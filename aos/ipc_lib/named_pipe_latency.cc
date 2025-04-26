#include <fcntl.h>
#include <inttypes.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <compare>
#include <random>
#include <ratio>
#include <thread>

#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"

#include "aos/events/epoll.h"
#include "aos/init.h"
#include "aos/ipc_lib/latency_lib.h"
#include "aos/logging/implementations.h"
#include "aos/realtime.h"
#include "aos/time/time.h"

// This is a demo program which uses named pipes to communicate.
// It measures both latency of a random timer thread, and latency of the
// pipe.

ABSL_FLAG(bool, sender, true, "If true, send signals to the other process.");
ABSL_FLAG(std::string, fifo, "/dev/shm/aos/named_pipe_latency",
          "FIFO to use for the test.");
ABSL_FLAG(int32_t, seconds, 10, "Duration of the test to run");
ABSL_FLAG(
    int32_t, latency_threshold, 1000,
    "Disable tracing when anything takes more than this many microseoncds");
ABSL_FLAG(int32_t, core, 7, "Core to pin to");
ABSL_FLAG(int32_t, sender_priority, 53, "RT priority to send at");
ABSL_FLAG(int32_t, receiver_priority, 52, "RT priority to receive at");
ABSL_FLAG(int32_t, timer_priority, 51, "RT priority to spin the timer at");

ABSL_FLAG(bool, log_latency, false, "If true, log the latency");

namespace chrono = ::std::chrono;

namespace aos {

void SenderThread() {
  int pipefd = open(absl::GetFlag(FLAGS_fifo).c_str(),
                    FD_CLOEXEC | O_NONBLOCK | O_WRONLY | O_NOATIME);
  const monotonic_clock::time_point end_time =
      monotonic_clock::now() + chrono::seconds(absl::GetFlag(FLAGS_seconds));
  // Standard mersenne_twister_engine seeded with 0
  ::std::mt19937 generator(0);

  // Sleep between 1 and 15 ms.
  ::std::uniform_int_distribution<> distribution(1000, 15000);

  SetCurrentThreadAffinity(MakeCpusetFromCpus({absl::GetFlag(FLAGS_core)}));
  SetCurrentThreadRealtimePriority(absl::GetFlag(FLAGS_sender_priority));
  while (true) {
    const monotonic_clock::time_point wakeup_time =
        monotonic_clock::now() + chrono::microseconds(distribution(generator));

    ::std::this_thread::sleep_until(wakeup_time);
    const monotonic_clock::time_point monotonic_now = monotonic_clock::now();
    char sent_time_buffer[8];
    memcpy(sent_time_buffer, &monotonic_now, sizeof(sent_time_buffer));
    PCHECK(write(pipefd, static_cast<void *>(sent_time_buffer),
                 sizeof(sent_time_buffer)) ==
           static_cast<int>(sizeof(sent_time_buffer)));

    if (monotonic_now > end_time) {
      break;
    }
  }

  {
    char sent_time_buffer[8];
    memset(sent_time_buffer, 0, sizeof(sent_time_buffer));
    PCHECK(write(pipefd, static_cast<void *>(sent_time_buffer),
                 sizeof(sent_time_buffer)) ==
           static_cast<int>(sizeof(sent_time_buffer)));
  }
  UnsetCurrentThreadRealtimePriority();

  PCHECK(close(pipefd) == 0);
}

void ReceiverThread() {
  int pipefd = open(absl::GetFlag(FLAGS_fifo).c_str(),
                    O_CLOEXEC | O_NONBLOCK | O_RDONLY | O_NOATIME);
  Tracing t;
  t.Start();

  chrono::nanoseconds max_wakeup_latency = chrono::nanoseconds(0);

  chrono::nanoseconds sum_latency = chrono::nanoseconds(0);
  int latency_count = 0;

  internal::EPoll epoll;

  epoll.OnReadable(pipefd, [&t, &epoll, &max_wakeup_latency, &sum_latency,
                            &latency_count, pipefd]() {
    char sent_time_buffer[8];
    const int ret = read(pipefd, static_cast<void *>(sent_time_buffer),
                         sizeof(sent_time_buffer));
    const monotonic_clock::time_point monotonic_now = monotonic_clock::now();
    CHECK_EQ(ret, 8);

    monotonic_clock::time_point sent_time;
    memcpy(&sent_time, sent_time_buffer, sizeof(sent_time_buffer));

    if (sent_time == monotonic_clock::epoch()) {
      epoll.Quit();
      return;
    }

    const chrono::nanoseconds wakeup_latency = monotonic_now - sent_time;

    sum_latency += wakeup_latency;
    ++latency_count;

    max_wakeup_latency = ::std::max(wakeup_latency, max_wakeup_latency);

    if (wakeup_latency >
        chrono::microseconds(absl::GetFlag(FLAGS_latency_threshold))) {
      t.Stop();
      AOS_LOG(INFO, "Stopped tracing, latency %" PRId64 "\n",
              static_cast<int64_t>(wakeup_latency.count()));
    }

    if (absl::GetFlag(FLAGS_log_latency)) {
      AOS_LOG(INFO, "dt: %8d.%03d\n",
              static_cast<int>(wakeup_latency.count() / 1000),
              static_cast<int>(wakeup_latency.count() % 1000));
    }
  });

  SetCurrentThreadAffinity(MakeCpusetFromCpus({absl::GetFlag(FLAGS_core)}));
  SetCurrentThreadRealtimePriority(absl::GetFlag(FLAGS_receiver_priority));
  epoll.Run();
  UnsetCurrentThreadRealtimePriority();
  epoll.DeleteFd(pipefd);

  const chrono::nanoseconds average_latency = sum_latency / latency_count;

  AOS_LOG(
      INFO,
      "Max named pip wakeup latency: %d.%03d microseconds, average: %d.%03d "
      "microseconds\n",
      static_cast<int>(max_wakeup_latency.count() / 1000),
      static_cast<int>(max_wakeup_latency.count() % 1000),
      static_cast<int>(average_latency.count() / 1000),
      static_cast<int>(average_latency.count() % 1000));

  PCHECK(close(pipefd) == 0);
}

int Main(int /*argc*/, char ** /*argv*/) {
  mkfifo(absl::GetFlag(FLAGS_fifo).c_str(), 0777);

  AOS_LOG(INFO, "Main!\n");
  ::std::thread t([]() {
    TimerThread(
        monotonic_clock::now() + chrono::seconds(absl::GetFlag(FLAGS_seconds)),
        absl::GetFlag(FLAGS_timer_priority));
  });

  ::std::thread st([]() { SenderThread(); });

  ReceiverThread();
  st.join();

  t.join();
  return 0;
}

}  // namespace aos

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  return ::aos::Main(argc, argv);
}
