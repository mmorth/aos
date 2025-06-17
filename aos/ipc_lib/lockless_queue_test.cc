#include "aos/ipc_lib/lockless_queue.h"

#include <sched.h>
#include <stdio.h>
#include <string.h>
#include <sys/signalfd.h>

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <compare>
#include <csignal>
#include <new>
#include <ostream>
#include <random>
#include <ratio>
#include <thread>
#include <type_traits>

#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "gtest/gtest.h"

#include "aos/events/epoll.h"
#include "aos/ipc_lib/event.h"
#include "aos/ipc_lib/lockless_queue_memory.h"
#include "aos/ipc_lib/lockless_queue_stepping.h"
#include "aos/ipc_lib/queue_racer.h"
#include "aos/ipc_lib/signalfd.h"
#include "aos/realtime.h"
#include "aos/util/phased_loop.h"

ABSL_FLAG(int32_t, min_iterations, 100,
          "Minimum number of stress test iterations to run");
ABSL_FLAG(int32_t, duration, 5, "Number of seconds to test for");
ABSL_FLAG(int32_t, print_rate, 60, "Number of seconds between status prints");

// The roboRIO can only handle 10 threads before exploding.  Set the default for
// ARM to 10.
ABSL_FLAG(int32_t, thread_count,
#if defined(__ARM_EABI__)
          10,
#else
          100,
#endif
          "Number of threads to race");

namespace aos::ipc_lib::testing {

namespace chrono = ::std::chrono;

class LocklessQueueTest : public ::testing::Test {
 public:
  static constexpr monotonic_clock::duration kChannelStorageDuration =
      std::chrono::milliseconds(500);

  LocklessQueueTest() {
    config_.num_watchers = 10;
    config_.num_senders = 100;
    config_.num_pinners = 5;
    config_.queue_size = 10000;
    // Exercise the alignment code.  This would throw off alignment.
    config_.message_data_size = 101;

    // Since our backing store is an array of uint64_t for alignment purposes,
    // normalize by the size.
    memory_.resize(LocklessQueueMemorySize(config_) / sizeof(uint64_t));

    Reset();
  }

  LocklessQueue queue() {
    return LocklessQueue(reinterpret_cast<LocklessQueueMemory *>(&(memory_[0])),
                         reinterpret_cast<LocklessQueueMemory *>(&(memory_[0])),
                         config_);
  }

  void Reset() { memset(&memory_[0], 0, LocklessQueueMemorySize(config_)); }

  // Runs until the signal is received.
  void RunUntilWakeup(Event *ready, int priority) {
    internal::EPoll epoll;
    SignalFd signalfd({kWakeupSignal});

    epoll.OnReadable(signalfd.fd(), [&signalfd, &epoll]() {
      signalfd_siginfo result = signalfd.Read();

      fprintf(stderr, "Got signal: %d\n", result.ssi_signo);
      epoll.Quit();
    });

    {
      // Register to be woken up *after* the signalfd is catching the signals.
      LocklessQueueWatcher watcher =
          LocklessQueueWatcher::Make(queue(), priority).value();

      // And signal we are now ready.
      ready->Set();

      epoll.Run();

      // Cleanup, ensuring the watcher is destroyed before the signalfd.
    }
    epoll.DeleteFd(signalfd.fd());
  }

  // Use a type with enough alignment that we are guarenteed that everything
  // will be aligned properly on the target platform.
  ::std::vector<uint64_t> memory_;

  LocklessQueueConfiguration config_;
};

// Tests that wakeup doesn't do anything if nothing was registered.
TEST_F(LocklessQueueTest, NoWatcherWakeup) {
  LocklessQueueWakeUpper wake_upper(queue());

  EXPECT_EQ(wake_upper.Wakeup(7), 0);
}

// Tests that wakeup doesn't do anything if a wakeup was registered and then
// unregistered.
TEST_F(LocklessQueueTest, UnregisteredWatcherWakeup) {
  LocklessQueueWakeUpper wake_upper(queue());

  {
    LocklessQueueWatcher::Make(queue(), 5).value();
  }

  EXPECT_EQ(wake_upper.Wakeup(7), 0);
}

// Tests that wakeup doesn't do anything if the thread dies.
TEST_F(LocklessQueueTest, DiedWatcherWakeup) {
  LocklessQueueWakeUpper wake_upper(queue());

  ::std::thread([this]() {
    // Use placement new so the destructor doesn't get run.
    ::std::aligned_storage<sizeof(LocklessQueueWatcher),
                           alignof(LocklessQueueWatcher)>::type data;
    new (&data)
        LocklessQueueWatcher(LocklessQueueWatcher::Make(queue(), 5).value());
  }).join();

  EXPECT_EQ(wake_upper.Wakeup(7), 0);
}

struct WatcherState {
  ::std::thread t;
  Event ready;
};

// Tests that too many watchers fails like expected.
TEST_F(LocklessQueueTest, TooManyWatchers) {
  // This is going to be a barrel of monkeys.
  // We need to spin up a bunch of watchers.  But, they all need to be in
  // different threads so they have different tids.
  ::std::vector<WatcherState> queues;
  // Reserve num_watchers WatcherState objects so the pointer value doesn't
  // change out from under us below.
  queues.reserve(config_.num_watchers);

  // Event used to trigger all the threads to unregister.
  Event cleanup;

  // Start all the threads.
  for (size_t i = 0; i < config_.num_watchers; ++i) {
    queues.emplace_back();

    WatcherState *s = &queues.back();
    queues.back().t = ::std::thread([this, &cleanup, s]() {
      LocklessQueueWatcher q = LocklessQueueWatcher::Make(queue(), 0).value();

      // Signal that this thread is ready.
      s->ready.Set();

      // And wait until we are asked to shut down.
      cleanup.Wait();
    });
  }

  // Wait until all the threads are actually going.
  for (WatcherState &w : queues) {
    w.ready.Wait();
  }

  // Now try to allocate another one.  This will fail.
  EXPECT_FALSE(LocklessQueueWatcher::Make(queue(), 0));

  // Trigger the threads to cleanup their resources, and wait until they are
  // done.
  cleanup.Set();
  for (WatcherState &w : queues) {
    w.t.join();
  }

  // We should now be able to allocate a wakeup.
  EXPECT_TRUE(LocklessQueueWatcher::Make(queue(), 0));
}

// Tests that too many watchers dies like expected.
TEST_F(LocklessQueueTest, TooManySenders) {
  ::std::vector<LocklessQueueSender> senders;
  for (size_t i = 0; i < config_.num_senders; ++i) {
    senders.emplace_back(
        LocklessQueueSender::Make(queue(), kChannelStorageDuration).value());
  }
  EXPECT_FALSE(LocklessQueueSender::Make(queue(), kChannelStorageDuration));
}

// Now, start 2 threads and have them receive the signals.
TEST_F(LocklessQueueTest, WakeUpThreads) {
  // Confirm that the wakeup signal is in range.
  EXPECT_LE(kWakeupSignal, SIGRTMAX);
  EXPECT_GE(kWakeupSignal, SIGRTMIN);

  LocklessQueueWakeUpper wake_upper(queue());

  // Event used to make sure the thread is ready before the test starts.
  Event ready1;
  Event ready2;

  // Start the thread.
  ::std::thread t1([this, &ready1]() { RunUntilWakeup(&ready1, 2); });
  ::std::thread t2([this, &ready2]() { RunUntilWakeup(&ready2, 1); });

  ready1.Wait();
  ready2.Wait();

  EXPECT_EQ(wake_upper.Wakeup(3), 2);

  t1.join();
  t2.join();

  // Clean up afterwords.  We are pretending to be RT when we are really not.
  // So we will be PI boosted up.
  UnsetCurrentThreadRealtimePriority();
}

// Do a simple send test.
TEST_F(LocklessQueueTest, Send) {
  LocklessQueueSender sender =
      LocklessQueueSender::Make(queue(), kChannelStorageDuration).value();
  LocklessQueueReader reader(queue());

  time::PhasedLoop loop(kChannelStorageDuration / (config_.queue_size - 1),
                        monotonic_clock::now());
  std::function<bool(const Context &)> should_read = [](const Context &) {
    return true;
  };

  // Send enough messages to wrap.
  for (int i = 0; i < 2 * static_cast<int>(config_.queue_size); ++i) {
    // Confirm that the queue index makes sense given the number of sends.
    EXPECT_EQ(reader.LatestIndex().index(),
              i == 0 ? QueueIndex::Invalid().index() : i - 1);

    // Send a trivial piece of data.
    char data[100];
    size_t s = snprintf(data, sizeof(data), "foobar%d", i);
    ASSERT_EQ(sender.Send(data, s, monotonic_clock::min_time,
                          realtime_clock::min_time, monotonic_clock::min_time,
                          0xffffffffu, UUID::Zero(), nullptr, nullptr, nullptr),
              LocklessQueueSender::Result::GOOD);

    // Confirm that the queue index still makes sense.  This is easier since the
    // empty case has been handled.
    EXPECT_EQ(reader.LatestIndex().index(), i);

    // Read a result from 5 in the past.
    monotonic_clock::time_point monotonic_sent_time;
    realtime_clock::time_point realtime_sent_time;
    monotonic_clock::time_point monotonic_remote_time;
    monotonic_clock::time_point monotonic_remote_transmit_time;
    realtime_clock::time_point realtime_remote_time;
    uint32_t remote_queue_index;
    UUID source_boot_uuid;
    char read_data[1024];
    size_t length;

    QueueIndex index = QueueIndex::Zero(config_.queue_size);
    if (i - 5 < 0) {
      index = index.DecrementBy(5 - i);
    } else {
      index = index.IncrementBy(i - 5);
    }
    LocklessQueueReader::Result read_result = reader.Read(
        index.index(), &monotonic_sent_time, &realtime_sent_time,
        &monotonic_remote_time, &monotonic_remote_transmit_time,
        &realtime_remote_time, &remote_queue_index, &source_boot_uuid, &length,
        &(read_data[0]), std::ref(should_read));

    // This should either return GOOD, or TOO_OLD if it is before the start of
    // the queue.
    if (read_result != LocklessQueueReader::Result::GOOD) {
      ASSERT_EQ(read_result, LocklessQueueReader::Result::TOO_OLD);
    }

    loop.SleepUntilNext();
  }
}

// Races a bunch of sending threads to see if it all works.
TEST_F(LocklessQueueTest, SendRace) {
  const size_t kNumMessages = 10000 / absl::GetFlag(FLAGS_thread_count);

  ::std::mt19937 generator(0);
  ::std::uniform_int_distribution<> write_wrap_count_distribution(0, 10);
  ::std::bernoulli_distribution race_reads_distribution;
  ::std::bernoulli_distribution set_should_read_distribution;
  ::std::bernoulli_distribution should_read_result_distribution;
  ::std::bernoulli_distribution wrap_writes_distribution;

  const chrono::seconds print_frequency(absl::GetFlag(FLAGS_print_rate));

  QueueRacer racer(queue(), absl::GetFlag(FLAGS_thread_count), kNumMessages);
  const monotonic_clock::time_point start_time = monotonic_clock::now();
  const monotonic_clock::time_point end_time =
      start_time + chrono::seconds(absl::GetFlag(FLAGS_duration));

  monotonic_clock::time_point monotonic_now = start_time;
  monotonic_clock::time_point next_print_time = start_time + print_frequency;
  uint64_t messages = 0;
  for (int i = 0;
       i < absl::GetFlag(FLAGS_min_iterations) || monotonic_now < end_time;
       ++i) {
    const bool race_reads = race_reads_distribution(generator);
    const bool set_should_read = set_should_read_distribution(generator);
    const bool should_read_result = should_read_result_distribution(generator);
    int write_wrap_count = write_wrap_count_distribution(generator);
    if (!wrap_writes_distribution(generator)) {
      write_wrap_count = 0;
    }
    EXPECT_NO_FATAL_FAILURE(racer.RunIteration(
        race_reads, write_wrap_count, set_should_read, should_read_result))
        << ": Running with race_reads: " << race_reads
        << ", and write_wrap_count " << write_wrap_count << " and on iteration "
        << i;

    messages += racer.CurrentIndex();

    monotonic_now = monotonic_clock::now();
    if (monotonic_now > next_print_time) {
      double elapsed_seconds = chrono::duration_cast<chrono::duration<double>>(
                                   monotonic_now - start_time)
                                   .count();
      printf("Finished iteration %d, %f iterations/sec, %f messages/second\n",
             i, i / elapsed_seconds,
             static_cast<double>(messages) / elapsed_seconds);
      next_print_time = monotonic_now + print_frequency;
    }
  }
}

namespace {

// Temporarily pins the current thread to the first 2 available CPUs.
// This speeds up the test on some machines a lot (~4x). It also preserves
// opportunities for the 2 threads to race each other.
class PinForTest {
 public:
  PinForTest() {
    cpu_set_t cpus = GetCurrentThreadAffinity();
    old_ = cpus;
    int number_found = 0;
    for (int i = 0; i < CPU_SETSIZE; ++i) {
      if (CPU_ISSET(i, &cpus)) {
        if (number_found < 2) {
          ++number_found;
        } else {
          CPU_CLR(i, &cpus);
        }
      }
    }
    SetCurrentThreadAffinity(cpus);
  }
  ~PinForTest() { SetCurrentThreadAffinity(old_); }

 private:
  cpu_set_t old_;
};

}  // namespace

class LocklessQueueTestTooFast : public LocklessQueueTest {
 public:
  LocklessQueueTestTooFast() {
    // Force a scenario where senders get rate limited
    config_.num_watchers = 1000;
    config_.num_senders = 100;
    config_.num_pinners = 5;
    config_.queue_size = 100;
    // Exercise the alignment code.  This would throw off alignment.
    config_.message_data_size = 101;

    // Since our backing store is an array of uint64_t for alignment purposes,
    // normalize by the size.
    memory_.resize(LocklessQueueMemorySize(config_) / sizeof(uint64_t));

    Reset();
  }
};

// Ensure we always return OK or MESSAGES_SENT_TOO_FAST under an extreme load
// on the Sender Queue.
TEST_F(LocklessQueueTestTooFast, MessagesSentTooFast) {
  PinForTest pin_cpu;
  uint64_t kNumMessages = 1000000;
  QueueRacer racer(queue(),
                   {absl::GetFlag(FLAGS_thread_count),
                    kNumMessages,
                    {LocklessQueueSender::Result::GOOD,
                     LocklessQueueSender::Result::MESSAGES_SENT_TOO_FAST},
                    std::chrono::milliseconds(500),
                    false});

  EXPECT_NO_FATAL_FAILURE(racer.RunIteration(false, 0, true, true));
}

// // Send enough messages to wrap the 32 bit send counter.
TEST_F(LocklessQueueTest, WrappedSend) {
  PinForTest pin_cpu;
  uint64_t kNumMessages = 0x100010000ul;
  QueueRacer racer(queue(), 1, kNumMessages);

  const monotonic_clock::time_point start_time = monotonic_clock::now();
  EXPECT_NO_FATAL_FAILURE(racer.RunIteration(false, 0, false, true));
  const monotonic_clock::time_point monotonic_now = monotonic_clock::now();
  double elapsed_seconds = chrono::duration_cast<chrono::duration<double>>(
                               monotonic_now - start_time)
                               .count();
  printf("Took %f seconds to write %" PRIu64 " messages, %f messages/s\n",
         elapsed_seconds, kNumMessages,
         static_cast<double>(kNumMessages) / elapsed_seconds);
}

#if defined(SUPPORTS_SHM_ROBUSTNESS_TEST)

// Verifies that LatestIndex points to the same message as the logic from
// "FetchNext", which increments the index until it gets "NOTHING_NEW" back.
// This is so we can confirm fetchers and watchers all see the same message at
// the same point in time.
int VerifyMessages(LocklessQueue *queue, LocklessQueueMemory *memory) {
  LocklessQueueReader reader(*queue);

  const ipc_lib::QueueIndex queue_index = reader.LatestIndex();
  if (!queue_index.valid()) {
    return 0;
  }

  // Now loop through the queue and make sure the number in the snprintf
  // increments.
  char last_data = '0';
  int i = 0;

  // Callback which isn't set so we don't exercise the conditional reading code.
  std::function<bool(const Context &)> should_read_callback;

  // Now, read as far as we can until we get NOTHING_NEW.  This simulates
  // FetchNext.
  while (true) {
    monotonic_clock::time_point monotonic_sent_time;
    realtime_clock::time_point realtime_sent_time;
    monotonic_clock::time_point monotonic_remote_time;
    monotonic_clock::time_point monotonic_remote_transmit_time;
    realtime_clock::time_point realtime_remote_time;
    uint32_t remote_queue_index;
    UUID source_boot_uuid;
    char read_data[1024];
    size_t length;

    LocklessQueueReader::Result read_result = reader.Read(
        i, &monotonic_sent_time, &realtime_sent_time, &monotonic_remote_time,
        &monotonic_remote_transmit_time, &realtime_remote_time,
        &remote_queue_index, &source_boot_uuid, &length, &(read_data[0]),
        should_read_callback);

    if (read_result != LocklessQueueReader::Result::GOOD) {
      if (read_result == LocklessQueueReader::Result::TOO_OLD) {
        ++i;
        continue;
      }
      ABSL_CHECK(read_result == LocklessQueueReader::Result::NOTHING_NEW)
          << ": " << static_cast<int>(read_result);
      break;
    }

    EXPECT_GT(read_data[LocklessQueueMessageDataSize(memory) - length + 6],
              last_data)
        << ": Got " << read_data << " for " << i;
    last_data = read_data[LocklessQueueMessageDataSize(memory) - length + 6];

    ++i;
  }

  // The latest queue index should match the fetched queue index.
  if (i == 0) {
    EXPECT_FALSE(queue_index.valid());
  } else {
    EXPECT_EQ(queue_index.index(), i - 1);
  }
  return i;
}

// Tests that at all points in the publish step, fetch == fetch next.  This
// means that there is an atomic point at which the message is viewed as visible
// to consumers.  Do this by killing the writer after each change to shared
// memory, and confirming fetch == fetch next each time.
TEST_F(LocklessQueueTest, FetchEqFetchNext) {
  SharedTid tid;

  // Make a small queue so it is easier to debug.
  LocklessQueueConfiguration config;
  config.num_watchers = 1;
  config.num_senders = 2;
  config.num_pinners = 0;
  config.queue_size = 3;
  config.message_data_size = 32;

  TestShmRobustness(
      config,
      [config, &tid](void *memory) {
        // Initialize the queue.
        LocklessQueue(
            reinterpret_cast<aos::ipc_lib::LocklessQueueMemory *>(memory),
            reinterpret_cast<aos::ipc_lib::LocklessQueueMemory *>(memory),
            config)
            .Initialize();
        tid.Set();
      },
      [config](void *memory) {
        LocklessQueue queue(
            reinterpret_cast<aos::ipc_lib::LocklessQueueMemory *>(memory),
            reinterpret_cast<aos::ipc_lib::LocklessQueueMemory *>(memory),
            config);
        // Now try to write some messages.  We will get killed a bunch as this
        // tries to happen.
        LocklessQueueSender sender =
            LocklessQueueSender::Make(queue, chrono::nanoseconds(1)).value();
        for (int i = 0; i < 5; ++i) {
          char data[100];
          size_t s = snprintf(data, sizeof(data), "foobar%d", i + 1);
          ASSERT_EQ(
              sender.Send(data, s + 1, monotonic_clock::min_time,
                          realtime_clock::min_time, monotonic_clock::min_time,
                          0xffffffffl, UUID::Zero(), nullptr, nullptr, nullptr),
              LocklessQueueSender::Result::GOOD);
        }
      },
      [config, &tid](void *raw_memory) {
        ::aos::ipc_lib::LocklessQueueMemory *const memory =
            reinterpret_cast<::aos::ipc_lib::LocklessQueueMemory *>(raw_memory);
        LocklessQueue queue(memory, memory, config);
        PretendThatOwnerIsDeadForTesting(&memory->queue_setup_lock, tid.Get());

        if (ABSL_VLOG_IS_ON(1)) {
          PrintLocklessQueueMemory(memory);
        }

        const int i = VerifyMessages(&queue, memory);

        LocklessQueueSender sender =
            LocklessQueueSender::Make(queue, chrono::nanoseconds(1)).value();
        {
          char data[100];
          size_t s = snprintf(data, sizeof(data), "foobar%d", i + 1);
          ASSERT_EQ(
              sender.Send(data, s + 1, monotonic_clock::min_time,
                          realtime_clock::min_time, monotonic_clock::min_time,
                          0xffffffffl, UUID::Zero(), nullptr, nullptr, nullptr),
              LocklessQueueSender::Result::GOOD);
        }

        // Now, make sure we can send 1 message and receive it to confirm we
        // haven't corrupted next_queue_index irrevocably.
        const int newi = VerifyMessages(&queue, memory);
        EXPECT_EQ(newi, i + 1);
      });
}

#endif

}  // namespace aos::ipc_lib::testing
