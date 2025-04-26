#include "aos/libc/aos_strsignal.h"

#include <string.h>

#include <csignal>
#include <functional>
#include <memory>
#include <thread>

#include "gtest/gtest.h"

#include "aos/sanitizers.h"

namespace aos::libc::testing {

// Tries a couple of easy ones.
TEST(StrsignalTest, Basic) {
  EXPECT_STREQ("Hangup", aos_strsignal(SIGHUP));
  EXPECT_STREQ("Broken pipe", aos_strsignal(SIGPIPE));
  EXPECT_STREQ("Real-time signal 2", aos_strsignal(SIGRTMIN + 2));
  EXPECT_STREQ("Unknown signal 155", aos_strsignal(155));
}

class SignalNameTester {
 public:
  void operator()() {
    for (int i = 0; i < SIGRTMAX + 5; ++i) {
      EXPECT_STREQ(strsignal(i), aos_strsignal(i));
    }
  }
};

// msan doesn't seem to like strsignal().
#if !defined(AOS_SANITIZE_MEMORY)
// Tests that all the signals give the same result as strsignal(3).
TEST(StrsignalTest, All) {
  // Sigh, strsignal allocates a buffer that uses pthread local storage.  This
  // interacts poorly with asan.  Spawning a thread causes the storage to get
  // cleaned up before asan checks.
  SignalNameTester t;
#if defined(AOS_SANITIZER_thread)
  // tsan doesn't like this usage of ::std::thread. It looks like
  // <https://gcc.gnu.org/bugzilla/show_bug.cgi?id=57507>.
  t();
#else
  ::std::thread thread(::std::ref(t));
  thread.join();
#endif
}
#endif

}  // namespace aos::libc::testing
