#include "aos/containers/inlined_vector.h"

#include "absl/flags/declare.h"
#include "absl/flags/flag.h"
#include "absl/flags/reflection.h"
#include "gtest/gtest.h"

#include "aos/realtime.h"
#include "aos/sanitizers.h"

ABSL_DECLARE_FLAG(bool, die_on_malloc);

namespace aos::testing {

// Checks that we don't malloc until/unless we need to increase the size of the
// vector.
TEST(SizedArrayTest, NoUnnecessaryMalloc) {
  absl::FlagSaver flag_saver;
  absl::SetFlag(&FLAGS_die_on_malloc, true);
  RegisterMallocHook();
  InlinedVector<int, 5> a;
  {
    aos::ScopedRealtime realtime;
    a.push_back(9);
    a.push_back(7);
    a.push_back(1);
    a.push_back(2);
    a.push_back(3);

    // And double-check that we can actually construct a new object at realtime.
    InlinedVector<int, 5> b;
  }
// Malloc hooks don't work with asan/msan.
#if !defined(AOS_SANITIZE_MEMORY) && !defined(AOS_SANITIZE_ADDRESS)
  EXPECT_DEATH(
      {
        aos::ScopedRealtime realtime;
        a.push_back(4);
      },
      "Malloced");
#endif
}

// Tests that we can create/define a vector with zero statically allocated
// elements (the absl::InlinedVector does not allow this for some reason).
TEST(SizedArrayTest, ZeroLengthVector) {
  InlinedVector<int, 0> zero;
  zero.push_back(1);
  ASSERT_EQ(1, zero[0]);
}
}  // namespace aos::testing
