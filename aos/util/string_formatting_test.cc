#include "aos/util/string_formatting.h"

#include "gtest/gtest.h"

namespace aos::util::testing {
TEST(StringFormattingTest, ValidateUtf8) {
  EXPECT_TRUE(ValidateUtf8("Test Ascii String!\n"));
  EXPECT_TRUE(ValidateUtf8("Test UTF-8 String! 😀\n"));
  EXPECT_FALSE(ValidateUtf8("Invalid Character \xFF"));
}
}  // namespace aos::util::testing
