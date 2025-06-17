#include "aos/fast_string_builder.h"

#include <charconv>

#include "absl/log/absl_check.h"

namespace aos {

FastStringBuilder::FastStringBuilder(std::size_t initial_size) {
  str_.reserve(initial_size);
}

void FastStringBuilder::Reset() { str_.resize(0); }

std::string_view FastStringBuilder::Result() const {
  return std::string_view(str_);
}

std::string FastStringBuilder::MoveResult() {
  std::string result;
  result.swap(str_);
  return result;
}

void FastStringBuilder::Resize(std::size_t chars_needed) {
  str_.resize(str_.size() + chars_needed);
}

void FastStringBuilder::Append(std::string_view str) {
  std::size_t index = str_.size();
  Resize(str.size());
  str_.replace(index, str.size(), str, 0);
}

void FastStringBuilder::Append(float val) {
  if (val == 0.0 && std::signbit(val)) {
    // By default a -0 will be represented as "-0" by std::to_chars, which some
    // JSON parsers will interpret as a "-0" *integer*, which causes them to
    // lose the sign bit. As such, force any parsers to interpret a -0 as a
    // floating point number.
    Append("-0.0");
    return;
  }
  std::size_t index = str_.size();
  constexpr std::size_t kMaxSize = 17;
  Resize(kMaxSize);
  const std::to_chars_result result =
      std::to_chars(str_.data() + index, str_.data() + index + kMaxSize, val);
  ABSL_CHECK(result.ec == std::errc())
      << std::make_error_code(result.ec).message();
  str_.resize(result.ptr - str_.data());
}

void FastStringBuilder::Append(double val) {
  if (val == 0.0 && std::signbit(val)) {
    // By default a -0 will be represented as "-0" by std::to_chars, which some
    // JSON parsers will interpret as a "-0" *integer*, which causes them to
    // lose the sign bit. As such, force any parsers to interpret a -0 as a
    // floating point number.
    Append("-0.0");
    return;
  }
  std::size_t index = str_.size();
  constexpr std::size_t kMaxSize = 25;
  Resize(kMaxSize);
  const std::to_chars_result result =
      std::to_chars(str_.data() + index, str_.data() + index + kMaxSize, val);
  ABSL_CHECK(result.ec == std::errc())
      << std::make_error_code(result.ec).message();
  str_.resize(result.ptr - str_.data());
}

void FastStringBuilder::AppendChar(char val) {
  Resize(1);
  str_[str_.size() - 1] = val;
}

void FastStringBuilder::AppendBool(bool val) {
  if (bool_to_str_) {
    Append(val ? kTrue : kFalse);
  } else {
    AppendChar(val ? '1' : '0');
  }
}

std::ostream &operator<<(std::ostream &os, const FastStringBuilder &builder) {
  os.write(builder.str_.data(), builder.str_.size());
  return os;
}

}  // namespace aos
