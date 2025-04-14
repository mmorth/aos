#ifndef AOS_UTIL_STRING_FORMATTING_H_
#define AOS_UTIL_STRING_FORMATTING_H_

#include <string>

namespace aos::util {

// Returns a formatted string with the specified fractional precision. Remove
// the trailing zeros after the first trailing zero to the right of the decimal
// point. This can be used for float or double types.
std::string FormatFloat(double value, int precision);

// Returns true if the provided string contains only valid UTF-8 characters.
bool ValidateUtf8(const std::string_view string);

}  // namespace aos::util

#endif  // AOS_UTIL_STRING_FORMATTING_H_
