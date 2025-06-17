#include "aos/logging/interface.h"

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <memory>
#include <ostream>
#include <string>
#include <type_traits>

#include "absl/log/absl_log.h"

#include "aos/die.h"
#include "aos/logging/context.h"
#include "aos/logging/implementations.h"
#include "aos/time/time.h"

namespace aos::logging {
namespace internal {

size_t ExecuteFormat(char *output, size_t output_size, const char *format,
                     va_list ap) {
  static const char *const continued = "...\n";
  const size_t size = output_size - strlen(continued);
  const int ret = vsnprintf(output, size, format, ap);
  typedef ::std::common_type<int, size_t>::type RetType;
  if (ret < 0) {
    ABSL_PLOG(FATAL) << "vsnprintf(" << output << ", " << size << ", " << format
                     << ", args) failed";
  } else if (static_cast<RetType>(ret) >= static_cast<RetType>(size)) {
    // Overwrite the '\0' at the end of the existing data and
    // copy in the one on the end of continued.
    memcpy(&output[size - 1], continued, strlen(continued) + 1);
  }
  return ::std::min<RetType>(ret, size);
}

}  // namespace internal

using internal::Context;

void VLog(log_level level, const char *format, va_list ap) {
  va_list ap1;
  va_copy(ap1, ap);

  Context *context = Context::Get();

  const std::shared_ptr<LogImplementation> implementation =
      context->implementation;
  // Log to the implementation if we have it, and stderr as a backup.
  if (implementation) {
    implementation->DoLog(level, format, ap1);
  } else {
    aos::logging::LogMessage message;
    aos::logging::internal::FillInMessage(level, context->MyName(),
                                          aos::monotonic_clock::now(), format,
                                          ap, &message);
    aos::logging::internal::PrintMessage(stderr, message);
  }
  va_end(ap1);

  if (level == FATAL) {
    VDie(format, ap);
  }
}

}  // namespace aos::logging

void log_do(log_level level, const char *format, ...) {
  va_list ap;
  va_start(ap, format);
  aos::logging::VLog(level, format, ap);
  va_end(ap);
}
