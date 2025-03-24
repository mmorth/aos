#ifndef AOS_EVENTS_LOGGING_LOGFILE_DECODER_OPTIONS_H_
#define AOS_EVENTS_LOGGING_LOGFILE_DECODER_OPTIONS_H_
#include "aos/events/logging/buffer_encoder.h"
#include "aos/events/logging/file_operations.h"

// This library largely exists to isolate behavior which needs to depend on
// whether or not the current build supports certain types of operations---in
// particular, whether or not we have support for s3-based file reading and
// LZMA-based file reading.
namespace aos::logger::internal {
std::unique_ptr<DataDecoder> ResolveDecoder(std::string_view filename,
                                            bool quiet);
std::unique_ptr<internal::FileOperations> MakeFileOperations(
    std::string_view filename);
}  // namespace aos::logger::internal
#endif  // AOS_EVENTS_LOGGING_LOGFILE_DECODER_OPTIONS_H_
