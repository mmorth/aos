#ifndef AOS_UTIL_LOG_TO_MCAP_LIB_H_
#define AOS_UTIL_LOG_TO_MCAP_LIB_H_

#include <string>
#include <vector>

namespace aos::util {

// Converts an AOS log to an MCAP log that can be fed into Foxglove.
//
// This function is intended to be called directly from main(). It's a separate
// file to let AOS users create their own "log_to_mcap" binaries. To try the AOS
// version out, run:
//  bazel run //aos/util:log_to_mcap -- --node NODE_NAME /path/to/logfile
//
// Then navigate to http://app.foxglove.dev (or use the desktop app),
// and use it to open the file (this doesn't upload the file to
// foxglove's servers or anything).
int ConvertLogToMcap(const std::vector<std::string> &log_paths,
                     std::string output_path);

}  // namespace aos::util

#endif  // AOS_UTIL_LOG_TO_MCAP_LIB_H_
