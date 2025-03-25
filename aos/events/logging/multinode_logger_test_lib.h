#ifndef AOS_EVENTS_LOGGING_MULTINODE_LOGGER_TEST_LIB_H
#define AOS_EVENTS_LOGGING_MULTINODE_LOGGER_TEST_LIB_H

#include "absl/flags/reflection.h"
#include "absl/strings/str_format.h"
#include "gmock/gmock.h"

#include "aos/events/event_loop.h"
#include "aos/events/logging/log_writer.h"
#include "aos/events/logging/snappy_encoder.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/network/testing_time_converter.h"
#include "aos/testing/path.h"
#include "aos/util/file.h"

#ifdef LZMA
#include "aos/events/logging/lzma_encoder.h"
#endif

namespace aos::logger {
class LogReader;
}

namespace aos::logger::testing {

struct CompressionParams {
  std::string_view extension;
  std::function<std::unique_ptr<DataEncoder>(size_t max_message_size)>
      encoder_factory;
};

enum class FileStrategy {
  // Use MinimalFileMultiNodeLogNamer
  kCombine,
  // Use MultiNodeFilesLogNamer
  kKeepSeparate,
};

enum class ForceTimestampBuffering {
  kForceBufferTimestamps,
  kAutoBuffer,
};

// Parameters to run all the tests with.
struct ConfigParams {
  // The config file to use.
  std::string config;
  // If true, the RemoteMessage channel should be shared between all the remote
  // channels.  If false, there will be 1 RemoteMessage channel per remote
  // channel.
  bool shared;
  // sha256 of the config.
  std::string_view sha256;
  // sha256 of the relogged config
  std::string_view relogged_sha256;
  // If kCombine, use MinimalFileMultiNodeLogNamer.
  FileStrategy file_strategy;
  // If kForceBufferTimestamps, set --force_timestamp_loading to force buffering
  // timestamps at the start.
  ForceTimestampBuffering timestamp_buffering;
};

struct LoggerState {
  void StartLogger(std::string logfile_base);

  std::unique_ptr<MultiNodeFilesLogNamer> MakeLogNamer(
      std::string logfile_base);

  std::unique_ptr<EventLoop> event_loop;
  std::unique_ptr<Logger> logger;
  const Configuration *configuration;
  const Node *node;
  MultiNodeFilesLogNamer *log_namer;
  CompressionParams params;
  FileStrategy file_strategy;
  aos::TimerHandler *start_timer;

  void AppendAllFilenames(std::vector<std::string> *filenames);

  ~LoggerState();
};

constexpr std::string_view kCombinedConfigSha1() {
  return "3d94be9641aaadaeaf7dafd234a988464e7cb04f3dc21f4acbb8431c4a1a9a04";
}
constexpr std::string_view kSplitConfigSha1() {
  return "3f7ddafc7c9ebccc64319d0e18a762184d863358d262603977f7e964a6bf235a";
}
constexpr std::string_view kReloggedSplitConfigSha1() {
  return "0e15b738fcd30642471f3e6cb545e8eec282c05913419e5f5c8ec49c354a5c5c";
}

LoggerState MakeLoggerState(NodeEventLoopFactory *node,
                            SimulatedEventLoopFactory *factory,
                            CompressionParams params,
                            FileStrategy file_strategy,
                            const Configuration *configuration = nullptr);
std::vector<std::vector<std::string>> ToLogReaderVector(
    const std::vector<LogFile> &log_files);
std::vector<CompressionParams> SupportedCompressionAlgorithms();
std::ostream &operator<<(std::ostream &ostream,
                         const CompressionParams &params);
std::ostream &operator<<(std::ostream &ostream, const ConfigParams &params);
std::vector<std::pair<std::vector<realtime_clock::time_point>,
                      std::vector<realtime_clock::time_point>>>
ConfirmReadable(
    const std::vector<std::string> &files,
    realtime_clock::time_point start_time = realtime_clock::min_time,
    realtime_clock::time_point end_time = realtime_clock::max_time);
// Counts the number of messages on a channel.  Returns (channel name, channel
// type, count) for every message matching matcher()
std::vector<std::tuple<std::string, std::string, int>> CountChannelsMatching(
    std::shared_ptr<const aos::Configuration> config, std::string_view filename,
    std::function<bool(const UnpackedMessageHeader *)> matcher);
// Counts the number of messages (channel, count) for all data messages.
std::vector<std::tuple<std::string, std::string, int>> CountChannelsData(
    std::shared_ptr<const aos::Configuration> config,
    std::string_view filename);
// Counts the number of messages (channel, count) for all timestamp messages.
std::vector<std::tuple<std::string, std::string, int>> CountChannelsTimestamp(
    std::shared_ptr<const aos::Configuration> config,
    std::string_view filename);

// Returns true if all of the max_out_of_order_duration's match the provided
// max_out_of_order_duration.
bool AllPartsMatchOutOfOrderDuration(
    const std::vector<LogFile> &files,
    std::chrono::nanoseconds max_out_of_order_duration =
        std::chrono::milliseconds(300));

// Skips checking the part file with boot_count 0 for 'node'.
bool AllRebootPartsMatchOutOfOrderDuration(
    const std::vector<LogFile> &files, const std::string node,
    std::chrono::nanoseconds max_out_of_order_duration =
        std::chrono::milliseconds(300));

class MultinodeLoggerTest : public ::testing::TestWithParam<
                                std::tuple<ConfigParams, CompressionParams>> {
 public:
  MultinodeLoggerTest();

  bool shared() const;
  FileStrategy file_strategy() const;

  std::vector<std::string> MakeLogFiles(std::string logfile_base1,
                                        std::string logfile_base2,
                                        size_t pi1_data_count = 1,
                                        size_t pi2_data_count = 1,
                                        size_t pi1_timestamp_count = 2,
                                        size_t pi2_timestamp_count = 2,
                                        bool relogged_config = false);

  std::vector<std::string> MakePi1RebootLogfiles();

  std::vector<std::string> MakePi1DeadNodeLogfiles();

  std::vector<std::vector<std::string>> StructureLogFiles();

  std::string Extension();

  LoggerState MakeLogger(NodeEventLoopFactory *node,
                         SimulatedEventLoopFactory *factory = nullptr,
                         const Configuration *configuration = nullptr);

  void StartLogger(LoggerState *logger, std::string logfile_base = "");

  void VerifyParts(const std::vector<LogFile> &sorted_parts,
                   const std::vector<std::string> &corrupted_parts = {});

  std::vector<std::pair<std::vector<realtime_clock::time_point>,
                        std::vector<realtime_clock::time_point>>>
  ConfirmReadable(
      const std::vector<std::string> &files,
      realtime_clock::time_point start_time = realtime_clock::min_time,
      realtime_clock::time_point end_time = realtime_clock::max_time);

  void AddExtension(std::string_view extension);

  bool HasSender(const aos::logger::LogReader &log_reader,
                 size_t channel_index);

  absl::FlagSaver flag_saver_;

  // Config and factory.
  aos::FlatbufferDetachedBuffer<aos::Configuration> config_;
  message_bridge::TestingTimeConverter time_converter_;
  SimulatedEventLoopFactory event_loop_factory_;

  NodeEventLoopFactory *const pi1_;
  const size_t pi1_index_;
  NodeEventLoopFactory *const pi2_;
  const size_t pi2_index_;

  std::string tmp_dir_;
  std::string logfile_base1_;
  std::string logfile_base2_;
  std::vector<std::string> pi1_reboot_logfiles_;
  std::vector<std::string> logfiles_;

  std::vector<std::vector<std::string>> structured_logfiles_;
};

typedef MultinodeLoggerTest MultinodeLoggerDeathTest;

}  // namespace aos::logger::testing

#endif  //  AOS_EVENTS_LOGGING_MULTINODE_LOGGER_TEST_LIB_H
