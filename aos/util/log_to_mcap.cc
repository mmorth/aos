#include <algorithm>
#include <memory>
#include <optional>
#include <ostream>
#include <set>
#include <string>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "flatbuffers/reflection_generated.h"

#include "aos/configuration.h"
#include "aos/events/event_loop.h"
#include "aos/events/logging/log_reader.h"
#include "aos/events/logging/logfile_sorting.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/flatbuffers.h"
#include "aos/init.h"
#include "aos/util/clock_publisher.h"
#include "aos/util/clock_timepoints_schema.h"
#include "aos/util/mcap_logger.h"

ABSL_FLAG(std::string, node, "", "Node to replay from the perspective of.");
ABSL_FLAG(std::string, mode, "flatbuffer", "json or flatbuffer serialization.");
ABSL_FLAG(
    bool, canonical_channel_names, false,
    "If set, use full channel names; by default, will shorten names to be the "
    "shortest possible version of the name (e.g., /aos instead of /pi/aos).");
ABSL_FLAG(bool, compress, true, "Whether to use LZ4 compression in MCAP file.");
ABSL_FLAG(bool, include_clocks, true,
          "Whether to add a /clocks channel that publishes all nodes' clock "
          "offsets.");
ABSL_FLAG(bool, fetch, false,
          "If set, *all* messages in the logfile will be included, including "
          "any that may have occurred prior to the start of the log. This "
          "can be used to see additional data, but given that data may be "
          "incomplete prior to the start of the log, you should be careful "
          "about interpretting data flow when using this flag.");
ABSL_FLAG(std::vector<std::string>, drop_channels, {},
          "A comma-separated list of MCAP topic names to drop. This looks like "
          "so: --drop_channels='/0/foo a.b.Msg1,/0/bar a.c.Msg2'.");

// Converts an AOS log to an MCAP log that can be fed into Foxglove. To try this
// out, run:
//  bazel run //aos/util:log_to_mcap -- --node NODE /path/to/log [/tmp/log.mcap]
//
// Then navigate to http://studio.foxglove.dev (or spin up your own instance
// locally), and use it to open the file (this doesn't upload the file to
// foxglove's servers or anything).
int main(int argc, char *argv[]) {
  aos::InitGoogle(&argc, &argv);

  std::string output_path = "/tmp/log.mcap";
  if (argc < 2) {
    LOG(FATAL) << "Usage: " << argv[0] << " path/to/log [output.mcap]";
  }
  if (argc >= 3) {
    output_path = argv[argc - 1];
    --argc;
  }

  const std::vector<aos::logger::LogFile> logfiles =
      aos::logger::SortParts(aos::logger::FindLogs(argc, argv));
  CHECK(!logfiles.empty());
  const std::set<std::string> logger_nodes = aos::logger::LoggerNodes(logfiles);
  CHECK_LT(0u, logger_nodes.size());
  const std::string logger_node = *logger_nodes.begin();
  std::string replay_node = absl::GetFlag(FLAGS_node);
  if (replay_node.empty()) {
    if (logger_nodes.size() == 1u) {
      LOG(INFO) << "Guessing \"" << logger_node
                << "\" as node given that --node was not specified.";
      replay_node = logger_node;
    } else {
      LOG(ERROR) << "Must supply a --node for log_to_mcap.";
      return 1;
    }
  }

  std::optional<aos::FlatbufferDetachedBuffer<aos::Configuration>> config;

  if (absl::GetFlag(FLAGS_include_clocks)) {
    aos::logger::LogReader config_reader(logfiles);

    if (aos::configuration::MultiNode(config_reader.configuration())) {
      CHECK(!replay_node.empty()) << ": Must supply a --node.";
    }

    const aos::Configuration *raw_config = config_reader.logged_configuration();
    // The ClockTimepoints message for multiple VPUs is bigger than the default
    // 1000 bytes. So we need to set a bigger size here.
    aos::ChannelT channel_overrides;
    channel_overrides.max_size = 2000;
    config = aos::configuration::AddChannelToConfiguration(
        raw_config, "/clocks",
        aos::FlatbufferSpan<reflection::Schema>(aos::ClockTimepointsSchema()),
        replay_node.empty()
            ? nullptr
            : aos::configuration::GetNode(raw_config, replay_node),
        channel_overrides);
  }

  aos::logger::LogReader reader(
      logfiles, config.has_value() ? &config.value().message() : nullptr);
  aos::SimulatedEventLoopFactory factory(reader.configuration());
  reader.RegisterWithoutStarting(&factory);

  if (aos::configuration::MultiNode(reader.configuration())) {
    CHECK(!replay_node.empty()) << ": Must supply a --node.";
  }

  const aos::Node *node =
      !aos::configuration::MultiNode(reader.configuration())
          ? nullptr
          : aos::configuration::GetNode(reader.configuration(), replay_node);

  std::unique_ptr<aos::EventLoop> clock_event_loop;
  std::unique_ptr<aos::ClockPublisher> clock_publisher;

  std::unique_ptr<aos::EventLoop> mcap_event_loop;
  std::unique_ptr<aos::McapLogger> relogger;
  auto startup_handler = [&relogger, &mcap_event_loop, &reader,
                          &clock_event_loop, &clock_publisher, &factory, node,
                          output_path]() {
    CHECK(!mcap_event_loop) << ": log_to_mcap does not support generating MCAP "
                               "files from multi-boot logs.";
    mcap_event_loop = reader.event_loop_factory()->MakeEventLoop("mcap", node);
    relogger = std::make_unique<aos::McapLogger>(
        mcap_event_loop.get(), output_path,
        absl::GetFlag(FLAGS_mode) == "flatbuffer"
            ? aos::McapLogger::Serialization::kFlatbuffer
            : aos::McapLogger::Serialization::kJson,
        absl::GetFlag(FLAGS_canonical_channel_names)
            ? aos::McapLogger::CanonicalChannelNames::kCanonical
            : aos::McapLogger::CanonicalChannelNames::kShortened,
        absl::GetFlag(FLAGS_compress) ? aos::McapLogger::Compression::kLz4
                                      : aos::McapLogger::Compression::kNone,
        absl::GetFlag(FLAGS_drop_channels));
    if (absl::GetFlag(FLAGS_include_clocks)) {
      clock_event_loop =
          reader.event_loop_factory()->MakeEventLoop("clock", node);
      clock_publisher = std::make_unique<aos::ClockPublisher>(
          &factory, clock_event_loop.get());
    }
  };
  if (absl::GetFlag(FLAGS_fetch)) {
    // Note: This condition is subtly different from just calling Fetch() on
    // every channel in OnStart(). Namely, if there is >1 message on a given
    // channel prior to the logfile start, then fetching in the reader OnStart()
    // is insufficient to get *all* log data.
    factory.GetNodeEventLoopFactory(node)->OnStartup(startup_handler);
  } else {
    reader.OnStart(node, startup_handler);
  }
  reader.event_loop_factory()->Run();
  reader.Deregister();
}
