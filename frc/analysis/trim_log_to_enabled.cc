#include <optional>

#include "absl/flags/flag.h"
#include "absl/flags/usage.h"
#include "absl/log/check.h"
#include "absl/log/log.h"

#include "aos/events/logging/log_reader.h"
#include "aos/init.h"
#include "aos/util/simulation_logger.h"
#include "frc/input/joystick_state_generated.h"

ABSL_FLAG(std::string, output_folder, "/tmp/trimmed/",
          "Name of the folder to write the trimmed log to.");
ABSL_FLAG(std::string, node, "roborio", "");
ABSL_FLAG(bool, auto, false, "If set, trim the log to just the auto mode.");
ABSL_FLAG(double, pre_enable_time_sec, 10.0,
          "Amount of time to leave in the new log before the first enable "
          "signal happens.");
ABSL_FLAG(double, post_enable_time_sec, 1.0,
          "Amount of time to leave in the new log after the final enable "
          "signal ends.");
ABSL_FLAG(double, force_start_monotonic, -1.0,
          "If set, time, in seconds, at which to forcibly trim the start "
          "of the log.");
ABSL_FLAG(
    double, force_end_monotonic, -1.0,
    "If set, time, in seconds, at which to forcibly trim the end of the log.");

int main(int argc, char *argv[]) {
  absl::SetProgramUsageMessage(
      "Trims the sections at the start/end of a log where the robot is "
      "disabled.");
  aos::InitGoogle(&argc, &argv);
  const std::vector<aos::logger::LogFile> logfiles =
      aos::logger::SortParts(aos::logger::FindLogs(argc, argv));
  std::optional<aos::monotonic_clock::time_point> start_time;
  std::optional<aos::monotonic_clock::time_point> end_time;
  bool printed_match = false;
  bool force_time_range = absl::GetFlag(FLAGS_force_start_monotonic) > 0;
  // We need to do two passes through the logfile; one to figure out when the
  // start/end times are, one to actually do the trimming.
  if (!force_time_range) {
    aos::logger::LogReader reader(logfiles);
    const aos::Node *roborio = aos::configuration::GetNode(
        reader.configuration(), absl::GetFlag(FLAGS_node));
    reader.Register();
    std::unique_ptr<aos::EventLoop> event_loop =
        reader.event_loop_factory()->MakeEventLoop(absl::GetFlag(FLAGS_node),
                                                   roborio);
    event_loop->MakeWatcher("/aos", [&start_time, &end_time, &printed_match,
                                     &event_loop](
                                        const frc::JoystickState &msg) {
      if (!printed_match && msg.match_type() != frc::MatchType::kNone) {
        LOG(INFO) << "Match Type: " << frc::EnumNameMatchType(msg.match_type());
        LOG(INFO) << "Match #: " << msg.match_number();
        printed_match = true;
      }

      if (msg.enabled() && (!absl::GetFlag(FLAGS_auto) || msg.autonomous())) {
        // Note that time is monotonic, so we don't need to e.g. do min's or
        // max's on the start/end time.
        if (!start_time.has_value()) {
          start_time = event_loop->context().monotonic_event_time;
        }
        end_time = event_loop->context().monotonic_event_time;
      }
    });

    reader.event_loop_factory()->Run();

    if (!printed_match) {
      LOG(INFO) << "No match info.";
    }
    if (!start_time.has_value()) {
      LOG(WARNING) << "Log does not ontain any JoystickState messages.";
      return 1;
    }
    LOG(INFO) << "First enable at " << start_time.value();
    LOG(INFO) << "Final enable at " << end_time.value();
    start_time.value() -= std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(
            absl::GetFlag(FLAGS_pre_enable_time_sec)));
    end_time.value() += std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(
            absl::GetFlag(FLAGS_post_enable_time_sec)));
  } else {
    CHECK_LT(absl::GetFlag(FLAGS_force_start_monotonic),
             absl::GetFlag(FLAGS_force_end_monotonic));
    start_time = aos::monotonic_clock::time_point(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(
                absl::GetFlag(FLAGS_force_start_monotonic))));
    end_time = aos::monotonic_clock::time_point(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(
                absl::GetFlag(FLAGS_force_end_monotonic))));
  }

  {
    aos::logger::LogReader reader(logfiles);
    const aos::Node *roborio = aos::configuration::GetNode(
        reader.configuration(), absl::GetFlag(FLAGS_node));
    reader.Register();
    std::unique_ptr<aos::EventLoop> event_loop =
        reader.event_loop_factory()->MakeEventLoop(absl::GetFlag(FLAGS_node),
                                                   roborio);
    auto exit_timer = event_loop->AddTimer(
        [&reader]() { reader.event_loop_factory()->Exit(); });
    exit_timer->Schedule(start_time.value());
    reader.event_loop_factory()->Run();
    const std::set<std::string> logger_nodes =
        aos::logger::LoggerNodes(logfiles);
    // Only start up loggers that generated the original set of logfiles.
    // This mostly exists to make it so that utilities like log_to_mcap can
    // easily auto-detect which node to replay as when consuming the input logs.
    auto loggers = aos::util::MakeLoggersForNodes(
        reader.event_loop_factory(), {logger_nodes.begin(), logger_nodes.end()},
        absl::GetFlag(FLAGS_output_folder));
    exit_timer->Schedule(end_time.value());

    reader.event_loop_factory()->Run();
  }

  LOG(INFO) << "Trimmed logs written to " << absl::GetFlag(FLAGS_output_folder);

  return EXIT_SUCCESS;
}
