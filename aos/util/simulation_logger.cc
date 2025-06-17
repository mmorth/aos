#include "aos/util/simulation_logger.h"

#include <algorithm>
#include <chrono>
#include <utility>

#include "absl/strings/str_cat.h"

#include "aos/configuration.h"
#include "aos/events/logging/logfile_utils.h"
#include "aos/time/time.h"

namespace aos::util {
LoggerState::LoggerState(aos::SimulatedEventLoopFactory *factory,
                         const aos::Node *node, std::string_view output_folder,
                         std::function<bool(const Channel *)> should_log,
                         bool do_skip_timing_report)
    : event_loop_(factory->MakeEventLoop("logger", node)),
      namer_(std::make_unique<aos::logger::MultiNodeFilesLogNamer>(
          absl::StrCat(output_folder, "/", logger::MaybeNodeName(node), "/"),
          event_loop_.get())),
      logger_(std::make_unique<aos::logger::Logger>(
          event_loop_.get(), event_loop_->configuration(),
          should_log ? should_log
                     : [](const aos::Channel *) { return true; })) {
  if (do_skip_timing_report) {
    event_loop_->SkipTimingReport();
  }
  event_loop_->SkipAosLog();

  // TODO (James, Maxwell) This shouldn't be necessary to have a delay here.
  // We'd like to have the logger start as soon as the event loop starts. The
  // logger must be started after (not on) `factory->send_delay()` amount of
  // time. Keep this simple, use two of those delays.
  TimerHandler *status_timer = event_loop_->AddTimer(
      [this]() { logger_->StartLogging(std::move(namer_)); });
  status_timer->Schedule(
      monotonic_clock::time_point(factory->send_delay() * 2));
}

std::vector<std::unique_ptr<LoggerState>> MakeLoggersForNodes(
    aos::SimulatedEventLoopFactory *factory,
    const std::vector<std::string> &nodes_to_log,
    std::string_view output_folder,
    std::function<bool(const Channel *)> should_log,
    bool do_skip_timing_report) {
  std::vector<std::unique_ptr<LoggerState>> loggers;
  for (const std::string &node : nodes_to_log) {
    loggers.emplace_back(std::make_unique<LoggerState>(
        factory, aos::configuration::GetNode(factory->configuration(), node),
        output_folder, should_log, do_skip_timing_report));
  }
  return loggers;
}

std::vector<std::unique_ptr<LoggerState>> MakeLoggersForAllNodes(
    aos::SimulatedEventLoopFactory *factory, std::string_view output_folder,
    std::function<bool(const Channel *)> should_log,
    bool do_skip_timing_report) {
  std::vector<std::unique_ptr<LoggerState>> loggers;
  for (const aos::Node *node :
       configuration::GetNodes(factory->configuration())) {
    loggers.emplace_back(std::make_unique<LoggerState>(
        factory, node, output_folder, should_log, do_skip_timing_report));
  }
  return loggers;
}

}  // namespace aos::util
