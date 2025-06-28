#include "aos/events/logging/log_reader.h"

#include <dirent.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/uio.h>

#include <climits>
#include <utility>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/strings/escaping.h"
#include "absl/types/span.h"
#include "flatbuffers/flatbuffers.h"
#include "openssl/sha.h"

#include "aos/events/event_loop.h"
#include "aos/events/logging/boot_timestamp.h"
#include "aos/events/logging/logfile_sorting.h"
#include "aos/events/logging/logger_generated.h"
#include "aos/flatbuffer_merge.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/network/multinode_timestamp_filter.h"
#include "aos/network/remote_message_generated.h"
#include "aos/network/remote_message_schema.h"
#include "aos/network/team_number.h"
#include "aos/network/timestamp_channel.h"
#include "aos/time/time.h"
#include "aos/util/file.h"
#include "aos/uuid.h"

ABSL_FLAG(bool, skip_missing_forwarding_entries, false,
          "If true, drop any forwarding entries with missing data.  If "
          "false, CHECK.");

ABSL_DECLARE_FLAG(bool, timestamps_to_csv);
ABSL_FLAG(
    bool, enable_timestamp_loading, true,
    "Enable loading all the timestamps into RAM at startup if they are in "
    "separate files.  This fixes any timestamp queueing problems for the cost "
    "of storing timestamps in RAM only on logs with timestamps logged in "
    "separate files from data.  Only disable this if you are reading a really "
    "long log file and are experiencing memory problems due to loading all the "
    "timestamps into RAM.");
ABSL_FLAG(
    bool, force_timestamp_loading, false,
    "Force loading all the timestamps into RAM at startup.  This fixes any "
    "timestamp queueing problems for the cost of storing timestamps in RAM and "
    "potentially reading each log twice.");

ABSL_FLAG(bool, skip_order_validation, false,
          "If true, ignore any out of orderness in replay");

ABSL_FLAG(
    double, time_estimation_buffer_seconds, 2.0,
    "The time to buffer ahead in the log file to accurately reconstruct time.");

ABSL_FLAG(
    std::string, start_time, "",
    "If set, start at this point in time in the log on the realtime clock. "
    "Times are expected to be in the form of 2021-01-15_15-30-35.000000000.");
ABSL_FLAG(
    std::string, end_time, "",
    "If set, end at this point in time in the log on the realtime clock. Times "
    "are expected to be in the form of 2021-01-15_15-30-35.000000000.");

ABSL_FLAG(bool, drop_realtime_messages_before_start, false,
          "If set, will drop any messages sent before the start of the "
          "logfile in realtime replay. Setting this guarantees consistency "
          "in timing with the original logfile, but means that you lose "
          "access to fetched low-frequency messages.");

ABSL_FLAG(
    double, threaded_look_ahead_seconds, 2.0,
    "Time, in seconds, to add to look-ahead when using multi-threaded replay. "
    "Can validly be zero, but higher values are encouraged for realtime replay "
    "in order to prevent the replay from ever having to block on waiting for "
    "the reader to find the next message.");

namespace aos {
namespace configuration {
// We don't really want to expose this publicly, but log reader doesn't really
// want to re-implement it.
void HandleMaps(const flatbuffers::Vector<flatbuffers::Offset<aos::Map>> *maps,
                std::string *name, std::string_view type, const Node *node);
}  // namespace configuration
namespace logger {
namespace {

namespace chrono = std::chrono;
using message_bridge::RemoteMessage;
}  // namespace

// Class to manage triggering events on the RT clock while replaying logs. Since
// the RT clock can only change when we get a message, we only need to update
// our timers when new messages are read.
class EventNotifier {
 public:
  EventNotifier(EventLoop *event_loop, std::function<void()> fn,
                std::string_view name,
                realtime_clock::time_point realtime_event_time)
      : event_loop_(event_loop),
        fn_(std::move(fn)),
        realtime_event_time_(realtime_event_time) {
    CHECK(event_loop_);
    event_timer_ = event_loop->AddTimer([this]() { HandleTime(); });

    if (event_loop_->node() != nullptr) {
      event_timer_->set_name(
          absl::StrCat(event_loop_->node()->name()->string_view(), "_", name));
    } else {
      event_timer_->set_name(name);
    }
  }

  ~EventNotifier() { event_timer_->Disable(); }

  // Sets the clock offset for realtime playback.
  void SetClockOffset(std::chrono::nanoseconds clock_offset) {
    clock_offset_ = clock_offset;
  }

  // Returns the event trigger time.
  realtime_clock::time_point realtime_event_time() const {
    return realtime_event_time_;
  }

  // Observes the next message and potentially calls the callback or updates the
  // timer.
  void ObserveNextMessage(monotonic_clock::time_point monotonic_message_time,
                          realtime_clock::time_point realtime_message_time) {
    if (realtime_message_time < realtime_event_time_) {
      return;
    }
    if (called_) {
      return;
    }

    // Move the callback wakeup time to the correct time (or make it now if
    // there's a gap in time) now that we know it is before the next
    // message.
    const monotonic_clock::time_point candidate_monotonic =
        (realtime_event_time_ - realtime_message_time) + monotonic_message_time;
    const monotonic_clock::time_point monotonic_now =
        event_loop_->monotonic_now();
    if (candidate_monotonic < monotonic_now) {
      // Whops, time went backwards.  Just do it now.
      HandleTime();
    } else {
      event_timer_->Schedule(candidate_monotonic + clock_offset_);
    }
  }

 private:
  void HandleTime() {
    if (!called_) {
      called_ = true;
      fn_();
    }
  }

  EventLoop *event_loop_ = nullptr;
  TimerHandler *event_timer_ = nullptr;
  std::function<void()> fn_;

  const realtime_clock::time_point realtime_event_time_ =
      realtime_clock::min_time;

  std::chrono::nanoseconds clock_offset_{0};

  bool called_ = false;
};

LogReader::LogReader(std::string_view filename,
                     const Configuration *replay_configuration,
                     const ReplayChannels *replay_channels)
    : LogReader(LogFilesContainer(SortParts({std::string(filename)})),
                replay_configuration, replay_channels) {}

LogReader::LogReader(std::vector<LogFile> log_files,
                     const Configuration *replay_configuration,
                     const ReplayChannels *replay_channels)
    : LogReader(LogFilesContainer(std::move(log_files)), replay_configuration,
                replay_channels) {}

LogReader::LogReader(LogFilesContainer log_files,
                     const Configuration *replay_configuration,
                     const ReplayChannels *replay_channels)
    : log_files_(std::move(log_files)),
      replay_configuration_(replay_configuration),
      replay_channels_(replay_channels),
      config_remapper_(log_files_.config().get(), replay_configuration_,
                       replay_channels_) {
  SetStartTime(absl::GetFlag(FLAGS_start_time));
  SetEndTime(absl::GetFlag(FLAGS_end_time));

  {
    // Log files container validates that log files shared the same config.
    const Configuration *config = log_files_.config().get();
    CHECK(config != nullptr);
  }

  if (replay_channels_ != nullptr) {
    CHECK(!replay_channels_->empty()) << "replay_channels is empty which means "
                                         "no messages will get replayed.";
  }

  if (!configuration::MultiNode(configuration())) {
    states_.resize(1);
  } else {
    if (replay_configuration) {
      CHECK_EQ(logged_configuration()->nodes()->size(),
               replay_configuration->nodes()->size())
          << ": Log file and replay config need to have matching nodes "
             "lists.";
      for (const Node *node : *logged_configuration()->nodes()) {
        if (configuration::GetNode(replay_configuration, node) == nullptr) {
          LOG(FATAL) << "Found node " << FlatbufferToJson(node)
                     << " in logged config that is not present in the replay "
                        "config.";
        }
      }
    }
    states_.resize(configuration()->nodes()->size());
  }

  before_send_callbacks_.resize(configuration()->channels()->size());
}

LogReader::~LogReader() {
  if (event_loop_factory_unique_ptr_) {
    Deregister();
  } else if (event_loop_factory_ != nullptr) {
    LOG(FATAL) << "Must call Deregister before the SimulatedEventLoopFactory "
                  "is destroyed";
  }
}

const Configuration *LogReader::logged_configuration() const {
  return config_remapper_.original_configuration();
}

const Configuration *LogReader::configuration() const {
  return config_remapper_.remapped_configuration();
}

std::vector<const Node *> LogReader::LoggedNodes() const {
  return configuration::GetNodes(logged_configuration());
}

monotonic_clock::time_point LogReader::monotonic_start_time(
    const Node *node) const {
  State *state =
      states_[configuration::GetNodeIndex(configuration(), node)].get();
  CHECK(state != nullptr) << ": Unknown node " << FlatbufferToJson(node);

  return state->monotonic_start_time(state->boot_count());
}

realtime_clock::time_point LogReader::realtime_start_time(
    const Node *node) const {
  State *state =
      states_[configuration::GetNodeIndex(configuration(), node)].get();
  CHECK(state != nullptr) << ": Unknown node " << FlatbufferToJson(node);

  return state->realtime_start_time(state->boot_count());
}

void LogReader::OnStart(std::function<void()> fn) {
  CHECK(!configuration::MultiNode(configuration()));
  OnStart(nullptr, std::move(fn));
}

void LogReader::OnStart(const Node *node, std::function<void()> fn) {
  const int node_index = configuration::GetNodeIndex(configuration(), node);
  CHECK_GE(node_index, 0);
  CHECK_LT(node_index, static_cast<int>(states_.size()));
  State *state = states_[node_index].get();
  CHECK(state != nullptr) << ": Unknown node " << FlatbufferToJson(node);

  state->OnStart(std::move(fn));
}

void LogReader::State::QueueThreadUntil(BootTimestamp time) {
  if (threading_ == ThreadedBuffering::kYes) {
    CHECK(!message_queuer_.has_value()) << "Can't start thread twice.";
    message_queuer_.emplace(
        [this](const BootTimestamp queue_until) {
          // This will be called whenever anything prompts us for any state
          // change; there may be wakeups that result in us not having any new
          // data to push (even if we aren't done), in which case we will return
          // nullopt but not done().
          if (last_queued_message_.has_value() &&
              queue_until < last_queued_message_) {
            return util::ThreadedQueue<Result<TimestampedMessage>,
                                       BootTimestamp>::PushResult{
                std::nullopt, false,
                last_queued_message_ == BootTimestamp::max_time()};
          }

          Result<TimestampedMessage *> message = timestamp_mapper_->Front();
          if (!message.has_value()) {
            return util::ThreadedQueue<Result<TimestampedMessage>,
                                       BootTimestamp>::PushResult{
                MakeError(message.error()), /*more_to_push=*/false,
                /*done=*/true};
          }
          // Upon reaching the end of the log, exit.
          if (message == nullptr) {
            last_queued_message_ = BootTimestamp::max_time();
            return util::ThreadedQueue<Result<TimestampedMessage>,
                                       BootTimestamp>::PushResult{
                std::nullopt,
                /*more_to_push=*/false, /*done=*/true};
          }

          last_queued_message_ = message.value()->monotonic_event_time;
          const util::ThreadedQueue<Result<TimestampedMessage>,
                                    BootTimestamp>::PushResult result{
              *message.value(), queue_until >= last_queued_message_, false};
          const Status pop_result = timestamp_mapper_->PopFront();
          if (!pop_result.has_value()) {
            return util::ThreadedQueue<Result<TimestampedMessage>,
                                       BootTimestamp>::PushResult{
                MakeError(pop_result.error()),
                /*more_to_push=*/false,
                /*done=*/true};
          }
          const Status seed_result = MaybeSeedSortedMessages();
          if (!seed_result.has_value()) {
            return util::ThreadedQueue<Result<TimestampedMessage>,
                                       BootTimestamp>::PushResult{
                MakeError(pop_result.error()),
                /*more_to_push=*/false,
                /*done=*/true};
          }
          return result;
        },
        time);
    // Spin until the first few seconds of messages are queued up so that we
    // don't end up with delays/inconsistent timing during the first few seconds
    // of replay.
    message_queuer_->WaitForNoMoreWork();
  }
}

void LogReader::State::OnStart(std::function<void()> fn) {
  on_starts_.emplace_back(std::move(fn));
}

void LogReader::State::RunOnStart() {
  SetRealtimeOffset(monotonic_start_time(boot_count()),
                    realtime_start_time(boot_count()));

  VLOG(1) << "Starting for node '" << MaybeNodeName(node()) << "' at time "
          << monotonic_start_time(boot_count());
  auto fn = [this]() {
    for (size_t i = 0; i < on_starts_.size(); ++i) {
      on_starts_[i]();
    }
  };
  if (event_loop_factory_) {
    event_loop_factory_->AllowApplicationCreationDuring(std::move(fn));
  } else {
    fn();
  }
  stopped_ = false;
  started_ = true;
}

void LogReader::OnEnd(std::function<void()> fn) {
  CHECK(!configuration::MultiNode(configuration()));
  OnEnd(nullptr, std::move(fn));
}

void LogReader::OnEnd(const Node *node, std::function<void()> fn) {
  const int node_index = configuration::GetNodeIndex(configuration(), node);
  CHECK_GE(node_index, 0);
  CHECK_LT(node_index, static_cast<int>(states_.size()));
  State *state = states_[node_index].get();
  CHECK(state != nullptr) << ": Unknown node " << FlatbufferToJson(node);

  state->OnEnd(std::move(fn));
}

void LogReader::State::OnEnd(std::function<void()> fn) {
  on_ends_.emplace_back(std::move(fn));
}

void LogReader::State::RunOnEnd() {
  VLOG(1) << "Ending for node '" << MaybeNodeName(node()) << "' at time "
          << monotonic_start_time(boot_count());
  auto fn = [this]() {
    for (size_t i = 0; i < on_ends_.size(); ++i) {
      on_ends_[i]();
    }
  };
  if (event_loop_factory_) {
    event_loop_factory_->AllowApplicationCreationDuring(std::move(fn));
  } else {
    fn();
  }

  stopped_ = true;
  started_ = true;
}

std::vector<
    std::pair<const aos::Channel *, NodeEventLoopFactory::ExclusiveSenders>>
LogReader::State::NonExclusiveChannels() {
  CHECK(node_event_loop_factory_ != nullptr);
  const aos::Configuration *config = node_event_loop_factory_->configuration();
  std::vector<
      std::pair<const aos::Channel *, NodeEventLoopFactory::ExclusiveSenders>>
      result{// Timing reports can be sent by logged and replayed applications.
             {aos::configuration::GetChannel(config, "/aos",
                                             "aos.timing.Report", "", node_),
              NodeEventLoopFactory::ExclusiveSenders::kNo},
             // AOS_LOG may be used in the log and in replay.
             {aos::configuration::GetChannel(
                  config, "/aos", "aos.logging.LogMessageFbs", "", node_),
              NodeEventLoopFactory::ExclusiveSenders::kNo}};
  for (const Node *const node : configuration::GetNodes(config)) {
    if (node == nullptr) {
      break;
    }
    const Channel *const old_timestamp_channel = aos::configuration::GetChannel(
        config,
        absl::StrCat("/aos/remote_timestamps/", node->name()->string_view()),
        "aos.message_bridge.RemoteMessage", "", node_, /*quiet=*/true);
    // The old-style remote timestamp channel can be populated from any
    // channel, simulated or replayed.
    if (old_timestamp_channel != nullptr) {
      result.push_back(std::make_pair(
          old_timestamp_channel, NodeEventLoopFactory::ExclusiveSenders::kNo));
    }
  }
  // Remove any channels that weren't found due to not existing in the
  // config.
  for (size_t ii = 0; ii < result.size();) {
    if (result[ii].first == nullptr) {
      result.erase(result.begin() + ii);
    } else {
      ++ii;
    }
  }
  return result;
}

void LogReader::Register() {
  event_loop_factory_unique_ptr_ =
      std::make_unique<SimulatedEventLoopFactory>(configuration());
  Register(event_loop_factory_unique_ptr_.get());
}

void LogReader::RegisterWithoutStarting(
    SimulatedEventLoopFactory *event_loop_factory) {
  CheckExpected(NonFatalRegisterWithoutStarting(event_loop_factory));
}

Status LogReader::NonFatalRegisterWithoutStarting(
    SimulatedEventLoopFactory *event_loop_factory) {
  event_loop_factory_ = event_loop_factory;
  config_remapper_.set_configuration(event_loop_factory_->configuration());

  const TimestampQueueStrategy timestamp_queue_strategy =
      ComputeTimestampQueueStrategy();

  filters_ =
      std::make_unique<message_bridge::MultiNodeNoncausalOffsetEstimator>(
          event_loop_factory_->configuration(), logged_configuration(),
          log_files_.boots(), absl::GetFlag(FLAGS_skip_order_validation),
          timestamp_queue_strategy ==
                  TimestampQueueStrategy::kQueueTimestampsAtStartup
              ? chrono::seconds(0)
              : chrono::duration_cast<chrono::nanoseconds>(
                    chrono::duration<double>(
                        absl::GetFlag(FLAGS_time_estimation_buffer_seconds))));

  std::vector<TimestampMapper *> timestamp_mappers;
  for (const Node *node : configuration::GetNodes(configuration())) {
    size_t node_index = configuration::GetNodeIndex(configuration(), node);
    std::string_view node_name = MaybeNodeName(node);

    // We don't run with threading on the buffering for simulated event loops
    // because we haven't attempted to validate how the interactions beteen the
    // buffering and the timestamp mapper works when running multiple nodes
    // concurrently.
    states_[node_index] = std::make_unique<State>(
        !log_files_.ContainsPartsForNode(node_name)
            ? nullptr
            : std::make_unique<TimestampMapper>(node_name, log_files_,
                                                timestamp_queue_strategy),
        timestamp_queue_strategy, filters_.get(),
        std::bind(&LogReader::NoticeRealtimeEnd, this), node,
        State::ThreadedBuffering::kNo, MaybeMakeReplayChannelIndices(node),
        before_send_callbacks_);
    State *state = states_[node_index].get();
    state->SetNodeEventLoopFactory(
        event_loop_factory_->GetNodeEventLoopFactory(node),
        event_loop_factory_);

    state->SetChannelCount(logged_configuration()->channels()->size());
    timestamp_mappers.emplace_back(state->timestamp_mapper());
  }
  filters_->SetTimestampMappers(std::move(timestamp_mappers));

  // Note: this needs to be set before any times are pulled, or we won't observe
  // the timestamps.
  event_loop_factory_->SetTimeConverter(filters_.get());

  for (const Node *node : configuration::GetNodes(configuration())) {
    const size_t node_index =
        configuration::GetNodeIndex(configuration(), node);
    State *state = states_[node_index].get();
    for (const Node *other_node : configuration::GetNodes(configuration())) {
      const size_t other_node_index =
          configuration::GetNodeIndex(configuration(), other_node);
      State *other_state = states_[other_node_index].get();
      if (other_state != state) {
        state->AddPeer(other_state);
      }
    }
  }

  // Now that every state has a peer, load all the timestamps into RAM.
  if (timestamp_queue_strategy ==
      TimestampQueueStrategy::kQueueTimestampsAtStartup) {
    for (std::unique_ptr<State> &state : states_) {
      AOS_RETURN_IF_ERROR(state->ReadTimestamps());
    }
  }

  // Register after making all the State objects so we can build references
  // between them.
  for (const Node *node : configuration::GetNodes(configuration())) {
    const size_t node_index =
        configuration::GetNodeIndex(configuration(), node);
    State *state = states_[node_index].get();

    // If we didn't find any log files with data in them, we won't ever get a
    // callback or be live.  So skip the rest of the setup.
    if (state->SingleThreadedOldestMessageTime() == BootTimestamp::max_time()) {
      continue;
    }

    ++live_nodes_;

    NodeEventLoopFactory *node_factory =
        event_loop_factory_->GetNodeEventLoopFactory(node);
    node_factory->OnStartup([this, state, node]() {
      ExitOrCheckExpected(RegisterDuringStartup(state->MakeEventLoop(), node));
    });
    node_factory->OnShutdown([this, state, node]() {
      // TODO(james): Does this actually respect the Result<> passed to Exit()?
      // Does it matter?
      // For now, default to a FATAL check in lieu of a more direct
      // verification.
      CheckExpected(RegisterDuringStartup(nullptr, node));
      state->DestroyEventLoop();
    });
  }

  if (live_nodes_ == 0) {
    LOG(FATAL)
        << "Don't have logs from any of the nodes in the replay config--are "
           "you sure that the replay config matches the original config?";
  }

  filters_->CheckGraph();

  for (std::unique_ptr<State> &state : states_) {
    AOS_RETURN_IF_ERROR(state->MaybeSeedSortedMessages());
  }

  // Forwarding is tracked per channel.  If it is enabled, we want to turn it
  // off.  Otherwise messages replayed will get forwarded across to the other
  // nodes, and also replayed on the other nodes.  This may not satisfy all
  // our users, but it'll start the discussion.
  if (configuration::NodesCount(event_loop_factory_->configuration()) > 1u) {
    for (size_t i = 0; i < logged_configuration()->channels()->size(); ++i) {
      const Channel *channel = logged_configuration()->channels()->Get(i);

      const Channel *remapped_channel = config_remapper_.RemapChannel(channel);

      event_loop_factory_->DisableForwarding(remapped_channel);
    }

    // If we are replaying a log, we don't want a bunch of redundant messages
    // from both the real message bridge and simulated message bridge.
    event_loop_factory_->PermanentlyDisableStatistics();
  }

  // Write pseudo start times out to file now that we are all setup.
  filters_->Start(event_loop_factory_);

  return Ok();
}

void LogReader::Register(SimulatedEventLoopFactory *event_loop_factory) {
  RegisterWithoutStarting(event_loop_factory);
  StartAfterRegister(event_loop_factory);
}

void LogReader::StartAfterRegister(
    SimulatedEventLoopFactory *event_loop_factory) {
  // We want to start the log file at the last start time of the log files
  // from all the nodes.  Compute how long each node's simulation needs to run
  // to move time to this point.
  distributed_clock::time_point start_time = distributed_clock::min_time;

  // TODO(austin): We want an "OnStart" callback for each node rather than
  // running until the last node.

  for (std::unique_ptr<State> &state : states_) {
    CHECK(state);
    VLOG(1) << "Start time is " << state->monotonic_start_time(0)
            << " for node '" << MaybeNodeName(state->node()) << "' now "
            << (state->event_loop() != nullptr ? state->monotonic_now()
                                               : monotonic_clock::min_time);
    if (state->monotonic_start_time(0) == monotonic_clock::min_time) {
      continue;
    }
    // And start computing the start time on the distributed clock now that
    // that works.
    start_time = std::max(start_time, CheckExpected(state->ToDistributedClock(
                                          state->monotonic_start_time(0))));
  }

  // TODO(austin): If a node doesn't have a start time, we might not queue
  // enough.  If this happens, we'll explode with a frozen error eventually.

  CHECK_GE(start_time, distributed_clock::epoch())
      << ": Hmm, we have a node starting before the start of time.  Offset "
         "everything.";

  {
    VLOG(1) << "Running until " << start_time << " in Register";
    event_loop_factory_->RunFor(start_time.time_since_epoch());
    VLOG(1) << "At start time";
  }

  for (std::unique_ptr<State> &state : states_) {
    // Make the RT clock be correct before handing it to the user.
    if (state->realtime_start_time(0) != realtime_clock::min_time) {
      state->SetRealtimeOffset(state->monotonic_start_time(0),
                               state->realtime_start_time(0));
    }
    VLOG(1) << "Start time is " << state->monotonic_start_time(0)
            << " for node '" << MaybeNodeName(state->node()) << "' now "
            << (state->event_loop() != nullptr ? state->monotonic_now()
                                               : monotonic_clock::min_time);
  }

  if (absl::GetFlag(FLAGS_timestamps_to_csv)) {
    filters_->Start(event_loop_factory);
  }
}

message_bridge::NoncausalOffsetEstimator *LogReader::GetFilter(
    const Node *node_a, const Node *node_b) {
  if (filters_) {
    return filters_->GetFilter(node_a, node_b);
  }
  return nullptr;
}

TimestampQueueStrategy LogReader::ComputeTimestampQueueStrategy() const {
  if ((log_files_.TimestampsStoredSeparately() &&
       absl::GetFlag(FLAGS_enable_timestamp_loading)) ||
      absl::GetFlag(FLAGS_force_timestamp_loading)) {
    return TimestampQueueStrategy::kQueueTimestampsAtStartup;
  } else {
    return TimestampQueueStrategy::kQueueTogether;
  }
}

// TODO(jkuszmaul): Make in-line modifications to
// ServerStatistics/ClientStatistics messages for ShmEventLoop-based replay to
// avoid messing up anything that depends on them having valid offsets.
void LogReader::Register(EventLoop *event_loop) {
  const TimestampQueueStrategy timestamp_queue_strategy =
      ComputeTimestampQueueStrategy();

  filters_ =
      std::make_unique<message_bridge::MultiNodeNoncausalOffsetEstimator>(
          event_loop->configuration(), logged_configuration(),
          log_files_.boots(), absl::GetFlag(FLAGS_skip_order_validation),
          timestamp_queue_strategy ==
                  TimestampQueueStrategy::kQueueTimestampsAtStartup
              ? chrono::seconds(0)
              : chrono::duration_cast<chrono::nanoseconds>(
                    chrono::duration<double>(
                        absl::GetFlag(FLAGS_time_estimation_buffer_seconds))));

  std::vector<TimestampMapper *> timestamp_mappers;
  for (const Node *node : configuration::GetNodes(configuration())) {
    auto node_name = MaybeNodeName(node);
    const size_t node_index =
        configuration::GetNodeIndex(configuration(), node);

    states_[node_index] = std::make_unique<State>(
        !log_files_.ContainsPartsForNode(node_name)
            ? nullptr
            : std::make_unique<TimestampMapper>(node_name, log_files_,
                                                timestamp_queue_strategy),
        timestamp_queue_strategy, filters_.get(),
        std::bind(&LogReader::NoticeRealtimeEnd, this), node,
        State::ThreadedBuffering::kYes, MaybeMakeReplayChannelIndices(node),
        before_send_callbacks_);
    State *state = states_[node_index].get();

    state->SetChannelCount(logged_configuration()->channels()->size());
    timestamp_mappers.emplace_back(state->timestamp_mapper());
  }

  filters_->SetTimestampMappers(std::move(timestamp_mappers));

  for (const Node *node : configuration::GetNodes(configuration())) {
    const size_t node_index =
        configuration::GetNodeIndex(configuration(), node);
    State *state = states_[node_index].get();
    for (const Node *other_node : configuration::GetNodes(configuration())) {
      const size_t other_node_index =
          configuration::GetNodeIndex(configuration(), other_node);
      State *other_state = states_[other_node_index].get();
      if (other_state != state) {
        state->AddPeer(other_state);
      }
    }
  }

  // Now that all the peers are added, we can buffer up all the timestamps if
  // needed.
  if (timestamp_queue_strategy ==
      TimestampQueueStrategy::kQueueTimestampsAtStartup) {
    for (std::unique_ptr<State> &state : states_) {
      CheckExpected(state->ReadTimestamps());
    }
  }

  for (const Node *node : configuration::GetNodes(configuration())) {
    if (node == nullptr || node->name()->string_view() ==
                               event_loop->node()->name()->string_view()) {
      CheckExpected(Register(event_loop, event_loop->node()));
    } else {
      CheckExpected(Register(nullptr, node));
    }
  }
}

Status LogReader::Register(EventLoop *event_loop, const Node *node) {
  State *state =
      states_[configuration::GetNodeIndex(configuration(), node)].get();

  // If we didn't find any log files with data in them, we won't ever get a
  // callback or be live.  So skip the rest of the setup.
  if (state->SingleThreadedOldestMessageTime() == BootTimestamp::max_time()) {
    return Ok();
  }

  if (event_loop != nullptr) {
    ++live_nodes_;
  }

  if (event_loop_factory_ != nullptr) {
    event_loop_factory_->GetNodeEventLoopFactory(node)->OnStartup(
        [this, event_loop, node]() {
          ExitOrCheckExpected(RegisterDuringStartup(event_loop, node));
        });
  } else {
    AOS_RETURN_IF_ERROR(RegisterDuringStartup(event_loop, node));
  }
  return Ok();
}

Result<void> LogReader::RegisterDuringStartup(EventLoop *event_loop,
                                              const Node *node) {
  if (event_loop != nullptr) {
    CHECK(event_loop->configuration() == configuration());
  }

  State *state =
      states_[configuration::GetNodeIndex(configuration(), node)].get();

  if (event_loop == nullptr) {
    state->ClearTimeFlags();
  }

  state->set_event_loop(event_loop);

  // We don't run timing reports when trying to print out logged data, because
  // otherwise we would end up printing out the timing reports themselves...
  // This is only really relevant when we are replaying into a simulation.
  if (event_loop != nullptr) {
    event_loop->SkipTimingReport();
    event_loop->SkipAosLog();
  }

  for (size_t logged_channel_index = 0;
       logged_channel_index < logged_configuration()->channels()->size();
       ++logged_channel_index) {
    // We skip creating the per channel setup here for channels not in
    // ReplayChannels. This avoids creating unnecessary Senders.
    const ReplayChannelIndices *replay_channel_indicies =
        state->GetReplayChannelIndices();
    if (replay_channel_indicies != nullptr) {
      auto const begin = replay_channel_indicies->cbegin();
      auto const end = replay_channel_indicies->cend();
      // TODO: benchmark strategies for channel_index matching
      if (!std::binary_search(begin, end, logged_channel_index)) {
        continue;
      }
    }

    const Channel *channel = config_remapper_.RemapChannel(
        logged_configuration()->channels()->Get(logged_channel_index));

    const bool logged = channel->logger() != LoggerConfig::NOT_LOGGED;
    message_bridge::NoncausalOffsetEstimator *filter = nullptr;

    State *source_state = nullptr;

    if (!configuration::ChannelIsSendableOnNode(channel, node) &&
        configuration::ChannelIsReadableOnNode(channel, node)) {
      const Node *source_node = configuration::GetNode(
          configuration(), channel->source_node()->string_view());

      // We've got a message which is being forwarded to this node.
      filter = GetFilter(node, source_node);

      source_state =
          states_[configuration::GetNodeIndex(configuration(), source_node)]
              .get();
    }

    // We are the source, and it is forwarded.
    const bool is_forwarded =
        configuration::ChannelIsSendableOnNode(channel, node) &&
        configuration::ConnectionCount(channel);

    state->SetChannel(
        logged_channel_index,
        configuration::ChannelIndex(configuration(), channel),
        event_loop && logged &&
                configuration::ChannelIsReadableOnNode(channel, node)
            ? event_loop->MakeRawSender(channel)
            : nullptr,
        filter, is_forwarded, source_state);

    if (is_forwarded && logged) {
      const Node *source_node = configuration::GetNode(
          configuration(), channel->source_node()->string_view());

      for (const Connection *connection : *channel->destination_nodes()) {
        const bool delivery_time_is_logged =
            configuration::ConnectionDeliveryTimeIsLoggedOnNode(connection,
                                                                source_node);

        if (delivery_time_is_logged) {
          State *destination_state =
              states_[configuration::GetNodeIndex(
                          configuration(), connection->name()->string_view())]
                  .get();
          if (destination_state) {
            destination_state->SetRemoteTimestampSender(
                logged_channel_index,
                event_loop ? state->RemoteTimestampSender(channel, connection)
                           : nullptr);
          }
        }
      }
    }
  }

  if (!event_loop) {
    state->ClearRemoteTimestampSenders();
    state->set_timer_handler(nullptr);
    state->set_startup_timer(nullptr);
    return Ok();
  }

  state->set_timer_handler(event_loop->AddTimer([this, state]() {
    if (state->MultiThreadedOldestMessageTime() == BootTimestamp::max_time()) {
      --live_nodes_;
      VLOG(1) << "Node '" << MaybeNodeName(state->event_loop()->node())
              << "' down!";
      if (exit_on_finish_ && live_nodes_ == 0 &&
          event_loop_factory_ != nullptr) {
        event_loop_factory_->Exit();
      }
      return;
    }

    Result<TimestampedMessage> timestamped_message_result = state->PopOldest();
    if (!timestamped_message_result.has_value()) {
      ExitOrCheckExpected(timestamped_message_result);
      return;
    }
    TimestampedMessage timestamped_message =
        std::move(timestamped_message_result.value());

    CHECK_EQ(timestamped_message.monotonic_event_time.boot,
             state->boot_count());

    const monotonic_clock::time_point monotonic_now =
        state->event_loop()->context().monotonic_event_time;
    if (event_loop_factory_ != nullptr) {
      // Only enforce exact timing in simulation.
      if (!absl::GetFlag(FLAGS_skip_order_validation)) {
        CHECK(monotonic_now == timestamped_message.monotonic_event_time.time)
            << ": " << FlatbufferToJson(state->event_loop()->node()) << " Now "
            << monotonic_now << " trying to send "
            << timestamped_message.monotonic_event_time << " failure "
            << state->DebugString();
      } else if (BootTimestamp{.boot = state->boot_count(),
                               .time = monotonic_now} !=
                 timestamped_message.monotonic_event_time) {
        LOG(WARNING) << "Check failed: monotonic_now == "
                        "timestamped_message.monotonic_event_time) ("
                     << monotonic_now << " vs. "
                     << timestamped_message.monotonic_event_time
                     << "): " << FlatbufferToJson(state->event_loop()->node())
                     << " Now " << monotonic_now << " trying to send "
                     << timestamped_message.monotonic_event_time << " failure "
                     << state->DebugString();
      }
    }

    if (timestamped_message.monotonic_event_time.time >
            state->monotonic_start_time(
                timestamped_message.monotonic_event_time.boot) ||
        event_loop_factory_ != nullptr ||
        !absl::GetFlag(FLAGS_drop_realtime_messages_before_start)) {
      if (timestamped_message.data != nullptr && !state->found_last_message()) {
        if (timestamped_message.monotonic_remote_time !=
                BootTimestamp::min_time() &&
            !absl::GetFlag(FLAGS_skip_order_validation) &&
            event_loop_factory_ != nullptr) {
          // Confirm that the message was sent on the sending node before the
          // destination node (this node).  As a proxy, do this by making sure
          // that time on the source node is past when the message was sent.
          //
          // TODO(austin): <= means that the cause message (which we know) could
          // happen after the effect even though we know they are at the same
          // time.  I doubt anyone will notice for a bit, but we should really
          // fix that.
          BootTimestamp monotonic_remote_now =
              state->monotonic_remote_now(timestamped_message.channel_index);
          if (!absl::GetFlag(FLAGS_skip_order_validation)) {
            CHECK_EQ(timestamped_message.monotonic_remote_time.boot,
                     monotonic_remote_now.boot)
                << state->event_loop()->node()->name()->string_view() << " to "
                << state->remote_node(timestamped_message.channel_index)
                       ->name()
                       ->string_view()
                << " while trying to send a message on "
                << configuration::CleanedChannelToString(
                       logged_configuration()->channels()->Get(
                           timestamped_message.channel_index))
                << " " << timestamped_message << " " << state->DebugString();
            CHECK_LE(timestamped_message.monotonic_remote_time,
                     monotonic_remote_now)
                << state->event_loop()->node()->name()->string_view() << " to "
                << state->remote_node(timestamped_message.channel_index)
                       ->name()
                       ->string_view()
                << " while trying to send a message on "
                << configuration::CleanedChannelToString(
                       logged_configuration()->channels()->Get(
                           timestamped_message.channel_index))
                << " " << state->DebugString();
          } else if (monotonic_remote_now.boot !=
                     timestamped_message.monotonic_remote_time.boot) {
            LOG(WARNING) << "Mismatched boots, " << monotonic_remote_now.boot
                         << " vs "
                         << timestamped_message.monotonic_remote_time.boot
                         << ".";
          } else if (timestamped_message.monotonic_remote_time >
                     monotonic_remote_now) {
            LOG(WARNING)
                << "Check failed: timestamped_message.monotonic_remote_time < "
                   "state->monotonic_remote_now(timestamped_message.channel_"
                   "index) ("
                << timestamped_message.monotonic_remote_time << " vs. "
                << state->monotonic_remote_now(
                       timestamped_message.channel_index)
                << ") " << state->event_loop()->node()->name()->string_view()
                << " to "
                << state->remote_node(timestamped_message.channel_index)
                       ->name()
                       ->string_view()
                << " currently " << timestamped_message.monotonic_event_time
                << " ("
                << state->ToDistributedClock(
                       timestamped_message.monotonic_event_time.time)
                << ") remote event time "
                << timestamped_message.monotonic_remote_time << " ("
                << state->RemoteToDistributedClock(
                       timestamped_message.channel_index,
                       timestamped_message.monotonic_remote_time.time)
                << ") " << state->DebugString();
          }
        }

        // If we have access to the factory, use it to fix the realtime time.
        state->SetRealtimeOffset(timestamped_message.monotonic_event_time.time,
                                 timestamped_message.realtime_event_time);

        VLOG(1) << "For node '" << MaybeNodeName(state->event_loop()->node())
                << "' sending at " << timestamped_message.monotonic_event_time
                << " : " << state->DebugString();
        // TODO(austin): std::move channel_data in and make that efficient in
        // simulation.
        state->Send(std::move(timestamped_message));
      } else if (state->found_last_message() ||
                 (!ignore_missing_data_ &&
                  // When starting up, we can have data which was sent before
                  // the log starts, but the timestamp was after the log
                  // starts. This is unreasonable to avoid, so ignore the
                  // missing data.
                  timestamped_message.monotonic_remote_time.time >=
                      state->monotonic_remote_start_time(
                          timestamped_message.monotonic_remote_time.boot,
                          timestamped_message.channel_index) &&
                  !absl::GetFlag(FLAGS_skip_missing_forwarding_entries))) {
        if (!state->found_last_message()) {
          // We've found a timestamp without data that we expect to have data
          // for. This likely means that we are at the end of the log file.
          // Record it and CHECK that in the rest of the log file, we don't find
          // any more data on that channel.  Not all channels will end at the
          // same point in time since they can be in different files.
          VLOG(1) << "Found the last message on channel "
                  << timestamped_message.channel_index << ", "
                  << configuration::CleanedChannelToString(
                         logged_configuration()->channels()->Get(
                             timestamped_message.channel_index))
                  << " on node '" << MaybeNodeName(state->event_loop()->node())
                  << "' at " << timestamped_message;

          // The user might be working with log files from 1 node but forgot to
          // configure the infrastructure to log data for a remote channel on
          // that node.  That can be very hard to debug, even though the log
          // reader is doing the right thing.  At least log a warning in that
          // case and tell the user what is happening so they can either update
          // their config to log the channel or can find a log with the data.
          const std::vector<std::string> logger_nodes =
              log_files_.logger_nodes();
          if (!logger_nodes.empty()) {
            // We have old logs which don't have the logger nodes logged.  In
            // that case, we can't be helpful :(
            bool data_logged = false;
            const Channel *channel = logged_configuration()->channels()->Get(
                timestamped_message.channel_index);
            for (const std::string &node : logger_nodes) {
              data_logged |=
                  configuration::ChannelMessageIsLoggedOnNode(channel, node);
            }
            if (!data_logged) {
              LOG(WARNING) << "Got a timestamp without any logfiles which "
                              "could contain data for channel "
                           << configuration::CleanedChannelToString(channel);
              LOG(WARNING) << "Only have logs logged on ["
                           << absl::StrJoin(logger_nodes, ", ") << "]";
              LOG(WARNING)
                  << "Dropping the rest of the data on "
                  << state->event_loop()->node()->name()->string_view();
              LOG(WARNING)
                  << "Consider using --skip_missing_forwarding_entries to "
                     "bypass this, update your config to log it, or add data "
                     "from one of the nodes it is logged on.";
            }
          }
          // The log file is now done, prod the callbacks.
          state->NotifyLogfileEnd();

          // Now that we found the end of one channel, artificially stop the
          // rest by setting the found_last_message bit.  It is confusing when
          // part of your data gets replayed but not all.  The rest of them will
          // get dropped as they are replayed to keep memory usage down.
          state->SetFoundLastMessage(true);

          // Vector storing if we've seen a nullptr message or not per channel.
          state->set_last_message(timestamped_message.channel_index);
        }

        // Make sure that once we have seen the last message on a channel,
        // data doesn't start back up again.  If the user wants to play
        // through events like this, they can set
        // --skip_missing_forwarding_entries or ignore_missing_data_.
        if (timestamped_message.data == nullptr) {
          state->set_last_message(timestamped_message.channel_index);
        } else {
          if (state->last_message(timestamped_message.channel_index)) {
            LOG(FATAL) << "Found missing data in the middle of the log file on "
                          "channel "
                       << timestamped_message.channel_index << " "
                       << configuration::StrippedChannelToString(
                              logged_configuration()->channels()->Get(
                                  timestamped_message.channel_index))
                       << " " << timestamped_message << " "
                       << state->DebugString();
          }
        }
      }
    } else {
      LOG(WARNING)
          << "Not sending data from before the start of the log file. "
          << timestamped_message.monotonic_event_time.time.time_since_epoch()
                 .count()
          << " start "
          << monotonic_start_time(state->node()).time_since_epoch().count()
          << " timestamped_message.data is null";
    }

    const Result<BootTimestamp> next_time_result =
        state->MultiThreadedOldestMessageTime();
    if (!next_time_result.has_value()) {
      ExitOrCheckExpected(next_time_result);
      return;
    }
    const BootTimestamp next_time = next_time_result.value();
    if (next_time != BootTimestamp::max_time()) {
      if (next_time.boot != state->boot_count()) {
        VLOG(1) << "Next message for node '"
                << MaybeNodeName(state->event_loop()->node())
                << "' is on the next boot, " << next_time << " now is "
                << state->monotonic_now();
        CHECK(event_loop_factory_);
        state->NotifyLogfileEnd();
        return;
      }
      if (event_loop_factory_ != nullptr) {
        VLOG(1) << "Scheduling for node '"
                << MaybeNodeName(state->event_loop()->node()) << "' wakeup for "
                << next_time.time << "("
                << state->ToDistributedClock(next_time.time)
                << " distributed), now is " << state->monotonic_now();
      } else {
        VLOG(1) << "Scheduling for node '"
                << MaybeNodeName(state->event_loop()->node()) << "' wakeup for "
                << next_time.time << ", now is " << state->monotonic_now();
      }
      // TODO(james): This can result in negative times getting passed-through
      // in realtime replay.
      state->Schedule(next_time.time);
    } else {
      VLOG(1) << "Node '" << MaybeNodeName(state->event_loop()->node())
              << "': No next message, scheduling shutdown";
      state->NotifyLogfileEnd();
      // Set a timer up immediately after now to die. If we don't do this,
      // then the watchers waiting on the message we just read will never get
      // called.
      // Doesn't apply to single-EventLoop replay since the watchers in question
      // are not under our control.
      if (event_loop_factory_ != nullptr) {
        state->Schedule(monotonic_now + event_loop_factory_->send_delay() +
                        std::chrono::nanoseconds(1));
      }
    }

    VLOG(1) << "Node '" << MaybeNodeName(state->event_loop()->node())
            << "': Done sending at "
            << state->event_loop()->context().monotonic_event_time << " now "
            << state->monotonic_now();
  }));

  AOS_RETURN_IF_ERROR(state->MaybeSeedSortedMessages());

  if (state->SingleThreadedOldestMessageTime() != BootTimestamp::max_time()) {
    state->set_startup_timer(
        event_loop->AddTimer([state]() { state->NotifyLogfileStart(); }));
    if (start_time_ != realtime_clock::min_time) {
      state->SetStartTimeFlag(start_time_);
    }
    if (end_time_ != realtime_clock::max_time) {
      state->SetEndTimeFlag(end_time_);
      ++live_nodes_with_realtime_time_end_;
    }
    event_loop->OnRun([this, state]() {
      const Result<BootTimestamp> next_time =
          state->SingleThreadedOldestMessageTime();
      if (!next_time.has_value()) {
        ExitOrCheckExpected(next_time);
        return;
      }
      CHECK_EQ(next_time->boot, state->boot_count());
      // Queue up messages and then set clock offsets (we don't want to set
      // clock offsets before we've done the work of getting the first messages
      // primed).
      state->QueueThreadUntil(
          next_time.value() +
          std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::duration<double>(
                  absl::GetFlag(FLAGS_threaded_look_ahead_seconds))));
      state->MaybeSetClockOffset();
      state->Schedule(next_time->time);
      state->SetUpStartupTimer();
    });
  }
  return Ok();
}

void LogReader::SetEndTime(std::string end_time) {
  if (end_time.empty()) {
    SetEndTime(realtime_clock::max_time);
  } else {
    std::optional<aos::realtime_clock::time_point> parsed_end_time =
        aos::realtime_clock::FromString(end_time);
    CHECK(parsed_end_time) << ": Failed to parse end time '" << end_time
                           << "'.  Expected a date in the format of "
                              "2021-01-15_15-30-35.000000000.";
    SetEndTime(*parsed_end_time);
  }
}

void LogReader::SetEndTime(realtime_clock::time_point end_time) {
  end_time_ = end_time;
}

void LogReader::SetStartTime(std::string start_time) {
  if (start_time.empty()) {
    SetStartTime(realtime_clock::min_time);
  } else {
    std::optional<aos::realtime_clock::time_point> parsed_start_time =
        aos::realtime_clock::FromString(start_time);
    CHECK(parsed_start_time) << ": Failed to parse start time '" << start_time
                             << "'.  Expected a date in the format of "
                                "2021-01-15_15-30-35.000000000.";
    SetStartTime(*parsed_start_time);
  }
}

void LogReader::SetStartTime(realtime_clock::time_point start_time) {
  start_time_ = start_time;
}

void LogReader::Deregister() {
  // Make sure that things get destroyed in the correct order, rather than
  // relying on getting the order correct in the class definition.
  for (std::unique_ptr<State> &state : states_) {
    if (state) {
      state->Deregister();
    }
  }

  event_loop_factory_unique_ptr_.reset();
  event_loop_factory_ = nullptr;
}

namespace {
// Checks if the specified channel name/type exists in the config and, depending
// on the value of conflict_handling, calls conflict_handler or just dies.
template <typename F>
void CheckAndHandleRemapConflict(
    std::string_view new_name, std::string_view new_type,
    const Configuration *config,
    ConfigRemapper::RemapConflict conflict_handling, F conflict_handler) {
  const Channel *existing_channel =
      configuration::GetChannel(config, new_name, new_type, "", nullptr, true);
  if (existing_channel != nullptr) {
    switch (conflict_handling) {
      case ConfigRemapper::RemapConflict::kDisallow:
        LOG(FATAL)
            << "Channel "
            << configuration::StrippedChannelToString(existing_channel)
            << " is already used--you can't remap a logged channel to it.";
        break;
      case ConfigRemapper::RemapConflict::kCascade:
        LOG(INFO) << "Automatically remapping "
                  << configuration::StrippedChannelToString(existing_channel)
                  << " to avoid conflicts.";
        conflict_handler();
        break;
    }
  }
}
}  // namespace

void LogReader::RemapLoggedChannel(
    std::string_view name, std::string_view type, std::string_view add_prefix,
    std::string_view new_type,
    ConfigRemapper::RemapConflict conflict_handling) {
  CheckEventsAreNotScheduled();
  config_remapper_.RemapOriginalChannel(name, type, nullptr, add_prefix,
                                        new_type, conflict_handling);
}

void LogReader::RemapLoggedChannel(
    std::string_view name, std::string_view type, const Node *node,
    std::string_view add_prefix, std::string_view new_type,
    ConfigRemapper::RemapConflict conflict_handling) {
  CheckEventsAreNotScheduled();
  config_remapper_.RemapOriginalChannel(name, type, node, add_prefix, new_type,
                                        conflict_handling);
}

void LogReader::RenameLoggedChannel(const std::string_view name,
                                    const std::string_view type,
                                    const std::string_view new_name,
                                    const std::vector<MapT> &add_maps) {
  CheckEventsAreNotScheduled();
  RenameLoggedChannel(name, type, nullptr, new_name, add_maps);
}

void LogReader::RenameLoggedChannel(const std::string_view name,
                                    const std::string_view type,
                                    const Node *const node,
                                    const std::string_view new_name,
                                    const std::vector<MapT> &add_maps) {
  CheckEventsAreNotScheduled();
  config_remapper_.RenameOriginalChannel(name, type, node, new_name, add_maps);
}

void LogReader::CheckEventsAreNotScheduled() {
  for (std::unique_ptr<State> &state : states_) {
    if (state) {
      CHECK(!state->event_loop())
          << ": Can't change the mapping after the events are scheduled.";
    }
  }
}

std::unique_ptr<const ReplayChannelIndices>
LogReader::MaybeMakeReplayChannelIndices(const Node *node) {
  if (replay_channels_ == nullptr) {
    return nullptr;
  } else {
    std::unique_ptr<ReplayChannelIndices> replay_channel_indices =
        std::make_unique<ReplayChannelIndices>();
    for (auto const &channel : *replay_channels_) {
      const Channel *ch = configuration::GetChannel(
          logged_configuration(), channel.first, channel.second, "", node);
      if (ch == nullptr) {
        LOG(WARNING) << "Channel: " << channel.first << " " << channel.second
                     << " not found in configuration for node: "
                     << node->name()->string_view() << " Skipping ...";
        continue;
      }
      const size_t channel_index =
          configuration::ChannelIndex(logged_configuration(), ch);
      replay_channel_indices->emplace_back(channel_index);
    }
    std::sort(replay_channel_indices->begin(), replay_channel_indices->end());
    return replay_channel_indices;
  }
}

std::vector<const Channel *> LogReader::RemappedChannels() const {
  return config_remapper_.RemappedChannels();
}

const Channel *LogReader::RemapChannel(const Channel *channel) {
  return config_remapper_.RemapChannel(channel);
}

LogReader::State::State(
    std::unique_ptr<TimestampMapper> timestamp_mapper,
    TimestampQueueStrategy timestamp_queue_strategy,
    message_bridge::MultiNodeNoncausalOffsetEstimator *multinode_filters,
    std::function<void()> notice_realtime_end, const Node *node,
    LogReader::State::ThreadedBuffering threading,
    std::unique_ptr<const ReplayChannelIndices> replay_channel_indices,
    const std::vector<std::function<SharedSpan(TimestampedMessage &)>>
        &before_send_callbacks)
    : timestamp_mapper_(std::move(timestamp_mapper)),
      timestamp_queue_strategy_(timestamp_queue_strategy),
      notice_realtime_end_(notice_realtime_end),
      node_(node),
      multinode_filters_(multinode_filters),
      threading_(threading),
      replay_channel_indices_(std::move(replay_channel_indices)),
      before_send_callbacks_(before_send_callbacks) {
  // If timestamp_mapper_ is nullptr, then there are no log parts associated
  // with this node. If there are no log parts for the node, there will be no
  // log data, and so we do not need to worry about the replay channel filters.
  if (replay_channel_indices_ != nullptr && timestamp_mapper_ != nullptr) {
    timestamp_mapper_->set_replay_channels_callback(
        [filter = replay_channel_indices_.get()](
            const TimestampedMessage &message) -> bool {
          auto const begin = filter->cbegin();
          auto const end = filter->cend();
          // TODO: benchmark strategies for channel_index matching
          return std::binary_search(begin, end, message.channel_index);
        });
  }
}

void LogReader::State::AddPeer(State *peer) {
  if (timestamp_mapper_ && peer->timestamp_mapper_) {
    timestamp_mapper_->AddPeer(peer->timestamp_mapper_.get());
  }
}

void LogReader::State::SetNodeEventLoopFactory(
    NodeEventLoopFactory *node_event_loop_factory,
    SimulatedEventLoopFactory *event_loop_factory) {
  node_event_loop_factory_ = node_event_loop_factory;
  event_loop_factory_ = event_loop_factory;
}

void LogReader::State::SetChannelCount(size_t count) {
  channels_.resize(count);
  remote_timestamp_senders_.resize(count);
  filters_.resize(count);
  channel_source_state_.resize(count);
  factory_channel_index_.resize(count);
  queue_index_map_.resize(count);
}

void LogReader::State::SetRemoteTimestampSender(
    size_t logged_channel_index, RemoteMessageSender *remote_timestamp_sender) {
  remote_timestamp_senders_[logged_channel_index] = remote_timestamp_sender;
}

void LogReader::State::SetChannel(
    size_t logged_channel_index, size_t factory_channel_index,
    std::unique_ptr<RawSender> sender,
    message_bridge::NoncausalOffsetEstimator *filter, bool is_forwarded,
    State *source_state) {
  channels_[logged_channel_index] = std::move(sender);
  filters_[logged_channel_index] = filter;
  channel_source_state_[logged_channel_index] = source_state;

  if (is_forwarded) {
    queue_index_map_[logged_channel_index] =
        std::make_unique<std::vector<State::ContiguousSentTimestamp>>();
  }

  factory_channel_index_[logged_channel_index] = factory_channel_index;
}

void LogReader::State::TrackMessageSendTiming(
    const RawSender &sender, monotonic_clock::time_point expected_send_time) {
  if (event_loop_ == nullptr || !timing_statistics_sender_.valid()) {
    return;
  }

  timing::MessageTimingT sample;
  sample.channel = configuration::ChannelIndex(event_loop_->configuration(),
                                               sender.channel());
  sample.expected_send_time = expected_send_time.time_since_epoch().count();
  sample.actual_send_time =
      sender.monotonic_sent_time().time_since_epoch().count();
  sample.send_time_error = aos::time::DurationInSeconds(
      expected_send_time - sender.monotonic_sent_time());
  send_timings_.push_back(sample);

  // Somewhat arbitrarily send out timing information in batches of 100. No need
  // to create excessive overhead in regenerated logfiles.
  // TODO(james): The overhead may be fine.
  constexpr size_t kMaxTimesPerStatisticsMessage = 100;
  CHECK(timing_statistics_sender_.valid());
  if (send_timings_.size() == kMaxTimesPerStatisticsMessage) {
    SendMessageTimings();
  }
}

void LogReader::State::SendMessageTimings() {
  if (send_timings_.empty() || !timing_statistics_sender_.valid()) {
    return;
  }
  auto builder = timing_statistics_sender_.MakeBuilder();
  std::vector<flatbuffers::Offset<timing::MessageTiming>> timing_offsets;
  for (const auto &timing : send_timings_) {
    timing_offsets.push_back(
        timing::MessageTiming::Pack(*builder.fbb(), &timing));
  }
  send_timings_.clear();
  flatbuffers::Offset<
      flatbuffers::Vector<flatbuffers::Offset<timing::MessageTiming>>>
      timings_offset = builder.fbb()->CreateVector(timing_offsets);
  timing::ReplayTiming::Builder timing_builder =
      builder.MakeBuilder<timing::ReplayTiming>();
  timing_builder.add_messages(timings_offset);
  timing_statistics_sender_.CheckOk(builder.Send(timing_builder.Finish()));
}

bool LogReader::State::Send(TimestampedMessage &&timestamped_message) {
  aos::RawSender *sender = channels_[timestamped_message.channel_index].get();
  CHECK(sender);
  uint32_t remote_queue_index = 0xffffffff;

  if (remote_timestamp_senders_[timestamped_message.channel_index] != nullptr) {
    State *source_state =
        channel_source_state_[timestamped_message.channel_index];
    CHECK(source_state != nullptr);
    std::vector<ContiguousSentTimestamp> *queue_index_map =
        source_state->queue_index_map_[timestamped_message.channel_index].get();
    CHECK(queue_index_map != nullptr);

    struct SentTimestamp {
      monotonic_clock::time_point monotonic_event_time;
      uint32_t queue_index;
    } search;

    CHECK_EQ(timestamped_message.monotonic_remote_time.boot,
             source_state->boot_count());
    search.monotonic_event_time =
        timestamped_message.monotonic_remote_time.time;
    search.queue_index = timestamped_message.remote_queue_index.index;

    // Find the sent time if available.
    auto element = std::lower_bound(
        queue_index_map->begin(), queue_index_map->end(), search,
        [](ContiguousSentTimestamp a, SentTimestamp b) {
          if (a.ending_monotonic_event_time < b.monotonic_event_time) {
            return true;
          }
          if (a.starting_monotonic_event_time > b.monotonic_event_time) {
            return false;
          }

          if (a.ending_queue_index < b.queue_index) {
            return true;
          }
          if (a.starting_queue_index >= b.queue_index) {
            return false;
          }

          // If it isn't clearly below or above, it is below.  Since we return
          // the last element <, this will return a match.
          return false;
        });

    // TODO(austin): Be a bit more principled here, but we will want to do that
    // after the logger rewrite.  We hit this when one node finishes, but the
    // other node isn't done yet.  So there is no send time, but there is a
    // receive time.
    if (element != queue_index_map->end()) {
      CHECK_EQ(timestamped_message.monotonic_remote_time.boot,
               source_state->boot_count());

      CHECK_GE(timestamped_message.monotonic_remote_time.time,
               element->starting_monotonic_event_time);
      CHECK_LE(timestamped_message.monotonic_remote_time.time,
               element->ending_monotonic_event_time);
      CHECK_GE(timestamped_message.remote_queue_index.index,
               element->starting_queue_index);
      CHECK_LE(timestamped_message.remote_queue_index.index,
               element->ending_queue_index);

      remote_queue_index = timestamped_message.remote_queue_index.index +
                           element->actual_queue_index -
                           element->starting_queue_index;
    } else {
      VLOG(1) << "No timestamp match in the map.";
    }
    CHECK_EQ(timestamped_message.monotonic_remote_time.boot,
             source_state->boot_count());
  }

  if (event_loop_factory_ != nullptr &&
      channel_source_state_[timestamped_message.channel_index] != nullptr &&
      multinode_filters_ != nullptr) {
    // Sanity check that we are using consistent boot uuids.
    State *source_state =
        channel_source_state_[timestamped_message.channel_index];
    CHECK(source_state != nullptr);
    CHECK(source_state->event_loop_ != nullptr);
    CHECK_EQ(multinode_filters_->boot_uuid(
                 configuration::GetNodeIndex(event_loop_->configuration(),
                                             source_state->node()),
                 timestamped_message.monotonic_remote_time.boot),
             source_state->event_loop_->boot_uuid());
  }

  SharedSpan to_send;
  // Right before sending allow the user to process the message.
  if (before_send_callbacks_[timestamped_message.channel_index]) {
    // Only channels which are forwarded and on the destination node have
    // channel_source_state_ set to non-null.  See RegisterDuringStartup.
    if (channel_source_state_[timestamped_message.channel_index] == nullptr) {
      // It is safe in this case since there is only one caller to Send, and the
      // data is not mutated after Send is called.
      to_send = before_send_callbacks_[timestamped_message.channel_index](
          timestamped_message);
      *timestamped_message.data.get() = to_send;
    } else {
      to_send = *timestamped_message.data;
    }
    if (!to_send) {
      return false;
    }
  } else {
    to_send = *timestamped_message.data;
  }

  // Send!  Use the replayed queue index here instead of the logged queue index
  // for the remote queue index.  This makes re-logging work.
  const UUID boot_uuid = [&]() -> UUID {
    if (channel_source_state_[timestamped_message.channel_index] != nullptr) {
      CHECK(multinode_filters_ != nullptr);
      return multinode_filters_->boot_uuid(
          configuration::GetNodeIndex(
              event_loop_->configuration(),
              channel_source_state_[timestamped_message.channel_index]->node()),
          timestamped_message.monotonic_remote_time.boot);
    } else {
      return event_loop_->boot_uuid();
    }
  }();
  const RawSender::Error err = sender->Send(
      std::move(to_send), timestamped_message.monotonic_remote_time.time,
      timestamped_message.realtime_remote_time,
      timestamped_message.monotonic_remote_transmit_time.time,
      remote_queue_index, boot_uuid);
  if (err != RawSender::Error::kOk) return false;
  if (monotonic_start_time(timestamped_message.monotonic_event_time.boot) <=
      timestamped_message.monotonic_event_time.time) {
    // Only track errors for non-fetched messages.
    TrackMessageSendTiming(
        *sender,
        timestamped_message.monotonic_event_time.time + clock_offset());
  }

  if (queue_index_map_[timestamped_message.channel_index]) {
    CHECK_EQ(timestamped_message.monotonic_event_time.boot, boot_count());
    if (queue_index_map_[timestamped_message.channel_index]->empty()) {
      // Nothing here, start a range with 0 length.
      ContiguousSentTimestamp timestamp;
      timestamp.starting_monotonic_event_time =
          timestamp.ending_monotonic_event_time =
              timestamped_message.monotonic_event_time.time;
      timestamp.starting_queue_index = timestamp.ending_queue_index =
          timestamped_message.queue_index.index;
      timestamp.actual_queue_index = sender->sent_queue_index();
      queue_index_map_[timestamped_message.channel_index]->emplace_back(
          timestamp);
    } else {
      // We've got something.  See if the next timestamp is still contiguous. If
      // so, grow it.
      ContiguousSentTimestamp *back =
          &queue_index_map_[timestamped_message.channel_index]->back();
      if ((back->starting_queue_index - back->actual_queue_index) ==
          (timestamped_message.queue_index.index -
           sender->sent_queue_index())) {
        back->ending_queue_index = timestamped_message.queue_index.index;
        back->ending_monotonic_event_time =
            timestamped_message.monotonic_event_time.time;
      } else {
        // Otherwise, make a new one.
        ContiguousSentTimestamp timestamp;
        timestamp.starting_monotonic_event_time =
            timestamp.ending_monotonic_event_time =
                timestamped_message.monotonic_event_time.time;
        timestamp.starting_queue_index = timestamp.ending_queue_index =
            timestamped_message.queue_index.index;
        timestamp.actual_queue_index = sender->sent_queue_index();
        queue_index_map_[timestamped_message.channel_index]->emplace_back(
            timestamp);
      }
    }

    // TODO(austin): Should we prune the map?  On a many day log, I only saw the
    // queue index diverge a couple of elements, which would be a very small
    // map.
  } else if (remote_timestamp_senders_[timestamped_message.channel_index] !=
             nullptr) {
    // TODO(james): Currently, If running replay against a single event loop,
    // remote timestamps will not get replayed because this code-path only
    // gets triggered on the event loop that receives the forwarded message
    // that the timestamps correspond to. This code, as written, also doesn't
    // correctly handle a non-zero clock_offset for the *_remote_time fields.
    State *source_state =
        channel_source_state_[timestamped_message.channel_index];
    CHECK(source_state != nullptr);

    flatbuffers::FlatBufferBuilder fbb;
    fbb.ForceDefaults(true);
    flatbuffers::Offset<flatbuffers::Vector<uint8_t>> boot_uuid_offset =
        event_loop_->boot_uuid().PackVector(&fbb);

    RemoteMessage::Builder message_header_builder(fbb);

    message_header_builder.add_channel_index(
        factory_channel_index_[timestamped_message.channel_index]);

    // Swap the remote and sent metrics.  They are from the sender's
    // perspective, not the receiver's perspective.
    message_header_builder.add_monotonic_sent_time(
        sender->monotonic_sent_time().time_since_epoch().count());
    message_header_builder.add_realtime_sent_time(
        sender->realtime_sent_time().time_since_epoch().count());
    message_header_builder.add_queue_index(sender->sent_queue_index());

    CHECK_EQ(timestamped_message.monotonic_remote_time.boot,
             source_state->boot_count());
    message_header_builder.add_monotonic_remote_time(
        timestamped_message.monotonic_remote_time.time.time_since_epoch()
            .count());
    message_header_builder.add_monotonic_remote_transmit_time(
        timestamped_message.monotonic_remote_transmit_time.time
            .time_since_epoch()
            .count());
    message_header_builder.add_realtime_remote_time(
        timestamped_message.realtime_remote_time.time_since_epoch().count());

    message_header_builder.add_remote_queue_index(remote_queue_index);
    message_header_builder.add_boot_uuid(boot_uuid_offset);

    fbb.Finish(message_header_builder.Finish());

    remote_timestamp_senders_[timestamped_message.channel_index]->Send(
        FlatbufferDetachedBuffer<RemoteMessage>(fbb.Release()),
        timestamped_message.monotonic_timestamp_time,
        source_state->boot_count());
  }

  return true;
}

LogReader::RemoteMessageSender::RemoteMessageSender(
    aos::Sender<message_bridge::RemoteMessage> sender, EventLoop *event_loop)
    : event_loop_(event_loop),
      sender_(std::move(sender)),
      timer_(event_loop->AddTimer([this]() { SendTimestamp(); })) {}

void LogReader::RemoteMessageSender::ScheduleTimestamp() {
  if (remote_timestamps_.empty()) {
    CHECK(timer_ != nullptr);
    timer_->Disable();
    scheduled_time_ = monotonic_clock::min_time;
    return;
  }

  if (scheduled_time_ != remote_timestamps_.front().monotonic_timestamp_time) {
    CHECK(timer_ != nullptr);
    timer_->Schedule(remote_timestamps_.front().monotonic_timestamp_time);
    scheduled_time_ = remote_timestamps_.front().monotonic_timestamp_time;
    CHECK_GE(scheduled_time_, event_loop_->monotonic_now())
        << event_loop_->node()->name()->string_view();
  }
}

void LogReader::RemoteMessageSender::Send(
    FlatbufferDetachedBuffer<RemoteMessage> remote_message,
    BootTimestamp monotonic_timestamp_time, size_t source_boot_count) {
  // There are 2 variants of logs.
  //   1) Logs without monotonic_timestamp_time
  //   2) Logs with monotonic_timestamp_time
  //
  // As of Jan 2021, we shouldn't have any more logs without
  // monotonic_timestamp_time.  We don't have data locked up in those logs worth
  // the effort of saving.
  //
  // This gives us 3 cases, 2 of which are undistinguishable.
  // 1) Old log without monotonic_timestamp_time.
  // 2) New log with monotonic_timestamp_time where the timestamp was logged
  //    remotely so we actually have monotonic_timestamp_time.
  // 3) New log, but the timestamp was logged on the node receiving the message
  //    so there is no monotonic_timestamp_time.
  //
  // Our goal when replaying is to accurately reproduce the state of the world
  // present when logging.  If a timestamp wasn't sent back across the network,
  // we shouldn't replay one back across the network.
  //
  // Given that we don't really care about 1, we can use the presence of the
  // timestamp to distinguish 2 and 3, and ignore 1.  If we don't have a
  // monotonic_timestamp_time, this means the message was logged locally and
  // remote timestamps can be ignored.
  if (monotonic_timestamp_time == BootTimestamp::min_time()) {
    return;
  }

  CHECK_EQ(monotonic_timestamp_time.boot, source_boot_count);

  remote_timestamps_.emplace(
      std::upper_bound(
          remote_timestamps_.begin(), remote_timestamps_.end(),
          monotonic_timestamp_time.time,
          [](const aos::monotonic_clock::time_point monotonic_timestamp_time,
             const Timestamp &timestamp) {
            return monotonic_timestamp_time <
                   timestamp.monotonic_timestamp_time;
          }),
      std::move(remote_message), monotonic_timestamp_time.time);
  ScheduleTimestamp();
}

void LogReader::RemoteMessageSender::SendTimestamp() {
  CHECK_EQ(event_loop_->context().monotonic_event_time, scheduled_time_)
      << event_loop_->node()->name()->string_view();
  CHECK(!remote_timestamps_.empty());

  // Send out all timestamps at the currently scheduled time.
  while (remote_timestamps_.front().monotonic_timestamp_time ==
         scheduled_time_) {
    CHECK_EQ(sender_.Send(std::move(remote_timestamps_.front().remote_message)),
             RawSender::Error::kOk);
    remote_timestamps_.pop_front();
    if (remote_timestamps_.empty()) {
      break;
    }
  }
  scheduled_time_ = monotonic_clock::min_time;

  ScheduleTimestamp();
}

LogReader::RemoteMessageSender *LogReader::State::RemoteTimestampSender(
    const Channel *channel, const Connection *connection) {
  message_bridge::ChannelTimestampFinder finder(event_loop_);
  // Look at any pre-created channel/connection pairs.
  {
    auto it =
        channel_timestamp_loggers_.find(std::make_pair(channel, connection));
    if (it != channel_timestamp_loggers_.end()) {
      return it->second.get();
    }
  }

  // That failed, so resolve the RemoteMessage channel timestamps will be logged
  // to.
  const Channel *timestamp_channel = finder.ForChannel(channel, connection);

  {
    // See if that has been created before.  If so, cache it in
    // channel_timestamp_loggers_ and return.
    auto it = timestamp_loggers_.find(timestamp_channel);
    if (it != timestamp_loggers_.end()) {
      CHECK(channel_timestamp_loggers_
                .try_emplace(std::make_pair(channel, connection), it->second)
                .second);
      return it->second.get();
    }
  }

  // Otherwise, make a sender, save it, and cache it.
  auto result = channel_timestamp_loggers_.try_emplace(
      std::make_pair(channel, connection),
      std::make_shared<RemoteMessageSender>(
          event_loop()->MakeSender<RemoteMessage>(
              timestamp_channel->name()->string_view()),
          event_loop()));

  CHECK(timestamp_loggers_.try_emplace(timestamp_channel, result.first->second)
            .second);
  return result.first->second.get();
}

Result<TimestampedMessage> LogReader::State::PopOldest() {
  // multithreaded
  if (message_queuer_.has_value()) {
    std::optional<Result<TimestampedMessage>> message = message_queuer_->Pop();
    CHECK(message.has_value()) << ": Unexpectedly ran out of messages.";
    // If there is an error during message reading, propagate it up.
    if (!message.value().has_value()) {
      return MakeError(message.value().error());
    }
    message_queuer_->SetState(
        message.value().value().monotonic_event_time +
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(
                absl::GetFlag(FLAGS_threaded_look_ahead_seconds))));
    VLOG(1) << "Popped " << message.value().value()
            << configuration::CleanedChannelToString(
                   event_loop_->configuration()->channels()->Get(
                       factory_channel_index_[message->value().channel_index]));
    return message.value();
  } else {  // single threaded
    CHECK(timestamp_mapper_ != nullptr);
    return timestamp_mapper_->Front().and_then(
        [this](TimestampedMessage *result_ptr) -> Result<TimestampedMessage> {
          CHECK(result_ptr != nullptr);

          TimestampedMessage result = std::move(*result_ptr);

          VLOG(2) << "Node '" << MaybeNodeName(event_loop_->node())
                  << "': PopOldest Popping " << result.monotonic_event_time;
          AOS_RETURN_IF_ERROR(timestamp_mapper_->PopFront());
          AOS_RETURN_IF_ERROR(MaybeSeedSortedMessages());

          CHECK_EQ(result.monotonic_event_time.boot, boot_count());

          VLOG(1) << "Popped " << result
                  << configuration::CleanedChannelToString(
                         event_loop_->configuration()->channels()->Get(
                             factory_channel_index_[result.channel_index]));
          return result;
        });
  }
}

Result<BootTimestamp> LogReader::State::MultiThreadedOldestMessageTime() {
  if (!message_queuer_.has_value()) {
    return SingleThreadedOldestMessageTime();
  }
  std::optional<Result<TimestampedMessage>> message = message_queuer_->Peek();
  if (!message.has_value()) {
    return BootTimestamp::max_time();
  }
  if (!message.value().has_value()) {
    return MakeError(message.value().error());
  }
  if (message.value().value().monotonic_event_time.boot == boot_count()) {
    ObserveNextMessage(message.value().value().monotonic_event_time.time,
                       message.value().value().realtime_event_time);
  }
  return message.value().value().monotonic_event_time;
}

Result<BootTimestamp> LogReader::State::SingleThreadedOldestMessageTime() {
  CHECK(!message_queuer_.has_value())
      << "Cannot use SingleThreadedOldestMessageTime() once the queuer thread "
         "is created.";
  if (timestamp_mapper_ == nullptr) {
    return BootTimestamp::max_time();
  }
  return timestamp_mapper_->Front().transform(
      [this](TimestampedMessage *result_ptr) {
        if (result_ptr == nullptr) {
          return BootTimestamp::max_time();
        }
        VLOG(2) << "Node '" << MaybeNodeName(node()) << "': oldest message at "
                << result_ptr->monotonic_event_time.time;
        if (result_ptr->monotonic_event_time.boot == boot_count()) {
          ObserveNextMessage(result_ptr->monotonic_event_time.time,
                             result_ptr->realtime_event_time);
        }
        return result_ptr->monotonic_event_time;
      });
}

Result<void> LogReader::State::ReadTimestamps() {
  if (!timestamp_mapper_) return Ok();
  AOS_RETURN_IF_ERROR(timestamp_mapper_->QueueTimestamps());

  // TODO(austin): Maybe make timestamp mapper do this so we don't need to clear
  // it out?
  //
  // If we don't clear the timestamp callback, we end up getting a callback both
  // when SplitTimestampBootMerger sees the timestamp, and when TimestampMapper
  // sees it.
  timestamp_mapper_->set_timestamp_callback([](TimestampedMessage *) {});
  return Ok();
}

Result<void> LogReader::State::MaybeSeedSortedMessages() {
  if (!timestamp_mapper_) return Ok();

  // The whole purpose of seeding is to make timestamp_mapper load timestamps.
  // So if we have already loaded them, skip seeding.
  if (timestamp_queue_strategy_ ==
      TimestampQueueStrategy::kQueueTimestampsAtStartup) {
    return Ok();
  }

  return timestamp_mapper_->QueueFor(
      chrono::duration_cast<chrono::seconds>(chrono::duration<double>(
          absl::GetFlag(FLAGS_time_estimation_buffer_seconds))));
}

void LogReader::State::Deregister() {
  if (started_ && !stopped_) {
    NotifyLogfileEnd();
  }
  for (size_t i = 0; i < channels_.size(); ++i) {
    channels_[i].reset();
  }
  ClearTimeFlags();
  channel_timestamp_loggers_.clear();
  timestamp_loggers_.clear();
  event_loop_unique_ptr_.reset();
  event_loop_ = nullptr;
  timer_handler_ = nullptr;
  node_event_loop_factory_ = nullptr;
  timing_statistics_sender_ = Sender<timing::ReplayTiming>();
}

void LogReader::State::SetStartTimeFlag(realtime_clock::time_point start_time) {
  if (start_time != realtime_clock::min_time) {
    start_event_notifier_ = std::make_unique<EventNotifier>(
        event_loop_, [this]() { NotifyFlagStart(); }, "flag_start", start_time);
  }
}

void LogReader::State::SetEndTimeFlag(realtime_clock::time_point end_time) {
  if (end_time != realtime_clock::max_time) {
    end_event_notifier_ = std::make_unique<EventNotifier>(
        event_loop_, [this]() { NotifyFlagEnd(); }, "flag_end", end_time);
  }
}

void LogReader::State::ObserveNextMessage(
    monotonic_clock::time_point monotonic_event,
    realtime_clock::time_point realtime_event) {
  if (start_event_notifier_) {
    start_event_notifier_->ObserveNextMessage(monotonic_event, realtime_event);
  }
  if (end_event_notifier_) {
    end_event_notifier_->ObserveNextMessage(monotonic_event, realtime_event);
  }
}

void LogReader::State::ClearTimeFlags() {
  start_event_notifier_.reset();
  end_event_notifier_.reset();
}

void LogReader::State::NotifyLogfileStart() {
  // If the start_event_notifier_ is set, that means that a realtime start time
  // was set manually; when the override is set, we want to delay any startup
  // handlers that would've happened before requested start time until that
  // start time.
  if (start_event_notifier_) {
    // Only call OnStart() if the start time for this node
    // (realtime_start_time())
    if (start_event_notifier_->realtime_event_time() >
        realtime_start_time(boot_count())) {
      VLOG(1) << "Skipping, " << start_event_notifier_->realtime_event_time()
              << " > " << realtime_start_time(boot_count());
      return;
    }
  }
  if (found_last_message_) {
    VLOG(1) << "Last message already found, bailing";
    return;
  }
  RunOnStart();
}

void LogReader::State::NotifyFlagStart() {
  // Should only be called if start_event_notifier_ has been set (which happens
  // as part of setting an explicit start time); only call the startup functions
  // that occurred *before* the start flag value.
  if (start_event_notifier_->realtime_event_time() >=
      realtime_start_time(boot_count())) {
    RunOnStart();
  }
}

void LogReader::State::NotifyLogfileEnd() {
  // Don't execute the OnEnd handlers if the logfile was ended artifically
  // early.
  if (found_last_message_) {
    return;
  }

  // Ensure that we only call OnEnd() if OnStart() was already called for this
  // boot (and don't call OnEnd() twice).
  if (!stopped_ && started_) {
    RunOnEnd();
  }
}

void LogReader::State::NotifyFlagEnd() {
  // Ensure that we only call OnEnd() if OnStart() was already called for this
  // boot (and don't call OnEnd() twice).
  if (!stopped_ && started_) {
    RunOnEnd();
    SetFoundLastMessage(true);
    CHECK(notice_realtime_end_);
    notice_realtime_end_();

    if (message_queuer_.has_value()) {
      message_queuer_->StopPushing();
    }
  }
}

void LogReader::State::MaybeSetClockOffset() {
  if (node_event_loop_factory_ == nullptr) {
    // If not running with simulated event loop, set the monotonic clock
    // offset.
    clock_offset_ = event_loop()->monotonic_now() - monotonic_start_time(0);

    if (start_event_notifier_) {
      start_event_notifier_->SetClockOffset(clock_offset_);
    }
    if (end_event_notifier_) {
      end_event_notifier_->SetClockOffset(clock_offset_);
    }
  }
}

void LogReader::SetRealtimeReplayRate(double replay_rate) {
  CHECK(event_loop_factory_ != nullptr)
      << ": Can't set replay rate without an event loop factory (have you "
         "called Register()?).";
  event_loop_factory_->SetRealtimeReplayRate(replay_rate);
}

void LogReader::NoticeRealtimeEnd() {
  CHECK_GE(live_nodes_with_realtime_time_end_, 1u);
  --live_nodes_with_realtime_time_end_;
  if (live_nodes_with_realtime_time_end_ == 0 && exit_on_finish() &&
      event_loop_factory_ != nullptr) {
    event_loop_factory_->Exit();
  }
}

bool LogReader::AreStatesInitialized() const {
  for (const auto &state : states_) {
    if (state) {
      return true;
    }
  }
  return false;
}

bool LogReader::HasSender(size_t logged_channel_index) const {
  for (const auto &state : states_) {
    if (state != nullptr) {
      if (state->HasSender(logged_channel_index)) {
        return true;
      }
    }
  }
  return false;
}

}  // namespace logger
}  // namespace aos
