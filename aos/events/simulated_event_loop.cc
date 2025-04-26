#include "aos/events/simulated_event_loop.h"

#include <algorithm>
#include <deque>
#include <optional>
#include <queue>
#include <string_view>
#include <vector>

#include "absl/container/btree_map.h"

#include "aos/events/aos_logging.h"
#include "aos/events/simulated_network_bridge.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/realtime.h"
#include "aos/sanitizers.h"
#include "aos/util/phased_loop.h"

// TODO(austin): If someone runs a SimulatedEventLoop on a RT thread with
// die_on_malloc set, it won't die.  Really, we need to go RT, or fall back to
// the base thread's original RT state to be actually accurate.

namespace aos {

class SimulatedEventLoop;
class SimulatedFetcher;
class SimulatedChannel;

using CheckSentTooFast = NodeEventLoopFactory::CheckSentTooFast;
using ExclusiveSenders = NodeEventLoopFactory::ExclusiveSenders;
using EventLoopOptions = NodeEventLoopFactory::EventLoopOptions;

namespace {

std::string NodeName(const Node *node) {
  if (node == nullptr) {
    return "";
  }

  return absl::StrCat(node->name()->string_view(), " ");
}

class ScopedMarkRealtimeRestorer {
 public:
  ScopedMarkRealtimeRestorer(bool rt) : rt_(rt), prior_(MarkRealtime(rt)) {}
  ~ScopedMarkRealtimeRestorer() { CHECK_EQ(rt_, MarkRealtime(prior_)); }

 private:
  const bool rt_;
  const bool prior_;
};

// Container for both a message, and the context for it for simulation.  This
// makes tracking the timestamps associated with the data easy.
struct SimulatedMessage final {
  SimulatedMessage(const SimulatedMessage &) = delete;
  SimulatedMessage &operator=(const SimulatedMessage &) = delete;
  ~SimulatedMessage();

  // Creates a SimulatedMessage with size bytes of storage.
  // This is a shared_ptr so we don't have to implement refcounting or copying.
  static std::shared_ptr<SimulatedMessage> Make(SimulatedChannel *channel,
                                                const SharedSpan data);

  // Context for the data.
  Context context;

  SimulatedChannel *const channel = nullptr;

  // Owning span to this message's data. Depending on the sender may either
  // represent the data of just the flatbuffer, or max channel size.
  SharedSpan data;

  // Mutable view of above data. If empty, this message is not mutable.
  absl::Span<uint8_t> mutable_data;

  // Determines whether this message is mutable. Used for Send where the user
  // fills out a message stored internally then gives us the size of data used.
  bool is_mutable() const { return data->size() == mutable_data.size(); }

  // Note: this should be private but make_shared requires it to be public.  Use
  // Make() above to construct.
  SimulatedMessage(SimulatedChannel *channel_in);
};

}  // namespace

// TODO(Brian): This should be in the anonymous namespace, but that annoys GCC
// for some reason...
class SimulatedWatcher : public WatcherState, public EventScheduler::Event {
 public:
  SimulatedWatcher(
      SimulatedEventLoop *simulated_event_loop, EventScheduler *scheduler,
      const Channel *channel,
      std::function<void(const Context &context, const void *message)> fn);

  ~SimulatedWatcher() override;

  bool has_run() const;

  void Handle() noexcept override;

  void Construct() override {}
  void Startup() override {}

  void Schedule(std::shared_ptr<SimulatedMessage> message);

  void HandleEvent() noexcept;

  void SetSimulatedChannel(SimulatedChannel *channel) {
    simulated_channel_ = channel;
  }

 private:
  void DoSchedule(monotonic_clock::time_point event_time);

  ::std::deque<std::shared_ptr<SimulatedMessage>> msgs_;

  SimulatedEventLoop *const simulated_event_loop_;
  const Channel *const channel_;
  EventScheduler *const scheduler_;
  EventHandler<SimulatedWatcher> event_;
  EventScheduler::Token token_;
  SimulatedChannel *simulated_channel_ = nullptr;
};

class SimulatedFactoryExitHandle : public ExitHandle {
 public:
  SimulatedFactoryExitHandle(SimulatedEventLoopFactory *factory)
      : factory_(factory) {
    ++factory_->exit_handle_count_;
  }
  ~SimulatedFactoryExitHandle() override {
    CHECK_GT(factory_->exit_handle_count_, 0);
    --factory_->exit_handle_count_;
  }

  void Exit(Result<void> status) override { factory_->Exit(status); }

 private:
  SimulatedEventLoopFactory *const factory_;
};

class SimulatedChannel {
 public:
  explicit SimulatedChannel(const Channel *channel,
                            std::chrono::nanoseconds channel_storage_duration,
                            const EventScheduler *scheduler)
      : channel_(channel),
        channel_storage_duration_(channel_storage_duration),
        next_queue_index_(ipc_lib::QueueIndex::Zero(number_buffers())),
        scheduler_(scheduler) {
    // Gut check that things fit.  Configuration validation should have caught
    // this before we get here.
    CHECK_LT(static_cast<size_t>(number_buffers()),
             std::numeric_limits<
                 decltype(available_buffer_indices_)::value_type>::max())
        << configuration::CleanedChannelToString(channel);
    available_buffer_indices_.resize(number_buffers());
    for (int i = 0; i < number_buffers(); ++i) {
      available_buffer_indices_[i] = i;
    }
  }

  ~SimulatedChannel() {
    latest_message_.reset();
    CHECK_EQ(static_cast<size_t>(number_buffers()),
             available_buffer_indices_.size());
    CHECK_EQ(0u, fetchers_.size())
        << configuration::StrippedChannelToString(channel());
    CHECK_EQ(0u, watchers_.size())
        << configuration::StrippedChannelToString(channel());
    CHECK_EQ(0, sender_count_)
        << configuration::StrippedChannelToString(channel());
  }

  // The number of messages we pretend to have in the queue.
  int queue_size() const {
    return configuration::QueueSize(channel()->frequency(),
                                    channel_storage_duration_);
  }

  std::chrono::nanoseconds channel_storage_duration() const {
    return channel_storage_duration_;
  }

  // The number of extra buffers (beyond the queue) we pretend to have.
  int number_scratch_buffers() const {
    return configuration::QueueScratchBufferSize(channel());
  }

  int number_buffers() const { return queue_size() + number_scratch_buffers(); }

  int GetBufferIndex() {
    CHECK(!available_buffer_indices_.empty()) << ": This should be impossible";
    const int result = available_buffer_indices_.back();
    available_buffer_indices_.pop_back();
    return result;
  }

  void FreeBufferIndex(int i) {
    // This extra checking has a large performance hit with sanitizers that
    // track memory accesses, so just skip it.
#if !defined(AOS_SANITIZE_MEMORY) && !defined(AOS_SANITIZE_ADDRESS)
    DCHECK(std::find(available_buffer_indices_.begin(),
                     available_buffer_indices_.end(),
                     i) == available_buffer_indices_.end())
        << ": Buffer is not in use: " << i;
#endif
    available_buffer_indices_.push_back(i);
  }

  // Makes a connected raw sender which calls Send below.
  ::std::unique_ptr<RawSender> MakeRawSender(SimulatedEventLoop *event_loop);

  // Makes a connected raw fetcher.
  ::std::unique_ptr<RawFetcher> MakeRawFetcher(EventLoop *event_loop);

  // Registers a watcher for the queue.
  void MakeRawWatcher(SimulatedWatcher *watcher);

  void RemoveWatcher(SimulatedWatcher *watcher) {
    watchers_.erase(std::find(watchers_.begin(), watchers_.end(), watcher));
  }

  // Sends the message to all the connected receivers and fetchers.  Returns the
  // sent queue index, or std::nullopt if messages were sent too fast.
  std::optional<uint32_t> Send(std::shared_ptr<SimulatedMessage> message,
                               CheckSentTooFast check_sent_too_fast);

  // Unregisters a fetcher.
  void UnregisterFetcher(SimulatedFetcher *fetcher);

  std::shared_ptr<SimulatedMessage> latest_message() { return latest_message_; }

  size_t max_size() const { return channel()->max_size(); }

  const std::string_view name() const {
    return channel()->name()->string_view();
  }

  const Channel *channel() const { return channel_; }

  void CountSenderCreated() {
    if (sender_count_ >= channel()->num_senders()) {
      LOG(FATAL) << "Failed to create sender on "
                 << configuration::CleanedChannelToString(channel())
                 << ", too many senders.";
    }
    CheckBufferCount();
    ++sender_count_;
  }

  void CountSenderDestroyed() {
    --sender_count_;
    CHECK_GE(sender_count_, 0);
    if (sender_count_ == 0) {
      allow_new_senders_ = true;
    }
  }

 private:
  void CheckBufferCount() {
    int reader_count = 0;
    if (channel()->read_method() == ReadMethod::PIN) {
      reader_count = watchers_.size() + fetchers_.size();
    }
    CHECK_LT(reader_count + sender_count_, number_scratch_buffers());
  }

  void CheckReaderCount() {
    if (channel()->read_method() != ReadMethod::PIN) {
      return;
    }
    CheckBufferCount();
    const int reader_count = watchers_.size() + fetchers_.size();
    if (reader_count >= channel()->num_readers()) {
      LOG(FATAL) << "Failed to create reader on "
                 << configuration::CleanedChannelToString(channel())
                 << ", too many readers.";
    }
  }

  const Channel *const channel_;
  const std::chrono::nanoseconds channel_storage_duration_;

  // List of all watchers.
  ::std::vector<SimulatedWatcher *> watchers_;

  // List of all fetchers.
  ::std::vector<SimulatedFetcher *> fetchers_;
  std::shared_ptr<SimulatedMessage> latest_message_;

  ipc_lib::QueueIndex next_queue_index_;

  int sender_count_ = 0;
  // Used to track when an exclusive sender has been created (e.g., for log
  // replay) and we want to prevent new senders from being accidentally created.
  bool allow_new_senders_ = true;

  std::vector<ipc_lib::QueueIndex::PackedIndexType> available_buffer_indices_;

  const EventScheduler *scheduler_;

  // Queue of all the message send times in the last channel_storage_duration_
  std::queue<monotonic_clock::time_point> last_times_;
};

namespace {

std::shared_ptr<SimulatedMessage> SimulatedMessage::Make(
    SimulatedChannel *channel, SharedSpan data) {
  // The allocations in here are due to infrastructure and don't count in the no
  // mallocs in RT code.
  ScopedNotRealtime nrt;

  auto message = std::make_shared<SimulatedMessage>(channel);
  message->context.size = data->size();
  message->context.data = data->data();
  message->data = std::move(data);

  return message;
}

SimulatedMessage::SimulatedMessage(SimulatedChannel *channel_in)
    : channel(channel_in) {
  context.buffer_index = channel->GetBufferIndex();
}

SimulatedMessage::~SimulatedMessage() {
  channel->FreeBufferIndex(context.buffer_index);
}

class SimulatedSender : public RawSender {
 public:
  SimulatedSender(SimulatedChannel *simulated_channel,
                  SimulatedEventLoop *event_loop);
  ~SimulatedSender() override;

  void *data() override {
    if (!message_) {
      // This API is safe to use in a RT context on a RT system.  So annotate it
      // accordingly.
      ScopedNotRealtime nrt;

      auto [span, mutable_span] =
          MakeSharedSpan(simulated_channel_->max_size());
      message_ = SimulatedMessage::Make(simulated_channel_, span);
      message_->mutable_data = mutable_span;
    }
    CHECK(message_->is_mutable());
    return message_->mutable_data.data();
  }

  size_t size() override { return simulated_channel_->max_size(); }

  Error DoSend(size_t length, monotonic_clock::time_point monotonic_remote_time,
               realtime_clock::time_point realtime_remote_time,
               monotonic_clock::time_point monotonic_remote_transmit_time,
               uint32_t remote_queue_index,
               const UUID &source_boot_uuid) override;

  Error DoSend(const void *msg, size_t size,
               monotonic_clock::time_point monotonic_remote_time,
               realtime_clock::time_point realtime_remote_time,
               monotonic_clock::time_point monotonic_remote_transmit_time,
               uint32_t remote_queue_index,
               const UUID &source_boot_uuid) override;

  Error DoSend(const SharedSpan data,
               aos::monotonic_clock::time_point monotonic_remote_time,
               aos::realtime_clock::time_point realtime_remote_time,
               monotonic_clock::time_point monotonic_remote_transmit_time,
               uint32_t remote_queue_index,
               const UUID &source_boot_uuid) override;

  int buffer_index() override {
    // First, ensure message_ is allocated.
    data();
    return message_->context.buffer_index;
  }

 private:
  SimulatedChannel *simulated_channel_;
  SimulatedEventLoop *simulated_event_loop_;

  std::shared_ptr<SimulatedMessage> message_;
};
}  // namespace

class SimulatedFetcher : public RawFetcher {
 public:
  explicit SimulatedFetcher(EventLoop *event_loop,
                            SimulatedChannel *simulated_channel)
      : RawFetcher(event_loop, simulated_channel->channel()),
        simulated_channel_(simulated_channel) {}
  ~SimulatedFetcher() {
    CHECK(!event_loop()->is_running()) << ": Can't make Fetcher while running";
    simulated_channel_->UnregisterFetcher(this);
  }

  std::pair<bool, monotonic_clock::time_point> DoFetchNext() override {
    return DoFetchNextIf(std::function<bool(const Context &context)>());
  }

  std::pair<bool, monotonic_clock::time_point> DoFetchNextIf(
      std::function<bool(const Context &context)> fn) override {
    // The allocations in here are due to infrastructure and don't count in the
    // no mallocs in RT code.
    ScopedNotRealtime nrt;
    if (msgs_.size() == 0) {
      return std::make_pair(false, monotonic_clock::min_time);
    }

    CHECK(!fell_behind_) << ": Got behind on "
                         << configuration::StrippedChannelToString(
                                simulated_channel_->channel())
                         << " on " << NodeName(event_loop()->node());

    if (fn) {
      Context context = msgs_.front()->context;
      context.data = nullptr;
      context.buffer_index = -1;

      if (!fn(context)) {
        return std::make_pair(false, monotonic_clock::min_time);
      }
    }

    SetMsg(std::move(msgs_.front()));
    msgs_.pop_front();
    return std::make_pair(true, event_loop()->monotonic_now());
  }

  std::pair<bool, monotonic_clock::time_point> DoFetch() override {
    return DoFetchIf(std::function<bool(const Context &context)>());
  }

  std::pair<bool, monotonic_clock::time_point> DoFetchIf(
      std::function<bool(const Context &context)> fn) override {
    // The allocations in here are due to infrastructure and don't count in the
    // no mallocs in RT code.
    ScopedNotRealtime nrt;
    if (msgs_.size() == 0) {
      // TODO(austin): Can we just do this logic unconditionally?  It is a lot
      // simpler.  And call clear, obviously.
      if (!msg_ && simulated_channel_->latest_message()) {
        std::shared_ptr<SimulatedMessage> latest_message =
            simulated_channel_->latest_message();

        if (fn) {
          Context context = latest_message->context;
          context.data = nullptr;
          context.buffer_index = -1;

          if (!fn(context)) {
            return std::make_pair(false, monotonic_clock::min_time);
          }
        }
        SetMsg(std::move(latest_message));
        return std::make_pair(true, event_loop()->monotonic_now());
      } else {
        return std::make_pair(false, monotonic_clock::min_time);
      }
    }

    if (fn) {
      Context context = msgs_.back()->context;
      context.data = nullptr;
      context.buffer_index = -1;

      if (!fn(context)) {
        return std::make_pair(false, monotonic_clock::min_time);
      }
    }

    // We've had a message enqueued, so we don't need to go looking for the
    // latest message from before we started.
    SetMsg(msgs_.back());
    msgs_.clear();
    fell_behind_ = false;
    return std::make_pair(true, event_loop()->monotonic_now());
  }

 private:
  friend class SimulatedChannel;

  // Updates the state inside RawFetcher to point to the data in msg_.
  void SetMsg(std::shared_ptr<SimulatedMessage> msg) {
    msg_ = std::move(msg);
    context_ = msg_->context;
    if (channel()->read_method() != ReadMethod::PIN) {
      context_.buffer_index = -1;
    }
    if (context_.remote_queue_index == 0xffffffffu) {
      context_.remote_queue_index = context_.queue_index;
    }
    if (context_.monotonic_remote_time == monotonic_clock::min_time) {
      context_.monotonic_remote_time = context_.monotonic_event_time;
    }
    if (context_.realtime_remote_time == realtime_clock::min_time) {
      context_.realtime_remote_time = context_.realtime_event_time;
    }
  }

  // Internal method for Simulation to add a message to the buffer.
  void Enqueue(std::shared_ptr<SimulatedMessage> buffer) {
    msgs_.emplace_back(std::move(buffer));
    if (fell_behind_ ||
        msgs_.size() > static_cast<size_t>(simulated_channel_->queue_size())) {
      fell_behind_ = true;
      // Might as well empty out all the intermediate messages now.
      while (msgs_.size() > 1) {
        msgs_.pop_front();
      }
    }
  }

  SimulatedChannel *simulated_channel_;
  std::shared_ptr<SimulatedMessage> msg_;

  // Messages queued up but not in use.
  ::std::deque<std::shared_ptr<SimulatedMessage>> msgs_;

  // Whether we're currently "behind", which means a FetchNext call will fail.
  bool fell_behind_ = false;
};

class SimulatedTimerHandler : public TimerHandler,
                              public EventScheduler::Event {
 public:
  explicit SimulatedTimerHandler(EventScheduler *scheduler,
                                 SimulatedEventLoop *simulated_event_loop,
                                 ::std::function<void()> fn);
  ~SimulatedTimerHandler() { Disable(); }

  void Schedule(monotonic_clock::time_point base,
                monotonic_clock::duration repeat_offset) override;

  void HandleEvent() noexcept;

  void Handle() noexcept override;

  void Disable() override;

  bool IsDisabled() override;

 private:
  SimulatedEventLoop *simulated_event_loop_;
  EventHandler<SimulatedTimerHandler> event_;
  EventScheduler *scheduler_;
  EventScheduler::Token token_;

  monotonic_clock::time_point base_;
  monotonic_clock::duration repeat_offset_;
  bool disabled_ = true;
};

class SimulatedPhasedLoopHandler : public PhasedLoopHandler,
                                   public EventScheduler::Event {
 public:
  SimulatedPhasedLoopHandler(EventScheduler *scheduler,
                             SimulatedEventLoop *simulated_event_loop,
                             ::std::function<void(int)> fn,
                             const monotonic_clock::duration interval,
                             const monotonic_clock::duration offset);
  ~SimulatedPhasedLoopHandler();

  void HandleEvent() noexcept;

  void Schedule(monotonic_clock::time_point sleep_time) override;

  void Handle() noexcept override;

 private:
  SimulatedEventLoop *simulated_event_loop_;
  EventHandler<SimulatedPhasedLoopHandler> event_;

  EventScheduler *scheduler_;
  EventScheduler::Token token_;
};

class SimulatedEventLoop : public EventLoop {
 public:
  explicit SimulatedEventLoop(
      EventScheduler *scheduler, NodeEventLoopFactory *node_event_loop_factory,
      absl::btree_map<SimpleChannel, std::unique_ptr<SimulatedChannel>>
          *channels,
      const Configuration *configuration,
      std::vector<SimulatedEventLoop *> *event_loops_, const Node *node,
      pid_t tid, EventLoopOptions options)
      : EventLoop(configuration),
        scheduler_(scheduler),
        node_event_loop_factory_(node_event_loop_factory),
        channels_(channels),
        event_loops_(event_loops_),
        node_(node),
        tid_(tid),
        startup_tracker_(std::make_shared<StartupTracker>()),
        options_(options) {
    ClearContext();
    startup_tracker_->loop = this;
    scheduler_->ScheduleOnStartup([startup_tracker = startup_tracker_]() {
      if (startup_tracker->loop) {
        startup_tracker->loop->Setup();
        startup_tracker->has_setup = true;
      }
    });

    event_loops_->push_back(this);
    scheduler_->ScheduleOnRun(&on_run_event_);
    on_run_scheduled_ = true;
  }

  ~SimulatedEventLoop() override {
    // Unregister any on_run callbacks to handle cleanup before they run
    // correctly.
    if (on_run_scheduled_) {
      scheduler_->DeleteOnRun(&on_run_event_);
    }

    // Trigger any remaining senders or fetchers to be cleared before destroying
    // the event loop so the book keeping matches.
    timing_report_sender_.reset();

    // Force everything with a registered fd with epoll to be destroyed now.
    timers_.clear();
    phased_loops_.clear();
    watchers_.clear();

    for (auto it = event_loops_->begin(); it != event_loops_->end(); ++it) {
      if (*it == this) {
        event_loops_->erase(it);
        break;
      }
    }
    VLOG(1) << scheduler_->distributed_now() << " " << NodeName(node())
            << monotonic_now() << " ~SimulatedEventLoop(\"" << name_ << "\")";
    startup_tracker_->loop = nullptr;
  }

  void SetIsRunning(bool running) {
    VLOG(1) << scheduler_->distributed_now() << " " << NodeName(node())
            << monotonic_now() << " " << name_ << " set_is_running(" << running
            << ")";
    CHECK(startup_tracker_->has_setup);

    set_is_running(running);
    if (running) {
      has_run_ = true;
    }
  }

  bool has_run() const { return has_run_; }

  std::chrono::nanoseconds send_delay() const { return send_delay_; }
  void set_send_delay(std::chrono::nanoseconds send_delay) {
    send_delay_ = send_delay;
  }

  monotonic_clock::time_point monotonic_now() const override {
    return node_event_loop_factory_->monotonic_now();
  }

  realtime_clock::time_point realtime_now() const override {
    return node_event_loop_factory_->realtime_now();
  }

  distributed_clock::time_point distributed_now() {
    return scheduler_->distributed_now();
  }

  std::unique_ptr<RawSender> MakeRawSender(const Channel *channel) override;

  std::unique_ptr<RawFetcher> MakeRawFetcher(const Channel *channel) override;

  void MakeRawWatcher(
      const Channel *channel,
      ::std::function<void(const Context &context, const void *message)>
          watcher) override;

  TimerHandler *AddTimer(::std::function<void()> callback) override {
    CHECK(!is_running());
    return NewTimer(::std::unique_ptr<TimerHandler>(
        new SimulatedTimerHandler(scheduler_, this, callback)));
  }

  PhasedLoopHandler *AddPhasedLoop(::std::function<void(int)> callback,
                                   const monotonic_clock::duration interval,
                                   const monotonic_clock::duration offset =
                                       ::std::chrono::seconds(0)) override {
    return NewPhasedLoop(
        ::std::unique_ptr<PhasedLoopHandler>(new SimulatedPhasedLoopHandler(
            scheduler_, this, callback, interval, offset)));
  }

  class OnRunEvent : public EventScheduler::Event {
   public:
    OnRunEvent(SimulatedEventLoop *loop) : loop_(loop) {}

    virtual void Handle() noexcept { loop_->DoOnRun(); }

   private:
    SimulatedEventLoop *loop_;
  };

  void OnRun(::std::function<void()> on_run) override {
    CHECK(!is_running()) << ": Cannot register OnRun callback while running.";
    CHECK(on_run_scheduled_)
        << "Registering OnRun callback after running on " << name();
    on_run_.emplace_back(std::move(on_run));
  }

  // Called by OnRunEvent when we need to process OnRun callbacks.
  void DoOnRun() {
    VLOG(1) << distributed_now() << " " << NodeName(node()) << monotonic_now()
            << " " << name() << " OnRun()";
    on_run_scheduled_ = false;
    while (!on_run_.empty()) {
      std::function<void()> fn = std::move(*on_run_.begin());
      on_run_.erase(on_run_.begin());

      logging::ScopedLogRestorer prev_logger;
      if (log_impl_) {
        prev_logger.Swap(log_impl_);
      }
      ScopedMarkRealtimeRestorer rt(runtime_realtime_priority() > 0);
      SetTimerContext(monotonic_now());
      fn();
      ClearContext();
    }
  }

  const Node *node() const override { return node_; }

  void set_name(const std::string_view name) override {
    name_ = std::string(name);
  }
  const std::string_view name() const override { return name_; }

  SimulatedChannel *GetSimulatedChannel(const Channel *channel);

  void SetRuntimeRealtimePriority(int priority) override {
    CHECK(!is_running()) << ": Cannot set realtime priority while running.";
    priority_ = priority;
  }

  int runtime_realtime_priority() const override { return priority_; }
  const cpu_set_t &runtime_affinity() const override { return affinity_; }

  void SetRuntimeAffinity(const cpu_set_t &affinity) override {
    CHECK(!is_running()) << ": Cannot set affinity while running.";
    affinity_ = affinity;
  }

  void Setup() {
    MaybeScheduleTimingReports();
    if (!skip_logger_) {
      log_sender_.Initialize(&name_,
                             MakeSender<logging::LogMessageFbs>("/aos"));
      log_impl_ = log_sender_.implementation();
    }
  }

  int NumberBuffers(const Channel *channel) override;

  const UUID &boot_uuid() const override {
    return node_event_loop_factory_->boot_uuid();
  }

  const EventLoopOptions &options() const { return options_; }

 private:
  friend class SimulatedTimerHandler;
  friend class SimulatedPhasedLoopHandler;
  friend class SimulatedWatcher;

  // We have a condition where we register a startup handler, but then get shut
  // down before it runs.  This results in a segfault if we are lucky, and
  // corruption otherwise.  To handle that, allocate a small object which points
  // back to us and can be freed when the function is freed.  That object can
  // then be updated when we get destroyed so setup is not called.
  struct StartupTracker {
    SimulatedEventLoop *loop = nullptr;
    bool has_setup = false;
  };

  void HandleEvent() {
    while (true) {
      if (EventCount() == 0 || PeekEvent()->event_time() > monotonic_now()) {
        break;
      }

      EventLoopEvent *event = PopEvent();
      event->HandleEvent();
    }
  }

  pid_t GetTid() override { return tid_; }

  EventScheduler *scheduler_;
  NodeEventLoopFactory *node_event_loop_factory_;
  absl::btree_map<SimpleChannel, std::unique_ptr<SimulatedChannel>> *channels_;
  std::vector<SimulatedEventLoop *> *event_loops_;

  ::std::string name_;

  int priority_ = 0;
  cpu_set_t affinity_ = DefaultAffinity();

  std::chrono::nanoseconds send_delay_;

  const Node *const node_;
  const pid_t tid_;

  AosLogToFbs log_sender_;
  std::shared_ptr<logging::LogImplementation> log_impl_ = nullptr;

  bool has_run_ = false;

  std::shared_ptr<StartupTracker> startup_tracker_;

  EventLoopOptions options_;

  // Event used by EventScheduler to run OnRun callbacks.
  OnRunEvent on_run_event_{this};

  // True if the on run callbacks have been scheduled in the scheduler, and
  // false if they have been executed.
  bool on_run_scheduled_;

  std::vector<std::function<void()>> on_run_;
};

void SimulatedEventLoopFactory::set_send_delay(
    std::chrono::nanoseconds send_delay) {
  send_delay_ = send_delay;
  for (std::unique_ptr<NodeEventLoopFactory> &node : node_factories_) {
    if (node) {
      for (SimulatedEventLoop *loop : node->event_loops_) {
        loop->set_send_delay(send_delay_);
      }
    }
  }
}

void SimulatedEventLoopFactory::SetRealtimeReplayRate(double replay_rate) {
  scheduler_scheduler_.SetReplayRate(replay_rate);
}

void SimulatedEventLoop::MakeRawWatcher(
    const Channel *channel,
    std::function<void(const Context &channel, const void *message)> watcher) {
  TakeWatcher(channel);

  std::unique_ptr<SimulatedWatcher> shm_watcher =
      std::make_unique<SimulatedWatcher>(this, scheduler_, channel,
                                         std::move(watcher));

  GetSimulatedChannel(channel)->MakeRawWatcher(shm_watcher.get());

  NewWatcher(std::move(shm_watcher));
  VLOG(1) << distributed_now() << " " << NodeName(node()) << monotonic_now()
          << " " << name() << " MakeRawWatcher(\""
          << configuration::StrippedChannelToString(channel) << "\")";

  // Order of operations gets kinda wonky if we let people make watchers after
  // running once.  If someone has a valid use case, we can reconsider.
  CHECK(!has_run()) << ": Can't add a watcher after running.";
}

std::unique_ptr<RawSender> SimulatedEventLoop::MakeRawSender(
    const Channel *channel) {
  TakeSender(channel);

  VLOG(1) << distributed_now() << " " << NodeName(node()) << monotonic_now()
          << " " << name() << " MakeRawSender(\""
          << configuration::StrippedChannelToString(channel) << "\")";
  return GetSimulatedChannel(channel)->MakeRawSender(this);
}

std::unique_ptr<RawFetcher> SimulatedEventLoop::MakeRawFetcher(
    const Channel *channel) {
  CHECK(!is_running()) << ": Can't make Fetcher while running";
  ChannelIndex(channel);

  if (!configuration::ChannelIsReadableOnNode(channel, node())) {
    LOG(FATAL) << "Channel { \"name\": \"" << channel->name()->string_view()
               << "\", \"type\": \"" << channel->type()->string_view()
               << "\" } is not able to be fetched on this node.  Check your "
                  "configuration.";
  }

  VLOG(1) << distributed_now() << " " << NodeName(node()) << monotonic_now()
          << " " << name() << " MakeRawFetcher(\""
          << configuration::StrippedChannelToString(channel) << "\")";
  return GetSimulatedChannel(channel)->MakeRawFetcher(this);
}

SimulatedChannel *SimulatedEventLoop::GetSimulatedChannel(
    const Channel *channel) {
  auto it = channels_->find(SimpleChannel(channel));
  if (it == channels_->end()) {
    it = channels_
             ->emplace(SimpleChannel(channel),
                       std::unique_ptr<SimulatedChannel>(new SimulatedChannel(
                           channel,
                           // There are a lot of tests which assume that 100 hz
                           // messages can actually be sent out at 100 hz and
                           // forwarded.  The jitter in wakeups causes small
                           // variation in timing.  Ignore that.
                           configuration::ChannelStorageDuration(
                               configuration(), channel) -
                               send_delay(),
                           scheduler_)))
             .first;
  }
  return it->second.get();
}

int SimulatedEventLoop::NumberBuffers(const Channel *channel) {
  return GetSimulatedChannel(channel)->number_buffers();
}

SimulatedWatcher::SimulatedWatcher(
    SimulatedEventLoop *simulated_event_loop, EventScheduler *scheduler,
    const Channel *channel,
    std::function<void(const Context &context, const void *message)> fn)
    : WatcherState(simulated_event_loop, channel, std::move(fn)),
      simulated_event_loop_(simulated_event_loop),
      channel_(channel),
      scheduler_(scheduler),
      event_(this),
      token_(scheduler_->InvalidToken()) {
  VLOG(1) << simulated_event_loop_->distributed_now() << " "
          << NodeName(simulated_event_loop_->node())
          << simulated_event_loop_->monotonic_now() << " "
          << simulated_event_loop_->name() << " Watching "
          << configuration::StrippedChannelToString(channel_);
}

SimulatedWatcher::~SimulatedWatcher() {
  VLOG(1) << simulated_event_loop_->distributed_now() << " "
          << NodeName(simulated_event_loop_->node())
          << simulated_event_loop_->monotonic_now() << " "
          << simulated_event_loop_->name() << " ~Watching "
          << configuration::StrippedChannelToString(channel_);
  simulated_event_loop_->RemoveEvent(&event_);
  if (token_ != scheduler_->InvalidToken()) {
    scheduler_->Deschedule(token_);
  }
  CHECK(simulated_channel_ != nullptr);
  simulated_channel_->RemoveWatcher(this);
}

bool SimulatedWatcher::has_run() const {
  return simulated_event_loop_->has_run();
}

void SimulatedWatcher::Schedule(std::shared_ptr<SimulatedMessage> message) {
  monotonic_clock::time_point event_time =
      simulated_event_loop_->monotonic_now();

  // Messages are queued in order.  If we are the first, add ourselves.
  // Otherwise, don't.
  if (msgs_.size() == 0) {
    event_.set_event_time(message->context.monotonic_event_time);
    simulated_event_loop_->AddEvent(&event_);

    DoSchedule(event_time);
  }

  msgs_.emplace_back(std::move(message));
}

void SimulatedWatcher::HandleEvent() noexcept {
  const monotonic_clock::time_point monotonic_now =
      simulated_event_loop_->monotonic_now();
  VLOG(1) << simulated_event_loop_->distributed_now() << " "
          << NodeName(simulated_event_loop_->node())
          << simulated_event_loop_->monotonic_now() << " "
          << simulated_event_loop_->name() << " Watcher "
          << configuration::StrippedChannelToString(channel_);
  CHECK_NE(msgs_.size(), 0u) << ": No events to handle.";

  logging::ScopedLogRestorer prev_logger;
  if (simulated_event_loop_->log_impl_) {
    prev_logger.Swap(simulated_event_loop_->log_impl_);
  }
  Context context = msgs_.front()->context;

  if (channel_->read_method() != ReadMethod::PIN) {
    context.buffer_index = -1;
  }
  if (context.remote_queue_index == 0xffffffffu) {
    context.remote_queue_index = context.queue_index;
  }
  if (context.monotonic_remote_time == monotonic_clock::min_time) {
    context.monotonic_remote_time = context.monotonic_event_time;
  }
  if (context.realtime_remote_time == realtime_clock::min_time) {
    context.realtime_remote_time = context.realtime_event_time;
  }

  {
    ScopedMarkRealtimeRestorer rt(
        simulated_event_loop_->runtime_realtime_priority() > 0);
    DoCallCallback([monotonic_now]() { return monotonic_now; }, context);
    simulated_event_loop_->ClearContext();
  }

  msgs_.pop_front();
  if (token_ != scheduler_->InvalidToken()) {
    scheduler_->Deschedule(token_);
    token_ = scheduler_->InvalidToken();
  }
  if (msgs_.size() != 0) {
    event_.set_event_time(msgs_.front()->context.monotonic_event_time);
    simulated_event_loop_->AddEvent(&event_);

    DoSchedule(event_.event_time());
  }
}

void SimulatedWatcher::Handle() noexcept {
  DCHECK(token_ != scheduler_->InvalidToken());
  token_ = scheduler_->InvalidToken();
  simulated_event_loop_->HandleEvent();
}

void SimulatedWatcher::DoSchedule(monotonic_clock::time_point event_time) {
  CHECK(token_ == scheduler_->InvalidToken())
      << ": May not schedule multiple times";
  token_ = scheduler_->Schedule(
      event_time + simulated_event_loop_->send_delay(), this);
}

void SimulatedChannel::MakeRawWatcher(SimulatedWatcher *watcher) {
  CheckReaderCount();
  watcher->SetSimulatedChannel(this);
  watchers_.emplace_back(watcher);
}

::std::unique_ptr<RawSender> SimulatedChannel::MakeRawSender(
    SimulatedEventLoop *event_loop) {
  CHECK(allow_new_senders_)
      << ": Attempted to create a new sender on exclusive channel "
      << configuration::StrippedChannelToString(channel_);
  std::optional<ExclusiveSenders> per_channel_option;
  for (const std::pair<const aos::Channel *, ExclusiveSenders> &per_channel :
       event_loop->options().per_channel_exclusivity) {
    if (per_channel.first->name()->string_view() ==
            channel_->name()->string_view() &&
        per_channel.first->type()->string_view() ==
            channel_->type()->string_view()) {
      CHECK(!per_channel_option.has_value())
          << ": Channel " << configuration::StrippedChannelToString(channel_)
          << " listed twice in per-channel list.";
      per_channel_option = per_channel.second;
    }
  }
  if (!per_channel_option.has_value()) {
    // This could just as easily be implemented by setting
    // per_channel_option to the global setting when we initialize it, but
    // then we'd lose track of whether a given channel appears twice in
    // the list.
    per_channel_option = event_loop->options().exclusive_senders;
  }
  if (per_channel_option.value() == ExclusiveSenders::kYes) {
    CHECK_EQ(0, sender_count_)
        << ": Attempted to add an exclusive sender on a channel with existing "
           "senders: "
        << configuration::StrippedChannelToString(channel_);
    allow_new_senders_ = false;
  }
  return ::std::unique_ptr<RawSender>(new SimulatedSender(this, event_loop));
}

::std::unique_ptr<RawFetcher> SimulatedChannel::MakeRawFetcher(
    EventLoop *event_loop) {
  CheckReaderCount();
  ::std::unique_ptr<SimulatedFetcher> fetcher(
      new SimulatedFetcher(event_loop, this));
  fetchers_.push_back(fetcher.get());
  return fetcher;
}

std::optional<uint32_t> SimulatedChannel::Send(
    std::shared_ptr<SimulatedMessage> message,
    CheckSentTooFast check_sent_too_fast) {
  const auto now = scheduler_->monotonic_now();
  // Remove times that are greater than or equal to a channel_storage_duration_
  // ago
  while (!last_times_.empty() &&
         (now >= channel_storage_duration_ + last_times_.front())) {
    last_times_.pop();
  }

  // Check that we are not sending messages too fast
  if (check_sent_too_fast == CheckSentTooFast::kYes &&
      static_cast<int>(last_times_.size()) >= queue_size()) {
    return std::nullopt;
  }

  const std::optional<uint32_t> queue_index = {next_queue_index_.index()};
  last_times_.push(now);

  message->context.queue_index = *queue_index;
  // Points to the actual data depending on the size set in context. Data may
  // allocate more than the actual size of the message, so offset from the back
  // of that to get the actual start of the data.
  message->context.data =
      message->data->data() + message->data->size() - message->context.size;

  DCHECK(channel()->has_schema())
      << ": Missing schema for channel "
      << configuration::StrippedChannelToString(channel());
  DCHECK(flatbuffers::Verify(
      *channel()->schema(), *channel()->schema()->root_table(),
      static_cast<const uint8_t *>(message->context.data),
      message->context.size))
      << ": Corrupted flatbuffer on " << channel()->name()->c_str() << " "
      << channel()->type()->c_str();

  next_queue_index_ = next_queue_index_.Increment();

  latest_message_ = std::move(message);
  for (SimulatedWatcher *watcher : watchers_) {
    if (watcher->has_run()) {
      watcher->Schedule(latest_message_);
    }
  }
  for (auto &fetcher : fetchers_) {
    fetcher->Enqueue(latest_message_);
  }
  return queue_index;
}

void SimulatedChannel::UnregisterFetcher(SimulatedFetcher *fetcher) {
  fetchers_.erase(::std::find(fetchers_.begin(), fetchers_.end(), fetcher));
}

SimulatedSender::SimulatedSender(SimulatedChannel *simulated_channel,
                                 SimulatedEventLoop *event_loop)
    : RawSender(event_loop, simulated_channel->channel()),
      simulated_channel_(simulated_channel),
      simulated_event_loop_(event_loop) {
  simulated_channel_->CountSenderCreated();
}

SimulatedSender::~SimulatedSender() {
  simulated_channel_->CountSenderDestroyed();
}

RawSender::Error SimulatedSender::DoSend(
    size_t length, monotonic_clock::time_point monotonic_remote_time,
    realtime_clock::time_point realtime_remote_time,
    monotonic_clock::time_point monotonic_remote_transmit_time,
    uint32_t remote_queue_index, const UUID &source_boot_uuid) {
  // The allocations in here are due to infrastructure and don't count in the
  // no mallocs in RT code.
  ScopedNotRealtime nrt;

  VLOG(1) << simulated_event_loop_->distributed_now() << " "
          << NodeName(simulated_event_loop_->node())
          << simulated_event_loop_->monotonic_now() << " "
          << simulated_event_loop_->name() << " Send "
          << configuration::StrippedChannelToString(channel());

  CHECK_LE(length, size()) << ": Attempting to send too big a message.";
  message_->context.monotonic_event_time =
      simulated_event_loop_->monotonic_now();
  message_->context.monotonic_remote_time = monotonic_remote_time;
  message_->context.remote_queue_index = remote_queue_index;
  message_->context.realtime_event_time = simulated_event_loop_->realtime_now();
  message_->context.realtime_remote_time = realtime_remote_time;
  message_->context.source_boot_uuid = source_boot_uuid;
  message_->context.monotonic_remote_transmit_time =
      monotonic_remote_transmit_time;
  CHECK_LE(length, message_->context.size);
  message_->context.size = length;

  const std::optional<uint32_t> optional_queue_index = simulated_channel_->Send(
      message_, simulated_event_loop_->options().check_sent_too_fast);

  // Check that we are not sending messages too fast
  if (!optional_queue_index) {
    VLOG(1) << simulated_event_loop_->distributed_now() << " "
            << NodeName(simulated_event_loop_->node())
            << simulated_event_loop_->monotonic_now() << " "
            << simulated_event_loop_->name() << "   -> SentTooFast "
            << configuration::StrippedChannelToString(channel())
            << ", Tried to send more than " << simulated_channel_->queue_size()
            << " (queue size) messages in the last "
            << std::chrono::duration<double>(
                   simulated_channel_->channel_storage_duration())
                   .count()
            << " seconds (channel storage duration)";
    return Error::kMessagesSentTooFast;
  }

  sent_queue_index_ = *optional_queue_index;
  monotonic_sent_time_ = simulated_event_loop_->monotonic_now();
  realtime_sent_time_ = simulated_event_loop_->realtime_now();

  // Drop the reference to the message so that we allocate a new message for
  // next time.  Otherwise we will continue to reuse the same memory for all
  // messages and corrupt it.
  message_.reset();
  return Error::kOk;
}

RawSender::Error SimulatedSender::DoSend(
    const void *msg, size_t size,
    monotonic_clock::time_point monotonic_remote_time,
    realtime_clock::time_point realtime_remote_time,
    monotonic_clock::time_point monotonic_remote_transmit_time,
    uint32_t remote_queue_index, const UUID &source_boot_uuid) {
  CHECK_LE(size, this->size())
      << ": Attempting to send too big a message on "
      << configuration::CleanedChannelToString(simulated_channel_->channel());

  // Allocates an aligned buffer in which to copy unaligned msg.
  auto [span, mutable_span] = MakeSharedSpan(size);
  message_ = SimulatedMessage::Make(simulated_channel_, span);

  // Now fill in the message.  size is already populated above, and
  // queue_index will be populated in simulated_channel_.
  memcpy(mutable_span.data(), msg, size);

  return DoSend(size, monotonic_remote_time, realtime_remote_time,
                monotonic_remote_transmit_time, remote_queue_index,
                source_boot_uuid);
}

RawSender::Error SimulatedSender::DoSend(
    const SharedSpan data, monotonic_clock::time_point monotonic_remote_time,
    realtime_clock::time_point realtime_remote_time,
    monotonic_clock::time_point monotonic_remote_transmit_time,
    uint32_t remote_queue_index, const UUID &source_boot_uuid) {
  CHECK_LE(data->size(), this->size())
      << ": Attempting to send too big a message on "
      << configuration::CleanedChannelToString(simulated_channel_->channel());

  // Constructs a message sharing the already allocated and aligned message
  // data.
  message_ = SimulatedMessage::Make(simulated_channel_, data);

  return DoSend(data->size(), monotonic_remote_time, realtime_remote_time,
                monotonic_remote_transmit_time, remote_queue_index,
                source_boot_uuid);
}

SimulatedTimerHandler::SimulatedTimerHandler(
    EventScheduler *scheduler, SimulatedEventLoop *simulated_event_loop,
    ::std::function<void()> fn)
    : TimerHandler(simulated_event_loop, std::move(fn)),
      simulated_event_loop_(simulated_event_loop),
      event_(this),
      scheduler_(scheduler),
      token_(scheduler_->InvalidToken()) {}

void SimulatedTimerHandler::Schedule(monotonic_clock::time_point base,
                                     monotonic_clock::duration repeat_offset) {
  CHECK_GE(base, monotonic_clock::epoch());
  // The allocations in here are due to infrastructure and don't count in the no
  // mallocs in RT code.
  ScopedNotRealtime nrt;
  Disable();
  const monotonic_clock::time_point monotonic_now =
      simulated_event_loop_->monotonic_now();
  base_ = base;
  repeat_offset_ = repeat_offset;
  token_ = scheduler_->Schedule(std::max(base, monotonic_now), this);
  event_.set_event_time(base_);
  simulated_event_loop_->AddEvent(&event_);
  disabled_ = false;
}

void SimulatedTimerHandler::Handle() noexcept {
  DCHECK(token_ != scheduler_->InvalidToken());
  token_ = scheduler_->InvalidToken();
  simulated_event_loop_->HandleEvent();
}

void SimulatedTimerHandler::HandleEvent() noexcept {
  const monotonic_clock::time_point monotonic_now =
      simulated_event_loop_->monotonic_now();
  VLOG(1) << simulated_event_loop_->distributed_now() << " "
          << NodeName(simulated_event_loop_->node()) << monotonic_now << " "
          << simulated_event_loop_->name() << " Timer '" << name() << "'";
  logging::ScopedLogRestorer prev_logger;
  if (simulated_event_loop_->log_impl_) {
    prev_logger.Swap(simulated_event_loop_->log_impl_);
  }
  if (token_ != scheduler_->InvalidToken()) {
    {
      ScopedNotRealtime nrt;
      scheduler_->Deschedule(token_);
    }
    token_ = scheduler_->InvalidToken();
  }
  if (repeat_offset_ != monotonic_clock::zero()) {
    // Reschedule.
    while (base_ <= monotonic_now) base_ += repeat_offset_;
    token_ = scheduler_->Schedule(base_, this);
    event_.set_event_time(base_);
    simulated_event_loop_->AddEvent(&event_);
    disabled_ = false;
  } else {
    disabled_ = true;
  }
  {
    ScopedMarkRealtimeRestorer rt(
        simulated_event_loop_->runtime_realtime_priority() > 0);
    Call([monotonic_now]() { return monotonic_now; }, monotonic_now);
    simulated_event_loop_->ClearContext();
  }
}

void SimulatedTimerHandler::Disable() {
  simulated_event_loop_->RemoveEvent(&event_);
  if (token_ != scheduler_->InvalidToken()) {
    {
      ScopedNotRealtime nrt;
      scheduler_->Deschedule(token_);
    }
    token_ = scheduler_->InvalidToken();
  }
  disabled_ = true;
}

bool SimulatedTimerHandler::IsDisabled() { return disabled_; }

SimulatedPhasedLoopHandler::SimulatedPhasedLoopHandler(
    EventScheduler *scheduler, SimulatedEventLoop *simulated_event_loop,
    ::std::function<void(int)> fn, const monotonic_clock::duration interval,
    const monotonic_clock::duration offset)
    : PhasedLoopHandler(simulated_event_loop, std::move(fn), interval, offset),
      simulated_event_loop_(simulated_event_loop),
      event_(this),
      scheduler_(scheduler),
      token_(scheduler_->InvalidToken()) {}

SimulatedPhasedLoopHandler::~SimulatedPhasedLoopHandler() {
  if (token_ != scheduler_->InvalidToken()) {
    scheduler_->Deschedule(token_);
    token_ = scheduler_->InvalidToken();
  }
  simulated_event_loop_->RemoveEvent(&event_);
}

void SimulatedPhasedLoopHandler::HandleEvent() noexcept {
  monotonic_clock::time_point monotonic_now =
      simulated_event_loop_->monotonic_now();
  VLOG(1) << simulated_event_loop_->scheduler_->distributed_now() << " "
          << NodeName(simulated_event_loop_->node()) << monotonic_now << " "
          << simulated_event_loop_->name() << " Phased loop '" << name() << "'";
  logging::ScopedLogRestorer prev_logger;
  if (simulated_event_loop_->log_impl_) {
    prev_logger.Swap(simulated_event_loop_->log_impl_);
  }

  {
    ScopedMarkRealtimeRestorer rt(
        simulated_event_loop_->runtime_realtime_priority() > 0);
    Call([monotonic_now]() { return monotonic_now; });
    simulated_event_loop_->ClearContext();
  }
}

void SimulatedPhasedLoopHandler::Handle() noexcept {
  DCHECK(token_ != scheduler_->InvalidToken());
  token_ = scheduler_->InvalidToken();
  simulated_event_loop_->HandleEvent();
}

void SimulatedPhasedLoopHandler::Schedule(
    monotonic_clock::time_point sleep_time) {
  // The allocations in here are due to infrastructure and don't count in the no
  // mallocs in RT code.
  ScopedNotRealtime nrt;
  simulated_event_loop_->RemoveEvent(&event_);
  if (token_ != scheduler_->InvalidToken()) {
    scheduler_->Deschedule(token_);
    token_ = scheduler_->InvalidToken();
  }
  token_ = scheduler_->Schedule(sleep_time, this);
  event_.set_event_time(sleep_time);
  simulated_event_loop_->AddEvent(&event_);
}

SimulatedEventLoopFactory::SimulatedEventLoopFactory(
    const Configuration *configuration)
    : configuration_(configuration),
      nodes_(
          // Don't crash if configuration is nullptr, handle it a bit better
          // before doing anything.
          configuration == nullptr ? std::vector<const Node *>{}
                                   : configuration::GetNodes(configuration_)) {
  CHECK(configuration_ != nullptr);
  CHECK(IsInitialized()) << ": Need to initialize AOS first.";
  for (const Node *node : nodes_) {
    node_factories_.emplace_back(
        new NodeEventLoopFactory(&scheduler_scheduler_, this, node));
  }

  if (configuration::NodesCount(configuration) > 1u) {
    bridge_ = std::make_unique<message_bridge::SimulatedMessageBridge>(this);
  }
}

SimulatedEventLoopFactory::~SimulatedEventLoopFactory() {
  CHECK_EQ(0, exit_handle_count_)
      << ": All ExitHandles must be destroyed before the factory";
}

NodeEventLoopFactory *SimulatedEventLoopFactory::GetNodeEventLoopFactory(
    std::string_view node) {
  return GetNodeEventLoopFactory(configuration::GetNode(configuration(), node));
}

NodeEventLoopFactory *SimulatedEventLoopFactory::GetNodeEventLoopFactory(
    const Node *node) {
  auto result = std::find_if(
      node_factories_.begin(), node_factories_.end(),
      [node](const std::unique_ptr<NodeEventLoopFactory> &node_factory) {
        return node_factory->node() == node;
      });

  CHECK(result != node_factories_.end())
      << ": Failed to find node " << FlatbufferToJson(node);

  return result->get();
}

void SimulatedEventLoopFactory::SetTimeConverter(
    TimeConverter *time_converter) {
  for (std::unique_ptr<NodeEventLoopFactory> &factory : node_factories_) {
    factory->SetTimeConverter(time_converter);
  }
  scheduler_scheduler_.SetTimeConverter(time_converter);
}

::std::unique_ptr<EventLoop> SimulatedEventLoopFactory::MakeEventLoop(
    std::string_view name, const Node *node) {
  if (node == nullptr) {
    CHECK(!configuration::MultiNode(configuration()))
        << ": Can't make a single node event loop in a multi-node world.";
  } else {
    CHECK(configuration::MultiNode(configuration()))
        << ": Can't make a multi-node event loop in a single-node world.";
  }
  return GetNodeEventLoopFactory(node)->MakeEventLoop(name);
}

NodeEventLoopFactory::NodeEventLoopFactory(
    EventSchedulerScheduler *scheduler_scheduler,
    SimulatedEventLoopFactory *factory, const Node *node)
    : scheduler_(configuration::GetNodeIndex(factory->configuration(), node)),
      factory_(factory),
      node_(node) {
  scheduler_scheduler->AddEventScheduler(&scheduler_);
  scheduler_.set_started([this]() {
    started_ = true;
    for (SimulatedEventLoop *event_loop : event_loops_) {
      event_loop->SetIsRunning(true);
    }
  });
  scheduler_.set_stopped([this]() {
    for (SimulatedEventLoop *event_loop : event_loops_) {
      event_loop->SetIsRunning(false);
    }
  });
  scheduler_.set_on_shutdown([this]() {
    VLOG(1) << scheduler_.distributed_now() << " " << NodeName(this->node())
            << monotonic_now() << " Shutting down node.";
    Shutdown();
    ScheduleStartup();
  });
  ScheduleStartup();
}

NodeEventLoopFactory::~NodeEventLoopFactory() {
  if (started_) {
    for (std::function<void()> &fn : on_shutdown_) {
      fn();
    }

    VLOG(1) << scheduler_.distributed_now() << " " << NodeName(node())
            << monotonic_now() << " Shutting down applications.";
    applications_.clear();
    started_ = false;
  }

  if (event_loops_.size() != 0u) {
    for (SimulatedEventLoop *event_loop : event_loops_) {
      LOG(ERROR) << scheduler_.distributed_now() << " " << NodeName(node())
                 << monotonic_now() << " Event loop '" << event_loop->name()
                 << "' failed to shut down";
    }
  }
  CHECK_EQ(event_loops_.size(), 0u) << "Event loop didn't exit";
}

void NodeEventLoopFactory::OnStartup(std::function<void()> &&fn) {
  CHECK(!scheduler_.is_running())
      << ": Can only register OnStartup handlers when not running.";
  on_startup_.emplace_back(std::move(fn));
  if (started_) {
    size_t on_startup_index = on_startup_.size() - 1;
    scheduler_.ScheduleOnStartup(
        [this, on_startup_index]() { on_startup_[on_startup_index](); });
  }
}

void NodeEventLoopFactory::OnShutdown(std::function<void()> &&fn) {
  on_shutdown_.emplace_back(std::move(fn));
}

void NodeEventLoopFactory::ScheduleStartup() {
  scheduler_.ScheduleOnStartup([this]() {
    UUID next_uuid = scheduler_.boot_uuid();
    if (boot_uuid_ != next_uuid) {
      CHECK_EQ(boot_uuid_, UUID::Zero())
          << ": Boot UUID changed without restarting.  Did TimeConverter "
             "change the boot UUID without signaling a restart, or did you "
             "change TimeConverter?";
      boot_uuid_ = next_uuid;
    }
    VLOG(1) << scheduler_.distributed_now() << " " << NodeName(this->node())
            << monotonic_now() << " Starting up node on boot " << boot_uuid_;
    Startup();
  });
}

void NodeEventLoopFactory::Startup() {
  CHECK(!started_);
  for (size_t i = 0; i < on_startup_.size(); ++i) {
    on_startup_[i]();
  }
}

void NodeEventLoopFactory::Shutdown() {
  for (SimulatedEventLoop *event_loop : event_loops_) {
    CHECK(!event_loop->is_running());
  }

  CHECK(started_);
  started_ = false;
  for (std::function<void()> &fn : on_shutdown_) {
    fn();
  }

  VLOG(1) << scheduler_.distributed_now() << " " << NodeName(node())
          << monotonic_now() << " Shutting down applications.";
  applications_.clear();

  if (event_loops_.size() != 0u) {
    for (SimulatedEventLoop *event_loop : event_loops_) {
      LOG(ERROR) << scheduler_.distributed_now() << " " << NodeName(node())
                 << monotonic_now() << " Event loop '" << event_loop->name()
                 << "' failed to shut down";
    }
  }
  CHECK_EQ(event_loops_.size(), 0u) << "Not all event loops shut down";
  boot_uuid_ = UUID::Zero();

  channels_.clear();
}

Result<void> SimulatedEventLoopFactory::GetAndClearExitStatus() {
  std::optional<Result<void>> exit_status;
  // Clear the stored exit_status_ and extract it to be returned.
  exit_status_.swap(exit_status);
  return exit_status.value_or(Result<void>{});
}

void SimulatedEventLoopFactory::RunFor(monotonic_clock::duration duration) {
  CheckExpected(NonFatalRunFor(duration));
}

Result<void> SimulatedEventLoopFactory::NonFatalRunFor(
    monotonic_clock::duration duration) {
  // This sets running to true too.
  const Result<void> result = scheduler_scheduler_.RunFor(duration);
  for (std::unique_ptr<NodeEventLoopFactory> &node : node_factories_) {
    if (node) {
      for (SimulatedEventLoop *loop : node->event_loops_) {
        CHECK(!loop->is_running());
      }
    }
  }
  return result.and_then([this]() { return GetAndClearExitStatus(); });
}

SimulatedEventLoopFactory::RunEndState SimulatedEventLoopFactory::RunUntil(
    realtime_clock::time_point now, const aos::Node *node) {
  return CheckExpected(NonFatalRunUntil(now, node));
}

Result<SimulatedEventLoopFactory::RunEndState>
SimulatedEventLoopFactory::NonFatalRunUntil(realtime_clock::time_point now,
                                            const aos::Node *node) {
  const Result<RunEndState> result =
      scheduler_scheduler_
          .RunUntil(now, &GetNodeEventLoopFactory(node)->scheduler_,
                    [this, &node](void) {
                      return GetNodeEventLoopFactory(node)->realtime_offset();
                    })
          .transform([](const bool events_remaining) {
            return events_remaining ? RunEndState::kEventsRemaining
                                    : RunEndState::kFinishedEventProcessing;
          });
  for (std::unique_ptr<NodeEventLoopFactory> &node : node_factories_) {
    if (node) {
      for (SimulatedEventLoop *loop : node->event_loops_) {
        CHECK(!loop->is_running());
      }
    }
  }
  return result.and_then([this](const RunEndState end_state) {
    return GetAndClearExitStatus().transform(
        [end_state]() { return end_state; });
  });
}

void SimulatedEventLoopFactory::Run() { CheckExpected(NonFatalRun()); }

Result<void> SimulatedEventLoopFactory::NonFatalRun() {
  // This sets running to true too.
  const Result<void> result = scheduler_scheduler_.Run();
  for (std::unique_ptr<NodeEventLoopFactory> &node : node_factories_) {
    if (node) {
      CHECK(!node->is_running());
      for (SimulatedEventLoop *loop : node->event_loops_) {
        CHECK(!loop->is_running());
      }
    }
  }
  return result.and_then([this]() { return GetAndClearExitStatus(); });
}

void SimulatedEventLoopFactory::Exit(Result<void> status) {
  if (!exit_status_.has_value()) {
    exit_status_ = std::move(status);
  } else {
    VLOG(1) << "Exit status is already set; not setting it again.";
  }
  scheduler_scheduler_.Exit();
}

std::unique_ptr<ExitHandle> SimulatedEventLoopFactory::MakeExitHandle() {
  return std::make_unique<SimulatedFactoryExitHandle>(this);
}

void SimulatedEventLoopFactory::DisableForwarding(const Channel *channel) {
  CHECK(bridge_) << ": Can't disable forwarding without a message bridge.";
  bridge_->DisableForwarding(channel);
}

void SimulatedEventLoopFactory::DisableStatistics() {
  CHECK(bridge_) << ": Can't disable statistics without a message bridge.";
  bridge_->DisableStatistics(
      message_bridge::SimulatedMessageBridge::DestroySenders::kNo);
}

void SimulatedEventLoopFactory::PermanentlyDisableStatistics() {
  CHECK(bridge_) << ": Can't disable statistics without a message bridge.";
  bridge_->DisableStatistics(
      message_bridge::SimulatedMessageBridge::DestroySenders::kYes);
}

void SimulatedEventLoopFactory::EnableStatistics() {
  CHECK(bridge_) << ": Can't enable statistics without a message bridge.";
  bridge_->EnableStatistics();
}

void SimulatedEventLoopFactory::SkipTimingReport() {
  CHECK(bridge_) << ": Can't skip timing reports without a message bridge.";

  for (std::unique_ptr<NodeEventLoopFactory> &node : node_factories_) {
    if (node) {
      node->SkipTimingReport();
    }
  }
}

void NodeEventLoopFactory::SkipTimingReport() {
  for (SimulatedEventLoop *event_loop : event_loops_) {
    event_loop->SkipTimingReport();
  }
  skip_timing_report_ = true;
}

void NodeEventLoopFactory::EnableStatistics() {
  CHECK(factory_->bridge_)
      << ": Can't enable statistics without a message bridge.";
  factory_->bridge_->EnableStatistics(node_);
}

void NodeEventLoopFactory::DisableStatistics() {
  CHECK(factory_->bridge_)
      << ": Can't disable statistics without a message bridge.";
  factory_->bridge_->DisableStatistics(node_);
}

::std::unique_ptr<EventLoop> NodeEventLoopFactory::MakeEventLoop(
    std::string_view name, EventLoopOptions options) {
  CHECK(!scheduler_.is_running() || !started_)
      << ": Can't create an event loop while running";

  pid_t tid = tid_;
  ++tid_;
  ::std::unique_ptr<SimulatedEventLoop> result(new SimulatedEventLoop(
      &scheduler_, this, &channels_, factory_->configuration(), &event_loops_,
      node_, tid, options));
  result->set_name(name);
  result->set_send_delay(factory_->send_delay());
  if (skip_timing_report_) {
    result->SkipTimingReport();
  }

  // TODO(austin): You shouldn't be able to make an event loop before t=0...

  VLOG(1) << scheduler_.distributed_now() << " " << NodeName(node())
          << monotonic_now() << " MakeEventLoop(\"" << result->name() << "\")";
  return result;
}

void SimulatedEventLoopFactory::AllowApplicationCreationDuring(
    std::function<void()> fn) {
  scheduler_scheduler_.TemporarilyStopAndRun(std::move(fn));
}

void NodeEventLoopFactory::Disconnect(const Node *other) {
  factory_->bridge_->Disconnect(node_, other);
}

void NodeEventLoopFactory::Connect(const Node *other) {
  factory_->bridge_->Connect(node_, other);
}

}  // namespace aos
