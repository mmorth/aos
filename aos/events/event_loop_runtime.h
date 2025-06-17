#ifndef AOS_EVENTS_EVENT_LOOP_RUNTIME_H_
#define AOS_EVENTS_EVENT_LOOP_RUNTIME_H_

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

// Exposes the primitives to implement an async Rust runtime on top of an
// EventLoop. This is not intended to be used directly, so the APIs are not
// particularly ergonomic for C++. See the Rust wrapper for detailed
// documentation.

#include <chrono>
#include <memory>
#include <optional>

#include "aos/events/event_loop.h"
#include "aos/for_rust.h"
#include "cxx.h"

namespace aos {

// An alternative version of Context to feed autocxx, to work around
// https://github.com/google/autocxx/issues/787.
/// <div rustbindgen replaces="aos::Context"></div>
struct RustContext {
  int64_t monotonic_event_time;
  int64_t realtime_event_time;

  int64_t monotonic_remote_time;
  int64_t realtime_remote_time;
  int64_t monotonic_remote_transmit_time;

  uint32_t queue_index;
  uint32_t remote_queue_index;

  size_t size;
  const void *data;

  int buffer_index;

  // Work around https://github.com/google/autocxx/issues/266.
  uint8_t source_boot_uuid[16];
};

static_assert(sizeof(Context) == sizeof(RustContext));
static_assert(alignof(Context) == alignof(RustContext));
static_assert(offsetof(Context, monotonic_event_time) ==
              offsetof(RustContext, monotonic_event_time));
static_assert(offsetof(Context, realtime_event_time) ==
              offsetof(RustContext, realtime_event_time));
static_assert(offsetof(Context, monotonic_remote_time) ==
              offsetof(RustContext, monotonic_remote_time));
static_assert(offsetof(Context, realtime_remote_time) ==
              offsetof(RustContext, realtime_remote_time));
static_assert(offsetof(Context, monotonic_remote_transmit_time) ==
              offsetof(RustContext, monotonic_remote_transmit_time));
static_assert(offsetof(Context, queue_index) ==
              offsetof(RustContext, queue_index));
static_assert(offsetof(Context, remote_queue_index) ==
              offsetof(RustContext, remote_queue_index));
static_assert(offsetof(Context, size) == offsetof(RustContext, size));
static_assert(offsetof(Context, data) == offsetof(RustContext, data));
static_assert(offsetof(Context, buffer_index) ==
              offsetof(RustContext, buffer_index));
static_assert(offsetof(Context, source_boot_uuid) ==
              offsetof(RustContext, source_boot_uuid));
static_assert(sizeof(Context::source_boot_uuid) ==
              sizeof(RustContext::source_boot_uuid));
static_assert(sizeof(RustContext) == sizeof(Context),
              "Update this when adding or removing fields");

// Similar to Rust's `Future<Output = Never>`.
class ApplicationFuture {
 public:
  ApplicationFuture() = default;
  virtual ~ApplicationFuture() = default;

  // Calls a Rust `Future::poll`, with a waker that will panic if used. Because
  // our Future's Output is Never, the inner Rust implementation can only return
  // Poll::Pending, which is equivalent to void.
  //
  // Returns true if it succeeded, or false if the Rust code paniced.
  virtual bool Poll() = 0;
};

// Similar to Rust's `Stream<Item = const Option&>`.
class WatcherForRust {
 public:
  WatcherForRust(std::unique_ptr<RawFetcher> fetcher)
      : fetcher_(std::move(fetcher)) {}
  ~WatcherForRust() = default;

  const Context *PollNext() {
    if (!fetcher_->FetchNext()) {
      return nullptr;
    }
    return &fetcher_->context();
  }

 private:
  const std::unique_ptr<RawFetcher> fetcher_;
};

class SenderForRust {
 public:
  SenderForRust(std::unique_ptr<RawSender> sender)
      : sender_(std::move(sender)) {}
  ~SenderForRust() = default;

  uint8_t *data() { return reinterpret_cast<uint8_t *>(sender_->data()); }
  size_t size() { return sender_->size(); }
  RawSender::Error SendBuffer(size_t size) { return sender_->Send(size); }
  RawSender::Error CopyAndSend(const uint8_t *data, size_t size) {
    return sender_->Send(data, size);
  }

 private:
  const std::unique_ptr<RawSender> sender_;
};

class FetcherForRust {
 public:
  FetcherForRust(std::unique_ptr<RawFetcher> fetcher)
      : fetcher_(std::move(fetcher)) {}
  ~FetcherForRust() = default;

  bool FetchNext() { return fetcher_->FetchNext(); }
  bool Fetch() { return fetcher_->Fetch(); }

  const Context &context() const { return fetcher_->context(); }

 private:
  const std::unique_ptr<RawFetcher> fetcher_;
};

class EventLoopRuntime;

class OnRunForRust {
 public:
  OnRunForRust(const EventLoopRuntime *runtime);
  ~OnRunForRust();

  bool is_running() const;

 private:
  const EventLoopRuntime *const runtime_;
};

class TimerForRust {
 public:
  static std::unique_ptr<TimerForRust> Make(const EventLoopRuntime *runtime);

  TimerForRust(const TimerForRust &) = delete;
  TimerForRust(TimerForRust &&) = delete;

  TimerForRust &operator=(const TimerForRust &) = delete;
  TimerForRust &operator=(TimerForRust &&) = delete;

  ~TimerForRust() { timer_->Disable(); }

  void Schedule(int64_t base, int64_t repeat_offset) {
    timer_->Schedule(
        monotonic_clock::time_point(std::chrono::nanoseconds(base)),
        std::chrono::nanoseconds(repeat_offset));
  }

  void Disable() { timer_->Disable(); }

  bool IsDisabled() const { return timer_->IsDisabled(); }

  void set_name(rust::Str name) { timer_->set_name(RustStrToStringView(name)); }
  rust::Str name() const { return StringViewToRustStr(timer_->name()); }

  // If true, the timer is expired.
  bool Poll();

 private:
  TimerForRust() = default;

  TimerHandler *timer_;
  bool expired_ = false;
};

class EventLoopRuntime {
 public:
  EventLoopRuntime(const EventLoop *event_loop)
      // SAFETY: A &EventLoop in Rust becomes a const EventLoop*. While
      // that's generally a reasonable convention, they are semantically
      // different. In Rust, a &mut EventLoop is very restrictive as it enforces
      // uniqueness of the reference. Additionally, a &EventLoop doesn't convey
      // const-ness in the C++ sense. So to make the FFI boundary more
      // ergonomic, we allow a &EventLoop passed from rust to be translated into
      // an EventLoop* in C++. This is safe so long as EventLoop is !Sync and no
      // &mut EventLoop references are constructed in Rust.
      : event_loop_(const_cast<EventLoop *>(event_loop)) {}
  ~EventLoopRuntime() {
    // Do this first, because it may hold child objects.
    task_.reset();
    ABSL_CHECK_EQ(child_count_, 0)
        << ": Some child objects were not destroyed first";
  }

  EventLoop *event_loop() const { return event_loop_; }

  void Spawn(std::unique_ptr<ApplicationFuture> task) const {
    ABSL_CHECK(!task_) << ": May only call Spawn once";
    task_ = std::move(task);
    DoPoll();
    // Just do this unconditionally, so we don't have to keep track of each
    // OnRun to only do it once. If Rust doesn't use OnRun, it's harmless to do
    // an extra poll.
    event_loop_->OnRun([this] { DoPoll(); });
  }

  const Configuration *configuration() const {
    return event_loop_->configuration();
  }
  const Node *node() const { return event_loop_->node(); }

  bool is_running() const { return event_loop_->is_running(); }

  // autocxx generates broken C++ code for `time_point`, see
  // https://github.com/google/autocxx/issues/787.
  int64_t monotonic_now() const {
    return std::chrono::nanoseconds(
               event_loop_->monotonic_now().time_since_epoch())
        .count();
  }
  int64_t realtime_now() const {
    return std::chrono::nanoseconds(
               event_loop_->realtime_now().time_since_epoch())
        .count();
  }

  rust::Str name() const { return StringViewToRustStr(event_loop_->name()); }

  WatcherForRust MakeWatcher(const Channel *channel) const {
    event_loop_->MakeRawNoArgWatcher(channel,
                                     [this](const Context &) { DoPoll(); });
    return WatcherForRust(event_loop_->MakeRawFetcher(channel));
  }

  SenderForRust MakeSender(const Channel *channel) const {
    return SenderForRust(event_loop_->MakeRawSender(channel));
  }

  FetcherForRust MakeFetcher(const Channel *channel) const {
    return FetcherForRust(event_loop_->MakeRawFetcher(channel));
  }

  OnRunForRust MakeOnRun() const { return OnRunForRust(this); }

  std::unique_ptr<TimerForRust> AddTimer() const {
    return TimerForRust::Make(this);
  }

  void SetRuntimeRealtimePriority(int priority) const {
    event_loop_->SetRuntimeRealtimePriority(priority);
  }

  void SetRuntimeAffinity(const cpu_set_t &cpuset) const {
    event_loop_->SetRuntimeAffinity(cpuset);
  }

 private:
  friend class OnRunForRust;
  friend class TimerForRust;

  // Polls the top-level future once. This is what all the callbacks should do.
  void DoPoll() const {
    if (task_) {
      ABSL_CHECK(task_->Poll()) << ": Rust panic, aborting";
    }
  }

  EventLoop *const event_loop_;

  // For Rust's EventLoopRuntime to be semantically equivelant to C++'s event
  // loop, we need the ability to have shared references (&EventLoopRuntime) on
  // the Rust side. Without that, the API would be overly restrictive to be
  // usable. In order for the generated code to use &self references on methods,
  // they need to be marked `const` on the C++ side. We use the `mutable`
  // keyword to allow mutation through `const` methods.
  //
  // SAFETY:
  //   * The event loop runtime must be `!Sync` in the Rust side (default).
  //   * We can't expose exclusive references (&mut) to either of the mutable
  //     fields on the Rust side from a shared reference (&).
  mutable std::unique_ptr<ApplicationFuture> task_;

  mutable int child_count_ = 0;
};

}  // namespace aos

#endif  // AOS_EVENTS_EVENT_LOOP_RUNTIME_H_
