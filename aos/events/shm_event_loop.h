#ifndef AOS_EVENTS_SHM_EVENT_LOOP_H_
#define AOS_EVENTS_SHM_EVENT_LOOP_H_

#include <vector>

#include "absl/types/span.h"

#include "aos/events/epoll.h"
#include "aos/events/event_loop.h"
#include "aos/events/event_loop_generated.h"
#include "aos/ipc_lib/shm_base.h"
#include "aos/ipc_lib/signalfd.h"
#include "aos/stl_mutex/stl_mutex.h"

ABSL_DECLARE_FLAG(std::string, application_name);

namespace aos {
namespace shm_event_loop_internal {

class ShmWatcherState;
class ShmTimerHandler;
class ShmPhasedLoopHandler;
class ShmSender;
class SimpleShmFetcher;
class ShmFetcher;
class ShmExitHandle;

}  // namespace shm_event_loop_internal

// Concrete implementation of EventLoop that is built from queues running out of
// shared memory.
//
// TODO(austin): Timing reports break multiple threads.  Need to add back in a
// mutex.
// This object must be interacted with from one thread, but the Senders
// and Fetchers may be used from multiple threads afterwords (as long as their
// destructors are called back in one thread again)
class ShmEventLoop : public EventLoop {
 public:
  ShmEventLoop(const Flatbuffer<Configuration> &configuration)
      : ShmEventLoop(&configuration.message()) {}
  ShmEventLoop(const Configuration *configuration);
  ShmEventLoop(const ShmEventLoop &) = delete;
  ~ShmEventLoop() override;

  void operator=(ShmEventLoop const &) = delete;

  // Runs the event loop until Exit is called, or ^C is caught.
  // TODO(james): Upgrade this to [[nodiscard]].
  Result<void> Run();
  // Exits the event loop.  async-signal-safe (see
  // https://man7.org/linux/man-pages/man7/signal-safety.7.html).
  // Will result in Run() returning a successful result when called.
  void Exit();

  // Exits the event loop with the provided status. Thread-safe, but not
  // async-safe.
  void ExitWithStatus(Result<void> status = {});

  // Constructs an exit handle for the EventLoop. The provided ExitHandle uses
  // ExitWithStatus().
  std::unique_ptr<ExitHandle> MakeExitHandle();

  aos::monotonic_clock::time_point monotonic_now() const override {
    return aos::monotonic_clock::now();
  }
  aos::realtime_clock::time_point realtime_now() const override {
    return aos::realtime_clock::now();
  }

  std::unique_ptr<RawSender> MakeRawSender(const Channel *channel) override;
  std::unique_ptr<RawFetcher> MakeRawFetcher(const Channel *channel) override;

  void MakeRawWatcher(
      const Channel *channel,
      std::function<void(const Context &context, const void *message)> watcher)
      override;
  void MakeRawNoArgWatcher(
      const Channel *channel,
      std::function<void(const Context &context)> watcher) override;

  TimerHandler *AddTimer(std::function<void()> callback) override;
  PhasedLoopHandler *AddPhasedLoop(std::function<void(int)> callback,
                                   const monotonic_clock::duration interval,
                                   const monotonic_clock::duration offset =
                                       std::chrono::seconds(0)) override;

  void OnRun(std::function<void()> on_run) override;

  void SetRuntimeRealtimePriority(int priority) override;
  void SetRuntimeAffinity(const cpu_set_t &cpuset) override;

  void set_name(const std::string_view name) override;
  const std::string_view name() const override { return name_; }
  const Node *node() const override { return node_; }

  int runtime_realtime_priority() const override { return priority_; }
  const cpu_set_t &runtime_affinity() const override { return affinity_; }
  const UUID &boot_uuid() const override { return boot_uuid_; }

  // Returns the epoll loop used to run the event loop.
  internal::EPoll *epoll() { return &epoll_; }

  // Returns the local mapping of the shared memory used by the watcher on the
  // specified channel. A watcher must be created on this channel before calling
  // this.
  absl::Span<char> GetWatcherSharedMemory(const Channel *channel);

  // Setting "use writeable memory" to false (the default) means that the
  // watcher will provide messages in a read-only memory region. Setting "use
  // writeable memory" to true means that the watcher will provide messages in a
  // writeable memory. Only use this if you absolutely know what you're doing.
  // You should only use this if you're interacting with something like CUDA
  // which expects writeable memory in its API. Note that regardless of this
  // setting, the API for watcher doesn't change. The messages will still be
  // `const`.
  void SetWatcherUseWritableMemory(const Channel *channel,
                                   bool use_writable_memory);

  // Returns the local mapping of the shared memory used by the provided Sender.
  template <typename T>
  absl::Span<char> GetSenderSharedMemory(aos::Sender<T> *sender) const {
    CheckCurrentThread();
    return GetShmSenderSharedMemory(GetRawSender(sender));
  }

  // Returns the local mapping of the private memory used by the provided
  // Fetcher to hold messages.
  //
  // Note that this may be the entire shared memory region held by this fetcher,
  // depending on its channel's read_method.
  template <typename T>
  absl::Span<const char> GetFetcherPrivateMemory(
      aos::Fetcher<T> *fetcher) const {
    CheckCurrentThread();
    return GetShmFetcherPrivateMemory(GetRawFetcher(fetcher));
  }

  // Returns the local mapping of the shared memory used by the provided
  // Fetcher to hold messages.
  //
  // Note that this may be the entire shared memory region held by this fetcher,
  // depending on its channel's read_method.
  //
  // Only use this if you really know what you're doing. See the docs for
  // SetFetcherUseWritableMemory() for more information.
  template <typename T>
  absl::Span<char> GetFetcherSharedMemory(aos::Fetcher<T> *fetcher) const {
    CheckCurrentThread();
    return GetShmFetcherSharedMemory(GetRawFetcher(fetcher));
  }

  // Setting "use writeable memory" to false (the default) means that the
  // fetcher will provide messages in a read-only memory region. Setting "use
  // writeable memory" to true means that the fetcher will provide messages in a
  // writeable memory. Only use this if you absolutely know what you're doing.
  // You should only use this if you're interacting with something like CUDA
  // which expects writeable memory in its API. Note that regardless of this
  // setting, the API for fetchers doesn't change. The messages will still be
  // `const`.
  template <typename T>
  void SetFetcherUseWritableMemory(aos::Fetcher<T> *fetcher,
                                   bool use_writable_memory) const {
    CheckCurrentThread();
    SetShmFetcherUseWritableMemory(GetRawFetcher(fetcher), use_writable_memory);
  }

  int NumberBuffers(const Channel *channel) override;

  // All public-facing APIs will verify this mutex is held when they are called.
  // For normal use with everything in a single thread, this is unnecessary.
  //
  // This is helpful as a safety check when using a ShmEventLoop with external
  // synchronization across multiple threads. It will NOT reliably catch race
  // conditions, but if you have a race condition triggered repeatedly it'll
  // probably catch it eventually.
  void CheckForMutex(aos::stl_mutex *check_mutex) {
    check_mutex_ = check_mutex;
  }

  // All public-facing APIs will verify they are called in this thread.
  // For normal use with the whole program in a single thread, this is
  // unnecessary. It's helpful as a safety check for programs with multiple
  // threads, where the EventLoop should only be interacted with from a single
  // one.
  void LockToThread() { check_tid_ = GetTid(); }

 private:
  friend class shm_event_loop_internal::ShmWatcherState;
  friend class shm_event_loop_internal::ShmTimerHandler;
  friend class shm_event_loop_internal::ShmPhasedLoopHandler;
  friend class shm_event_loop_internal::ShmSender;
  friend class shm_event_loop_internal::SimpleShmFetcher;
  friend class shm_event_loop_internal::ShmFetcher;
  friend class shm_event_loop_internal::ShmExitHandle;

  using EventLoop::SendTimingReport;

  void CheckCurrentThread() const;

  void HandleEvent();

  // Returns the TID of the event loop.
  pid_t GetTid() override;

  // Private method to access the shared memory mapping of a ShmSender.
  absl::Span<char> GetShmSenderSharedMemory(const aos::RawSender *sender) const;

  // Private method to access the private memory mapping of a ShmFetcher.
  absl::Span<const char> GetShmFetcherPrivateMemory(
      const aos::RawFetcher *fetcher) const;

  // Private method to access the shared memory mapping of a ShmFetcher.
  absl::Span<char> GetShmFetcherSharedMemory(
      const aos::RawFetcher *fetcher) const;

  void SetShmFetcherUseWritableMemory(aos::RawFetcher *fetcher,
                                      bool use_writable_memory) const;

  const UUID boot_uuid_;

  int exit_handle_count_ = 0;

  // Capture the --shm_base flag at construction time.  This makes it much
  // easier to make different shared memory regions for doing things like
  // multi-node tests.
  std::string shm_base_;

  std::vector<std::function<void()>> on_run_;
  int priority_ = 0;
  cpu_set_t affinity_ = DefaultAffinity();
  std::string name_;
  const Node *const node_;

  aos::stl_mutex *check_mutex_ = nullptr;
  std::optional<pid_t> check_tid_;

  internal::EPoll epoll_;

  // Only set during Run().
  std::unique_ptr<ipc_lib::SignalFd> signalfd_;

  // Calls to Exit() are guaranteed to be thread-safe, so the exit_status_mutex_
  // guards access to the exit_status_.
  aos::stl_mutex exit_status_mutex_;
  // Once exit_status_ is set once, we will not set it again until we have
  // actually exited. This is to try to provide consistent behavior in cases
  // where Exit() is called multiple times before Run() is aactually terminates
  // execution.
  std::optional<Result<void>> exit_status_{};
  // Used by the Exit() call to provide an async-safe way of indicating that
  // Exit() was called.
  // Will be set once Exit() or ExitWithStatus() has been called.
  // Note: std::atomic<> is not necessarily guaranteed to be lock-free, although
  // std::atomic_flag is, and so is safe to use in Exit().
  std::atomic_flag observed_exit_ = ATOMIC_FLAG_INIT;
};

}  // namespace aos

#endif  // AOS_EVENTS_SHM_EVENT_LOOP_H_
