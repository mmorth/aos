#ifndef AOS_STL_MUTEX_H_
#define AOS_STL_MUTEX_H_

#include <mutex>
#include <ostream>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

#include "aos/ipc_lib/aos_sync.h"
#include "aos/macros.h"

namespace aos {

// A mutex with the same API and semantics as ::std::mutex, with the addition of
// methods for checking if the previous owner died and a constexpr default
// constructor.
// Definitely safe to put in SHM.
// This uses the pthread_mutex semantics for owner-died: once somebody dies with
// the lock held, anybody else who takes it will see true for owner_died() until
// one of them calls consistent(). It is an error to call unlock() when
// owner_died() returns true.
class stl_mutex {
 public:
  constexpr stl_mutex() : native_handle_() {}

  void lock() {
    const int ret = mutex_grab(&native_handle_);
    switch (ret) {
      case 0:
        break;
      case 1:
        owner_died_ = true;
        break;
      default:
        ABSL_LOG(FATAL) << "mutex_grab(" << &native_handle_
                        << ") failed: " << ret;
    }
  }

  bool try_lock() {
    const int ret = mutex_trylock(&native_handle_);
    switch (ret) {
      case 0:
        return true;
      case 1:
        owner_died_ = true;
        return true;
      case 4:
        return false;
      default:
        ABSL_LOG(FATAL) << "mutex_trylock(" << &native_handle_
                        << ") failed: " << ret;
    }
  }

  void unlock() {
    ABSL_CHECK(!owner_died_);
    mutex_unlock(&native_handle_);
  }

  typedef aos_mutex *native_handle_type;
  native_handle_type native_handle() { return &native_handle_; }

  bool owner_died() const { return owner_died_; }
  void consistent() { owner_died_ = false; }

  // Returns whether this mutex is locked by the current thread. This is very
  // hard to use reliably, please think very carefully before using it for
  // anything beyond probabilistic assertion checks.
  bool is_locked() const { return mutex_islocked(&native_handle_); }

 private:
  aos_mutex native_handle_;

  bool owner_died_ = false;

  DISALLOW_COPY_AND_ASSIGN(stl_mutex);
};

// A mutex with the same API and semantics as ::std::recursive_mutex, with the
// addition of methods for checking if the previous owner died and a constexpr
// default constructor.
// Definitely safe to put in SHM.
// This uses the pthread_mutex semantics for owner-died: once somebody dies with
// the lock held, anybody else who takes it will see true for owner_died() until
// one of them calls consistent(). It is an error to call unlock() or lock()
// again when owner_died() returns true.
class stl_recursive_mutex {
 public:
  constexpr stl_recursive_mutex() {}

  void lock() {
    if (mutex_.is_locked()) {
      ABSL_CHECK(!owner_died());
      ++recursive_locks_;
    } else {
      mutex_.lock();
      if (mutex_.owner_died()) {
        recursive_locks_ = 0;
      } else {
        ABSL_CHECK_EQ(0, recursive_locks_);
      }
    }
  }
  bool try_lock() {
    if (mutex_.is_locked()) {
      ABSL_CHECK(!owner_died());
      ++recursive_locks_;
      return true;
    } else {
      if (mutex_.try_lock()) {
        if (mutex_.owner_died()) {
          recursive_locks_ = 0;
        } else {
          ABSL_CHECK_EQ(0, recursive_locks_);
        }
        return true;
      } else {
        return false;
      }
    }
  }
  void unlock() {
    if (recursive_locks_ == 0) {
      mutex_.unlock();
    } else {
      --recursive_locks_;
    }
  }

  typedef stl_mutex::native_handle_type native_handle_type;
  native_handle_type native_handle() { return mutex_.native_handle(); }

  bool owner_died() const { return mutex_.owner_died(); }
  void consistent() { mutex_.consistent(); }

 private:
  stl_mutex mutex_;
  int recursive_locks_ = 0;

  DISALLOW_COPY_AND_ASSIGN(stl_recursive_mutex);
};

// Convenient typedefs for various types of locking objects.
typedef ::std::lock_guard<stl_mutex> mutex_lock_guard;
typedef ::std::lock_guard<stl_recursive_mutex> recursive_lock_guard;
typedef ::std::unique_lock<stl_mutex> mutex_unique_lock;
typedef ::std::unique_lock<stl_recursive_mutex> recursive_unique_lock;

}  // namespace aos

#endif  // AOS_STL_MUTEX_H_
