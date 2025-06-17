#include "aos/ipc_lib/memory_mapped_queue.h"

#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <chrono>
#include <limits>
#include <ostream>
#include <thread>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "absl/strings/str_cat.h"
#include "flatbuffers/string.h"

#include "aos/ipc_lib/index.h"
#include "aos/util/file.h"

namespace aos::ipc_lib {

std::string ShmFolder(std::string_view shm_base, const Channel *channel) {
  ABSL_CHECK(channel->has_name());
  ABSL_CHECK_EQ(channel->name()->string_view()[0], '/');
  return absl::StrCat(shm_base, channel->name()->string_view(), "/");
}

std::string ShmPath(std::string_view shm_base, const Channel *channel) {
  ABSL_CHECK(channel->has_type());
  return ShmFolder(shm_base, channel) + channel->type()->str() + ".v7";
}

void PageFaultDataWrite(char *data, size_t size, const long page_size) {
  // This just has to divide the actual page size. Being smaller will make this
  // a bit slower than necessary, but not much. 1024 is a pretty conservative
  // choice (most pages are probably 4096).
  const size_t pages = (size + page_size - 1) / page_size;
  for (size_t i = 0; i < pages; ++i) {
    char zero = 0;
    // We need to ensure there's a writable pagetable entry, but avoid modifying
    // the data.
    //
    // Even if you lock the data into memory, some kernels still seem to lazily
    // create the actual pagetable entries. This means we need to somehow
    // "write" to the page.
    //
    // Also, this takes place while other processes may be concurrently
    // opening/initializing the memory, so we need to avoid corrupting that.
    //
    // This is the simplest operation I could think of which achieves that:
    // "store 0 if it's already 0".
    __atomic_compare_exchange_n(&data[i * page_size], &zero, 0, true,
                                __ATOMIC_RELAXED, __ATOMIC_RELAXED);
  }
}

void PageFaultDataRead(const char *data, size_t size, const long page_size) {
  const size_t pages = (size + page_size - 1) / page_size;
  for (size_t i = 0; i < pages; ++i) {
    // We need to ensure there's a readable pagetable entry.
    __atomic_load_n(&data[i * page_size], __ATOMIC_RELAXED);
  }
}

LocklessQueueConfiguration MakeQueueConfiguration(
    const Configuration *configuration, const Channel *channel) {
  LocklessQueueConfiguration config;

  config.num_watchers = channel->num_watchers();
  config.num_senders = channel->num_senders();
  // The value in the channel will default to 0 if readers are configured to
  // copy.
  config.num_pinners = channel->num_readers();
  config.queue_size = configuration::QueueSize(configuration, channel);
  ABSL_CHECK_LT(config.queue_size,
                std::numeric_limits<QueueIndex::PackedIndexType>::max())
      << ": More messages/second configured than the queue can hold on "
      << configuration::CleanedChannelToString(channel) << ", "
      << channel->frequency() << "hz for "
      << std::chrono::duration<double>(
             configuration::ChannelStorageDuration(configuration, channel))
             .count()
      << "sec";
  config.message_data_size = channel->max_size();

  return config;
}

MemoryMappedQueue::MemoryMappedQueue(std::string_view shm_base,
                                     uint32_t permissions,
                                     const Configuration *config,
                                     const Channel *channel)
    : config_(MakeQueueConfiguration(config, channel)) {
  const long kSystemPageSize = sysconf(_SC_PAGESIZE);
  std::string path = ShmPath(shm_base, channel);

  size_ = ipc_lib::LocklessQueueMemorySize(config_);

  util::MkdirP(path, permissions);

  // There are 2 cases.  Either the file already exists, or it does not
  // already exist and we need to create it.  Start by trying to create it. If
  // that fails, the file has already been created and we can open it
  // normally..  Once the file has been created it will never be deleted.
  int fd =
      open(path.c_str(), O_RDWR | O_CREAT | O_EXCL, O_CLOEXEC | permissions);
  if ((fd == -1) && (errno == EEXIST)) {
    ABSL_VLOG(1) << path << " already created.";
    // File already exists.
    fd = open(path.c_str(), O_RDWR, O_CLOEXEC);
    ABSL_PCHECK(fd != -1) << ": Failed to open " << path;
    while (true) {
      struct stat st;
      ABSL_PCHECK(fstat(fd, &st) == 0);
      if (st.st_size != 0) {
        ABSL_CHECK_EQ(static_cast<size_t>(st.st_size), size_)
            << ": Size of " << path
            << " doesn't match expected size of backing queue file.  Did the "
               "queue definition change?";
        break;
      } else {
        // The creating process didn't get around to it yet.  Give it a bit.
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        ABSL_VLOG(1) << path << " is zero size, waiting";
      }
    }
  } else {
    ABSL_VLOG(1) << "Created " << path;
    ABSL_PCHECK(fd != -1) << ": Failed to open " << path;
    ABSL_PCHECK(ftruncate(fd, size_) == 0);
  }

  data_ = mmap(NULL, size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  ABSL_PCHECK(data_ != MAP_FAILED);
  const_data_ = mmap(NULL, size_, PROT_READ, MAP_SHARED, fd, 0);
  ABSL_PCHECK(const_data_ != MAP_FAILED);
  ABSL_PCHECK(close(fd) == 0);
  PageFaultDataWrite(static_cast<char *>(data_), size_, kSystemPageSize);
  PageFaultDataRead(static_cast<const char *>(const_data_), size_,
                    kSystemPageSize);

  ipc_lib::InitializeLocklessQueueMemory(memory(), config_);
}

MemoryMappedQueue::~MemoryMappedQueue() {
  ABSL_PCHECK(munmap(data_, size_) == 0);
  ABSL_PCHECK(munmap(const_cast<void *>(const_data_), size_) == 0);
}

}  // namespace aos::ipc_lib
