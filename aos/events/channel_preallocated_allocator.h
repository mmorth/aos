#ifndef AOS_EVENTS_CHANNEL_PREALLOCATED_ALLOCATOR_
#define AOS_EVENTS_CHANNEL_PREALLOCATED_ALLOCATOR_

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "flatbuffers/flatbuffers.h"

#include "aos/configuration.h"
#include "aos/configuration_generated.h"

namespace aos {

class ChannelPreallocatedAllocator : public flatbuffers::Allocator {
 public:
  ChannelPreallocatedAllocator(uint8_t *data, size_t size,
                               const Channel *channel)
      : data_(data), size_(size), channel_(channel) {}

  ChannelPreallocatedAllocator(const ChannelPreallocatedAllocator &) = delete;
  ChannelPreallocatedAllocator(ChannelPreallocatedAllocator &&other)
      : data_(other.data_), size_(other.size_), channel_(other.channel_) {
    ABSL_CHECK(!is_allocated()) << ": May not overwrite in-use allocator";
    ABSL_CHECK(!other.is_allocated());
  }

  ChannelPreallocatedAllocator &operator=(
      const ChannelPreallocatedAllocator &) = delete;
  ChannelPreallocatedAllocator &operator=(
      ChannelPreallocatedAllocator &&other) {
    ABSL_CHECK(!is_allocated()) << ": May not overwrite in-use allocator";
    ABSL_CHECK(!other.is_allocated());
    data_ = other.data_;
    size_ = other.size_;
    channel_ = other.channel_;
    return *this;
  }
  ~ChannelPreallocatedAllocator() override { ABSL_CHECK(!is_allocated_); }

  // TODO(austin): Read the contract for these.
  uint8_t *allocate(size_t size) override {
    if (is_allocated_) {
      ABSL_LOG(FATAL)
          << "Can't allocate more memory with a fixed size allocator on "
             "channel "
          << configuration::CleanedChannelToString(channel_);
    }

    ABSL_CHECK_LE(size, size_)
        << ": Tried to allocate more space than available on channel "
        << configuration::CleanedChannelToString(channel_);

    is_allocated_ = true;
    return data_;
  }

  void deallocate(uint8_t *data, size_t size) override {
    ABSL_CHECK_EQ(data, data_)
        << ": Deallocating data not allocated here on channel "
        << configuration::CleanedChannelToString(channel_);
    ABSL_CHECK_LE(size, size_)
        << ": Tried to deallocate more space than available on channel "
        << configuration::CleanedChannelToString(channel_);
    is_allocated_ = false;
  }

  uint8_t *reallocate_downward(uint8_t * /*old_p*/, size_t /*old_size*/,
                               size_t new_size, size_t /*in_use_back*/,
                               size_t /*in_use_front*/) override {
    ABSL_LOG(FATAL)
        << "Requested " << new_size
        << " bytes (includes extra for room to grow even more), max size "
        << channel_->max_size() << " for channel "
        << configuration::CleanedChannelToString(channel_)
        << ".  Increase the memory reserved to at least " << new_size << ".";
    return nullptr;
  }

  void Reset() { is_allocated_ = false; }
  bool is_allocated() const { return is_allocated_; }

  bool allocated() { return is_allocated_; }

  size_t size() const { return size_; }
  const uint8_t *data() const { return data_; }

 private:
  bool is_allocated_ = false;
  uint8_t *data_;
  size_t size_;
  const Channel *channel_;
};

}  // namespace aos

#endif  // AOS_EVENTS_CHANNEL_PREALLOCATED_ALLOCATOR_
