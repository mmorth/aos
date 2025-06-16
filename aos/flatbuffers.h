#ifndef AOS_FLATBUFFERS_H_
#define AOS_FLATBUFFERS_H_

#include <array>
#include <optional>
#include <string_view>
#include <vector>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/types/span.h"
#include "flatbuffers/flatbuffers.h"  // IWYU pragma: export

#include "aos/containers/resizeable_buffer.h"
#include "aos/ipc_lib/data_alignment.h"
#include "aos/macros.h"
#include "aos/util/file.h"
#if __has_feature(memory_sanitizer) || __has_feature(address_sanitizer)
#include "aos/realtime.h"
#endif

namespace aos {

// This class is a base class for all sizes of array backed allocators.
class FixedAllocatorBase : public flatbuffers::Allocator {
 public:
  ~FixedAllocatorBase() override { CHECK(!is_allocated_); }

  // TODO(austin): Read the contract for these.
  uint8_t *allocate(size_t) override;

  void deallocate(uint8_t *allocated_data, size_t allocated_size) override {
    DCHECK_LE(allocated_size, size());
    DCHECK_EQ(allocated_data, data());
    CHECK(is_allocated_);
    is_allocated_ = false;
  }

  uint8_t *reallocate_downward(uint8_t *, size_t, size_t, size_t,
                               size_t) override;

  virtual const uint8_t *data() const = 0;
  virtual uint8_t *data() = 0;
  virtual size_t size() const = 0;

  void Reset() {
    CHECK(!is_allocated_);
    is_allocated_ = false;
  }
  bool is_allocated() const { return is_allocated_; }

  bool allocated() { return is_allocated_; }

 private:
  bool is_allocated_ = false;
};

// This class is a fixed memory allocator which holds the data for a flatbuffer
// in a vector.
class FixedAllocator : public FixedAllocatorBase {
 public:
  FixedAllocator(size_t size) : buffer_(size, 0) {}

  uint8_t *data() override { return &buffer_[0]; }
  const uint8_t *data() const override { return &buffer_[0]; }
  size_t size() const override { return buffer_.size(); }

  // Releases the data in the buffer.
  std::vector<uint8_t> release() { return std::move(buffer_); }

 private:
  std::vector<uint8_t> buffer_;
};

// This class adapts a preallocated memory region to an Allocator.
class PreallocatedAllocator : public FixedAllocatorBase {
 public:
  PreallocatedAllocator(void *data, size_t size) : data_(data), size_(size) {}
  PreallocatedAllocator(const PreallocatedAllocator &) = delete;
  PreallocatedAllocator(PreallocatedAllocator &&other)
      : data_(other.data_), size_(other.size_) {
    CHECK(!is_allocated()) << ": May not overwrite in-use allocator";
    CHECK(!other.is_allocated());
  }

  PreallocatedAllocator &operator=(const PreallocatedAllocator &) = delete;
  PreallocatedAllocator &operator=(PreallocatedAllocator &&other) {
    CHECK(!is_allocated()) << ": May not overwrite in-use allocator";
    CHECK(!other.is_allocated());
    data_ = other.data_;
    size_ = other.size_;
    return *this;
  }

  uint8_t *data() final {
    CHECK(data_ != nullptr);
    return reinterpret_cast<uint8_t *>(data_);
  }
  const uint8_t *data() const final {
    CHECK(data_ != nullptr);
    return reinterpret_cast<const uint8_t *>(data_);
  }
  size_t size() const final { return size_; }

 private:
  void *data_;
  size_t size_;
};

// Base class representing an object which holds the memory representing a root
// flatbuffer.
template <typename T>
class Flatbuffer {
 public:
  virtual ~Flatbuffer() {}
  // Returns the MiniReflectTypeTable for T.
  static const flatbuffers::TypeTable *MiniReflectTypeTable() {
    return T::MiniReflectTypeTable();
  }

  // Returns a message from the buffer.
  virtual const T &message() const = 0;
  // Returns a mutable message.  It can be mutated via the flatbuffer rules.
  virtual T *mutable_message() = 0;

  // Wipes out the data buffer. This is handy to mark an instance as freed, and
  // make attempts to use it fail more obviously.
  void Wipe() { memset(span().data(), 0, span().size()); }

  // Returns true if the flatbuffer is valid. Returns false if either:
  // * The flatbuffer is incorrectly constructed (e.g., it points to memory
  // locations outside of the current memory buffer).
  // * The flatbuffer is too complex, and the flatbuffer verifier chosen to bail
  // when attempting to traverse the tree of tables.
  bool Verify() const {
    if (span().size() < 4u) {
      return false;
    }
    flatbuffers::Verifier v(span().data(), span().size());
    return v.VerifyTable(&message());
  }

 protected:
  virtual absl::Span<uint8_t> span() = 0;
  virtual absl::Span<const uint8_t> span() const = 0;
};

// Base class for non-size prefixed flatbuffers.  span() means different things
// across the 2 types, so you end up with a different GetRoot.
template <typename T>
class NonSizePrefixedFlatbuffer : public Flatbuffer<T> {
 public:
  const T &message() const override {
    return *flatbuffers::GetRoot<T>(
        reinterpret_cast<const void *>(this->span().data()));
  }
  T *mutable_message() override {
    return flatbuffers::GetMutableRoot<T>(
        reinterpret_cast<void *>(this->span().data()));
  }

  absl::Span<uint8_t> span() override = 0;
  absl::Span<const uint8_t> span() const override = 0;
};

// Non-owning Span backed flatbuffer.
template <typename T>
class FlatbufferSpan : public NonSizePrefixedFlatbuffer<T> {
 public:
  // Builds a flatbuffer pointing to the contents of a span.
  FlatbufferSpan(const absl::Span<const uint8_t> data) : data_(data) {}
  // Builds a Flatbuffer pointing to the contents of another flatbuffer.
  FlatbufferSpan(const NonSizePrefixedFlatbuffer<T> &other) {
    data_ = other.span();
  }

  // Copies the data from the other flatbuffer.
  FlatbufferSpan &operator=(const NonSizePrefixedFlatbuffer<T> &other) {
    data_ = other.span();
    return *this;
  }

  virtual ~FlatbufferSpan() override {}

  absl::Span<uint8_t> span() override {
    LOG(FATAL) << "Unimplemented";
    return absl::Span<uint8_t>(nullptr, 0);
  }
  absl::Span<const uint8_t> span() const override { return data_; }

 private:
  absl::Span<const uint8_t> data_;
};

// ResizeableBuffer backed flatbuffer.
template <typename T>
class FlatbufferVector : public NonSizePrefixedFlatbuffer<T> {
 public:
  // Builds a Flatbuffer around a ResizeableBuffer.
  FlatbufferVector(ResizeableBuffer &&data) : data_(std::move(data)) {}

  // Builds a Flatbuffer by copying the data from the other flatbuffer.
  FlatbufferVector(const NonSizePrefixedFlatbuffer<T> &other) {
    data_.resize(other.span().size());
    CHECK(other.span().data());
    memcpy(data_.data(), other.span().data(), data_.size());
  }

  // Copy constructor.
  FlatbufferVector(const FlatbufferVector<T> &other) : data_(other.data_) {}

  // Move constructor.
  FlatbufferVector(FlatbufferVector<T> &&other)
      : data_(std::move(other.data_)) {}

  // Copies the data from the other flatbuffer.
  FlatbufferVector &operator=(const FlatbufferVector<T> &other) {
    data_ = other.data_;
    return *this;
  }
  FlatbufferVector &operator=(FlatbufferVector<T> &&other) {
    data_ = std::move(other.data_);
    return *this;
  }

  // Constructs an empty flatbuffer of type T.
  static FlatbufferVector<T> Empty() {
    return FlatbufferVector<T>(ResizeableBuffer());
  }

  virtual ~FlatbufferVector() override {}

  absl::Span<uint8_t> span() override {
    return absl::Span<uint8_t>(data_.data(), data_.size());
  }
  absl::Span<const uint8_t> span() const override {
    return absl::Span<const uint8_t>(data_.data(), data_.size());
  }

 private:
  ResizeableBuffer data_;
};

// This object associates the message type with the memory storing the
// flatbuffer.  This only stores root tables.
//
// From a usage point of view, pointers to the data are very different than
// pointers to the tables.
template <typename T>
class FlatbufferDetachedBuffer final : public NonSizePrefixedFlatbuffer<T> {
 public:
  // Builds a Flatbuffer by taking ownership of the buffer.
  FlatbufferDetachedBuffer(flatbuffers::DetachedBuffer &&buffer)
      : buffer_(::std::move(buffer)) {}

  // Builds a flatbuffer by taking ownership of the buffer from the other
  // flatbuffer.
  FlatbufferDetachedBuffer(FlatbufferDetachedBuffer &&fb)
      : buffer_(::std::move(fb.buffer_)) {}
  FlatbufferDetachedBuffer &operator=(FlatbufferDetachedBuffer &&fb) {
    ::std::swap(buffer_, fb.buffer_);
    return *this;
  }

  virtual ~FlatbufferDetachedBuffer() override {}

  // Constructs an empty flatbuffer of type T.
  static FlatbufferDetachedBuffer<T> Empty() {
    flatbuffers::FlatBufferBuilder fbb;
    fbb.ForceDefaults(true);
    const auto end = fbb.EndTable(fbb.StartTable());
    fbb.Finish(flatbuffers::Offset<flatbuffers::Table>(end));
    return FlatbufferDetachedBuffer<T>(fbb.Release());
  }

  // Returns references to the buffer, and the data.
  const flatbuffers::DetachedBuffer &buffer() const { return buffer_; }

  absl::Span<uint8_t> span() override {
    return absl::Span<uint8_t>(buffer_.data(), buffer_.size());
  }
  absl::Span<const uint8_t> span() const override {
    return absl::Span<const uint8_t>(buffer_.data(), buffer_.size());
  }

 private:
  flatbuffers::DetachedBuffer buffer_;
};

// Array backed flatbuffer which manages building of the flatbuffer.
template <typename T, size_t Size>
class FlatbufferFixedAllocatorArray final
    : public NonSizePrefixedFlatbuffer<T> {
 public:
  FlatbufferFixedAllocatorArray()
      :
#if __has_feature(memory_sanitizer) || __has_feature(address_sanitizer)
        // See the comments in Reset() for why we're using a vector here when
        // running with sanitizers.
        buffer_(std::make_optional<std::vector<uint8_t>>(Size)),
#else
        buffer_(std::make_optional<std::array<uint8_t, Size>>()),
#endif
        allocator_(
            std::make_optional<PreallocatedAllocator>(buffer_->data(), Size)) {
  }

  FlatbufferFixedAllocatorArray(const FlatbufferFixedAllocatorArray &) = delete;
  void operator=(const NonSizePrefixedFlatbuffer<T> &) = delete;

  void CopyFrom(const NonSizePrefixedFlatbuffer<T> &other) {
    DCHECK(buffer_.has_value());
    DCHECK(allocator_.has_value());
    CHECK(!allocator_->is_allocated()) << ": May not overwrite while building";
    CHECK_LE(other.span().size(), Size)
        << ": Source flatbuffer is larger than the target.";
    memcpy(buffer_->begin(), other.span().data(), other.span().size());
    data_ = buffer_->data();
    size_ = other.span().size();
  }

  // Resets the internal memory. This invalidates the references previously
  // acquired via `message()` or `mutable_message()`.
  void Reset() {
    DCHECK(buffer_.has_value());
    DCHECK(allocator_.has_value());
    CHECK(!allocator_->is_allocated() || data_ != nullptr)
        << ": May not reset while building";
#if __has_feature(memory_sanitizer) || __has_feature(address_sanitizer)
    // Since the user can get a raw pointer to the flatbuffer contained in the
    // allocator, it's possible that the user holds on to this pointer across
    // calls to `Reset()`. Depending on how the user ends up using pointer,
    // undefined behaviour can be invoked. That's not ideal. We really want to
    // make it more obvious that pointers should not be held on to across
    // `Reset()` calls. Unfortunately, the various sanitizers don't have support
    // for arena-style custom memory allocators. To work around that, we
    // forcibly instantiate a whole new buffer object. It has to live on the
    // heap so that the sanitizers can track invalid pointers properly.
    fbb_ = flatbuffers::FlatBufferBuilder{};
    allocator_.reset();
    {
      aos::ScopedNotRealtime not_realtime;
      buffer_ = std::make_optional<std::vector<uint8_t>>(Size);
    }
    allocator_ =
        std::make_optional<PreallocatedAllocator>(buffer_->data(), Size);
#endif
    fbb_ = flatbuffers::FlatBufferBuilder(Size, &*allocator_);
    fbb_.ForceDefaults(true);
    data_ = nullptr;
    size_ = 0;
  }

  flatbuffers::FlatBufferBuilder *fbb() {
    DCHECK(buffer_.has_value());
    DCHECK(allocator_.has_value());
    CHECK(!allocator_->allocated())
        << ": Array backed flatbuffer can only be built once";
    fbb_ = flatbuffers::FlatBufferBuilder(Size, &*allocator_);
    fbb_.ForceDefaults(true);
    return &fbb_;
  }

  void Finish(flatbuffers::Offset<T> root) {
    DCHECK(buffer_.has_value());
    DCHECK(allocator_.has_value());
    CHECK(allocator_->allocated()) << ": Cannot finish if not building";
    fbb_.Finish(root);
    data_ = fbb_.GetBufferPointer();
    size_ = fbb_.GetSize();
    DCHECK_LE(size_, Size);
  }

  absl::Span<uint8_t> span() override {
    return absl::Span<uint8_t>(data_, size_);
  }
  absl::Span<const uint8_t> span() const override {
    return absl::Span<const uint8_t>(data_, size_);
  }

 private:
#if __has_feature(memory_sanitizer) || __has_feature(address_sanitizer)
  // See the comments in Reset() for why we're using a vector here when running
  // with sanitizers.
  std::optional<std::vector<uint8_t>> buffer_;
#else
  std::optional<std::array<uint8_t, Size>> buffer_;
#endif
  std::optional<PreallocatedAllocator> allocator_;
  flatbuffers::FlatBufferBuilder fbb_;
  uint8_t *data_ = nullptr;
  size_t size_ = 0;
};

template <typename T>
class SizePrefixedFlatbuffer : public Flatbuffer<T> {
 public:
  const T &message() const override {
    return *flatbuffers::GetSizePrefixedRoot<T>(
        reinterpret_cast<const void *>(this->span().data()));
  }

  T *mutable_message() override {
    return flatbuffers::GetMutableSizePrefixedRoot<T>(
        reinterpret_cast<void *>(this->span().data()));
  }

  absl::Span<uint8_t> span() override = 0;
  absl::Span<const uint8_t> span() const override = 0;
};

// This object associates the message type with the memory storing the
// flatbuffer.  This only stores root tables.
//
// From a usage point of view, pointers to the data are very different than
// pointers to the tables.
template <typename T>
class SizePrefixedFlatbufferDetachedBuffer final
    : public SizePrefixedFlatbuffer<T> {
 public:
  // Builds a Flatbuffer by taking ownership of the buffer.
  SizePrefixedFlatbufferDetachedBuffer(flatbuffers::DetachedBuffer &&buffer)
      : buffer_(::std::move(buffer)) {
    CHECK_GE(buffer_.size(), sizeof(flatbuffers::uoffset_t));
  }

  // Builds a flatbuffer by taking ownership of the buffer from the other
  // flatbuffer.
  SizePrefixedFlatbufferDetachedBuffer(
      SizePrefixedFlatbufferDetachedBuffer &&fb)
      : buffer_(::std::move(fb.buffer_)) {}
  SizePrefixedFlatbufferDetachedBuffer &operator=(
      SizePrefixedFlatbufferDetachedBuffer &&fb) {
    ::std::swap(buffer_, fb.buffer_);
    return *this;
  }

  virtual ~SizePrefixedFlatbufferDetachedBuffer() override {}

  static SizePrefixedFlatbufferDetachedBuffer<T> Empty() {
    flatbuffers::FlatBufferBuilder fbb;
    fbb.ForceDefaults(true);
    const auto end = fbb.EndTable(fbb.StartTable());
    fbb.FinishSizePrefixed(flatbuffers::Offset<flatbuffers::Table>(end));
    return SizePrefixedFlatbufferDetachedBuffer<T>(fbb.Release());
  }

  flatbuffers::DetachedBuffer Release() {
    flatbuffers::FlatBufferBuilder fbb;
    fbb.ForceDefaults(true);
    const auto end = fbb.EndTable(fbb.StartTable());
    fbb.FinishSizePrefixed(flatbuffers::Offset<flatbuffers::Table>(end));
    flatbuffers::DetachedBuffer result = fbb.Release();
    std::swap(result, buffer_);
    return result;
  }

  // Returns references to the buffer, and the data.
  absl::Span<uint8_t> span() override {
    return absl::Span<uint8_t>(buffer_.data(), buffer_.size());
  }
  absl::Span<const uint8_t> span() const override {
    return absl::Span<const uint8_t>(buffer_.data(), buffer_.size());
  }

 private:
  flatbuffers::DetachedBuffer buffer_;
};

// ResizeableBuffer backed flatbuffer.
template <typename T>
class SizePrefixedFlatbufferVector : public SizePrefixedFlatbuffer<T> {
 public:
  // Builds a Flatbuffer around a ResizeableBuffer.
  SizePrefixedFlatbufferVector(ResizeableBuffer &&data)
      : data_(std::move(data)) {}

  // Builds a Flatbuffer by copying the data from the other flatbuffer.
  SizePrefixedFlatbufferVector(const SizePrefixedFlatbuffer<T> &other)
      : SizePrefixedFlatbufferVector(other.span()) {}

  // Builds a flatbuffer by copying the data from the provided span.
  SizePrefixedFlatbufferVector(const absl::Span<const uint8_t> span) {
    data_.resize(span.size());
    memcpy(data_.data(), span.data(), data_.size());
  }

  // Copy constructor.
  SizePrefixedFlatbufferVector(const SizePrefixedFlatbufferVector<T> &other)
      : data_(other.data_) {}

  // Move constructor.
  SizePrefixedFlatbufferVector(SizePrefixedFlatbufferVector<T> &&other)
      : data_(std::move(other.data_)) {}

  // Copies the data from the other flatbuffer.
  SizePrefixedFlatbufferVector &operator=(
      const SizePrefixedFlatbufferVector<T> &other) {
    data_ = other.data_;
    return *this;
  }
  SizePrefixedFlatbufferVector &operator=(
      SizePrefixedFlatbufferVector<T> &&other) {
    data_ = std::move(other.data_);
    return *this;
  }

  // Constructs an empty flatbuffer of type T.
  static SizePrefixedFlatbufferVector<T> Empty() {
    return SizePrefixedFlatbufferVector<T>(ResizeableBuffer());
  }

  virtual ~SizePrefixedFlatbufferVector() override {}

  absl::Span<uint8_t> span() override {
    return absl::Span<uint8_t>(data_.data(), data_.size());
  }
  absl::Span<const uint8_t> span() const override {
    return absl::Span<const uint8_t>(data_.data(), data_.size());
  }

 private:
  ResizeableBuffer data_;
};

// Non-owning Span backed flatbuffer.
template <typename T>
class SizePrefixedFlatbufferSpan : public SizePrefixedFlatbuffer<T> {
 public:
  // Builds a flatbuffer pointing to the contents of a span.
  SizePrefixedFlatbufferSpan(const absl::Span<const uint8_t> data)
      : data_(data) {}
  // Builds a Flatbuffer pointing to the contents of another flatbuffer.
  SizePrefixedFlatbufferSpan(const SizePrefixedFlatbuffer<T> &other) {
    data_ = other.span();
  }

  // Points to the data in the other flatbuffer.
  SizePrefixedFlatbufferSpan &operator=(
      const SizePrefixedFlatbuffer<T> &other) {
    data_ = other.span();
    return *this;
  }

  ~SizePrefixedFlatbufferSpan() override {}

  absl::Span<uint8_t> span() override {
    LOG(FATAL) << "Unimplemented";
    return absl::Span<uint8_t>(nullptr, 0);
  }
  absl::Span<const uint8_t> span() const override { return data_; }

 private:
  absl::Span<const uint8_t> data_;
};

inline flatbuffers::DetachedBuffer CopySpanAsDetachedBuffer(
    absl::Span<const uint8_t> span) {
  // Copy the data from the span.
  uint8_t *buf = flatbuffers::DefaultAllocator().allocate(span.size());
  memcpy(buf, span.data(), span.size());
  // Then give it to a DetachedBuffer to manage.
  return flatbuffers::DetachedBuffer(nullptr, false, buf, span.size(), buf,
                                     span.size());
}

// MMap a flatbuffer on disk.
template <typename T>
class FlatbufferMMap : public NonSizePrefixedFlatbuffer<T> {
 public:
  // Builds a Flatbuffer by mmaping the data from a flatbuffer saved on disk.
  FlatbufferMMap(const std::string &flatbuffer_path,
                 util::FileOptions options = util::FileOptions::kReadable) {
    span_ = util::MMapFile(flatbuffer_path, options);
  }

  // Copies the reference to the mapped memory.
  FlatbufferMMap(const FlatbufferMMap &) = default;
  FlatbufferMMap &operator=(const FlatbufferMMap<T> &other) = default;

  // Moves the reference to the mapped memory from one pointer to another.
  FlatbufferMMap(FlatbufferMMap &&) = default;
  FlatbufferMMap &operator=(FlatbufferMMap<T> &&other) = default;

  absl::Span<uint8_t> span() override { return *span_; }
  absl::Span<const uint8_t> span() const override { return *span_; }

 private:
  std::shared_ptr<absl::Span<uint8_t>> span_;
};

// The regular flatbuffer API makes it surprisingly irritating to unpack an
// Object into an ObjectT without a bunch of extra lines.
template <typename T>
T::NativeTableType UnpackFlatbuffer(const T *fbs) {
  typename T::NativeTableType object;
  fbs->UnPackTo(&object);
  return object;
}

}  // namespace aos

#endif  // AOS_FLATBUFFERS_H_
