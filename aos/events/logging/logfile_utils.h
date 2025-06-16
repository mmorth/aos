#ifndef AOS_EVENTS_LOGGING_LOGFILE_UTILS_H_
#define AOS_EVENTS_LOGGING_LOGFILE_UTILS_H_

#include <sys/uio.h>

#include <chrono>
#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

#include "absl/container/btree_set.h"
#include "absl/types/span.h"
#include "flatbuffers/flatbuffers.h"

#include "aos/configuration.h"
#include "aos/containers/resizeable_buffer.h"
#include "aos/events/event_loop.h"
#include "aos/events/logging/boot_timestamp.h"
#include "aos/events/logging/buffer_encoder.h"
#include "aos/events/logging/log_backend.h"
#include "aos/events/logging/logfile_sorting.h"
#include "aos/events/logging/logger_generated.h"
#include "aos/flatbuffers.h"
#include "aos/network/remote_message_generated.h"
#include "aos/util/status.h"

namespace aos::logger {

enum class LogType : uint8_t {
  // The message originated on this node and should be logged here.
  kLogMessage,
  // The message originated on another node, but only the delivery times are
  // logged here.
  kLogDeliveryTimeOnly,
  // The message originated on the other node and should be logged on this node.
  kLogRemoteMessage
};

// This class manages efficiently writing a sequence of detached buffers to a
// file.  It encodes them, queues them up, and batches the write operation.

class DetachedBufferWriter {
 public:
  // Marker struct for one of our constructor overloads.
  struct already_out_of_space_t {};

  DetachedBufferWriter(std::unique_ptr<LogSink> log_sink,
                       std::unique_ptr<DataEncoder> encoder);
  // Creates a dummy instance which won't even open a file. It will act as if
  // opening the file ran out of space immediately.
  DetachedBufferWriter(already_out_of_space_t);
  DetachedBufferWriter(DetachedBufferWriter &&other);
  DetachedBufferWriter(const DetachedBufferWriter &) = delete;

  virtual ~DetachedBufferWriter();

  DetachedBufferWriter &operator=(DetachedBufferWriter &&other);
  DetachedBufferWriter &operator=(const DetachedBufferWriter &) = delete;

  std::string_view name() const { return log_sink_->name(); }

  // This will be true until Close() is called, unless the file couldn't be
  // created due to running out of space.
  bool is_open() const { return log_sink_->is_open(); }

  // Queues up a finished FlatBufferBuilder to be encoded and written.
  //
  // Triggers a flush if there's enough data queued up.
  //
  // Steals the detached buffer from it.
  // Returns the duration of time spent on encoding the message.
  std::chrono::nanoseconds CopyMessage(DataEncoder::Copier *copier,
                                       aos::monotonic_clock::time_point now);

  // Indicates we got ENOSPC when trying to write. After this returns true, no
  // further data is written.
  bool ran_out_of_space() const { return ran_out_of_space_; }

  // To avoid silently failing to write logfiles, you must call this before
  // destruction if ran_out_of_space() is true and the situation has been
  // handled.
  void acknowledge_out_of_space() {
    CHECK(ran_out_of_space_);
    acknowledge_ran_out_of_space_ = true;
  }

  // Fully flushes and closes the underlying file now. No additional data may be
  // enqueued after calling this.
  //
  // This will be performed in the destructor automatically.
  //
  // Note that this may set ran_out_of_space().
  void Close();

  // Returns the total number of bytes written and currently queued.
  size_t total_bytes() const {
    if (!encoder_) {
      return 0;
    }
    return encoder_->total_bytes();
  }

  WriteStats *WriteStatistics() const { return log_sink_->WriteStatistics(); }

 private:
  // Performs a single writev call with as much of the data we have queued up as
  // possible.  now is the time we flushed at, to be recorded in
  // last_flush_time_.
  //
  // This will normally take all of the data we have queued up, unless an
  // encoder has spit out a big enough chunk all at once that we can't manage
  // all of it.
  void Flush(aos::monotonic_clock::time_point now);

  // Flushes data if we've reached the threshold to do that as part of normal
  // operation either due to the outstanding queued data, or because we have
  // passed our flush period.  now is the current time to save some CPU grabbing
  // the current time.  It just needs to be close.
  void FlushAtThreshold(aos::monotonic_clock::time_point now);

  std::unique_ptr<LogSink> log_sink_;
  std::unique_ptr<DataEncoder> encoder_;

  bool ran_out_of_space_ = false;
  bool acknowledge_ran_out_of_space_ = false;

  aos::monotonic_clock::time_point last_flush_time_ =
      aos::monotonic_clock::min_time;
};

// Repacks the provided RemoteMessage into fbb.
flatbuffers::Offset<MessageHeader> PackRemoteMessage(
    flatbuffers::FlatBufferBuilder *fbb,
    const message_bridge::RemoteMessage *msg, int channel_index,
    const aos::monotonic_clock::time_point monotonic_timestamp_time);

constexpr flatbuffers::uoffset_t PackRemoteMessageSize() { return 96u; }
size_t PackRemoteMessageInline(
    uint8_t *data, const message_bridge::RemoteMessage *msg, int channel_index,
    const aos::monotonic_clock::time_point monotonic_timestamp_time,
    size_t start_byte, size_t end_byte);

// Packes a message pointed to by the context into a MessageHeader.
flatbuffers::Offset<MessageHeader> PackMessage(
    flatbuffers::FlatBufferBuilder *fbb, const Context &context,
    int channel_index, LogType log_type);

// Returns the size that the packed message from PackMessage or
// PackMessageInline will be.
flatbuffers::uoffset_t PackMessageSize(LogType log_type, size_t data_size);

// Packs the provided message pointed to by context into the provided buffer.
// This is equivalent to PackMessage, but doesn't require allocating a
// FlatBufferBuilder underneath.
size_t PackMessageInline(uint8_t *data, const Context &contex,
                         int channel_index, LogType log_type, size_t start_byte,
                         size_t end_byte);

// Class to read chunks out of a log file.
class SpanReader {
 public:
  // It creates a reader and makes proper decoder based on information encoded
  // in the filename.
  SpanReader(std::string_view filename, bool quiet = false);

  // Opens new reader from provided decoder.
  SpanReader(std::string_view filename, std::unique_ptr<DataDecoder> decoder);

  std::string_view filename() const { return filename_; }

  size_t TotalRead() const { return total_read_; }
  size_t TotalConsumed() const { return total_consumed_; }
  bool IsIncomplete() const {
    return is_finished_ && total_consumed_ < total_read_;
  }

  // Returns a span with the data for the next message from the log file,
  // including the size.  The result is only guarenteed to be valid until
  // ReadMessage() or PeekMessage() is called again.
  absl::Span<const uint8_t> ReadMessage();

  // Returns a span with the data for the next message without consuming it.
  // Multiple calls to PeekMessage return the same data.  ReadMessage or
  // ConsumeMessage must be called to get the next message.
  absl::Span<const uint8_t> PeekMessage();
  // Consumes the message so the next call to ReadMessage or PeekMessage returns
  // new data.  This does not invalidate the data.
  void ConsumeMessage();

 private:
  // TODO(austin): Optimization:
  //   Allocate the 256k blocks like we do today.  But, refcount them with
  //   shared_ptr pointed to by the messageheader that is returned.  This avoids
  //   the copy.  Need to do more benchmarking.
  //   And (Brian): Consider just mmapping the file and handing out refcounted
  //   pointers into that too.

  // Reads a chunk of data into data_.  Returns false if no data was read.
  bool ReadBlock();

  std::string filename_;

  // File reader and data decoder.
  std::unique_ptr<DataDecoder> decoder_;

  // Vector to read into.
  ResizeableBuffer data_;

  // Amount of data consumed already in data_.
  size_t consumed_data_ = 0;

  // Accumulates the total volume of bytes read from filename_
  size_t total_read_ = 0;

  // Accumulates the total volume of read bytes that were 'consumed' into
  // messages. May be less than total_read_, if the last message (span) is
  // either truncated or somehow corrupt.
  size_t total_consumed_ = 0;

  // Reached the end, no more readable messages.
  bool is_finished_ = false;
};

//  Class to borrow log readers from pool based on their ids. This is used as a
//  factory and helps with performance when construction or descrution of
//  decoders are not free. For instance,, S3 fetchers are slow to destroy.
class ReadersPool {
 public:
  virtual ~ReadersPool() = default;

  // Borrow reader from pool based on the id.
  virtual SpanReader *BorrowReader(std::string_view id) = 0;
};

class LogReadersPool : public ReadersPool {
 public:
  explicit LogReadersPool(const LogSource *log_source = nullptr,
                          size_t pool_size = 50);

  SpanReader *BorrowReader(std::string_view id) override;

 private:
  const LogSource *log_source_;
  std::vector<SpanReader> part_readers_;
  const size_t pool_size_;
};

// Reads the last header from a log file.  This handles any duplicate headers
// that were written.
std::optional<SizePrefixedFlatbufferVector<LogFileHeader>> ReadHeader(
    SpanReader *span_reader);
std::optional<SizePrefixedFlatbufferVector<LogFileHeader>> ReadHeader(
    std::string_view filename);
// Reads the Nth message from a log file, excluding the header.  Note: this
// doesn't handle duplicate headers.
std::optional<SizePrefixedFlatbufferVector<MessageHeader>> ReadNthMessage(
    std::string_view filename, size_t n);

class UnpackedMessageHeader;

// Class which handles reading the header and messages from the log file.  This
// handles any per-file state left before merging below.
class MessageReader {
 public:
  // TODO (Alexei): it's deprecated and needs to be removed.
  explicit MessageReader(std::string_view filename)
      : MessageReader(SpanReader(filename)) {}

  explicit MessageReader(SpanReader span_reader);

  std::string_view filename() const { return span_reader_.filename(); }

  // Returns the header from the log file.
  const LogFileHeader *log_file_header() const {
    return &raw_log_file_header_.message();
  }

  // Returns the raw data of the header from the log file.
  const SizePrefixedFlatbufferVector<LogFileHeader> &raw_log_file_header()
      const {
    return raw_log_file_header_;
  }

  // Returns the minimum amount of data needed to queue up for sorting before
  // we're guarenteed to not see data out of order.
  std::chrono::nanoseconds max_out_of_order_duration() const {
    return max_out_of_order_duration_;
  }

  // Returns the newest timestamp read out of the log file.
  monotonic_clock::time_point newest_timestamp() const {
    return newest_timestamp_;
  }

  // Returns the next message if there is one (nullptr if we reached the end of
  // the log).
  // Returns an error if we encountered an issue in reading the log.
  std::shared_ptr<UnpackedMessageHeader> ReadMessage();

  // The time at which we need to read another chunk from the logfile.
  monotonic_clock::time_point queue_data_time() const {
    return newest_timestamp() - max_out_of_order_duration();
  }

  // Flag value setters for testing
  void set_crash_on_corrupt_message_flag(bool b) {
    crash_on_corrupt_message_flag_ = b;
  }
  void set_ignore_corrupt_messages_flag(bool b) {
    ignore_corrupt_messages_flag_ = b;
  }

 private:
  // Log chunk reader.
  SpanReader span_reader_;

  // Vector holding the raw data for the log file header.
  SizePrefixedFlatbufferVector<LogFileHeader> raw_log_file_header_;

  // Minimum amount of data to queue up for sorting before we are guarenteed
  // to not see data out of order.
  std::chrono::nanoseconds max_out_of_order_duration_;

  // Timestamp of the newest message in a channel queue.
  monotonic_clock::time_point newest_timestamp_ = monotonic_clock::min_time;

  // Total volume of verifiable messages from the beginning of the file.
  // TODO - are message counts also useful?
  size_t total_verified_before_ = 0;

  // Total volume of messages with corrupted flatbuffer formatting, if any.
  // Excludes corrupted message content.
  // TODO - if the layout included something as simple as a CRC (relatively
  // fast and robust enough) for each span, then corrupted content could be
  // included in this check.
  size_t total_corrupted_ = 0;

  // Total volume of verifiable messages intermixed with corrupted messages,
  // if any. Will be == 0 if total_corrupted_ == 0.
  size_t total_verified_during_ = 0;

  // Total volume of verifiable messages found after the last corrupted one,
  // if any. Will be == 0 if total_corrupted_ == 0.
  size_t total_verified_after_ = 0;

  bool is_corrupted() const { return total_corrupted_ > 0; }

  bool crash_on_corrupt_message_flag_ = true;
  bool ignore_corrupt_messages_flag_ = false;
};

// A class to seamlessly read messages from a list of part files.
class PartsMessageReader {
 public:
  // TODO (Alexei): it's deprecated, need to removed.
  explicit PartsMessageReader(LogParts log_parts)
      : PartsMessageReader(LogPartsAccess(std::nullopt, std::move(log_parts))) {
  }

  explicit PartsMessageReader(LogPartsAccess log_parts_access);

  std::string_view filename() const { return message_reader_.filename(); }

  // Returns the LogParts that holds the filenames we are reading.
  const LogParts &parts() const { return log_parts_access_.parts(); }

  const LogFileHeader *log_file_header() const {
    return message_reader_.log_file_header();
  }

  // Returns the minimum amount of data needed to queue up for sorting before
  // we are guarenteed to not see data out of order.
  std::chrono::nanoseconds max_out_of_order_duration() const {
    return max_out_of_order_duration_;
  }

  // Returns the newest timestamp read out of the log file.
  monotonic_clock::time_point newest_timestamp() const {
    return newest_timestamp_;
  }

  // Returns the next message if there is one (nullptr if we reached the end of
  // the logfiles).
  // Returns an error if we encountered an issue in reading the log.
  // Note: reading the next message may change the max_out_of_order_duration().
  [[nodiscard]] Result<std::shared_ptr<UnpackedMessageHeader>> ReadMessage();

  // Returns the boot count for the requested node, or std::nullopt if we don't
  // know.
  std::optional<size_t> boot_count(size_t node_index) const {
    CHECK_GE(node_index, 0u);
    CHECK_LT(node_index, boot_counts_.size());
    return boot_counts_[node_index];
  }

 private:
  static SpanReader MakeSpanReader(const LogPartsAccess &log_parts_access,
                                   size_t part_number);

  // Opens the next log and updates message_reader_.  Sets done_ if there is
  // nothing more to do.
  void NextLog();
  void ComputeBootCounts();

  const LogPartsAccess log_parts_access_;
  size_t next_part_index_ = 1u;
  bool done_ = false;

  MessageReader message_reader_;
  // We instantiate the next one early, to allow implementations to prefetch.
  // TODO(Brian): To get optimal performance when downloading, this needs more
  // communication with the implementation to prioritize the next part and add
  // more parallelism when it helps. Maybe some kind of a queue of parts in
  // order, and the implementation gets to pull however many make sense off the
  // front?
  std::optional<MessageReader> next_message_reader_;

  // True after we have seen a message after the start of the log.  The
  // guarentees on logging essentially are that all data from before the
  // starting time of the log may be arbitrarily out of order, but once we get
  // max_out_of_order_duration past the start, everything will remain within
  // max_out_of_order_duration.  We shouldn't see anything before the start
  // after we've seen a message that is at least max_out_of_order_duration after
  // the start.
  bool after_start_ = false;

  monotonic_clock::time_point newest_timestamp_ = monotonic_clock::min_time;

  // Per node boot counts.
  std::vector<std::optional<size_t>> boot_counts_;

  const std::chrono::nanoseconds max_out_of_order_duration_;
};

// Stores MessageHeader as a flat header and inline, aligned block of data.
class UnpackedMessageHeader {
 public:
  UnpackedMessageHeader(
      uint32_t channel_index, monotonic_clock::time_point monotonic_sent_time,
      realtime_clock::time_point realtime_sent_time, uint32_t queue_index,
      std::optional<monotonic_clock::time_point> monotonic_remote_time,
      std::optional<realtime_clock::time_point> realtime_remote_time,
      monotonic_clock::time_point monotonic_remote_transmit_time,
      std::optional<uint32_t> remote_queue_index,
      monotonic_clock::time_point monotonic_timestamp_time,
      bool has_monotonic_timestamp_time, absl::Span<const uint8_t> span)
      : channel_index(channel_index),
        monotonic_sent_time(monotonic_sent_time),
        realtime_sent_time(realtime_sent_time),
        queue_index(queue_index),
        monotonic_remote_time(monotonic_remote_time),
        realtime_remote_time(realtime_remote_time),
        monotonic_remote_transmit_time(monotonic_remote_transmit_time),
        remote_queue_index(remote_queue_index),
        monotonic_timestamp_time(monotonic_timestamp_time),
        has_monotonic_timestamp_time(has_monotonic_timestamp_time),
        span(span) {}
  UnpackedMessageHeader(const UnpackedMessageHeader &) = delete;
  UnpackedMessageHeader &operator=(const UnpackedMessageHeader &) = delete;

  // The channel.
  uint32_t channel_index = 0xffffffff;

  monotonic_clock::time_point monotonic_sent_time;
  realtime_clock::time_point realtime_sent_time;

  // The local queue index.
  uint32_t queue_index = 0xffffffff;

  std::optional<aos::monotonic_clock::time_point> monotonic_remote_time;

  std::optional<realtime_clock::time_point> realtime_remote_time;
  aos::monotonic_clock::time_point monotonic_remote_transmit_time;
  std::optional<uint32_t> remote_queue_index;

  // This field is defaulted in the flatbuffer, so we need to store both the
  // possibly defaulted value and whether it is defaulted.
  monotonic_clock::time_point monotonic_timestamp_time;
  bool has_monotonic_timestamp_time;

  static std::shared_ptr<UnpackedMessageHeader> MakeMessage(
      const MessageHeader &message);

  // Note: we are storing a span here because we need something to put in the
  // SharedSpan pointer that RawSender takes.  We are using the aliasing
  // constructor of shared_ptr to avoid the allocation, and it needs a nice
  // pointer to track.
  absl::Span<const uint8_t> span;

  char actual_data[];

 private:
  ~UnpackedMessageHeader() {}

  static void DestroyAndFree(UnpackedMessageHeader *p) {
    p->~UnpackedMessageHeader();
    free(p);
  }
};

std::ostream &operator<<(std::ostream &os,
                         const UnpackedMessageHeader &message);

// Struct to hold a message as it gets sorted on a single node.
struct Message {
  // The channel.
  uint32_t channel_index = 0xffffffff;
  // The local queue index.
  // TODO(austin): Technically the boot inside queue_index is redundant with
  // timestamp.  In practice, it is less error-prone to duplicate it.  Maybe a
  // function to return the combined struct?
  BootQueueIndex queue_index;
  // The local timestamp.
  BootTimestamp timestamp;

  // Remote boot when this is a timestamp.
  size_t monotonic_remote_boot = 0xffffff;

  size_t monotonic_timestamp_boot = 0xffffff;

  // Pointer to the unpacked header.
  std::shared_ptr<UnpackedMessageHeader> header;

  // Pointer to a pointer to the span with the flatbuffer to publish in it.  The
  // second layer of indirection lets us modify all copies of a message when
  // sending inside the log reader.
  std::shared_ptr<SharedSpan> data;

  bool operator<(const Message &m2) const;
  bool operator<=(const Message &m2) const;
  bool operator>=(const Message &m2) const;
  bool operator==(const Message &m2) const;
};

std::ostream &operator<<(std::ostream &os, const Message &m);

// Structure to hold a full message and all the timestamps, which may or may not
// have been sent from a remote node.  The remote_queue_index will be invalid if
// this message is from the point of view of the node which sent it.
struct TimestampedMessage {
  uint32_t channel_index = 0xffffffff;

  BootQueueIndex queue_index;
  BootTimestamp monotonic_event_time;
  realtime_clock::time_point realtime_event_time = realtime_clock::min_time;

  BootQueueIndex remote_queue_index;
  BootTimestamp monotonic_remote_time;
  realtime_clock::time_point realtime_remote_time = realtime_clock::min_time;

  BootTimestamp monotonic_remote_transmit_time;

  BootTimestamp monotonic_timestamp_time;

  // Pointer to a pointer to the data.  If the outer pointer isn't populated, no
  // data exists to send, we only have the timestamps. If the inner pointer is
  // nullptr, the user has marked the message as something to not send.
  std::shared_ptr<SharedSpan> data;
};

std::ostream &operator<<(std::ostream &os, const TimestampedMessage &m);

// Class to sort the resulting messages from a PartsMessageReader.
class MessageSorter {
 public:
  // TODO (Alexei): it's deperecated and need to be removed.
  explicit MessageSorter(LogParts log_parts)
      : MessageSorter(LogPartsAccess(std::nullopt, std::move(log_parts))) {}

  explicit MessageSorter(const LogPartsAccess log_parts_access);

  // Returns the parts that this is sorting messages from.
  const LogParts &parts() const { return parts_message_reader_.parts(); }

  monotonic_clock::time_point monotonic_start_time() const {
    return parts().monotonic_start_time;
  }
  realtime_clock::time_point realtime_start_time() const {
    return parts().realtime_start_time;
  }

  // The time this data is sorted until.
  monotonic_clock::time_point sorted_until() const { return sorted_until_; }

  // Returns the next sorted message from the log file; nullptr if at the end of
  // the log. Returns an error if we encounter an error in reading the logfile.
  [[nodiscard]] Result<const Message *> Front();
  // Pops the front message.  This should only be called after a call to
  // Front().
  void PopFront();

  // Returns a debug string representing the contents of this sorter.
  std::string DebugString() const;

 private:
  // Log parts reader we are wrapping.
  PartsMessageReader parts_message_reader_;
  // Cache of the time we are sorted until.
  aos::monotonic_clock::time_point sorted_until_ = monotonic_clock::min_time;

  // Timestamp of the last message returned.  Used to make sure nothing goes
  // backwards.
  monotonic_clock::time_point last_message_time_ = monotonic_clock::min_time;

  // Set used for efficient sorting of messages.  We can benchmark and evaluate
  // other data structures if this proves to be the bottleneck.
  absl::btree_set<Message> messages_;

  // Mapping from channel to source node.
  // TODO(austin): Should we put this in Boots so it can be cached for everyone?
  std::vector<size_t> source_node_index_;
};

// Class to run merge sort on the messages associated with specific node and
// boot.
class PartsMerger {
 public:
  PartsMerger(SelectedLogParts &&selected_parts);

  // Copying and moving will mess up the internal raw pointers.  Just don't do
  // it.
  PartsMerger(PartsMerger const &) = delete;
  PartsMerger(PartsMerger &&) = delete;
  void operator=(PartsMerger const &) = delete;
  void operator=(PartsMerger &&) = delete;

  // Node index in the configuration of this node.
  int node() const { return node_; }

  std::string_view node_name() const {
    return configuration::NodeName(configuration().get(), node());
  }

  // List of parts being sorted together.
  std::vector<const LogParts *> Parts() const;

  const std::shared_ptr<const Configuration> configuration() const {
    return message_sorters_[0].parts().config;
  }

  monotonic_clock::time_point monotonic_start_time() const {
    return monotonic_start_time_;
  }
  realtime_clock::time_point realtime_start_time() const {
    return realtime_start_time_;
  }

  // Returns the oldest message observed in this set of parts.  This could be
  // before the start time if we fetched it at the start of logging from long
  // ago.
  monotonic_clock::time_point monotonic_oldest_time() {
    if (monotonic_oldest_time_ == monotonic_clock::max_time) {
      VLOG(1) << "No oldest message time, fetching " << node_name();
      (void)Front();
    }
    return monotonic_oldest_time_;
  }

  // The time this data is sorted until.
  monotonic_clock::time_point sorted_until() const { return sorted_until_; }

  // Returns the next sorted message from the set of log files; nullptr if at
  // the end of the log.
  // Returns an error on a failure to read the log files.
  [[nodiscard]] Result<const Message *> Front();
  // Pops the front message.  This should only be called after a call to
  // Front().
  void PopFront();

 private:
  // Unsorted list of all parts sorters.
  std::vector<MessageSorter> message_sorters_;

  // Pointer to the parts sorter holding the current Front message if one
  // exists, or nullptr if a new one needs to be found.
  MessageSorter *current_ = nullptr;
  // Cached sorted_until value.
  aos::monotonic_clock::time_point sorted_until_ = monotonic_clock::min_time;

  // Cached node.
  int node_;

  // Timestamp of the last message returned.  Used to make sure nothing goes
  // backwards.
  monotonic_clock::time_point last_message_time_ = monotonic_clock::min_time;

  realtime_clock::time_point realtime_start_time_ = realtime_clock::max_time;
  monotonic_clock::time_point monotonic_start_time_ = monotonic_clock::max_time;
  monotonic_clock::time_point monotonic_oldest_time_ =
      monotonic_clock::max_time;
};

// Class to concatenate multiple boots worth of logs into a single per-node
// stream.
class BootMerger {
 public:
  BootMerger(std::string_view node_name, const LogFilesContainer &log_files,
             const std::vector<StoredDataType> &types);

  // Copying and moving will mess up the internal raw pointers.  Just don't do
  // it.
  BootMerger(BootMerger const &) = delete;
  BootMerger(BootMerger &&) = delete;
  void operator=(BootMerger const &) = delete;
  void operator=(BootMerger &&) = delete;

  // Node index in the configuration of this node.
  int node() const { return node_; }
  std::string_view node_name() const;

  // List of parts being sorted together.
  std::vector<const LogParts *> Parts() const;

  const std::shared_ptr<const Configuration> configuration() const {
    return configuration_;
  }

  monotonic_clock::time_point monotonic_start_time(size_t boot) const;
  realtime_clock::time_point realtime_start_time(size_t boot) const;
  monotonic_clock::time_point monotonic_oldest_time(size_t boot) const;

  bool started() const;

  // Returns the next sorted message from the set of log files; nullptr if at
  // the end of the log.
  // Returns an error on a failure to read the log files.
  [[nodiscard]] Result<const Message *> Front();
  // Pops the front message.  This should only be called after a call to
  // Front().
  void PopFront();

 private:
  int index_ = 0;

  // TODO(austin): Sanjay points out this is pretty inefficient.  Don't keep so
  // many things open.
  // A list of all the parts mergers.  Only the boots with something to sort are
  // instantiated.
  std::vector<std::unique_ptr<PartsMerger>> parts_mergers_;

  std::shared_ptr<const Configuration> configuration_;
  int node_;
};

enum class TimestampQueueStrategy {
  // Read the timestamps at the same time as all the other data.
  kQueueTogether,
  // Read the timestamps first.
  kQueueTimestampsAtStartup,
};

// Class to manage queueing up timestamps from BootMerger and notifying
// TimestampMapper of them.
class SplitTimestampBootMerger {
 public:
  SplitTimestampBootMerger(std::string_view node_name,
                           const LogFilesContainer &log_files,
                           TimestampQueueStrategy timestamp_queue_strategy);

  // Copying and moving will mess up the internal raw pointers.  Just don't do
  // it.
  SplitTimestampBootMerger(SplitTimestampBootMerger const &) = delete;
  SplitTimestampBootMerger(SplitTimestampBootMerger &&) = delete;
  void operator=(SplitTimestampBootMerger const &) = delete;
  void operator=(SplitTimestampBootMerger &&) = delete;

  // Reads all timestamps into a member variable queue, and calls the function
  // on each timestamp.  This only saves timestamps, which are defined as
  // messages sent on this node, but not originally from this node.  To make
  // that distinction, source_node is provided which has a list of which node
  // index is the source node for each channel, where the channel index is the
  // array index.
  // Returns an error if there were any issues while attempting to read the
  // logfile.
  [[nodiscard]] Status QueueTimestamps(
      std::function<void(TimestampedMessage *)> fn,
      const std::vector<size_t> &source_node);

  // Node index in the configuration of this node.
  int node() const { return boot_merger_.node(); }
  // Returns the name of the node this class is sorting for.
  std::string_view node_name() const;

  std::shared_ptr<const Configuration> configuration() const {
    return boot_merger_.configuration();
  }

  monotonic_clock::time_point monotonic_start_time(size_t boot) const;
  realtime_clock::time_point realtime_start_time(size_t boot) const;
  monotonic_clock::time_point monotonic_oldest_time(size_t boot) const;

  // Returns true if the log has been started.
  bool started() const {
    // Timestamps don't count, so only track boot_merger_.
    return boot_merger_.started();
  }

  // Returns the next sorted message from the set of log files; nullptr if we
  // have reached the end of the log files.
  // Returns an error if there were any issues while attempting to read the
  // log.
  [[nodiscard]] Result<const Message *> Front();

  // Pops the front message.  This should only be called after a call to
  // Front().
  void PopFront();

 private:
  enum class MessageSource {
    kTimestampMessage,
    kBootMerger,
  };

  MessageSource message_source_ = MessageSource::kBootMerger;

  // Boot merger for data and potentially timestamps.
  BootMerger boot_merger_;

  // Boot merger for just timestamps.  Any data read from here is to be ignored.
  std::unique_ptr<BootMerger> timestamp_boot_merger_;

  // Deque of all the timestamp messages.
  std::deque<Message> timestamp_messages_;

  // Start times for each boot.
  std::vector<monotonic_clock::time_point> monotonic_start_time_;
  std::vector<realtime_clock::time_point> realtime_start_time_;

  // Tracks if QueueTimestamps loaded any timestamps.
  bool queue_timestamps_ran_ = false;
};

// Class to match timestamps with the corresponding data from other nodes.
//
// This class also buffers data for the node it represents, and supports
// notifying when new data is queued as well as queueing until a point in time.
class TimestampMapper {
 public:
  TimestampMapper(std::string_view node_name,
                  const LogFilesContainer &log_files,
                  TimestampQueueStrategy timestamp_queue_strategy);

  // Copying and moving will mess up the internal raw pointers.  Just don't do
  // it.
  TimestampMapper(TimestampMapper const &) = delete;
  TimestampMapper(TimestampMapper &&) = delete;
  void operator=(TimestampMapper const &) = delete;
  void operator=(TimestampMapper &&) = delete;

  // TODO(austin): It would be super helpful to provide a way to queue up to
  // time X without matching timestamps, and to then be able to pull the
  // timestamps out of this queue.  This lets us bootstrap time estimation
  // without exploding memory usage worst case.

  const Configuration *configuration() const { return configuration_.get(); }

  // Returns which node this is sorting for.
  size_t node() const { return boot_merger_.node(); }

  // The start time of this log.
  monotonic_clock::time_point monotonic_start_time(size_t boot) const {
    return boot_merger_.monotonic_start_time(boot);
  }
  realtime_clock::time_point realtime_start_time(size_t boot) const {
    return boot_merger_.realtime_start_time(boot);
  }
  // Returns the oldest timestamp on a message on this boot.
  monotonic_clock::time_point monotonic_oldest_time(size_t boot) const {
    return boot_merger_.monotonic_oldest_time(boot);
  }

  // Uses timestamp_mapper as the peer for its node. Only one mapper may be set
  // for each node.  Peers are used to look up the data for timestamps on this
  // node.
  void AddPeer(TimestampMapper *timestamp_mapper);

  // Returns true if anything has been queued up.
  bool started() const { return boot_merger_.started(); }

  // Returns the next message for this node; nullptr if at the end of the log.
  // Returns an error upon failing to read the log.
  [[nodiscard]] Result<TimestampedMessage *> Front();
  // Pops the next message.  Front must be called first.
  // Returns
  [[nodiscard]] Status PopFront();

  // Returns debug information about this node.
  std::string DebugString() const;

  // Queues just the timestamps so that the timestamp callback gets called.
  // Note, the timestamp callback will get called when they get returned too, so
  // make sure to unset it if you don't want to be called twice.
  [[nodiscard]] Status QueueTimestamps();

  // Queues data the provided time.
  [[nodiscard]] Status QueueUntil(BootTimestamp queue_time);
  // Queues until we have time_estimation_buffer of data in the queue.
  [[nodiscard]] Status QueueFor(
      std::chrono::nanoseconds time_estimation_buffer);

  // Queues until the condition is met.
  template <typename T>
  [[nodiscard]] Status QueueUntilCondition(T fn) {
    while (true) {
      if (fn()) {
        break;
      }
      const Result<bool> queued_message = QueueMatched();
      if (!queued_message.has_value()) {
        return MakeError(queued_message.error());
      }
      if (!queued_message.value()) {
        return Ok();
      }
    }
    return Ok();
  }

  // Sets the callback that can be used to skip messages.
  void set_replay_channels_callback(
      std::function<bool(const TimestampedMessage &)> fn) {
    replay_channels_callback_ = fn;
  }

  // Sets a callback to be called whenever a full message is queued.
  void set_timestamp_callback(std::function<void(TimestampedMessage *)> fn) {
    timestamp_callback_ = fn;
  }

 private:
  // Result of MaybeQueueMatched
  enum class MatchResult : uint8_t {
    kEndOfFile,  // End of the log file being read
    kQueued,     // Message was queued
    kSkipped     // Message was skipped over
  };

  // The state for a remote node.  This holds the data that needs to be matched
  // with the remote node's timestamps.
  struct NodeData {
    // True if we should save data here.  This should be true if any of the
    // bools in delivered below are true.
    bool any_delivered = false;

    // True if we have a peer and therefore should be saving data for it.
    bool save_for_peer = false;

    // Peer pointer.  This node is only to be considered if a peer is set.
    TimestampMapper *peer = nullptr;

    struct ChannelData {
      // Deque per channel.  This contains the data from the outside
      // TimestampMapper node which is relevant for the node this NodeData
      // points to.
      std::deque<Message> messages;
      // Bool tracking per channel if a message is delivered to the node this
      // NodeData represents.
      bool delivered = false;
      // The TTL for delivery.
      std::chrono::nanoseconds time_to_live = std::chrono::nanoseconds(0);
    };

    // Vector with per channel data.
    std::vector<ChannelData> channels;
  };

  // Returns (and forgets about) the data for the provided timestamp message
  // showing when it was delivered to this node.
  Result<Message> MatchingMessageFor(const Message &message);

  // Queues up a single message into our message queue, and any nodes that this
  // message is delivered to.  Returns true if one was available, false
  // otherwise.
  // Returns an error if an error was encountered in log reading.
  [[nodiscard]] Result<bool> Queue();

  // Queues up a single matched message into our matched message queue.  Returns
  // true if one was queued, and false otherwise.
  // Returns an error if an error was encountered in log reading.
  [[nodiscard]] Result<bool> QueueMatched();

  // Queues a message if the replay_channels_callback is passed and the end of
  // the log file has not been reached.
  // Returns an error if an error was encountered in log reading.
  [[nodiscard]] Result<MatchResult> MaybeQueueMatched();

  // Queues up data until we have at least one message >= to time t.
  // Useful for triggering a remote node to read enough data to have the
  // timestamp you care about available.
  [[nodiscard]] Status QueueUnmatchedUntil(BootTimestamp t);

  // Queues m into matched_messages_.
  void QueueMessage(const Message *m);

  // If a replay_channels_callback was set and the callback returns false, a
  // matched message is popped and true is returned. Otherwise false is
  // returned.
  bool CheckReplayChannelsAndMaybePop(const TimestampedMessage &message);

  // Returns the name of the node this class is sorting for.
  std::string_view node_name() const {
    return configuration::NodeName(configuration(), node());
  }

  // The node merger to source messages from.
  SplitTimestampBootMerger boot_merger_;

  std::shared_ptr<const Configuration> configuration_;

  // The buffer of messages for this node.  These are not matched with any
  // remote data.
  std::deque<Message> messages_;
  // The node index for the source node for each channel.
  std::vector<size_t> source_node_;

  // Vector per node.  Not all nodes will have anything.
  std::vector<NodeData> nodes_data_;

  // Latest message to return.
  std::deque<TimestampedMessage> matched_messages_;

  // Tracks the state of the first message in matched_messages_.  Do we need to
  // update it, is it valid, or should we return nullptr?
  enum class FirstMessage {
    kNeedsUpdate,
    kInMessage,
    kNullptr,
  };
  FirstMessage first_message_ = FirstMessage::kNeedsUpdate;

  // Timestamp of the last message returned.  Used to make sure nothing goes
  // backwards.
  BootTimestamp last_message_time_ = BootTimestamp::min_time();
  BootTimestamp last_popped_message_time_ = BootTimestamp::min_time();
  // Time this node is queued up until.  Used for caching.
  BootTimestamp queued_until_ = BootTimestamp::min_time();

  std::function<void(TimestampedMessage *)> timestamp_callback_;
  std::function<bool(TimestampedMessage &)> replay_channels_callback_;
};

// Returns the node name, or an empty string if we are a single node.
inline std::string_view MaybeNodeName(const Node *node) {
  if (node != nullptr) {
    return node->name()->string_view();
  }
  return "";
}

// Class to copy a RemoteMessage into the provided buffer.
class RemoteMessageCopier : public DataEncoder::Copier {
 public:
  RemoteMessageCopier(const message_bridge::RemoteMessage *message,
                      int channel_index,
                      aos::monotonic_clock::time_point monotonic_timestamp_time,
                      EventLoop *event_loop)
      : DataEncoder::Copier(PackRemoteMessageSize()),
        message_(message),
        channel_index_(channel_index),
        monotonic_timestamp_time_(monotonic_timestamp_time),
        event_loop_(event_loop) {}

  monotonic_clock::time_point end_time() const { return end_time_; }

  size_t Copy(uint8_t *data, size_t start_byte, size_t end_byte) final {
    size_t result = PackRemoteMessageInline(data, message_, channel_index_,
                                            monotonic_timestamp_time_,
                                            start_byte, end_byte);
    end_time_ = event_loop_->monotonic_now();
    return result;
  }

 private:
  const message_bridge::RemoteMessage *message_;
  int channel_index_;
  aos::monotonic_clock::time_point monotonic_timestamp_time_;
  EventLoop *event_loop_;
  monotonic_clock::time_point end_time_;
};

// Class to copy a context into the provided buffer.
class ContextDataCopier : public DataEncoder::Copier {
 public:
  ContextDataCopier(const Context &context, int channel_index, LogType log_type,
                    EventLoop *event_loop)
      : DataEncoder::Copier(PackMessageSize(log_type, context.size)),
        context_(context),
        channel_index_(channel_index),
        log_type_(log_type),
        event_loop_(event_loop) {}

  monotonic_clock::time_point end_time() const { return end_time_; }

  size_t Copy(uint8_t *data, size_t start_byte, size_t end_byte) final {
    size_t result = PackMessageInline(data, context_, channel_index_, log_type_,
                                      start_byte, end_byte);
    end_time_ = event_loop_->monotonic_now();
    return result;
  }

 private:
  const Context &context_;
  const int channel_index_;
  const LogType log_type_;
  EventLoop *event_loop_;
  monotonic_clock::time_point end_time_;
};

}  // namespace aos::logger

#endif  // AOS_EVENTS_LOGGING_LOGFILE_UTILS_H_
