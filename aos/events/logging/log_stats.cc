#include <iomanip>
#include <iostream>
#include <queue>

#include "absl/flags/flag.h"
#include "absl/flags/usage.h"
#include "absl/strings/str_format.h"

#include "aos/events/logging/log_reader.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/time/time.h"

ABSL_FLAG(
    std::string, name, "",
    "Name to match for printing out channels. Empty means no name filter.");

ABSL_FLAG(std::string, node, "", "Node to print stats out for.");

ABSL_FLAG(bool, excessive_size_only, false,
          "Only print channels that have a set max message size that is more "
          "than double of the max message size.");

ABSL_FLAG(double, skip_seconds, 0.0,
          "If set to a positive value, skip this many seconds at the beginning "
          "of the log. Otherwise, process the log from the very beginning.");
ABSL_FLAG(
    double, run_for, 0.0,
    "If set to a positive value, only process the log for this many seconds. "
    "Otherwise, process the log until the end of the log.");

ABSL_FLAG(
    bool, print_repack_size_diffs, false,
    "Analyze how many bytes could be saved in each message when converted to "
    "JSON and back. This can be helpful to identify code that is generating "
    "inefficiently packed flatbuffer messages.");

// This class implements a histogram for tracking message period
// percentiles.
class Histogram {
 public:
  Histogram(size_t buckets = 1024)
      : max_value_bucket_(0.01), values_(buckets, 0.0), counts_(buckets, 0) {}

  // Adds a new sample to the histogram, potentially downsampling the existing
  // data.
  void Add(double value) {
    if (value < max_value_bucket_) {
      const ssize_t bucket = static_cast<size_t>(
          std::floor(value * values_.size() / max_value_bucket_));
      CHECK_GE(bucket, 0);
      CHECK_LT(bucket, static_cast<ssize_t>(values_.size()));
      values_[bucket] += value;
      if (all_counts_ == 0 || value > max_value_) {
        max_value_ = value;
      }
      if (all_counts_ == 0 || value < min_value_) {
        min_value_ = value;
      }
      ++counts_[bucket];
      ++all_counts_;
    } else {
      // Double all the bucket sizes by merging adjacent buckets and doubling
      // the max value.  If this isn't enough, we'll recurse inside Add and
      // do it again until it fits.
      max_value_bucket_ *= 2.0;
      for (size_t bucket = 0; bucket < values_.size() / 2; ++bucket) {
        values_[bucket] = values_[bucket * 2] + values_[bucket * 2 + 1];
        counts_[bucket] = counts_[bucket * 2] + counts_[bucket * 2 + 1];
      }
      for (size_t bucket = values_.size() / 2; bucket < values_.size();
           ++bucket) {
        values_[bucket] = 0.0;
        counts_[bucket] = 0;
      }
      Add(value);
    }
  }

  // Prints out the percentiles for a couple of critical numbers.
  std::string Percentile() const {
    const size_t percentile5 = all_counts_ / 20;
    double percentile5_value = 0.0;
    const size_t percentile50 = all_counts_ / 2;
    double percentile50_value = 0.0;
    const size_t percentile95 = all_counts_ - percentile5;
    double percentile95_value = 0.0;

    size_t count = 0;
    for (size_t i = 0; i < values_.size(); ++i) {
      if (count < percentile5 && count + counts_[i] >= percentile5) {
        percentile5_value = values_[i] / counts_[i];
      }
      if (count < percentile50 && count + counts_[i] >= percentile50) {
        percentile50_value = values_[i] / counts_[i];
      }
      if (count < percentile95 && count + counts_[i] >= percentile95) {
        percentile95_value = values_[i] / counts_[i];
      }
      count += counts_[i];
    }

    // Assume here that these are periods in seconds.  Convert to ms for
    // readability.  This isn't super generic, but that's fine for now.
    return absl::StrFormat(
        "[max %.3fms 95%%:%.3fms 50%%:%.3fms 5%%:%.3fms min %.3fms]",
        max_value_ * 1000., percentile95_value * 1000.,
        percentile50_value * 1000., percentile5_value * 1000.,
        min_value_ * 1000.);
  }

 private:
  // The size of the largest bucket.  Used to figure out which bucket something
  // goes into.
  double max_value_bucket_;
  // Max and min values overall we have seen.
  double max_value_ = 0;
  double min_value_ = 0;
  // A list of the sum of values and counts for those per bucket.
  std::vector<double> values_;
  std::vector<size_t> counts_;
  // Total number of samples.
  size_t all_counts_ = 0;
};

class ChannelStats {
 public:
  ChannelStats(const aos::Channel *channel, const aos::Node *destination_node,
               aos::SimulatedEventLoopFactory *factory,
               const reflection::Schema *schema)
      : channel_(channel),
        config_(factory->configuration()),
        factory_(factory),
        schema_([&]() {
          CHECK(schema != nullptr);
          return schema;
        }()),
        destination_node_(destination_node),
        flatbuffer_type_(schema) {
    // Multi-node channel
    if (channel_->has_source_node() && channel_->has_destination_nodes() &&
        channel_->destination_nodes()->size() > 0) {
      CHECK(destination_node_)
          << "Should have destination node for forwarded channel: "
          << channel_->name()->string_view();
      source_node_ = aos::configuration::GetNode(
          config_, channel_->source_node()->string_view());
      CHECK(source_node_) << "Node not in config: "
                          << channel_->source_node()->string_view();
    }
  }

  // Adds a sample to the statistics.
  void Add(const aos::Context &context) {
    if (context.size > 0 && context.data != nullptr) {
      // Perform a very naive repacking of the message. Ideally, we'd use
      // something like RecursiveCopyFlatBuffer that works with schemas.
      // For now, this is good enough.
      const std::string json = aos::FlatbufferToJson(
          schema_, static_cast<const uint8_t *>(context.data));
      flatbuffers::DetachedBuffer buffer =
          aos::JsonToFlatbuffer(json, flatbuffer_type_);

      const ssize_t packed_size_reduction = static_cast<ssize_t>(context.size) -
                                            static_cast<ssize_t>(buffer.size());
      max_packed_size_reduction_ =
          std::max(max_packed_size_reduction_, packed_size_reduction);
      total_packed_size_reduction_ += packed_size_reduction;
    }

    max_message_size_ = std::max(max_message_size_, context.size);
    total_message_size_ += context.size;
    total_num_messages_++;
    channel_end_time_ = context.realtime_event_time;
    first_message_time_ =
        std::min(first_message_time_, context.monotonic_event_time);
    if (current_message_time_ != aos::monotonic_clock::min_time) {
      histogram_.Add(std::chrono::duration<double>(
                         context.monotonic_event_time - current_message_time_)
                         .count());
    }
    current_message_time_ = context.monotonic_event_time;
    channel_storage_duration_messages_.push(current_message_time_);
    while (channel_storage_duration_messages_.front() +
               aos::configuration::ChannelStorageDuration(config_, channel_) <=
           current_message_time_) {
      channel_storage_duration_messages_.pop();
    }
    max_messages_per_period_ = std::max(
        max_messages_per_period_, channel_storage_duration_messages_.size());

    // Only count latency if this message is forwarded and the remote time was
    // filled
    if (source_node_ != nullptr &&
        context.monotonic_remote_time != context.monotonic_event_time) {
      // Convert times to distributed clock so they can be compared across nodes
      const aos::distributed_clock::time_point remote_time = CheckExpected(
          factory_->GetNodeEventLoopFactory(source_node_)
              ->ToDistributedClock(context.monotonic_remote_time));

      const aos::distributed_clock::time_point event_time =
          CheckExpected(factory_->GetNodeEventLoopFactory(destination_node_)
                            ->ToDistributedClock(context.monotonic_event_time));
      // Add the current latency to the sum
      total_latency_ += event_time - remote_time;

      num_messages_with_remote_++;
    }
  }

  std::string Percentile() const { return histogram_.Percentile(); }

  double SecondsActive() const {
    return aos::time::DurationInSeconds(current_message_time_ -
                                        first_message_time_);
  }

  size_t max_message_size() const { return max_message_size_; }
  size_t total_num_messages() const { return total_num_messages_; }

  ssize_t max_packed_size_reduction() const {
    return max_packed_size_reduction_;
  }

  double avg_messages_per_sec() const {
    return total_num_messages_ / SecondsActive();
  }
  double max_messages_per_sec() const {
    return max_messages_per_period_ /
           std::min(SecondsActive(),
                    std::chrono::duration<double>(
                        aos::configuration::ChannelStorageDuration(config_,
                                                                   channel_))
                        .count());
  }
  size_t avg_message_size() const {
    return total_message_size_ / total_num_messages_;
  }
  size_t avg_message_bandwidth() const {
    return total_message_size_ / SecondsActive();
  }

  ssize_t avg_packed_size_reduction() const {
    return total_packed_size_reduction_ / total_num_messages_;
  }

  aos::realtime_clock::time_point channel_end_time() const {
    return channel_end_time_;
  }

  const aos::Channel *channel() const { return channel_; }

  std::string AvgLatency() {
    if (num_messages_with_remote_ == 0) {
      return "";
    }

    std::stringstream ss;
    ss << std::setprecision(3);

    const double avg_latency =
        std::chrono::duration<double, std::milli>(total_latency_).count() /
        num_messages_with_remote_;
    ss << '[' << source_node_->name()->string_view() << "->"
       << destination_node_->name()->string_view() << " " << avg_latency
       << "ms latency avg]";

    return ss.str();
  }

 private:
  // pointer to the channel for which stats are collected
  const aos::Channel *channel_;
  const aos::Configuration *config_;
  aos::SimulatedEventLoopFactory *factory_;
  const reflection::Schema *const schema_;
  aos::realtime_clock::time_point channel_end_time_ =
      aos::realtime_clock::min_time;
  aos::monotonic_clock::time_point first_message_time_ =
      // needs to be higher than time in the logfile!
      aos::monotonic_clock::max_time;
  aos::monotonic_clock::time_point current_message_time_ =
      aos::monotonic_clock::min_time;

  // Buffer of the last N seconds of messages, for N = channel_storage_duration.
  std::queue<aos::monotonic_clock::time_point>
      channel_storage_duration_messages_;
  size_t max_messages_per_period_ = 0;

  // channel stats to collect per channel
  int total_num_messages_ = 0;
  size_t max_message_size_ = 0;
  size_t total_message_size_ = 0;

  // The size reduction (in bytes) after a naive repacking. A negative number
  // indicates that the repacking generated a _bigger_ message than the
  // original message.
  ssize_t max_packed_size_reduction_ = 0;
  ssize_t total_packed_size_reduction_ = 0;

  // Count of messages which had remote timestamps
  size_t num_messages_with_remote_ = 0;
  // Sum of latencies in all messages sent on this channel if multinode
  aos::distributed_clock::duration total_latency_;

  Histogram histogram_;

  const aos::Node *source_node_ = nullptr;
  const aos::Node *destination_node_;

  const aos::FlatbufferType flatbuffer_type_;
};

struct LogfileStats {
  // All relevant stats on to logfile level
  size_t logfile_length = 0;
  int total_log_messages = 0;
  aos::realtime_clock::time_point logfile_end_time =
      aos::realtime_clock::min_time;
};

class LogStatsApplication {
 public:
  LogStatsApplication(aos::EventLoop *event_loop,
                      aos::SimulatedEventLoopFactory *log_reader_factory,
                      aos::logger::LogReader *reader)
      : event_loop_(event_loop),
        node_(event_loop_->node()),
        realtime_start_time_(reader->realtime_start_time(node_)),
        channel_stats_() {
    event_loop_->SkipTimingReport();
    event_loop_->SkipAosLog();

    // Read channel info and store in vector.
    bool found_channel = false;
    const flatbuffers::Vector<flatbuffers::Offset<aos::Channel>> *channels =
        event_loop_->configuration()->channels();

    int channel_stats_index = 0;  // Iterate through the channel_stats.
    for (flatbuffers::uoffset_t i = 0; i < channels->size(); i++) {
      const aos::Channel *channel = channels->Get(i);
      if (!aos::configuration::ChannelIsReadableOnNode(channel, node_)) {
        continue;
      }

      if (channel->name()->string_view().find(absl::GetFlag(FLAGS_name)) ==
          std::string::npos) {
        continue;
      }

      // Add a record to the stats vector.
      channel_stats_.push_back(ChannelStats{
          channel, node_, log_reader_factory,
          aos::configuration::GetSchema(event_loop_->configuration(),
                                        channel->type()->string_view())});
      // Lambda to read messages, parse for information, and update stats.
      auto watcher = [this, channel_stats_index](const aos::Context &context) {
        this->UpdateStats(context, channel_stats_index);
      };
      if (absl::GetFlag(FLAGS_print_repack_size_diffs)) {
        event_loop_->MakeRawWatcher(
            channel, std::bind(watcher, ::std::placeholders::_1));
      } else {
        event_loop_->MakeRawNoArgWatcher(channel, watcher);
      }
      channel_stats_index++;
      // TODO (Stephan): Frequency of messages per second
      // - Sliding window
      // - Max / Deviation
      found_channel = true;
    }
    if (!found_channel) {
      LOG(FATAL) << "Could not find any channels.";
    }
  }

  ~LogStatsApplication() { PrintStats(); }

  void PrintStats() {
    // Print out the stats per channel and for the logfile.
    for (size_t i = 0; i != channel_stats_.size(); i++) {
      if (!absl::GetFlag(FLAGS_excessive_size_only) ||
          (channel_stats_[i].max_message_size() * 2) <
              static_cast<size_t>(channel_stats_[i].channel()->max_size())) {
        if (channel_stats_[i].total_num_messages() > 0) {
          std::cout << channel_stats_[i].channel()->name()->string_view() << " "
                    << channel_stats_[i].channel()->type()->string_view()
                    << "\n";

          logfile_stats_.total_log_messages +=
              channel_stats_[i].total_num_messages();
          logfile_stats_.logfile_end_time =
              std::max(logfile_stats_.logfile_end_time,
                       channel_stats_[i].channel_end_time());

          if (!absl::GetFlag(FLAGS_excessive_size_only)) {
            std::cout << "   " << channel_stats_[i].total_num_messages()
                      << " msgs, " << channel_stats_[i].avg_messages_per_sec()
                      << "hz avg, " << channel_stats_[i].max_messages_per_sec()
                      << "hz max, " << channel_stats_[i].channel()->frequency()
                      << "hz configured max";
          }
          std::cout << " " << channel_stats_[i].avg_message_size()
                    << " bytes avg, "
                    << channel_stats_[i].avg_message_bandwidth()
                    << " bytes/sec avg, "
                    << channel_stats_[i].max_message_size() << " bytes max / "
                    << channel_stats_[i].channel()->max_size() << "bytes, "
                    << channel_stats_[i].Percentile() << ", "
                    << channel_stats_[i].AvgLatency();
          std::cout << std::endl;
          if (absl::GetFlag(FLAGS_print_repack_size_diffs)) {
            std::cout << "   " << channel_stats_[i].avg_packed_size_reduction()
                      << " bytes packed reduction avg, "
                      << channel_stats_[i].max_packed_size_reduction()
                      << " bytes packed reduction max";
            std::cout << std::endl;
          }
        }
      }
    }
    std::cout << std::setfill('-') << std::setw(80) << "-"
              << "\nLogfile statistics:\n"
              << "Log starts at:\t" << realtime_start_time_ << "\n"
              << "Log ends at:\t" << logfile_stats_.logfile_end_time << "\n"
              << "Log file size:\t" << logfile_stats_.logfile_length << "\n"
              << "Total messages:\t" << logfile_stats_.total_log_messages
              << "\n";
  }

  void UpdateStats(const aos::Context &context, int channel_stats_index) {
    // Run the ChannelStats update.
    channel_stats_[channel_stats_index].Add(context);
    // Update the overall logfile statistics.
    logfile_stats_.logfile_length += context.size;
  }

 private:
  aos::EventLoop *event_loop_;
  const aos::Node *node_;
  aos::realtime_clock::time_point realtime_start_time_;
  std::vector<ChannelStats> channel_stats_;
  LogfileStats logfile_stats_;
};

int main(int argc, char **argv) {
  absl::SetProgramUsageMessage(
      "Usage: \n"
      "  log_stats [args] logfile1 logfile2 ...\n"
      "This program provides statistics on a given log file. Supported "
      "statistics are:\n"
      " - Logfile start time;\n"
      " - Total messages per channel/type;\n"
      " - Max message size per channel/type;\n"
      " - Frequency of messages per second;\n"
      " - Total logfile size and number of messages;\n"
      " - Average latency per forwarded channel/type.\n"
      "Pass a logfile (path/filename) and use --name "
      "flag to specify a channel to listen on.");

  aos::InitGoogle(&argc, &argv);

  if (argc < 2) {
    LOG(FATAL) << "Expected at least 1 logfile as an argument.";
  }

  aos::logger::LogReader reader(
      aos::logger::SortParts(aos::logger::FindLogs(argc, argv)));

  aos::SimulatedEventLoopFactory event_loop_factory(reader.configuration());
  reader.RegisterWithoutStarting(&event_loop_factory);

  const aos::Node *node = nullptr;

  if (aos::configuration::MultiNode(reader.configuration())) {
    if (absl::GetFlag(FLAGS_node).empty()) {
      LOG(INFO) << "Need a --node specified.  The log file has:";
      for (const aos::Node *node : reader.LoggedNodes()) {
        LOG(INFO) << "  " << node->name()->string_view();
      }
      reader.Deregister();
      return 1;
    } else {
      node = aos::configuration::GetNode(reader.configuration(),
                                         absl::GetFlag(FLAGS_node));
    }
  }

  LogStatsApplication *log_stats_application = nullptr;

  if (absl::GetFlag(FLAGS_skip_seconds) > 0) {
    reader.SetStartTime(
        reader.realtime_start_time(node) +
        std::chrono::duration_cast<aos::realtime_clock::duration>(
            std::chrono::duration<double>(absl::GetFlag(FLAGS_skip_seconds))));
  }

  // Start the LogStatsApplication when the logfile starts.
  aos::NodeEventLoopFactory *node_factory =
      event_loop_factory.GetNodeEventLoopFactory(node);
  reader.OnStart(node, [&log_stats_application, &event_loop_factory,
                        node_factory, &reader]() {
    log_stats_application = node_factory->AlwaysStart<LogStatsApplication>(
        "log_stats_application", &event_loop_factory, &reader);
  });
  reader.OnEnd(node, [&log_stats_application, node_factory]() {
    CHECK(log_stats_application != nullptr);
    node_factory->Stop(log_stats_application);
    log_stats_application = nullptr;
  });

  if (absl::GetFlag(FLAGS_run_for) > 0.0) {
    event_loop_factory.RunFor(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(absl::GetFlag(FLAGS_run_for))));
  } else {
    event_loop_factory.Run();
  }

  std::cout << std::endl;

  // Cleanup the created processes
  reader.Deregister();

  return 0;
}
