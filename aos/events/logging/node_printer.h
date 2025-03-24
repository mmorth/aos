#include "absl/flags/flag.h"

#include "aos/aos_cli_utils.h"
#include "aos/events/event_loop.h"
#include "aos/events/simulated_event_loop.h"

ABSL_DECLARE_FLAG(std::string, name);
ABSL_DECLARE_FLAG(std::string, type);
ABSL_DECLARE_FLAG(bool, print);
ABSL_DECLARE_FLAG(int64_t, max_vector_size);
ABSL_DECLARE_FLAG(bool, pretty);
ABSL_DECLARE_FLAG(double, monotonic_start_time);
ABSL_DECLARE_FLAG(double, monotonic_end_time);
ABSL_DECLARE_FLAG(bool, hex);

namespace aos::logging {

// Creates a Printer object based on the command line flags that the user
// specified.
aos::Printer MakePrinter();

// This class prints out all data from a node on a boot.
class NodePrinter {
 public:
  NodePrinter(aos::EventLoop *event_loop,
              aos::SimulatedEventLoopFactory *factory, aos::Printer *printer);

  // Tells the printer when the log starts and stops.
  void SetStarted(bool started, aos::monotonic_clock::time_point monotonic_now,
                  aos::realtime_clock::time_point realtime_now);

 private:
  struct MessageInfo {
    std::string node_name;
    std::unique_ptr<aos::RawFetcher> fetcher;
  };

  aos::SimulatedEventLoopFactory *factory_;
  aos::NodeEventLoopFactory *node_factory_;
  aos::EventLoop *event_loop_;

  std::string node_name_;

  bool started_ = false;

  aos::Printer *printer_ = nullptr;
};

}  // namespace aos::logging
