#include "aos/configuration.h"
#include "foo/Ping_lib.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "absl/flags/flag.h"

// Provide a --config flag that can be used to point to the config that the
// application will use. Generally defaulted to the actual path that will be
// used on the real system (most applications default to a name of
// aos_config.json, by convention).
ABSL_FLAG(std::string, config, "foo/Pongpong_config.json", "Path to the config.");

int main(int argc, char **argv) {
  // Various common initialization steps, including command line flag parsing.
  aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  // Create a shared-memory based EventLoop using the provided config.
  // This is currently the only EventLoop implementation for using on realtime
  // systems.
  aos::ShmEventLoop event_loop(&config.message());

  aos::Pong Pong(&event_loop);

  // Actually run the EventLoop. This will block until event_loop.Exit() is
  // called or we receive a signal to exit (e.g., a Ctrl-C on the command line).
  event_loop.Run();

  return 0;
}