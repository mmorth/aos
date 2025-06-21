#include "foo/ping_lib.h"

#include "aos/json_to_flatbuffer.h"
#include "absl/flags/flag.h"
#include "absl/log/log.h"
#include "absl/log/check.h"

ABSL_FLAG(int32_t, sleep_ms, 10, "Time to sleep between pings");

namespace aos {

namespace chrono = std::chrono;

Ping::Ping(EventLoop *event_loop)
    : event_loop_(event_loop),
      sender_(event_loop_->MakeSender<examples::Ping>("/test")) {
  timer_handle_ = event_loop_->AddTimer([this]() { SendPing(); });
  timer_handle_->set_name("ping");

  event_loop_->MakeWatcher(
      "/test", [this](const examples::Pong &pong) { HandlePong(pong); });

  event_loop_->OnRun([this]() {
    timer_handle_->Schedule(event_loop_->monotonic_now(),
                            chrono::milliseconds(FLAGS_sleep_ms));
  });
}

void Ping::SendPing() {
  ++count_;
  aos::Sender<examples::Ping>::Builder builder = sender_.MakeBuilder();
  examples::Ping::Builder ping_builder = builder.MakeBuilder<examples::Ping>();
  ping_builder.add_value(count_);
  ping_builder.add_send_time(
      event_loop_->monotonic_now().time_since_epoch().count());
  builder.CheckOk(builder.Send(ping_builder.Finish()));
}

void Ping::HandlePong(const examples::Pong &pong) {
  const aos::monotonic_clock::time_point monotonic_send_time(
      chrono::nanoseconds(pong.initial_send_time()));
  const aos::monotonic_clock::time_point monotonic_now =
      event_loop_->monotonic_now();

  const chrono::nanoseconds round_trip_time =
      monotonic_now - monotonic_send_time;

  if (pong.value() == count_) {
    LOG(INFO) << "Elapsed time " << round_trip_time.count() << " ns "
              << FlatbufferToJson(&pong);
  }
}

}  // namespace aos