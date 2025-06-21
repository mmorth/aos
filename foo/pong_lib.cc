#include "foo/pong_lib.h"

#include "aos/events/event_loop.h"
#include "foo/ping_generated.h"

namespace aos {

Pong::Pong(EventLoop *event_loop)
    : event_loop_(event_loop),
      sender_(event_loop_->MakeSender<examples::Pong>("/test")) {
  event_loop_->MakeWatcher("/test", [this](const examples::Ping &ping) {
    last_value_ = ping.value();
    last_send_time_ = ping.send_time();
    aos::Sender<examples::Pong>::Builder builder = sender_.MakeBuilder();
    examples::Pong::Builder pong_builder =
        builder.MakeBuilder<examples::Pong>();
    pong_builder.add_value(ping.value());
    pong_builder.add_initial_send_time(ping.send_time());
    builder.CheckOk(builder.Send(pong_builder.Finish()));
  });
}

}  // namespace aos