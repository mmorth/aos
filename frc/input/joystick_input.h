#ifndef AOS_INPUT_JOYSTICK_INPUT_H_
#define AOS_INPUT_JOYSTICK_INPUT_H_

#include <atomic>

#include "aos/events/event_loop.h"
#include "frc/input/driver_station_data.h"

namespace frc::input {

// A class for handling joystick packet values.
// It will call RunIteration each time a new packet is received.
//
// This class automatically handles updating ::aos::joystick_state and logging
// (at INFO) button edges.
class JoystickInput {
 public:
  explicit JoystickInput(::aos::EventLoop *event_loop)
      : event_loop_(event_loop) {
    event_loop_->MakeWatcher("/frc",
                             [this](const frc::JoystickState &joystick_state) {
                               this->HandleData(&joystick_state);
                             });
    event_loop->SetRuntimeRealtimePriority(28);
  }

 protected:
  int mode() const { return mode_; }

 private:
  void HandleData(const frc::JoystickState *joystick_state);

  // Subclasses should do whatever they want with data here.
  virtual void RunIteration(const driver_station::Data &data) = 0;

  ::aos::EventLoop *event_loop_;
  driver_station::Data data_;

  int mode_;
};

}  // namespace frc::input

#endif  // AOS_INPUT_JOYSTICK_INPUT_H_
