#ifndef FRC_WPILIB_JOYSTICK_SENDER_H_
#define FRC_WPILIB_JOYSTICK_SENDER_H_

#include <atomic>

#include "aos/events/shm_event_loop.h"
#include "frc/input/joystick_state_generated.h"

namespace frc::wpilib {

class JoystickSender {
 public:
  JoystickSender(::aos::ShmEventLoop *event_loop);

 private:
  aos::ShmEventLoop *event_loop_;
  aos::Sender<frc::JoystickState> joystick_state_sender_;
  const uint16_t team_id_;
};

}  // namespace frc::wpilib

#endif  // FRC_WPILIB_JOYSTICK_SENDER_H_
