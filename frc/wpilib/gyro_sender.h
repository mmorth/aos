#ifndef FRC_WPILIB_GYRO_H_
#define FRC_WPILIB_GYRO_H_

#include <atomic>
#include <cstdint>

#include "aos/events/event_loop.h"
#include "aos/events/shm_event_loop.h"
#include "frc/input/robot_state_generated.h"
#include "frc/queues/gyro_generated.h"
#include "frc/queues/gyro_uid_generated.h"
#include "frc/wpilib/gyro_interface.h"
#include "frc/zeroing/averager.h"

namespace frc::wpilib {

// Handles reading the gyro over SPI and sending out angles on a queue.
//
// This is designed to be passed into ::std::thread's constructor so it will run
// as a separate thread.
class GyroSender {
 public:
  GyroSender(::aos::ShmEventLoop *event_loop);

  enum class State { INITIALIZING, RUNNING };

 private:
  // Initializes the gyro and then loops until Exit() is called on the event
  // loop, taking readings.
  void Loop(const int iterations);

  ::aos::EventLoop *event_loop_;
  ::aos::Fetcher<frc::RobotState> joystick_state_fetcher_;
  ::aos::Sender<::frc::sensors::Uid> uid_sender_;
  ::aos::Sender<::frc::sensors::GyroReading> gyro_reading_sender_;

  // Readings per second.
  static constexpr int kReadingRate = 200;

  GyroInterface gyro_;

  State state_ = State::INITIALIZING;

  // In radians, ready to send out.
  double angle_ = 0;
  // Calibrated offset.
  double zero_offset_ = 0;

  ::aos::monotonic_clock::time_point last_initialize_time_ =
      ::aos::monotonic_clock::min_time;
  int startup_cycles_left_ = 2 * kReadingRate;

  zeroing::Averager<double, 6 * kReadingRate> zeroing_data_;

  bool zeroed_ = false;
};

}  // namespace frc::wpilib

#endif  // FRC_WPILIB_GYRO_H_
