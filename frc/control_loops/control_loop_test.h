#ifndef AOS_CONTROLS_CONTROL_LOOP_TEST_H_
#define AOS_CONTROLS_CONTROL_LOOP_TEST_H_

#include <chrono>
#include <string_view>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "gtest/gtest.h"

#include "aos/events/simulated_event_loop.h"
#include "aos/flatbuffers.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/network/testing_time_converter.h"
#include "aos/testing/test_logging.h"
#include "aos/time/time.h"
#include "frc/input/joystick_state_generated.h"
#include "frc/input/robot_state_generated.h"

namespace frc::testing {

// Handles setting up the environment that all control loops need to actually
// run.
// This includes sending the queue messages and Clear()ing the queues when
// appropriate.
// It also includes dealing with ::aos::time.
template <typename TestBaseClass>
class ControlLoopTestTemplated : public TestBaseClass {
 public:
  ControlLoopTestTemplated(
      aos::FlatbufferDetachedBuffer<aos::Configuration> configuration,
      ::std::chrono::nanoseconds dt = kTimeTick,
      std::vector<std::vector<aos::logger::BootTimestamp>> times = {})
      : configuration_(std::move(configuration)),
        time_converter_(
            aos::configuration::NodesCount(&configuration_.message())),
        event_loop_factory_(&configuration_.message()),
        dt_(dt) {
    event_loop_factory()->SetTimeConverter(&time_converter_);

    // We need to setup the time converter before any event loop has been
    // created.  Otherwise, the event loop will read the boot uuid and we'll be
    // unable to change it.
    if (times.empty()) {
      time_converter_.StartEqual();
    } else {
      for (const std::vector<aos::logger::BootTimestamp> &time : times) {
        time_converter_.AddMonotonic(time);
      }
    }
    robot_status_event_loop_ = MakeEventLoop(
        "robot_status",
        aos::configuration::MultiNode(event_loop_factory_.configuration())
            ? aos::configuration::GetNode(event_loop_factory_.configuration(),
                                          "roborio")
            : nullptr);
    aos::testing::EnableTestLogging();
    robot_state_sender_ =
        robot_status_event_loop_->MakeSender<frc::RobotState>("/frc");
    joystick_state_sender_ =
        robot_status_event_loop_->MakeSender<frc::JoystickState>("/frc");

    // Schedule the robot status send 1 nanosecond before the loop runs.
    send_robot_state_phased_loop_ = robot_status_event_loop_->AddPhasedLoop(
        [this](int) { SendRobotState(); }, dt_,
        dt - ::std::chrono::nanoseconds(1));

    send_joystick_state_timer_ =
        robot_status_event_loop_->AddTimer([this]() { SendJoystickState(); });

    robot_status_event_loop_->OnRun([this]() {
      send_joystick_state_timer_->Schedule(
          robot_status_event_loop_->monotonic_now(), dt_);
    });
  }
  virtual ~ControlLoopTestTemplated() {}

  void set_team_id(uint16_t team_id) { team_id_ = team_id; }
  uint16_t team_id() const { return team_id_; }

  void set_alliance(frc::Alliance alliance) { alliance_ = alliance; }
  frc::Alliance alliance() const { return alliance_; }

  // Sets the enabled/disabled bit and (potentially) rebroadcasts the robot
  // state messages.
  void SetEnabled(bool enabled) {
    if (enabled_ != enabled) {
      enabled_ = enabled;
      SendJoystickState();
      SendRobotState();
      send_joystick_state_timer_->Schedule(
          robot_status_event_loop_->monotonic_now(), dt_);
    }
  }

  // Simulate a reset of the process reading sensors, which tells loops that all
  // index counts etc will be reset.
  void SimulateSensorReset() { ++reader_pid_; }

  // Sets the battery voltage in robot_state.
  void set_battery_voltage(double battery_voltage) {
    battery_voltage_ = battery_voltage;
  }

  ::std::unique_ptr<::aos::EventLoop> MakeEventLoop(
      std::string_view name, const aos::Node *node = nullptr) {
    return event_loop_factory_.MakeEventLoop(name, node);
  }

  void set_send_delay(std::chrono::nanoseconds send_delay) {
    event_loop_factory_.set_send_delay(send_delay);
  }

  void RunFor(aos::monotonic_clock::duration duration) {
    event_loop_factory_.RunFor(duration);
  }

  ::aos::monotonic_clock::time_point monotonic_now() {
    return robot_status_event_loop_->monotonic_now();
  }

  ::std::chrono::nanoseconds dt() const { return dt_; }

  const aos::Configuration *configuration() const {
    return &configuration_.message();
  }

  aos::SimulatedEventLoopFactory *event_loop_factory() {
    return &event_loop_factory_;
  }

  aos::message_bridge::TestingTimeConverter *time_converter() {
    return &time_converter_;
  }

 private:
  // Sends out all of the required queue messages.
  void SendJoystickState() {
    if (monotonic_now() >= kDSPacketTime + last_ds_time_ ||
        last_enabled_ != enabled_) {
      auto new_state = joystick_state_sender_.MakeBuilder();
      frc::JoystickState::Builder builder =
          new_state.template MakeBuilder<frc::JoystickState>();

      builder.add_fake(true);

      builder.add_enabled(enabled_);
      builder.add_autonomous(false);
      builder.add_team_id(team_id_);
      builder.add_alliance(alliance_);

      CHECK_EQ(new_state.Send(builder.Finish()), aos::RawSender::Error::kOk);

      last_ds_time_ = monotonic_now();
      last_enabled_ = enabled_;
    }
  }

  bool last_enabled_ = false;

  void SendRobotState() {
    auto new_state = robot_state_sender_.MakeBuilder();

    frc::RobotState::Builder builder =
        new_state.template MakeBuilder<frc::RobotState>();

    builder.add_reader_pid(reader_pid_);
    builder.add_outputs_enabled(enabled_);
    builder.add_browned_out(false);

    builder.add_is_3v3_active(true);
    builder.add_is_5v_active(true);
    builder.add_voltage_3v3(3.3);
    builder.add_voltage_5v(5.0);

    builder.add_voltage_roborio_in(battery_voltage_);
    builder.add_voltage_battery(battery_voltage_);

    new_state.CheckOk(new_state.Send(builder.Finish()));
  }

  static constexpr ::std::chrono::microseconds kTimeTick{5000};
  static constexpr ::std::chrono::milliseconds kDSPacketTime{20};

  aos::FlatbufferDetachedBuffer<aos::Configuration> configuration_;

  aos::message_bridge::TestingTimeConverter time_converter_;

  aos::SimulatedEventLoopFactory event_loop_factory_;

  const ::std::chrono::nanoseconds dt_;

  uint16_t team_id_ = 118;
  frc::Alliance alliance_ = frc::Alliance::kInvalid;
  int32_t reader_pid_ = 1;
  double battery_voltage_ = 12.4;

  ::aos::monotonic_clock::time_point last_ds_time_ =
      ::aos::monotonic_clock::min_time;

  bool enabled_ = false;

  ::std::unique_ptr<::aos::EventLoop> robot_status_event_loop_;

  ::aos::Sender<frc::RobotState> robot_state_sender_;
  ::aos::Sender<frc::JoystickState> joystick_state_sender_;

  ::aos::PhasedLoopHandler *send_robot_state_phased_loop_ = nullptr;
  ::aos::TimerHandler *send_joystick_state_timer_ = nullptr;
};

typedef ControlLoopTestTemplated<::testing::Test> ControlLoopTest;

template <typename TestBaseClass>
constexpr ::std::chrono::microseconds
    ControlLoopTestTemplated<TestBaseClass>::kTimeTick;

template <typename TestBaseClass>
constexpr ::std::chrono::milliseconds
    ControlLoopTestTemplated<TestBaseClass>::kDSPacketTime;

}  // namespace frc::testing

#endif  // AOS_CONTROLS_CONTROL_LOOP_TEST_H_
