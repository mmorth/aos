#ifndef FRC_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_TEST_LIB_H_
#define FRC_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_TEST_LIB_H_

#include <queue>
#include <vector>

#include "aos/events/event_loop.h"
#include "frc/control_loops/control_loops_generated.h"
#include "frc/control_loops/drivetrain/drivetrain_config.h"
#include "frc/control_loops/drivetrain/drivetrain_goal_generated.h"
#include "frc/control_loops/drivetrain/drivetrain_output_generated.h"
#include "frc/control_loops/drivetrain/drivetrain_position_generated.h"
#include "frc/control_loops/drivetrain/drivetrain_status_generated.h"
#include "frc/control_loops/state_feedback_loop.h"
#include "frc/queues/gyro_generated.h"
#include "frc/wpilib/imu_batch_generated.h"

namespace frc::control_loops::drivetrain::testing {

const DrivetrainConfig<double> &GetTestDrivetrainConfig();

class DrivetrainPlant : public StateFeedbackPlant<4, 2, 2> {
 public:
  explicit DrivetrainPlant(StateFeedbackPlant<4, 2, 2> &&other)
      : StateFeedbackPlant<4, 2, 2>(::std::move(other)) {}

  void CheckU(const Eigen::Matrix<double, 2, 1> &U) override;

  double left_voltage_offset() const { return left_voltage_offset_; }
  void set_left_voltage_offset(double left_voltage_offset) {
    left_voltage_offset_ = left_voltage_offset;
  }

  double right_voltage_offset() const { return right_voltage_offset_; }
  void set_right_voltage_offset(double right_voltage_offset) {
    right_voltage_offset_ = right_voltage_offset;
  }

 private:
  double left_voltage_offset_ = 0.0;
  double right_voltage_offset_ = 0.0;
};

// Class which simulates the drivetrain and sends out queue messages containing
// the position.
class DrivetrainSimulation {
 public:
  // Constructs a motor simulation.
  // TODO(aschuh) Do we want to test the clutch one too?
  DrivetrainSimulation(::aos::EventLoop *event_loop,
                       ::aos::EventLoop *imu_event_loop,
                       const DrivetrainConfig<double> &dt_config,
                       aos::monotonic_clock::duration imu_read_period =
                           frc::controls::kLoopFrequency);
  DrivetrainSimulation(::aos::EventLoop *event_loop,
                       const DrivetrainConfig<double> &dt_config)
      : DrivetrainSimulation(event_loop, nullptr, dt_config) {}

  // Resets the plant.
  void Reinitialize();

  // Returns the position of the drivetrain.
  double GetLeftPosition() const { return drivetrain_plant_.Y(0, 0); }
  double GetRightPosition() const { return drivetrain_plant_.Y(1, 0); }

  void set_left_voltage_offset(double left_voltage_offset) {
    drivetrain_plant_.set_left_voltage_offset(left_voltage_offset);
  }
  void set_right_voltage_offset(double right_voltage_offset) {
    drivetrain_plant_.set_right_voltage_offset(right_voltage_offset);
  }

  Eigen::Matrix<double, 5, 1> state() const { return state_; }

  Eigen::Matrix<double, 5, 1> *mutable_state() { return &state_; }

  ::Eigen::Matrix<double, 2, 1> GetPosition() const {
    return state_.block<2, 1>(0, 0);
  }

  void MaybePlot();

  // Set whether we should send out the drivetrain Position and IMU messages
  // (this will keep sending the "truth" message).
  void set_send_messages(const bool send_messages) {
    if (!send_messages && !imu_readings_.empty()) {
      // Flush current IMU readings
      SendImuMessage();
    }
    send_messages_ = send_messages;
  }

  void set_imu_faulted(const bool fault_imu) { imu_faulted_ = fault_imu; }
  void set_accel_sin_magnitude(double magnitude) {
    accel_sin_wave_magnitude_ = magnitude;
  }

 private:
  struct ImuReading {
    Eigen::Vector3d gyro;
    Eigen::Vector3d accel;
    // On the 2022 robot, encoders are read as part of the same procedure that
    // reads the IMU.
    Eigen::Vector2d encoders;
    int64_t timestamp;
    bool faulted;
  };

  // Sends out the position queue messages.
  void SendPositionMessage();
  // Reads and stores the IMU state
  void ReadImu();
  // Sends out the IMU messages.
  void SendImuMessage();
  // Sends out the "truth" status message.
  void SendTruthMessage();

  // Simulates the drivetrain moving for one timestep.
  void Simulate();

  ::aos::EventLoop *event_loop_;
  ::aos::EventLoop *imu_event_loop_;
  ::aos::Fetcher<frc::RobotState> robot_state_fetcher_;

  ::aos::Sender<::frc::control_loops::drivetrain::Position>
      drivetrain_position_sender_;
  ::aos::Sender<::frc::control_loops::drivetrain::Status>
      drivetrain_truth_sender_;
  ::aos::Fetcher<::frc::control_loops::drivetrain::Output>
      drivetrain_output_fetcher_;
  ::aos::Fetcher<::frc::control_loops::drivetrain::Status>
      drivetrain_status_fetcher_;
  ::aos::Sender<::frc::IMUValuesBatch> imu_sender_;
  ::aos::Sender<::frc::sensors::GyroReading> gyro_sender_;

  bool imu_faulted_ = false;

  double last_yaw_rate_ = 0.0;
  int imu_data_counter_ = 0;
  std::queue<ImuReading> imu_readings_;

  DrivetrainConfig<double> dt_config_;

  DrivetrainPlant drivetrain_plant_;

  // This state is [x, y, theta, left_velocity, right_velocity].
  ::Eigen::Matrix<double, 5, 1> state_ = ::Eigen::Matrix<double, 5, 1>::Zero();
  ::std::unique_ptr<
      StateFeedbackLoop<2, 2, 2, double, StateFeedbackHybridPlant<2, 2, 2>,
                        HybridKalman<2, 2, 2>>>
      velocity_drivetrain_;
  double last_left_position_;
  double last_right_position_;

  Eigen::Matrix<double, 2, 1> last_U_;

  // Last robot acceleration, in m/s/s.
  Eigen::Vector3d last_acceleration_{0, 0, 1};

  bool left_gear_high_ = false;
  bool right_gear_high_ = false;
  bool first_ = true;

  ::std::vector<double> actual_x_;
  ::std::vector<double> actual_y_;
  ::std::vector<double> trajectory_x_;
  ::std::vector<double> trajectory_y_;

  bool send_messages_ = true;
  // Magnitude of sine wave to feed into the measured accelerations.
  double accel_sin_wave_magnitude_ = 0.0;
};

}  // namespace frc::control_loops::drivetrain::testing

#endif  // FRC_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_TEST_LIB_H_
