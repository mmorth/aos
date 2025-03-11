#include "frc/control_loops/drivetrain/drivetrain_test_lib.h"

#include <chrono>

#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "gtest/gtest.h"

#include "frc/control_loops/drivetrain/trajectory.h"
#include "frc/control_loops/state_feedback_loop.h"
#if defined(SUPPORT_PLOT)
#include "third_party/matplotlib-cpp/matplotlibcpp.h"
#endif
#include "frc/control_loops/drivetrain/test_robot/drivetrain_dog_motor_plant.h"
#include "frc/control_loops/drivetrain/test_robot/hybrid_velocity_drivetrain.h"
#include "frc/control_loops/drivetrain/test_robot/kalman_drivetrain_motor_plant.h"
#include "frc/control_loops/drivetrain/test_robot/polydrivetrain_dog_motor_plant.h"
#include "frc/wpilib/imu_batch_generated.h"

ABSL_FLAG(bool, plot, false, "If true, plot");

namespace frc::control_loops::drivetrain::testing {

namespace {
// TODO(Comran): Make one that doesn't depend on the actual values for a
// specific robot.
const constants::ShifterHallEffect kThreeStateDriveShifter{0.0, 0.0, 0.25,
                                                           0.75};

StateFeedbackPlant<4, 2, 2, double> MakePlantFromConfig(
    const DrivetrainConfig<double> &dt_config) {
  ::std::vector<
      ::std::unique_ptr<StateFeedbackPlantCoefficients<4, 2, 2, double>>>
      coefs;
  for (size_t ii = 0;
       ii < dt_config.make_drivetrain_loop().plant().coefficients_size();
       ++ii) {
    coefs.emplace_back(new StateFeedbackPlantCoefficients<4, 2, 2, double>(
        dt_config.make_drivetrain_loop().plant().coefficients(ii)));
  }
  return StateFeedbackPlant<4, 2, 2, double>(std::move(coefs));
}

}  // namespace

namespace chrono = ::std::chrono;

const DrivetrainConfig<double> &GetTestDrivetrainConfig() {
  static DrivetrainConfig<double> kDrivetrainConfig{
      ::frc::control_loops::drivetrain::ShifterType::kHallEffectShifter,
      ::frc::control_loops::drivetrain::LoopType::kClosedLoop,
      ::frc::control_loops::drivetrain::GyroType::kImuZGyro,
      ImuType::kImuFlippedX,
      ::frc::control_loops::drivetrain::test_robot::MakeDrivetrainLoop,
      ::frc::control_loops::drivetrain::test_robot::MakeVelocityDrivetrainLoop,
      ::frc::control_loops::drivetrain::test_robot::MakeKFDrivetrainLoop,
      ::frc::control_loops::drivetrain::test_robot::
          MakeHybridVelocityDrivetrainLoop,

      chrono::duration_cast<chrono::nanoseconds>(chrono::duration<double>(
          ::frc::control_loops::drivetrain::test_robot::kDt)),
      ::frc::control_loops::drivetrain::test_robot::kRobotRadius,
      ::frc::control_loops::drivetrain::test_robot::kWheelRadius,
      ::frc::control_loops::drivetrain::test_robot::kV,

      ::frc::control_loops::drivetrain::test_robot::kHighGearRatio,
      ::frc::control_loops::drivetrain::test_robot::kLowGearRatio,
      ::frc::control_loops::drivetrain::test_robot::kJ,
      ::frc::control_loops::drivetrain::test_robot::kMass,
      kThreeStateDriveShifter,
      kThreeStateDriveShifter,
      false,
      0,

      0.25,
      1.00,
      1.00,
      false,
      Eigen::Matrix3d::Identity(),
      /*is_simulated=*/true};

  return kDrivetrainConfig;
};

void DrivetrainPlant::CheckU(const Eigen::Matrix<double, 2, 1> &U) {
  EXPECT_LE(U(0, 0), U_max(0, 0) + 0.00001 + left_voltage_offset_);
  EXPECT_GE(U(0, 0), U_min(0, 0) - 0.00001 + left_voltage_offset_);
  EXPECT_LE(U(1, 0), U_max(1, 0) + 0.00001 + right_voltage_offset_);
  EXPECT_GE(U(1, 0), U_min(1, 0) - 0.00001 + right_voltage_offset_);
}

DrivetrainSimulation::DrivetrainSimulation(
    ::aos::EventLoop *event_loop, ::aos::EventLoop *imu_event_loop,
    const DrivetrainConfig<double> &dt_config,
    aos::monotonic_clock::duration imu_read_period)
    : event_loop_(event_loop),
      imu_event_loop_(imu_event_loop),
      robot_state_fetcher_(event_loop_->MakeFetcher<::aos::RobotState>("/aos")),
      drivetrain_position_sender_(
          event_loop_->MakeSender<::frc::control_loops::drivetrain::Position>(
              "/drivetrain")),
      drivetrain_truth_sender_(
          event_loop_->TryMakeSender<::frc::control_loops::drivetrain::Status>(
              "/drivetrain/truth")),
      drivetrain_output_fetcher_(
          event_loop_->MakeFetcher<::frc::control_loops::drivetrain::Output>(
              "/drivetrain")),
      drivetrain_status_fetcher_(
          event_loop_->MakeFetcher<::frc::control_loops::drivetrain::Status>(
              "/drivetrain")),
      imu_sender_(
          event_loop->TryMakeSender<::frc::IMUValuesBatch>("/drivetrain")),
      dt_config_(dt_config),
      drivetrain_plant_(MakePlantFromConfig(dt_config_)),
      velocity_drivetrain_(
          ::std::unique_ptr<StateFeedbackLoop<2, 2, 2, double,
                                              StateFeedbackHybridPlant<2, 2, 2>,
                                              HybridKalman<2, 2, 2>>>(
              new StateFeedbackLoop<2, 2, 2, double,
                                    StateFeedbackHybridPlant<2, 2, 2>,
                                    HybridKalman<2, 2, 2>>(
                  dt_config_.make_hybrid_drivetrain_velocity_loop()))) {
  if (imu_event_loop_ != nullptr) {
    CHECK(!imu_sender_);
    imu_sender_ =
        imu_event_loop_->MakeSender<::frc::IMUValuesBatch>("/localizer");
    gyro_sender_ =
        event_loop_->MakeSender<::frc::sensors::GyroReading>("/drivetrain");
  }
  CHECK(imu_sender_);
  Reinitialize();
  last_U_.setZero();
  event_loop_->AddPhasedLoop(
      [this](int) {
        // Skip this the first time.
        if (!first_) {
          Simulate();
          if (absl::GetFlag(FLAGS_plot)) {
            EXPECT_TRUE(drivetrain_status_fetcher_.Fetch());

            ::Eigen::Matrix<double, 2, 1> actual_position = GetPosition();
            actual_x_.push_back(actual_position(0));
            actual_y_.push_back(actual_position(1));

            trajectory_x_.push_back(
                drivetrain_status_fetcher_->trajectory_logging()->x());
            trajectory_y_.push_back(
                drivetrain_status_fetcher_->trajectory_logging()->y());
          }
        }
        first_ = false;
        SendPositionMessage();
        SendTruthMessage();
        SendImuMessage();
      },
      dt_config_.dt);
  event_loop_->AddPhasedLoop([this](int) { ReadImu(); }, imu_read_period);
}

void DrivetrainSimulation::Reinitialize() {
  drivetrain_plant_.mutable_X(0, 0) = 0.0;
  drivetrain_plant_.mutable_X(1, 0) = 0.0;
  drivetrain_plant_.mutable_X(2, 0) = 0.0;
  drivetrain_plant_.mutable_X(3, 0) = 0.0;
  drivetrain_plant_.mutable_Y() = drivetrain_plant_.C() * drivetrain_plant_.X();
  last_left_position_ = drivetrain_plant_.Y(0, 0);
  last_right_position_ = drivetrain_plant_.Y(1, 0);
}

void DrivetrainSimulation::SendTruthMessage() {
  if (!drivetrain_truth_sender_) {
    return;
  }
  auto builder = drivetrain_truth_sender_.MakeBuilder();
  auto status_builder =
      builder.MakeBuilder<frc::control_loops::drivetrain::Status>();
  status_builder.add_x(state_.x());
  status_builder.add_y(state_.y());
  status_builder.add_theta(state_(2));
  CHECK_EQ(builder.Send(status_builder.Finish()), aos::RawSender::Error::kOk);
}

void DrivetrainSimulation::SendPositionMessage() {
  const double left_encoder = GetLeftPosition();
  const double right_encoder = GetRightPosition();

  if (send_messages_) {
    flatbuffers::FlatBufferBuilder fbb;
    frc::control_loops::drivetrain::Position::Builder position_builder(fbb);
    position_builder.add_left_encoder(left_encoder);
    position_builder.add_right_encoder(right_encoder);
    position_builder.add_left_shifter_position(left_gear_high_ ? 1.0 : 0.0);
    position_builder.add_right_shifter_position(right_gear_high_ ? 1.0 : 0.0);
    fbb.Finish(position_builder.Finish());
    aos::FlatbufferDetachedBuffer<frc::control_loops::drivetrain::Position>
        position(fbb.Release());
    CHECK_EQ(drivetrain_position_sender_.Send(position),
             aos::RawSender::Error::kOk);
  }
}

void DrivetrainSimulation::ReadImu() {
  // Don't accumalate readings when we aren't sending them
  if (!send_messages_) {
    return;
  }

  const Eigen::Vector3d gyro =
      dt_config_.imu_transform.inverse() *
      Eigen::Vector3d(0.0, 0.0,
                      (drivetrain_plant_.X(3, 0) - drivetrain_plant_.X(1, 0)) /
                          (dt_config_.robot_radius * 2.0));

  // Acceleration due to gravity, in m/s/s.
  constexpr double kG = 9.80665;
  const Eigen::Vector3d accel =
      dt_config_.imu_transform.inverse() *
      Eigen::Vector3d(last_acceleration_.x() / kG, last_acceleration_.y() / kG,
                      last_acceleration_.z() + 1.0);
  const int64_t timestamp =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          event_loop_->monotonic_now().time_since_epoch())
          .count();
  last_yaw_rate_ = gyro.z();
  imu_readings_.push({.gyro = gyro,
                      .accel = accel,
                      .encoders = {GetLeftPosition(), GetRightPosition()},
                      .timestamp = timestamp,
                      .faulted = imu_faulted_});
}

void DrivetrainSimulation::SendImuMessage() {
  if (!send_messages_) {
    return;
  }

  flatbuffers::FlatBufferBuilder fbb;
  std::vector<flatbuffers::Offset<IMUValues>> imu_values;

  // Send all the IMU readings and pop the ones we have sent
  while (!imu_readings_.empty()) {
    const auto imu_reading = imu_readings_.front();
    imu_readings_.pop();

    frc::ADIS16470DiagStat::Builder diag_stat_builder(fbb);
    diag_stat_builder.add_clock_error(false);
    diag_stat_builder.add_memory_failure(imu_reading.faulted);
    diag_stat_builder.add_sensor_failure(false);
    diag_stat_builder.add_standby_mode(false);
    diag_stat_builder.add_spi_communication_error(false);
    diag_stat_builder.add_flash_memory_update_error(false);
    diag_stat_builder.add_data_path_overrun(false);

    const auto diag_stat_offset = diag_stat_builder.Finish();

    frc::IMUValues::Builder imu_builder(fbb);
    imu_builder.add_self_test_diag_stat(diag_stat_offset);

    imu_builder.add_gyro_x(imu_reading.gyro.x());
    imu_builder.add_gyro_y(imu_reading.gyro.y());
    imu_builder.add_gyro_z(imu_reading.gyro.z());

    imu_builder.add_accelerometer_x(imu_reading.accel.x());
    imu_builder.add_accelerometer_y(imu_reading.accel.y());
    imu_builder.add_accelerometer_z(imu_reading.accel.z());
    imu_builder.add_monotonic_timestamp_ns(imu_reading.timestamp);

    if (imu_event_loop_ != nullptr) {
      imu_builder.add_pico_timestamp_us(imu_reading.timestamp / 1000);
      imu_builder.add_data_counter(imu_data_counter_++);
      imu_builder.add_checksum_failed(false);
      imu_builder.add_left_encoder(imu_reading.encoders(0));
      imu_builder.add_right_encoder(imu_reading.encoders(1));
    }

    imu_values.push_back(imu_builder.Finish());
  }

  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<frc::IMUValues>>>
      imu_values_offset = fbb.CreateVector(imu_values);
  frc::IMUValuesBatch::Builder imu_values_batch_builder(fbb);
  imu_values_batch_builder.add_readings(imu_values_offset);
  fbb.Finish(imu_values_batch_builder.Finish());
  aos::FlatbufferDetachedBuffer<frc::IMUValuesBatch> message = fbb.Release();
  CHECK_EQ(imu_sender_.Send(message), aos::RawSender::Error::kOk);
  if (gyro_sender_) {
    auto builder = gyro_sender_.MakeBuilder();
    sensors::GyroReading::Builder reading_builder =
        builder.MakeBuilder<sensors::GyroReading>();
    reading_builder.add_angle(state_(2));
    reading_builder.add_velocity(last_yaw_rate_);
    CHECK_EQ(builder.Send(reading_builder.Finish()),
             aos::RawSender::Error::kOk);
  }
}

// Simulates the drivetrain moving for one timestep.
void DrivetrainSimulation::Simulate() {
  last_left_position_ = drivetrain_plant_.Y(0, 0);
  last_right_position_ = drivetrain_plant_.Y(1, 0);
  ::Eigen::Matrix<double, 2, 1> U = last_U_;
  if (send_messages_) {
    EXPECT_TRUE(drivetrain_output_fetcher_.Fetch());
    last_U_ << drivetrain_output_fetcher_->left_voltage(),
        drivetrain_output_fetcher_->right_voltage();
    left_gear_high_ = drivetrain_output_fetcher_->left_high();
    right_gear_high_ = drivetrain_output_fetcher_->right_high();
  } else {
    U = U.Zero();
  }
  {
    robot_state_fetcher_.Fetch();
    const double scalar = robot_state_fetcher_.get()
                              ? robot_state_fetcher_->voltage_battery() / 12.0
                              : 1.0;
    last_U_ *= scalar;
  }

  if (left_gear_high_) {
    if (right_gear_high_) {
      drivetrain_plant_.set_index(3);
    } else {
      drivetrain_plant_.set_index(2);
    }
  } else {
    if (right_gear_high_) {
      drivetrain_plant_.set_index(1);
    } else {
      drivetrain_plant_.set_index(0);
    }
  }

  U(0, 0) += drivetrain_plant_.left_voltage_offset();
  U(1, 0) += drivetrain_plant_.right_voltage_offset();
  drivetrain_plant_.Update(U);
  double dt_float = ::aos::time::DurationInSeconds(dt_config_.dt);
  const auto dynamics = [this](const ::Eigen::Matrix<double, 5, 1> &X,
                               const ::Eigen::Matrix<double, 2, 1> &U) {
    return ContinuousDynamics(velocity_drivetrain_->plant(),
                              dt_config_.Tlr_to_la(), X, U);
  };
  const Eigen::Matrix<double, 5, 1> last_state = state_;
  state_ = RungeKuttaU(dynamics, state_, U, dt_float);
  // Calculate Xdot from the actual state change rather than getting Xdot at the
  // current state_.
  // TODO(james): This seemed to help make the simulation perform better, but
  // I'm not sure that it is actually helping. Regardless, we should be
  // calculating Xdot at all the intermediate states at the 2 kHz that
  // the IMU sends at, rather than doing a sample-and-hold like we do now.
  const Eigen::Matrix<double, 5, 1> Xdot = (state_ - last_state) / dt_float;

  const double yaw_rate = Xdot(2);
  const double longitudinal_velocity = (state_(4) + state_(3)) / 2.0;
  const double centripetal_accel = yaw_rate * longitudinal_velocity;
  // TODO(james): Allow inputting arbitrary calibrations, e.g., for testing
  // situations where the IMU is not perfectly flat in the CG of the robot.
  last_acceleration_ << (Xdot(3, 0) + Xdot(4, 0)) / 2.0, centripetal_accel, 0.0;
  double accel_disturbance =
      std::sin(10.0 * 2 * M_PI *
               aos::time::DurationInSeconds(
                   event_loop_->monotonic_now().time_since_epoch())) *
      accel_sin_wave_magnitude_;
  last_acceleration_.z() += accel_disturbance;
}

void DrivetrainSimulation::MaybePlot() {
#if defined(SUPPORT_PLOT)
  if (absl::GetFlag(FLAGS_plot)) {
    std::cout << "Plotting." << ::std::endl;
    matplotlibcpp::figure();
    matplotlibcpp::plot(actual_x_, actual_y_, {{"label", "actual position"}});
    matplotlibcpp::plot(trajectory_x_, trajectory_y_,
                        {{"label", "trajectory position"}});
    matplotlibcpp::legend();
    matplotlibcpp::show();
  }
#endif
}

}  // namespace frc::control_loops::drivetrain::testing
