#ifndef FRC_WPILIB_SENSOR_READER_H_
#define FRC_WPILIB_SENSOR_READER_H_

#include <atomic>
#include <chrono>

#include "aos/events/event_loop.h"
#include "aos/events/shm_event_loop.h"
#include "aos/stl_mutex/stl_mutex.h"
#include "aos/time/time.h"
#include "frc/control_loops/control_loops_static.h"
#include "frc/control_loops/drivetrain/drivetrain_position_static.h"
#include "frc/input/robot_state_generated.h"
#include "frc/wpilib/ahal/DigitalGlitchFilter.h"
#include "frc/wpilib/ahal/DigitalInput.h"
#include "frc/wpilib/ahal/DriverStation.h"
#include "frc/wpilib/dma.h"
#include "frc/wpilib/dma_edge_counting.h"
#include "frc/wpilib/encoder_and_potentiometer.h"

using ::aos::monotonic_clock;
namespace chrono = ::std::chrono;

namespace frc::wpilib {

class SensorReader {
 public:
  SensorReader(::aos::ShmEventLoop *event_loop);
  virtual ~SensorReader() {}

  // Updates the fast and medium encoder filter frequencies.
  void UpdateFastEncoderFilterHz(int hz);
  void UpdateMediumEncoderFilterHz(int hz);

  // Sets the left drivetrain encoder.
  void set_drivetrain_left_encoder(::std::unique_ptr<frc::Encoder> encoder);

  // Sets the right drivetrain encoder.
  void set_drivetrain_right_encoder(::std::unique_ptr<frc::Encoder> encoder);

  // Adds a sensor to DMA.
  void AddToDMA(DMASampleHandlerInterface *handler) {
    if (!dma_synchronizer_) {
      dma_synchronizer_.reset(
          new ::frc::wpilib::DMASynchronizer(std::make_unique<DMA>()));
    }
    dma_synchronizer_->Add(handler);
  }

  // Sets PWM trigger mode.  If true, synchronize the control loops with the PWM
  // pulses.  The sensors are sampled 50 uS after the falling edge of the PWM
  // pulse.
  void set_pwm_trigger(bool trigger) { pwm_trigger_ = trigger; }

  // Stops the pwm trigger on the next iteration.
  void Quit() { run_ = false; }

  virtual void RunIteration() = 0;
  // Runs the DMA iteration after the synchronizer.  This only gets run if a
  // sensor has been added to DMA.
  virtual void RunDmaIteration() {}

 protected:
  // Copies a DMAEncoder to a IndexPosition with the correct unit and direction
  // changes.
  void CopyPosition(const ::frc::wpilib::DMAEncoder &encoder,
                    ::frc::IndexPositionStatic *position,
                    double encoder_counts_per_revolution, double encoder_ratio,
                    bool reverse) {
    const double multiplier = reverse ? -1.0 : 1.0;
    position->set_encoder(multiplier *
                          encoder_translate(encoder.polled_encoder_value(),
                                            encoder_counts_per_revolution,
                                            encoder_ratio));
    position->set_latched_encoder(
        multiplier * encoder_translate(encoder.last_encoder_value(),
                                       encoder_counts_per_revolution,
                                       encoder_ratio));
    position->set_index_pulses(encoder.index_posedge_count());
  }

  // Copies a AbsoluteEncoderAndPotentiometer to a PotAndAbsolutePosition with
  // the correct unit and direction changes.
  void CopyPosition(
      const ::frc::wpilib::AbsoluteEncoderAndPotentiometer &encoder,
      ::frc::PotAndAbsolutePositionStatic *position,
      double encoder_counts_per_revolution, double encoder_ratio,
      ::std::function<double(double)> potentiometer_translate, bool reverse,
      double pot_offset) {
    const double multiplier = reverse ? -1.0 : 1.0;
    position->set_pot(multiplier * potentiometer_translate(
                                       encoder.ReadPotentiometerVoltage()) +
                      pot_offset);
    position->set_encoder(multiplier *
                          encoder_translate(encoder.ReadRelativeEncoder(),
                                            encoder_counts_per_revolution,
                                            encoder_ratio));

    position->set_absolute_encoder((reverse
                                        ? (1.0 - encoder.ReadAbsoluteEncoder())
                                        : encoder.ReadAbsoluteEncoder()) *
                                   encoder_ratio * (2.0 * M_PI));
  }

  void CopyPosition(
      const ::frc::wpilib::DMAAbsoluteEncoderAndPotentiometer &encoder,
      ::frc::PotAndAbsolutePositionStatic *position,
      double encoder_counts_per_revolution, double encoder_ratio,
      ::std::function<double(double)> potentiometer_translate, bool reverse,
      double pot_offset) {
    const double multiplier = reverse ? -1.0 : 1.0;
    position->set_pot(multiplier * potentiometer_translate(
                                       encoder.ReadPotentiometerVoltage()) +
                      pot_offset);
    position->set_encoder(multiplier *
                          encoder_translate(encoder.ReadRelativeEncoder(),
                                            encoder_counts_per_revolution,
                                            encoder_ratio));

    position->set_absolute_encoder((reverse
                                        ? (1.0 - encoder.ReadAbsoluteEncoder())
                                        : encoder.ReadAbsoluteEncoder()) *
                                   encoder_ratio * (2.0 * M_PI));
  }

  // Copies an AbsoluteEncoderAndPotentiometer to an AbsoluteAndAbsolutePosition
  // with the correct unit and direction changes.
  void CopyPosition(const ::frc::wpilib::AbsoluteAndAbsoluteEncoder &encoder,
                    ::frc::AbsoluteAndAbsolutePositionStatic *position,
                    double encoder_counts_per_revolution, double encoder_ratio,
                    double single_turn_encoder_ratio, bool reverse) {
    const double multiplier = reverse ? -1.0 : 1.0;
    position->set_encoder(multiplier *
                          encoder_translate(encoder.ReadRelativeEncoder(),
                                            encoder_counts_per_revolution,
                                            encoder_ratio));

    position->set_absolute_encoder((reverse
                                        ? (1.0 - encoder.ReadAbsoluteEncoder())
                                        : encoder.ReadAbsoluteEncoder()) *
                                   encoder_ratio * (2.0 * M_PI));

    position->set_single_turn_absolute_encoder(
        (reverse ? (1.0 - encoder.ReadSingleTurnAbsoluteEncoder())
                 : encoder.ReadSingleTurnAbsoluteEncoder()) *
        single_turn_encoder_ratio * (2.0 * M_PI));
  }

  // Copies a DMAEdgeCounter to a HallEffectAndPosition with the correct unit
  // and direction changes.
  void CopyPosition(const ::frc::wpilib::DMAEdgeCounter &counter,
                    ::frc::HallEffectAndPositionStatic *position,
                    double encoder_counts_per_revolution, double encoder_ratio,
                    bool reverse) {
    const double multiplier = reverse ? -1.0 : 1.0;
    position->set_encoder(multiplier *
                          encoder_translate(counter.polled_encoder(),
                                            encoder_counts_per_revolution,
                                            encoder_ratio));
    position->set_current(!counter.polled_value());
    position->set_posedge_count(counter.negative_count());
    position->set_negedge_count(counter.positive_count());
    position->set_posedge_value(
        multiplier * encoder_translate(counter.last_negative_encoder_value(),
                                       encoder_counts_per_revolution,
                                       encoder_ratio));
    position->set_negedge_value(
        multiplier * encoder_translate(counter.last_positive_encoder_value(),
                                       encoder_counts_per_revolution,
                                       encoder_ratio));
  }

  // Copies a Absolute Encoder with the correct unit
  // and direction changes.
  void CopyPosition(const ::frc::wpilib::AbsoluteEncoder &encoder,
                    ::frc::AbsolutePositionStatic *position,
                    double encoder_counts_per_revolution, double encoder_ratio,
                    bool reverse) {
    const double multiplier = reverse ? -1.0 : 1.0;
    position->set_encoder(multiplier *
                          encoder_translate(encoder.ReadRelativeEncoder(),
                                            encoder_counts_per_revolution,
                                            encoder_ratio));

    position->set_absolute_encoder((reverse
                                        ? (1.0 - encoder.ReadAbsoluteEncoder())
                                        : encoder.ReadAbsoluteEncoder()) *
                                   encoder_ratio * (2.0 * M_PI));
  }

  void CopyPosition(const ::frc::wpilib::DMAEncoderAndPotentiometer &encoder,
                    ::frc::PotAndIndexPositionStatic *position,
                    ::std::function<double(int32_t)> encoder_translate,
                    ::std::function<double(double)> potentiometer_translate,
                    bool reverse, double pot_offset) {
    const double multiplier = reverse ? -1.0 : 1.0;
    position->set_encoder(multiplier *
                          encoder_translate(encoder.polled_encoder_value()));
    position->set_pot(multiplier * potentiometer_translate(
                                       encoder.polled_potentiometer_voltage()) +
                      pot_offset);
    position->set_latched_encoder(
        multiplier * encoder_translate(encoder.last_encoder_value()));
    position->set_latched_pot(
        multiplier *
            potentiometer_translate(encoder.last_potentiometer_voltage()) +
        pot_offset);
    position->set_index_pulses(encoder.index_posedge_count());
  }

  // Copies a relative digital encoder.
  void CopyPosition(const ::frc::Encoder &encoder,
                    ::frc::RelativePositionStatic *position,
                    double encoder_counts_per_revolution, double encoder_ratio,
                    bool reverse) {
    const double multiplier = reverse ? -1.0 : 1.0;
    position->set_encoder(multiplier *
                          encoder_translate(encoder.GetRaw(),
                                            encoder_counts_per_revolution,
                                            encoder_ratio));
  }

  // Copies a potentiometer
  void CopyPosition(const ::frc::AnalogInput &input,
                    ::frc::RelativePositionStatic *position,
                    ::std::function<double(double)> potentiometer_translate,
                    bool reverse, double pot_offset) {
    const double multiplier = reverse ? -1.0 : 1.0;
    position->set_encoder(
        multiplier * potentiometer_translate(input.GetVoltage()) + pot_offset);
  }

  double encoder_translate(int32_t value, double counts_per_revolution,
                           double ratio) {
    return static_cast<double>(value) / counts_per_revolution * ratio *
           (2.0 * M_PI);
  }

  void SendDrivetrainPosition(
      aos::Sender<control_loops::drivetrain::PositionStatic>::StaticBuilder
          builder,
      std::function<double(double input)> velocity_translate,
      std::function<double(double input)> encoder_to_meters, bool left_inverted,
      bool right_inverted);

  ::aos::EventLoop *event_loop_;
  aos::Sender<frc::RobotState> robot_state_sender_;

  frc::DigitalGlitchFilter fast_encoder_filter_, medium_encoder_filter_;

  ::std::unique_ptr<frc::Encoder> drivetrain_left_encoder_,
      drivetrain_right_encoder_;

 private:
  // Gets called right before the DMA synchronizer is up and running.
  virtual void Start() {}

  // Sets up everything during startup.
  void DoStart();

  // Runs a single iteration.
  void Loop();

  // Returns the monotonic time of the start of the first PWM cycle.
  // Returns min_time if no start time could be calculated.
  monotonic_clock::time_point GetPWMStartTime();

  bool pwm_trigger_ = false;

  ::std::unique_ptr<::frc::wpilib::DMASynchronizer> dma_synchronizer_;

  ::std::atomic<bool> run_{true};
  ::frc::DriverStation *ds_;

  const int32_t my_pid_;

  // Pointer to the timer handler used to modify the wakeup.
  ::aos::TimerHandler *timer_handler_;

  // Last time we got called.
  ::aos::monotonic_clock::time_point last_monotonic_now_ =
      ::aos::monotonic_clock::min_time;
  // The current period.
  chrono::microseconds period_ = chrono::microseconds(5000);
};

}  // namespace frc::wpilib

#endif  // FRC_WPILIB_SENSOR_READER_H_
