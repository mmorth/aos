#ifndef FRC_AUTONOMOUS_EXTENDED_AUTONOMOUS_ACTOR_H_
#define FRC_AUTONOMOUS_EXTENDED_AUTONOMOUS_ACTOR_H_

#include <memory>

#include "aos/actions/actions.h"
#include "aos/actions/actor.h"
#include "aos/events/shm_event_loop.h"
#include "frc/autonomous/auto_generated.h"
#include "frc/autonomous/base_autonomous_actor.h"
#include "frc/control_loops/control_loops_generated.h"
#include "frc/control_loops/drivetrain/drivetrain_config.h"
#include "frc/control_loops/drivetrain/drivetrain_goal_generated.h"
#include "frc/control_loops/drivetrain/drivetrain_status_generated.h"

namespace frc::autonomous {

class UserButtonLocalizedAutonomousActor : public BaseAutonomousActor {
 public:
  explicit UserButtonLocalizedAutonomousActor(
      ::aos::EventLoop *event_loop,
      const control_loops::drivetrain::DrivetrainConfig<double> &dt_config);

  bool RunAction(
      const ::frc::autonomous::AutonomousActionParams *params) override;

 protected:
  virtual bool Run(const ::frc::autonomous::AutonomousActionParams *params) = 0;
  virtual void SendStartingPosition(const Eigen::Vector3d &start) = 0;
  virtual void Replan() = 0;
  virtual void Reset() = 0;

  void MaybeSendStartingPosition();

  ::aos::Fetcher<frc::RobotState> robot_state_fetcher_;
  ::aos::Fetcher<frc::JoystickState> joystick_state_fetcher_;

  frc::Alliance alliance_ = frc::Alliance::kInvalid;
  bool is_planned_ = false;
  bool sent_starting_position_ = false;

  std::optional<Eigen::Vector3d> starting_position_;

 private:
  void DoReplan();
  void DoReset();

  aos::TimerHandler *replan_timer_;
  aos::TimerHandler *button_poll_;

  bool user_indicated_safe_to_reset_ = false;
};

}  // namespace frc::autonomous

#endif  // FRC_AUTONOMOUS_EXTENDED_AUTONOMOUS_ACTOR_H_
