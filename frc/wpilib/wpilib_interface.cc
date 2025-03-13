#include "frc/wpilib/wpilib_interface.h"

#include "aos/events/event_loop.h"
#include "aos/logging/logging.h"
#include "frc/input/robot_state_generated.h"
#include "frc/wpilib/ahal/Utility.h"
#include "hal/HAL.h"

namespace frc::wpilib {

flatbuffers::Offset<frc::RobotState> PopulateRobotState(
    aos::Sender<frc::RobotState>::Builder *builder, int32_t my_pid) {
  int32_t status = 0;

  frc::RobotState::Builder robot_state_builder =
      builder->MakeBuilder<frc::RobotState>();

  robot_state_builder.add_reader_pid(my_pid);
  robot_state_builder.add_outputs_enabled(HAL_GetSystemActive(&status));
  robot_state_builder.add_browned_out(HAL_GetBrownedOut(&status));

  robot_state_builder.add_is_3v3_active(HAL_GetUserActive3V3(&status));
  robot_state_builder.add_is_5v_active(HAL_GetUserActive5V(&status));
  robot_state_builder.add_voltage_3v3(HAL_GetUserVoltage3V3(&status));
  robot_state_builder.add_voltage_5v(HAL_GetUserVoltage5V(&status));

  robot_state_builder.add_voltage_roborio_in(HAL_GetVinVoltage(&status));
  robot_state_builder.add_voltage_battery(HAL_GetVinVoltage(&status));
  robot_state_builder.add_user_button(frc::GetUserButton());

  if (status != 0) {
    AOS_LOG(FATAL, "Failed to get robot state: %d\n", status);
  }

  return robot_state_builder.Finish();
}

}  // namespace frc::wpilib
