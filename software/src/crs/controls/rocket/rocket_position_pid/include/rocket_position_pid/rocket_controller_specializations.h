#ifndef ROCKET_POSITION_PID_ROCKET_CONTROLLER_SPECIALIZATIONS
#define ROCKET_POSITION_PID_ROCKET_CONTROLLER_SPECIALIZATIONS

#include "rocket_6_dof_model/rocket_6_dof_state.h"
#include "rocket_6_dof_model/rocket_6_dof_input.h"
#include "rocket_6_dof_model/rocket_6_dof_params.h"
#include "rocket_control_commons/rocket_controller.h"
#include "rocket_control_commons/rocket_6_dof_allocation.h"
#include "rocket_position_pid/rocket_high_level_pid_controller.h"

namespace crs_controls
{
// Rocket controller config template specialications
template <>
struct rocket_controller_config<RocketHighLevelPidController, RocketAttitudeController, Rocket6DofAllocation>
{
  rocket_high_level_pid_controller_config high_level_controller_config;

  rocket_attitude_controller_config low_level_controller_config;

  rocket_6_dof_allocation_config allocation_config;
};

// Rocket controller template specializations
typedef RocketController<RocketHighLevelPidController, RocketAttitudeController, Rocket6DofAllocation>
    Rocket6DofPidController;

}  // namespace crs_controls

#endif /* ROCKET_POSITION_PID_ROCKET_CONTROLLER_SPECIALIZATIONS*/
