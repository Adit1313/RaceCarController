#ifndef ROCKET_POSITION_PID_ROCKET_HIGH_LEVEL_PID_CONTROLLER
#define ROCKET_POSITION_PID_ROCKET_HIGH_LEVEL_PID_CONTROLLER

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "rocket_6_dof_model/rocket_6_dof_state.h"
#include "rocket_6_dof_model/rocket_6_dof_params.h"

#include "rocket_control_commons/rocket_high_level_controller_base.h"
#include "rocket_position_pid/rocket_high_level_pid_controller_config.h"

namespace crs_controls
{
/**
 * Controller to stabilize the rocket position.
 */
class RocketHighLevelPidController : public RocketHighLevelControllerBase<rocket_high_level_pid_controller_config>
{
private:
  Eigen::Vector3d I_velocity_error_integrator_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d I_previous_velocity_ = Eigen::Vector3d::Zero();

  /**
   * @brief Position P Control policy which produces a velocity setpoint.
   *
   * @param I_current_position the current rocket position in the inertial frame
   * @param I_desired_position the desired rocket position in the inertial frame
   * @return a velocity setpoint in the inertial frame
   */
  Eigen::Vector3d computeLinearVelocitySetpoint(Eigen::Vector3d I_current_position, Eigen::Vector3d I_desired_position);

  /**
   * @brief Velocity PID Control policy which produces a force setpoint.
   *
   * @param I_current_velocity the current rocket velocity in the inertial frame
   * @param I_desired_velocity the desired rocket velocity in the inertial frame
   * @return a force setpoint in the inertial frame
   */
  Eigen::Vector3d computeForceSetpoint(Eigen::Vector3d I_current_velocity, Eigen::Vector3d I_desired_velocity);

  /**
   * @brief compute attitude setpoint for a given thrust vector based on the current attitude.
   *
   * @param thrust_vector the desired trust vector in the inertial frame
   * @param q_IB_current the current attitude of the rocket
   * @return a quaternion setpoint which corresponds to the desired thrust vector
   */
  Eigen::Quaterniond computeAttitudeSetpoint(Eigen::Vector3d thrust_vector, Eigen::Quaterniond q_IB_current);

public:
  RocketHighLevelPidController(rocket_high_level_pid_controller_config config,
                               std::shared_ptr<crs_models::rocket_6_dof_model::rocket_6_dof_params> model);

  /**
   * @brief Compute the high level control commands to control the rocket.
   *
   * @param current_state is the current system state of the rocket.
   * @param desired_state is the desired system state of the rocket.
   * @return rocket_high_level_controller_control_input depending on the system
   */
  rocket_high_level_controller_control_input
  getControlInput(const crs_models::rocket_6_dof_model::rocket_6_dof_state& current_state,
                  const crs_models::rocket_6_dof_model::rocket_6_dof_state& desired_state) override;

  /**
   * @brief Returns wether the controller is initialized
   *
   * @return true if controller is already initialized
   * @return false if controller is not initialized
   */
  bool isInitializing()
  {
    return false;
  }
};

}  // namespace crs_controls

#endif /* ROCKET_POSITION_PID_ROCKET_HIGH_LEVEL_PID_CONTROLLER */
