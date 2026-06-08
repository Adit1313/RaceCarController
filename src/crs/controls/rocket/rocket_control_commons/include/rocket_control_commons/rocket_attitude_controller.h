#ifndef ROCKET_CONTROL_COMMONS_ROCKET_ATTITUDE_CONTROLLER
#define ROCKET_CONTROL_COMMONS_ROCKET_ATTITUDE_CONTROLLER

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rocket_6_dof_model/rocket_6_dof_state.h>
// #include <rocket_6_dof_model/rocket_6_dof_params.h>

#include "rocket_control_commons/rocket_attitude_controller_config.h"

#include "commons/filter.h"

namespace crs_controls
{
/**
 * Controller to stabilize the rocket attitude.
 */
class RocketAttitudeController
{
private:
  rocket_attitude_controller_config config_;

  Eigen::Vector3d rate_integrator_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d rate_prev_ = Eigen::Vector3d::Zero();

  std::vector<double> debug_states_ = std::vector<double>(16);

  bool is_armed_ = false;
  bool is_in_autonomous_mode_ = false;

  // TODO(@naefjo): make parametric
  // H(s) = omega_0 / (omega_0 + s) with omega_0 = 100Hz, discretized with tustin at 300Hz
  Filter d_filter_x_;
  Filter d_filter_y_;
  Filter d_filter_z_;

  /**
   * Executes attitude control loop.
   *
   * P controller which uses the current attitude quaternion and the desired setpoint
   * quaternion to compute an angular rate.
   */
  Eigen::Vector3d computeAngularRateSetpoint(const Eigen::Quaterniond& current_quaternion,
                                             const Eigen::Quaterniond& desired_quaternion);

  /**
   * Executes the attitude rate control loop.
   *
   * PID controller which uses the current and desired rate to compute a torque setpoint.
   */
  Eigen::Vector3d computeTorqueSetpoint(const Eigen::Vector3d& current_rate, const Eigen::Vector3d& desired_rate);

public:
  // Constructor
  RocketAttitudeController(rocket_attitude_controller_config config);

  /**
   * Executes attitude controller, returning a new input to apply
   *
   * @param current_state describes the full current state of the rocket.
   * @param desired_state describes the desired state of the rocket. Note however that we currently
   * only use the desired attitude quaternion to produce a torque setpoint.
   *
   */
  Eigen::Vector3d getControlInput(const crs_models::rocket_6_dof_model::rocket_6_dof_state& current_state,
                                  const Eigen::Quaterniond& quaternion_setpoint);

  /**
   *  Sets the internal config
   *  Allows e.g. to update PID gains during runtime, or change target velocity.
   */
  void setConfig(rocket_attitude_controller_config config)
  {
    config_ = config;
  }

  rocket_attitude_controller_config& getConfig()
  {
    return config_;
  }

  /**
   * @brief Returns wether the controller is currently initializing
   *
   * @return true if controller is initializing
   * @return false if controller is not initializing
   */
  bool isInitializing()
  {
    return false;
  }

  /**
   * @brief Returns the loop rate of the controller.
   *
   *  @return float which specifies the loop rate of the controller in Hz
   */
  float getLoopRate()
  {
    return config_.loop_rate;
  }

  /**
   * @brief Debug function which can be used to return some useful debug info from the controller
   *
   * @return const std::vector<double> containing relevant debug info about the controller
   */
  const std::vector<double> getDebugControllerState() const
  {
    return debug_states_;
  }

  /**
   * @brief Set internal state variables of the controller
   *
   * @param internal_state internal state parameters to be set in the controller.
   * takes the format [is_armed, is_in_autonomous_mode]
   */
  void setInternalControllerState(const std::vector<bool>& internal_state);
};

}  // namespace crs_controls

#endif /* ROCKET_CONTROL_COMMONS_ROCKET_ATTITUDE_CONTROLLER */
