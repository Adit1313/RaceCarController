
#include "rocket_control_commons/rocket_attitude_controller.h"

namespace crs_controls
{

Eigen::Vector3d RocketAttitudeController::computeAngularRateSetpoint(const Eigen::Quaterniond& current_quaternion,
                                                                     const Eigen::Quaterniond& desired_quaternion)
{
  // NOTE(@naefjo): current_quaternion encodes the rotation I_r = q_IB * B_r!!!

  // NOTE(@naefjo): q_e = q_{BI} * q_(IB_des) = q_{BB_des}
  Eigen::Quaterniond error_quaternion = current_quaternion.inverse() * desired_quaternion;

  double sign_q_w = error_quaternion.w() > 0.0 ? 1.0 : -1.0;
  return 2.0 * sign_q_w * config_.attitude_p_gain.cwiseProduct(error_quaternion.vec());
}

Eigen::Vector3d RocketAttitudeController::computeTorqueSetpoint(const Eigen::Vector3d& current_rate,
                                                                const Eigen::Vector3d& desired_rate)
{
  Eigen::Vector3d rate_error = desired_rate - current_rate;

  Eigen::Vector3d filtered_rate =
      Eigen::Vector3d(d_filter_x_.process(current_rate.x()), d_filter_y_.process(current_rate.y()),
                      d_filter_z_.process(current_rate.z()));

  Eigen::Vector3d d_term = config_.rate_d_gain.cwiseProduct((filtered_rate - rate_prev_) * config_.loop_rate);

  rate_integrator_ += config_.rate_i_gain.cwiseProduct(rate_error) / config_.loop_rate;

  if (!is_in_autonomous_mode_)
  {
    rate_integrator_.setZero();
  }

  rate_integrator_ = rate_integrator_.cwiseMax(-config_.rate_i_limits).cwiseMin(config_.rate_i_limits);

  Eigen::Vector3d torque_setpoint = config_.rate_p_gain.cwiseProduct(rate_error) + rate_integrator_ - d_term;

  rate_prev_ = filtered_rate;

  return torque_setpoint;
}

RocketAttitudeController::RocketAttitudeController(rocket_attitude_controller_config config)
  : config_(config)
  , d_filter_x_(config.derivative_filter_b, config.derivative_filter_a)
  , d_filter_y_(config.derivative_filter_b, config.derivative_filter_a)
  , d_filter_z_(config.derivative_filter_b, config.derivative_filter_a)
{
  std::cout << "Low level controller config:\n" << config_ << std::endl;
}

Eigen::Vector3d
RocketAttitudeController::getControlInput(const crs_models::rocket_6_dof_model::rocket_6_dof_state& current_state,
                                          const Eigen::Quaterniond& quaternion_setpoint)
{
  if (!is_armed_)
  {
    return Eigen::Vector3d::Zero();
  }

  Eigen::Quaterniond current_state_quaternion = Eigen::Quaterniond(
      current_state.quaternion_w, current_state.quaternion_x, current_state.quaternion_y, current_state.quaternion_z);

  Eigen::Vector3d current_rate = Eigen::Vector3d(current_state.angular_velocity_x, current_state.angular_velocity_y,
                                                 current_state.angular_velocity_z);

  Eigen::Vector3d rate_setpoint = computeAngularRateSetpoint(current_state_quaternion, quaternion_setpoint);

  Eigen::Vector3d torque_setpoint = computeTorqueSetpoint(current_rate, rate_setpoint);

  for (int i = 0; i < 3; ++i)
  {
    debug_states_[i] = rate_setpoint[i];
    debug_states_[3 + i] = rate_integrator_[i];
    debug_states_[6 + i] = torque_setpoint[i];
    debug_states_[9 + i] = rate_setpoint[i] - current_rate[i];
    debug_states_[12 + i] = current_rate[i];
  }

  // NOTE(@naefjo): currently, fins are not included in the model so this is the best we can do to achieve the desired
  // torque
  return torque_setpoint;
}

void RocketAttitudeController::setInternalControllerState(const std::vector<bool>& internal_state)
{
  is_armed_ = internal_state[0];
  is_in_autonomous_mode_ = internal_state[1];
}

}  // namespace crs_controls
