#include "rocket_position_pid/rocket_high_level_pid_controller.h"

namespace crs_controls
{

Eigen::Vector3d RocketHighLevelPidController::computeLinearVelocitySetpoint(Eigen::Vector3d I_current_position,
                                                                            Eigen::Vector3d I_desired_position)
{
  Eigen::Vector3d I_position_error = I_desired_position - I_current_position;
  Eigen::Vector3d I_velocity_setpoint = this->config_.position_p_gain.cwiseProduct(I_position_error);

  // Limit maximum velocity
  I_velocity_setpoint = I_velocity_setpoint.cwiseMax(-0.5).cwiseMin(0.5);
  return I_velocity_setpoint;
}

Eigen::Vector3d RocketHighLevelPidController::computeForceSetpoint(Eigen::Vector3d I_current_velocity,
                                                                   Eigen::Vector3d I_desired_velocity)
{
  Eigen::Vector3d I_velocity_error = I_desired_velocity - I_current_velocity;

  I_velocity_error_integrator_ +=
      this->config_.velocity_i_gain.cwiseProduct(I_velocity_error) / this->config_.loop_rate;

  if (!(this->is_in_autonomous_mode_))
  {
    I_velocity_error_integrator_.setZero();
  }

  I_velocity_error_integrator_ =
      I_velocity_error_integrator_.cwiseMax(-this->config_.velocity_i_limits).cwiseMin(this->config_.velocity_i_limits);

  Eigen::Vector3d I_force_setpoint =
      this->config_.velocity_p_gain.cwiseProduct(I_velocity_error) + I_velocity_error_integrator_ -
      this->config_.velocity_d_gain.cwiseProduct((I_current_velocity - I_previous_velocity_) * this->config_.loop_rate);

  I_force_setpoint *= this->model_->mass;

  I_previous_velocity_ = I_current_velocity;

  return I_force_setpoint;
}

Eigen::Quaterniond RocketHighLevelPidController::computeAttitudeSetpoint(Eigen::Vector3d thrust_vector,
                                                                         Eigen::Quaterniond q_IB_current)
{
  Eigen::Matrix3d R_IB = q_IB_current.toRotationMatrix();
  // NOTE(@naefjo): Decompose the current attitude into R(x)*R(y)*R(z), i.e. first a rotation around yaw/x followed
  // by two consecutive rotations around pitch and roll (y and z).
  Eigen::Vector3d xyz_euler_angles =
      Eigen::Vector3d(std::atan2(-R_IB(1, 2), R_IB(2, 2)),
                      std::atan2(R_IB(0, 2), std::sqrt(R_IB(1, 2) * R_IB(1, 2) + R_IB(2, 2) * R_IB(2, 2))),
                      std::atan2(-R_IB(0, 1), R_IB(0, 0)));

  // Define the B_local frame as the one whose yaw is aligned with the rocket but the yz-plane is parallel to the
  // yz-plane of the inertial frame.
  Eigen::AngleAxisd yaw_angle(xyz_euler_angles.x(), Eigen::Vector3d::UnitX());
  Eigen::Quaterniond q_IB_local = Eigen::Quaterniond(yaw_angle);
  Eigen::Vector3d Bl_thrust_vector = q_IB_local.inverse() * thrust_vector;

  // Derivation based on Bl_Thrust = R(y)*R(z)*[total_thrust, 0, 0] and then solving for angles y and z.
  double y_axis_angle = std::atan(-Bl_thrust_vector.z() / Bl_thrust_vector.x());
  double z_axis_angle = std::asin(Bl_thrust_vector.y() / Bl_thrust_vector.norm());

  y_axis_angle = std::min(std::max(y_axis_angle, -this->config_.max_angle), this->config_.max_angle);
  z_axis_angle = std::min(std::max(z_axis_angle, -this->config_.max_angle), this->config_.max_angle);

  // Reconstruct the setpoint q_IB_des by composing the yaw, roll, pitch rotations.
  return Eigen::Quaterniond(yaw_angle * Eigen::AngleAxisd(y_axis_angle, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(z_axis_angle, Eigen::Vector3d::UnitZ()));
}

RocketHighLevelPidController::RocketHighLevelPidController(
    rocket_high_level_pid_controller_config config,
    std::shared_ptr<crs_models::rocket_6_dof_model::rocket_6_dof_params> model)
  : RocketHighLevelControllerBase<rocket_high_level_pid_controller_config>(config, model)
{
  this->debug_states_ = std::vector<double>(16);
  this->config_.max_angle = this->config_.max_angle * M_PI / 180.0;
}

rocket_high_level_controller_control_input
RocketHighLevelPidController::getControlInput(const crs_models::rocket_6_dof_model::rocket_6_dof_state& current_state,
                                              const crs_models::rocket_6_dof_model::rocket_6_dof_state& desired_state)
{
  if (!(this->is_armed_))
  {
    return { Eigen::Quaterniond::Identity(), 0.0 };
  }

  Eigen::Vector3d I_current_position =
      Eigen::Vector3d(current_state.position_x, current_state.position_y, current_state.position_z);
  Eigen::Vector3d I_current_velocity =
      Eigen::Vector3d(current_state.velocity_x, current_state.velocity_y, current_state.velocity_z);
  Eigen::Vector3d I_desired_position =
      Eigen::Vector3d(desired_state.position_x, desired_state.position_y, desired_state.position_z);

  Eigen::Vector3d I_desired_velocity = computeLinearVelocitySetpoint(I_current_position, I_desired_position);
  Eigen::Vector3d I_force_setpoint = computeForceSetpoint(I_current_velocity, I_desired_velocity);

  // gravity compensation
  I_force_setpoint.x() += this->model_->gravity_constant * this->model_->mass;

  Eigen::Quaterniond q_IB_current = Eigen::Quaterniond(current_state.quaternion_w, current_state.quaternion_x,
                                                       current_state.quaternion_y, current_state.quaternion_z);

  Eigen::Quaterniond attitude_setpoint = computeAttitudeSetpoint(I_force_setpoint, q_IB_current);

  for (int i = 0; i < 4; ++i)
  {
    this->debug_states_[i] = attitude_setpoint.coeffs()[i];
  }

  for (int i = 0; i < 3; ++i)
  {
    this->debug_states_[4 + i] = I_desired_velocity[i];
    this->debug_states_[7 + i] = I_velocity_error_integrator_[i];
    this->debug_states_[10 + i] = I_force_setpoint[i];
    this->debug_states_[13 + i] = I_desired_position[i];
  }

  return { attitude_setpoint, I_force_setpoint.norm() };
}

}  // namespace crs_controls
