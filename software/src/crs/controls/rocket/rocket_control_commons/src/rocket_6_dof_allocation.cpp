#include "rocket_control_commons/rocket_6_dof_allocation.h"

#include <cmath>

namespace crs_controls
{

double Rocket6DofAllocation::mapGimbalToServoAngle(const double gimbal_angle,
                                                   const double rotation_axis_tilt_angle) const
{
  // NOTE(@naefjo): Disclaimer, I have no idea how this works, this is shamelessly copy-pasted from lukas spannagl's
  // matlab code :)
  // Source:
  // https://gitlab.ethz.ch/ics-group/students/lukas-spannagl/general/-/blob/master/Control%20Allocation/HelperFunctions/map_angle_new.m
  // Thesis;
  // https://gitlab.ethz.ch/ics-group/students/lukas-spannagl/general/-/blob/master/MT_Report/MT_Lukas_Spannagl.pdf?ref_type=heads
  const double a = model_->gimbal_a;
  const double b = model_->gimbal_b;
  const double c = model_->gimbal_c;
  const double d = model_->gimbal_d;
  const double e = model_->gimbal_e;

  // Crazy voodoo magic
  const double g = a * a - b * b + c * c + d * d + e * e - 2 * std::cos(gimbal_angle) * c * d +
                   2 * std::cos(rotation_axis_tilt_angle) * std::sin(gimbal_angle) * c * e;
  const double h = 2 * a * (d - c * std::cos(gimbal_angle));
  const double k = 2 * a * (e + c * std::cos(rotation_axis_tilt_angle) * std::sin(gimbal_angle));
  const double value_in_sqrt = -g * g + h * h + k * k;
  const double value_in_atan = (k - std::sqrt(value_in_sqrt)) / (g - h);
  double result = 2 * std::atan(value_in_atan);

  // If we encounter a nan value in the servo angle computation, we set the angle to the maximum angle
  // possible.
  if (std ::isnan(result))
  {
    result = ((gimbal_angle > 0.0f) - (gimbal_angle < 0.0f)) * M_PI_2;
  }

  return result;
}

double Rocket6DofAllocation::constrainTorque(const double torque_x, const double thrust_magnitude)
{
  double constrainted_torque = torque_x;
  if (torque_x > config_.constraint_coefficients(0, 0) * thrust_magnitude + config_.constraint_coefficients(0, 1))
  {
    constrainted_torque =
        config_.constraint_coefficients(0, 0) * thrust_magnitude + config_.constraint_coefficients(0, 1);
  }
  else if (torque_x > config_.constraint_coefficients(1, 0) * thrust_magnitude + config_.constraint_coefficients(1, 1))
  {
    constrainted_torque =
        config_.constraint_coefficients(1, 0) * thrust_magnitude + config_.constraint_coefficients(1, 1);
  }

  if (torque_x < config_.constraint_coefficients(2, 0) * thrust_magnitude + config_.constraint_coefficients(2, 1))
  {
    constrainted_torque =
        config_.constraint_coefficients(2, 0) * thrust_magnitude + config_.constraint_coefficients(2, 1);
  }
  else if (torque_x < config_.constraint_coefficients(3, 0) * thrust_magnitude + config_.constraint_coefficients(3, 1))
  {
    constrainted_torque =
        config_.constraint_coefficients(3, 0) * thrust_magnitude + config_.constraint_coefficients(3, 1);
  }

  debug_[0] = thrust_magnitude;
  debug_[1] = constrainted_torque;
  return constrainted_torque;
}

Rocket6DofAllocation::Rocket6DofAllocation(std::shared_ptr<crs_models::rocket_6_dof_model::rocket_6_dof_params> model,
                                           rocket_6_dof_allocation_config config)
  : model_(model), config_(config), debug_(2)
{
}

crs_models::rocket_6_dof_model::rocket_6_dof_input Rocket6DofAllocation::getControlInput(const Eigen::Vector3d& force,
                                                                                         const Eigen::Vector3d& torque)
{
  const Eigen::Vector3d B_thrust_vector = Eigen::Vector3d(force.x(), force.y() - torque.z() / model_->thrust_cog_offset,
                                                          force.z() + torque.y() / model_->thrust_cog_offset);

  const double thrust_magnitude = B_thrust_vector.norm();
  const double denominator = std::sqrt(B_thrust_vector.squaredNorm() - std::pow(B_thrust_vector.y(), 2));
  const double theta_gimbal_1 = -std::asin(B_thrust_vector.z() / denominator);
  const double theta_gimbal_2 = std::asin(B_thrust_vector.y() / B_thrust_vector.norm());
  const double phi_servo_1 = mapGimbalToServoAngle(theta_gimbal_1, 0.0);
  const double phi_servo_2 = mapGimbalToServoAngle(theta_gimbal_2, theta_gimbal_1);

  const double constrainted_torque = constrainTorque(torque.x(), thrust_magnitude);
  return { thrust_magnitude, constrainted_torque, phi_servo_1, phi_servo_2 };
}

}  // namespace crs_controls
