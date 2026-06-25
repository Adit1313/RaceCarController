#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

#include <dynamic_models/utils/data_conversion.h>
#include "rocket_6_dof_model/rocket_6_dof_input.h"
#include "rocket_6_dof_model/rocket_6_dof_state.h"

namespace commons
{
template <>
const std::vector<casadi::MX> asCasadiSym<crs_models::rocket_6_dof_model::rocket_6_dof_state>()
{
  return { casadi::MX::sym("x"),          casadi::MX::sym("y"),          casadi::MX::sym("z"),
           casadi::MX::sym("v_x"),        casadi::MX::sym("v_y"),        casadi::MX::sym("v_z"),
           casadi::MX::sym("quat_x"),     casadi::MX::sym("quat_y"),     casadi::MX::sym("quat_z"),
           casadi::MX::sym("quat_w"),     casadi::MX::sym("angular_vx"), casadi::MX::sym("angular_vy"),
           casadi::MX::sym("angular_vz"), casadi::MX::sym("thrust_x"),   casadi::MX::sym("thrust_y"),
           casadi::MX::sym("thrust_z") };
}

template <>
const std::vector<casadi::MX> asCasadiSym<crs_models::rocket_6_dof_model::rocket_6_dof_input>()
{
  return { casadi::MX::sym("thrust_x"), casadi::MX::sym("thrust_y"), casadi::MX::sym("thrust_z") };
}

template <>
std::vector<double*> convertToVector<crs_models::rocket_6_dof_model::rocket_6_dof_state>(
    crs_models::rocket_6_dof_model::rocket_6_dof_state& state)
{
  return { &state.position_x,         &state.position_y,       &state.position_z,         &state.velocity_x,
           &state.velocity_y,         &state.velocity_z,       &state.quaternion_x,       &state.quaternion_y,
           &state.quaternion_z,       &state.quaternion_w,     &state.angular_velocity_x, &state.angular_velocity_y,
           &state.angular_velocity_z, &state.thrust_magnitude, &state.torque_x,           &state.servo_angle_1,
           &state.servo_angle_2 };
}

template <>
std::vector<const double*> convertToConstVector<crs_models::rocket_6_dof_model::rocket_6_dof_state>(
    const crs_models::rocket_6_dof_model::rocket_6_dof_state& state)
{
  return { &state.position_x,         &state.position_y,       &state.position_z,         &state.velocity_x,
           &state.velocity_y,         &state.velocity_z,       &state.quaternion_x,       &state.quaternion_y,
           &state.quaternion_z,       &state.quaternion_w,     &state.angular_velocity_x, &state.angular_velocity_y,
           &state.angular_velocity_z, &state.thrust_magnitude, &state.torque_x,           &state.servo_angle_1,
           &state.servo_angle_2 };
}

template <>
std::vector<double*> convertToVector<crs_models::rocket_6_dof_model::rocket_6_dof_input>(
    crs_models::rocket_6_dof_model::rocket_6_dof_input& input)
{
  return { &input.thrust_magnitude, &input.torque, &input.servo_angle_1, &input.servo_angle_2 };
}

template <>
std::vector<const double*> convertToConstVector<crs_models::rocket_6_dof_model::rocket_6_dof_input>(
    const crs_models::rocket_6_dof_model::rocket_6_dof_input& input)
{
  return { &input.thrust_magnitude, &input.torque, &input.servo_angle_1, &input.servo_angle_2 };
}

template <>
crs_models::rocket_6_dof_model::rocket_6_dof_state
convertToState<crs_models::rocket_6_dof_model::rocket_6_dof_state, 17>(const Eigen::Matrix<double, 17, 1>& vector)
{
  crs_models::rocket_6_dof_model::rocket_6_dof_state state;
  state.position_x = vector(0, 0);
  state.position_y = vector(1, 0);
  state.position_z = vector(2, 0);
  state.velocity_x = vector(3, 0);
  state.velocity_y = vector(4, 0);
  state.velocity_z = vector(5, 0);
  state.quaternion_x = vector(6, 0);
  state.quaternion_y = vector(7, 0);
  state.quaternion_z = vector(8, 0);
  state.quaternion_w = vector(9, 0);
  state.angular_velocity_x = vector(10, 0);
  state.angular_velocity_y = vector(11, 0);
  state.angular_velocity_z = vector(12, 0);
  state.thrust_magnitude = vector(13, 0);
  state.torque_x = vector(14, 0);
  state.servo_angle_1 = vector(15, 0);
  state.servo_angle_2 = vector(16, 0);

  return state;
}

template <>
crs_models::rocket_6_dof_model::rocket_6_dof_input
convertToState<crs_models::rocket_6_dof_model::rocket_6_dof_input, 4>(const Eigen::Matrix<double, 4, 1>& vector)
{
  crs_models::rocket_6_dof_model::rocket_6_dof_input input;
  input.thrust_magnitude = vector(0, 0);
  input.torque = vector(1, 0);
  input.servo_angle_1 = vector(2, 0);
  input.servo_angle_2 = vector(3, 0);

  return input;
}

template <>
Eigen::Matrix<double, 17, 1> convertToEigen<crs_models::rocket_6_dof_model::rocket_6_dof_state, 17>(
    const crs_models::rocket_6_dof_model::rocket_6_dof_state& state)
{
  Eigen::Matrix<double, 17, 1> matrix;
  matrix(0, 0) = state.position_x;
  matrix(1, 0) = state.position_y;
  matrix(2, 0) = state.position_z;
  matrix(3, 0) = state.velocity_x;
  matrix(4, 0) = state.velocity_y;
  matrix(5, 0) = state.velocity_z;
  matrix(6, 0) = state.quaternion_x;
  matrix(7, 0) = state.quaternion_y;
  matrix(8, 0) = state.quaternion_z;
  matrix(9, 0) = state.quaternion_w;
  matrix(10, 0) = state.angular_velocity_x;
  matrix(11, 0) = state.angular_velocity_y;
  matrix(12, 0) = state.angular_velocity_z;
  matrix(13, 0) = state.thrust_magnitude;
  matrix(14, 0) = state.torque_x;
  matrix(15, 0) = state.servo_angle_1;
  matrix(16, 0) = state.servo_angle_2;
  return matrix;
}

template <>
crs_models::rocket_6_dof_model::rocket_6_dof_state
convertToState<crs_models::rocket_6_dof_model::rocket_6_dof_state, -1>(const Eigen::Matrix<double, -1, 1>& vector)
{
  return convertToState<crs_models::rocket_6_dof_model::rocket_6_dof_state, 17>(vector);
}

template <>
Eigen::Matrix<double, -1, 1> convertToEigen<crs_models::rocket_6_dof_model::rocket_6_dof_state, -1>(
    const crs_models::rocket_6_dof_model::rocket_6_dof_state& state)
{
  return convertToEigen<crs_models::rocket_6_dof_model::rocket_6_dof_state, 17>(state);
}
}  // namespace commons
