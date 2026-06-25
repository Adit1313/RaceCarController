#include <Eigen/Core>
#include <vector>

#include "stacked_model/pacejka_imu_bias_car_state.h"
#include "stacked_model/pacejka_imu_bias_car_input.h"
#include <dynamic_models/utils/data_conversion.h>

namespace commons
{
template <>
const std::vector<casadi::MX> asCasadiSym<crs_models::stacked_model::pacejka_imu_bias_car_state>()
{
  return { casadi::MX::sym("x"),       casadi::MX::sym("y"),       casadi::MX::sym("yaw"),
           casadi::MX::sym("v_x"),     casadi::MX::sym("v_y"),     casadi::MX::sym("yaw_rate"),
           casadi::MX::sym("bias_ax"), casadi::MX::sym("bias_ay"), casadi::MX::sym("bias_dyaw") };
}

template <>
const std::vector<casadi::MX> asCasadiSym<crs_models::stacked_model::pacejka_imu_bias_car_input>()
{
  return { casadi::MX::sym("torque"), casadi::MX::sym("steer") };
}

template <>
std::vector<double*> convertToVector<crs_models::stacked_model::pacejka_imu_bias_car_state>(
    crs_models::stacked_model::pacejka_imu_bias_car_state& state)
{
  return { &state.pos_x,    &state.pos_y,   &state.yaw,     &state.vel_x,    &state.vel_y,
           &state.yaw_rate, &state.bias_ax, &state.bias_ay, &state.bias_dyaw };
}
template <>

std::vector<const double*> convertToConstVector<crs_models::stacked_model::pacejka_imu_bias_car_state>(
    const crs_models::stacked_model::pacejka_imu_bias_car_state& state)
{
  return { &state.pos_x,    &state.pos_y,   &state.yaw,     &state.vel_x,    &state.vel_y,
           &state.yaw_rate, &state.bias_ax, &state.bias_ay, &state.bias_dyaw };
}

template <>
std::vector<double*> convertToVector<crs_models::stacked_model::pacejka_imu_bias_car_input>(
    crs_models::stacked_model::pacejka_imu_bias_car_input& input)
{
  return { &input.torque, &input.steer };
}

template <>
std::vector<const double*> convertToConstVector<crs_models::stacked_model::pacejka_imu_bias_car_input>(
    const crs_models::stacked_model::pacejka_imu_bias_car_input& input)
{
  return { &input.torque, &input.steer };
}

template <>
crs_models::stacked_model::pacejka_imu_bias_car_state
convertToState<crs_models::stacked_model::pacejka_imu_bias_car_state, 9>(const Eigen::Matrix<double, 9, 1>& vector)
{
  crs_models::stacked_model::pacejka_imu_bias_car_state state;
  state.pos_x = vector(0, 0);
  state.pos_y = vector(1, 0);
  state.yaw = vector(2, 0);
  state.vel_x = vector(3, 0);
  state.vel_y = vector(4, 0);
  state.yaw_rate = vector(5, 0);
  state.bias_ax = vector(6, 0);
  state.bias_ay = vector(7, 0);
  state.bias_dyaw = vector(8, 0);
  return state;
}

template <>
Eigen::Matrix<double, 9, 1> convertToEigen<crs_models::stacked_model::pacejka_imu_bias_car_state, 9>(
    const crs_models::stacked_model::pacejka_imu_bias_car_state& state)
{
  Eigen::Matrix<double, 9, 1> matrix;
  matrix(0, 0) = state.pos_x;
  matrix(1, 0) = state.pos_y;
  matrix(2, 0) = state.yaw;
  matrix(3, 0) = state.vel_x;
  matrix(4, 0) = state.vel_y;
  matrix(5, 0) = state.yaw_rate;
  matrix(6, 0) = state.bias_ax;
  matrix(7, 0) = state.bias_ay;
  matrix(8, 0) = state.bias_dyaw;
  return matrix;
}

template <>
crs_models::stacked_model::pacejka_imu_bias_car_state
convertToState<crs_models::stacked_model::pacejka_imu_bias_car_state, -1>(const Eigen::Matrix<double, -1, 1>& vector)
{
  return convertToState<crs_models::stacked_model::pacejka_imu_bias_car_state, 9>(vector);
}

template <>
Eigen::Matrix<double, -1, 1> convertToEigen<crs_models::stacked_model::pacejka_imu_bias_car_state, -1>(
    const crs_models::stacked_model::pacejka_imu_bias_car_state& state)
{
  return convertToEigen<crs_models::stacked_model::pacejka_imu_bias_car_state, 9>(state);
}

}  // namespace commons
