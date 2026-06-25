#include <Eigen/Core>
#include <vector>

#include <dynamic_models/utils/data_conversion.h>
#include "imu_bias/imu_bias_state.h"
#include "imu_bias/imu_bias_input.h"

namespace commons
{

template <>
std::vector<double*> convertToVector<crs_models::imu_bias::imu_bias_state>(crs_models::imu_bias::imu_bias_state& state)
{
  return { &state.bias_ax, &state.bias_ay, &state.bias_dyaw };
}
template <>

std::vector<const double*>
convertToConstVector<crs_models::imu_bias::imu_bias_state>(const crs_models::imu_bias::imu_bias_state& state)
{
  return { &state.bias_ax, &state.bias_ay, &state.bias_dyaw };
}

template <>
std::vector<const double*> convertToConstVector<crs_models::imu_bias::imu_bias_input>(
    [[maybe_unused]] const crs_models::imu_bias::imu_bias_input& control_input)
{
  return {};
}

template <>
crs_models::imu_bias::imu_bias_state
convertToState<crs_models::imu_bias::imu_bias_state, 3>(const Eigen::Matrix<double, 3, 1>& vector)
{
  crs_models::imu_bias::imu_bias_state state;
  state.bias_ax = vector(1, 1);
  state.bias_ay = vector(2, 2);
  state.bias_dyaw = vector(0, 0);
  return state;
}

template <>
Eigen::Matrix<double, 3, 1>
convertToEigen<crs_models::imu_bias::imu_bias_state, 3>(const crs_models::imu_bias::imu_bias_state& state)
{
  Eigen::Matrix<double, 3, 1> matrix;
  matrix(1, 0) = state.bias_ax;
  matrix(2, 0) = state.bias_ay;
  matrix(0, 0) = state.bias_dyaw;
  return matrix;
}

template <>
crs_models::imu_bias::imu_bias_state
convertToState<crs_models::imu_bias::imu_bias_state, -1>(const Eigen::Matrix<double, -1, 1>& vector)
{
  return convertToState<crs_models::imu_bias::imu_bias_state, 3>(vector);
}

template <>
Eigen::Matrix<double, -1, 1>
convertToEigen<crs_models::imu_bias::imu_bias_state, -1>(const crs_models::imu_bias::imu_bias_state& state)
{
  return convertToEigen<crs_models::imu_bias::imu_bias_state, 3>(state);
}
}  // namespace commons
