#include <ros_crs_utils/parameter_io.h>

#include <stacked_model/pacejka_imu_bias_car_state.h>

namespace parameter_io
{

template <>
crs_models::stacked_model::pacejka_imu_bias_car_state getState(const ros::NodeHandle& nh)
{
  crs_models::stacked_model::pacejka_imu_bias_car_state state;
  std::vector<double> state_as_vec;
  if (!nh.getParam("value", state_as_vec))
  {
    ROS_WARN_STREAM("Could not load initial state from parameters.");
  }
  else
  {
    state.pos_x = state_as_vec[0];
    state.pos_y = state_as_vec[1];
    state.yaw = state_as_vec[2];
    state.vel_x = state_as_vec[3];
    state.vel_y = state_as_vec[4];
    state.yaw_rate = state_as_vec[5];
    state.bias_ax = state_as_vec[6];
    state.bias_ay = state_as_vec[7];
    state.bias_dyaw = state_as_vec[8];
  }
  return state;
}

}  // namespace parameter_io
