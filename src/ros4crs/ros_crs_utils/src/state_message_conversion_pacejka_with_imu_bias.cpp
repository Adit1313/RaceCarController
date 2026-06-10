#include "ros_crs_utils/state_message_conversion.h"
#include <cmath>
#include <crs_msgs/car_input.h>
#include <crs_msgs/car_state_cart.h>
#include <crs_msgs/car_with_imu_bias_state_cart.h>

#include <pacejka_model/pacejka_car_input.h>
#include <pacejka_model/pacejka_car_state.h>
#include <stacked_model/pacejka_imu_bias_car_state.h>
#include <stacked_model/pacejka_imu_bias_car_input.h>

namespace message_conversion
{

template <>
crs_msgs::car_input convertToRosInput(const crs_models::stacked_model::pacejka_imu_bias_car_input input)
{
  return convertToRosInput<crs_msgs::car_input, crs_models::pacejka_model::pacejka_car_input>(input);
}

template <>
crs_models::stacked_model::pacejka_imu_bias_car_input convertToCrsInput(const crs_msgs::car_input input)
{
  crs_models::pacejka_model::pacejka_car_input crs_pacejka_input =
      convertToCrsInput<crs_msgs::car_input, crs_models::pacejka_model::pacejka_car_input>(input);
  return crs_models::stacked_model::pacejka_imu_bias_car_input(crs_pacejka_input);
}

// Convert message into pacejka_car_state e.g. for controller
template <>
crs_models::pacejka_model::pacejka_car_state convertMsgToState(const crs_msgs::car_with_imu_bias_state_cart& msg)
{
  crs_models::pacejka_model::pacejka_car_state state;
  state.pos_x = msg.x;
  state.pos_y = msg.y;
  state.yaw = msg.yaw;
  state.vel_x = msg.vx_b;
  state.vel_y = msg.vy_b;
  state.yaw_rate = msg.dyaw;

  return state;
}

// Convert message into pacejka_imu_bias_car_state e.g. for estimator
template <>
crs_models::stacked_model::pacejka_imu_bias_car_state
convertMsgToState(const crs_msgs::car_with_imu_bias_state_cart& msg)
{
  crs_models::stacked_model::pacejka_imu_bias_car_state state;
  state.pos_x = msg.x;
  state.pos_y = msg.y;
  state.yaw = msg.yaw;
  state.vel_x = msg.vx_b;
  state.vel_y = msg.vy_b;
  state.yaw_rate = msg.dyaw;
  state.bias_ax = msg.bias_ax_b;
  state.bias_ay = msg.bias_ay_b;
  state.bias_dyaw = msg.bias_dyaw;

  return state;
}

template <>
crs_msgs::car_state_cart convertStateToRosMsg(const crs_models::stacked_model::pacejka_imu_bias_car_state& state,
                                              const crs_models::stacked_model::pacejka_imu_bias_car_input& input)
{
  crs_msgs::car_state_cart state_msg;
  // Position and orientation
  state_msg.x = state.pos_x;
  state_msg.y = state.pos_y;
  state_msg.yaw = state.yaw;
  state_msg.dyaw = state.yaw_rate;

  state_msg.vx_b = state.vel_x;
  state_msg.vy_b = state.vel_y;

  // Conversion to different coordinates
  state_msg.v_tot = std::hypot(state_msg.vx_b, state_msg.vy_b);
  state_msg.vx_w = state_msg.vx_b * std::cos(state_msg.yaw) - state_msg.vy_b * std::sin(state_msg.yaw);
  state_msg.vy_w = state_msg.vx_b * std::sin(state_msg.yaw) + state_msg.vy_b * std::cos(state_msg.yaw);

  state_msg.additional_data = { state.bias_ax, state.bias_ay, state.bias_dyaw };

  // WARN, not initialized:
  // state_msgs.droll;
  // state_msgs.dpitch;
  // state_msgs.roll;
  // state_msgs.pitch;
  // state_msg.z;
  // state_msg.ax_w;
  // state_msg.ay_w;
  // state_msg.az_w;
  // state_msg.ax_b;
  // state_msg.ay_b;
  // state_msg.az_b;
  // state_msg.steer;

  state_msg.header.stamp = ros::Time::now();
  return state_msg;
}

template <>
crs_msgs::car_with_imu_bias_state_cart
convertStateToRosMsg(const crs_models::stacked_model::pacejka_imu_bias_car_state& state,
                     const crs_models::stacked_model::pacejka_imu_bias_car_input& input)
{
  crs_msgs::car_with_imu_bias_state_cart state_msg;
  // Position and orientation
  state_msg.x = state.pos_x;
  state_msg.y = state.pos_y;
  state_msg.yaw = state.yaw;
  state_msg.dyaw = state.yaw_rate;

  state_msg.vx_b = state.vel_x;
  state_msg.vy_b = state.vel_y;

  state_msg.bias_ax_b = state.bias_ax;
  state_msg.bias_ay_b = state.bias_ay;
  state_msg.bias_dyaw = state.bias_dyaw;

  // Conversion to different coordinates
  state_msg.v_tot = std::hypot(state_msg.vx_b, state_msg.vy_b);
  state_msg.vx_w = state_msg.vx_b * std::cos(state_msg.yaw) - state_msg.vy_b * std::sin(state_msg.yaw);
  state_msg.vy_w = state_msg.vx_b * std::sin(state_msg.yaw) + state_msg.vy_b * std::cos(state_msg.yaw);

  // WARN, not initialized:
  // state_msgs.droll;
  // state_msgs.dpitch;
  // state_msgs.roll;
  // state_msgs.pitch;
  // state_msg.z;
  // state_msg.ax_w;
  // state_msg.ay_w;
  // state_msg.az_w;
  // state_msg.ax_b;
  // state_msg.ay_b;
  // state_msg.az_b;
  // state_msg.steer;

  state_msg.header.stamp = ros::Time::now();
  return state_msg;
}

}  // namespace message_conversion
