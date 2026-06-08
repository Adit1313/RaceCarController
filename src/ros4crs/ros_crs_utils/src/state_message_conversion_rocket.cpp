#include "ros_crs_utils/state_message_conversion.h"
#include <cmath>
#include <crs_msgs/rocket_input.h>
#include <crs_msgs/rocket_state.h>

#include <rocket_6_dof_model/rocket_6_dof_input.h>
#include <rocket_6_dof_model/rocket_6_dof_state.h>

namespace message_conversion
{

template <>
crs_msgs::rocket_input convertToRosInput(const crs_models::rocket_6_dof_model::rocket_6_dof_input input)
{
  crs_msgs::rocket_input ros_input;
  ros_input.thrust_magnitude = input.thrust_magnitude;
  ros_input.torque = input.torque;
  ros_input.servo_angle_1 = input.servo_angle_1;
  ros_input.servo_angle_2 = input.servo_angle_2;
  ros_input.header.stamp = ros::Time::now();
  return ros_input;
}

template <>
crs_models::rocket_6_dof_model::rocket_6_dof_input convertToCrsInput(const crs_msgs::rocket_input input)
{
  return crs_models::rocket_6_dof_model::rocket_6_dof_input(input.thrust_magnitude, input.torque, input.servo_angle_1,
                                                            input.servo_angle_2);
}

template <>
crs_models::rocket_6_dof_model::rocket_6_dof_state convertMsgToState(const crs_msgs::rocket_state& msg)
{
  crs_models::rocket_6_dof_model::rocket_6_dof_state state;
  state.position_x = msg.pos_x;
  state.position_y = msg.pos_y;
  state.position_z = msg.pos_z;
  state.velocity_x = msg.vel_x;
  state.velocity_y = msg.vel_y;
  state.velocity_z = msg.vel_z;
  state.quaternion_x = msg.quat_x;
  state.quaternion_y = msg.quat_y;
  state.quaternion_z = msg.quat_z;
  state.quaternion_w = msg.quat_w;
  state.angular_velocity_x = msg.ang_vel_x;
  state.angular_velocity_y = msg.ang_vel_y;
  state.angular_velocity_z = msg.ang_vel_z;
  state.thrust_magnitude = msg.thrust_magnitude;
  state.torque_x = msg.torque_x;
  state.servo_angle_1 = msg.servo_angle_1;
  state.servo_angle_2 = msg.servo_angle_2;

  return state;
}

template <>
crs_msgs::rocket_state convertStateToRosMsg(const crs_models::rocket_6_dof_model::rocket_6_dof_state& state,
                                            const crs_models::rocket_6_dof_model::rocket_6_dof_input& input)
{
  crs_msgs::rocket_state state_msg;
  state_msg.pos_x = state.position_x;
  state_msg.pos_y = state.position_y;
  state_msg.pos_z = state.position_z;
  state_msg.vel_x = state.velocity_x;
  state_msg.vel_y = state.velocity_y;
  state_msg.vel_z = state.velocity_z;
  state_msg.quat_x = state.quaternion_x;
  state_msg.quat_y = state.quaternion_y;
  state_msg.quat_z = state.quaternion_z;
  state_msg.quat_w = state.quaternion_w;
  state_msg.ang_vel_x = state.angular_velocity_x;
  state_msg.ang_vel_y = state.angular_velocity_y;
  state_msg.ang_vel_z = state.angular_velocity_z;
  state_msg.thrust_magnitude = state.thrust_magnitude;
  state_msg.torque_x = state.torque_x;
  state_msg.servo_angle_1 = state.servo_angle_1;
  state_msg.servo_angle_2 = state.servo_angle_2;

  state_msg.header.stamp = ros::Time::now();
  return state_msg;
}

}  // namespace message_conversion
