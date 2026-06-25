#include "ros_controllers/ros_joystick_controller.h"
#include <crs_msgs/car_input.h>
#include <cmath>
#include <algorithm>

namespace ros_controllers
{

JoystickController::JoystickController(ros::NodeHandle nh, ros::NodeHandle nh_private)
  : nh_(nh), nh_private_(nh_private)
{
  input_publisher_ = nh_private_.advertise<crs_msgs::car_input>("control_input", 1);

  if (!nh_private_.getParam("controller_params/max_throttle", max_throttle_))
  {
    ROS_WARN_STREAM("JoystickController: Did not load max_throttle. Using default!");
    max_throttle_ = 1.0;
  }
  if (!nh_private_.getParam("controller_params/max_steer", max_steer_))
  {
    ROS_WARN_STREAM("JoystickController: Did not load max_steer. Using default!");
    max_steer_ = 20 * M_PI / 180.0;  // 20 deg is appropriate for the buggies
  }

  joy_sub_ = nh_.subscribe("joy", 1, &JoystickController::joystickCallback, this);
}

void JoystickController::joystickCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  // These trigger values may change from controller to controller.
  // This is currently for the Xbox 360 controller.
  double right_trigger = joy->axes[5];  // right trigger
  double left_trigger = joy->axes[2];   // left trigger
  double joy_steer = joy->axes[0];      // left stick, left/right axis

  double forward_torque = -max_throttle_ / 2.0 * (right_trigger - 1.0);
  double reverse_torque = max_throttle_ / 2.0 * (left_trigger - 1.0);

  // Torque from the two triggers can "overlap" and cancel out
  double torque = forward_torque + reverse_torque;

  crs_msgs::car_input joystick_input;

  joystick_input.torque = std::clamp(torque, -max_throttle_, max_throttle_);
  joystick_input.steer = std::clamp(max_steer_ * joy_steer, -max_steer_, max_steer_);
  joystick_input.velocity = NAN;
  joystick_input.header.stamp = ros::Time::now();
  joystick_input.header.frame_id = "crs_frame";

  input_publisher_.publish(joystick_input);
}

}  // namespace ros_controllers
