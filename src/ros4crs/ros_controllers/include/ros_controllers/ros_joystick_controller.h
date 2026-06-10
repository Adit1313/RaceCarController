#pragma once

#include "ros/ros.h"
#include <sensor_msgs/Joy.h>

namespace ros_controllers
{

class JoystickController
{
private:
  // Node handles.
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher input_publisher_;
  ros::Subscriber joy_sub_;

  /** Maximum magnitude of torque applied to drive motor. */
  double max_throttle_;
  /** Maximum steering angle commanded to car. */
  double max_steer_;

public:
  JoystickController(ros::NodeHandle nh, ros::NodeHandle nh_private);

  /**
   * @brief Processes a joystick message and publishes a car_input message
   *
   * A joystick driver receives the physical button presses and publishes
   * them on the /joy topic. This function is a callback for the /joy topic
   * and converts a joystick configuration to publish a car_input message.
   */
  void joystickCallback(const sensor_msgs::Joy::ConstPtr& joy);
};

}  // namespace ros_controllers
