#ifndef ROCKPARA_ACTUATORS_ROCKPARA_ACTUATORS_H
#define ROCKPARA_ACTUATORS_ROCKPARA_ACTUATORS_H

/**
 * @file    rockpara_actuators.h
 * @author  Jason Shengjie Hu (shhu@ethz.ch)
 * @brief   ros driver for servo motors and rotors
 */

#include <array>

#include "Eigen/Core"
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"

#include "crs_msgs/rocket_input.h"

#include "rockpara_actuators/pca9685.h"

namespace ns_rockpara_actuators
{

struct rocket_actuator_parameters
{
  Eigen::Matrix<double, 10, 1> coefficients_pwm_1;
  Eigen::Matrix<double, 10, 1> coefficients_pwm_2;

  int top_propeller_spin_direction;
};

class RockparaActuators
{
public:
  RockparaActuators(ros::NodeHandle& node_handle);
  ~RockparaActuators();

private:
  // Parameters to turn thrust and torque into pwm commands for the main motor
  rocket_actuator_parameters actuator_parameters_;
  // hardware driver
  PCA9685 driver_pca9685_;
  int pwm_duty_min_servo_, pwm_duty_range_servo_, servo1_channel_, servo2_channel_;
  int pwm_duty_min_motor_, pwm_duty_range_motor_, upper_motor_channel_, lower_motor_channel_;
  bool is_armed_;
  bool manual_override_;
  float capped_throttle_;
  void initHardwareDriver();
  void setServo(const int& channel, const float& val);
  void setMotor(const int& channel, const float& val);
  void setUpperLowerMotors(const float& upper, const float& lower);

  // ros
  ros::Publisher pub_executed_;
  void setupSubscribers(ros::NodeHandle& node_handle);
  void loadParameters(ros::NodeHandle& node_handle);

  ros::Subscriber sub_manual_commands_;
  void callbackManualCommands(const std_msgs::Float32MultiArray::ConstPtr& msg);

  ros::Subscriber sub_arm_;
  void callbackArm(const std_msgs::Bool::ConstPtr& msg);

  ros::Subscriber sub_disarm_;
  void callbackDisarm(const std_msgs::Bool::ConstPtr& msg);

  ros::Subscriber sub_manual_override_;
  void callbackManualOverride(const std_msgs::Bool::ConstPtr& msg);

  ros::Subscriber sub_auto_commands_;
  void callbackAutonomousCommands(const crs_msgs::rocket_input::ConstPtr& msg);

  /**
   * @brief Constructs the feature vector of polynomials up to 3rd order between thrust and torque.
   *
   * @param thrust is the total desired thrust magnitude along the body x-axis
   * @param torque is the desired torque the coaxial propeller pair hould produce around the body x-axis
   * @return Eigen::Matrix<double, 10, 1> consisting of the polynomial variables in order of increasing power
   */
  const Eigen::Matrix<double, 10, 1> constructThrustMappingFeatures(const double thrust, const double torque) const;

  /**
   * @brief Converts the desired thrust and torque commands into the corresponding main motor pwm commands.
   *
   * @param B_thrust_x is the total desired thrust magnitude along the body x-axis
   * @param B_torque_x is the desired torque the coaxial propeller pair hould produce around the body x-axis
   * @return std::array<double, 2> consisting of pwm_1 and pwm_2
   */
  const std::array<double, 2> convertThrustAndTorqueToPWMCommands(const double B_thrust_x,
                                                                  const double B_torque_x) const;

  // helper function(s)
  int convertToPWM(const float& val, const int& min_val, const int& val_range, const float& in_min, const float& in_max,
                   const bool& inverted);
};
}  // namespace ns_rockpara_actuators

#endif
