#include "rockpara_actuators/rockpara_actuators.h"
#include <vector>
#include <cmath>

#define MIN_ARMED_FRACTION 0.05

using namespace ns_rockpara_actuators;

RockparaActuators::RockparaActuators(ros::NodeHandle& node_handle)
{
  initHardwareDriver();
  loadParameters(node_handle);
  is_armed_ = false;
  capped_throttle_ = 0.0;
  pub_executed_ = node_handle.advertise<std_msgs::Float32MultiArray>("commands_executed", 10);
  setupSubscribers(node_handle);
}

RockparaActuators::~RockparaActuators()
{
  for (register int i = 0; i < 20; i++)
  {
    setUpperLowerMotors(0.0, 0.0);
    usleep(10000);
  }
  gpioTerminate();
  ROS_INFO("Motors set to 0. PIGPIO terminated.");
}

void RockparaActuators::initHardwareDriver()
{
  // initialize pigpio
  int gpio_status = gpioInitialise();

  if (gpio_status < 0)
  {
    fprintf(stderr, "pigpio initialisation failed.\n");
    exit(1);
  }

  driver_pca9685_.init();
  usleep(10000);
  if (driver_pca9685_.testConn())
  {
    ROS_INFO("Connection test passed.");
  }
  else
  {
    exit(1);
  }

  float freq = 50.0;
  ROS_INFO("Set PWM frequencey to: %f", driver_pca9685_.setFrequency(freq));
}

void RockparaActuators::loadParameters(ros::NodeHandle& node_handle)
{
  // Load torque_pwm mapping parameters
  std::vector<double> temp_pwm_1;
  std::vector<double> temp_pwm_2;
  if (!(node_handle.getParam("coefficient_vector_pwm_1", temp_pwm_1) &&
        node_handle.getParam("coefficient_vector_pwm_2", temp_pwm_2) &&
        node_handle.getParam("top_propeller_spin_direction", actuator_parameters_.top_propeller_spin_direction)))
  {
    ROS_ERROR_STREAM("could not read all the necessary rocket actuator parameters");
    ros::shutdown();
  }

  actuator_parameters_.coefficients_pwm_1 = Eigen::Map<Eigen::Matrix<double, 10, 1>>(temp_pwm_1.data());
  actuator_parameters_.coefficients_pwm_2 = Eigen::Map<Eigen::Matrix<double, 10, 1>>(temp_pwm_2.data());

  // Load driver parameters
  node_handle.getParam("servo1_channel", servo1_channel_);
  node_handle.getParam("servo2_channel", servo2_channel_);
  node_handle.getParam("pwm_duty_min_servo", pwm_duty_min_servo_);
  int pwm_duty_max_servo;
  node_handle.getParam("pwm_duty_max_servo", pwm_duty_max_servo);
  pwm_duty_range_servo_ = pwm_duty_max_servo - pwm_duty_min_servo_;

  node_handle.getParam("upper_motor_channel", upper_motor_channel_);
  node_handle.getParam("lower_motor_channel", lower_motor_channel_);
  node_handle.getParam("pwm_duty_min_motor", pwm_duty_min_motor_);
  int pwm_duty_max_motor;
  node_handle.getParam("pwm_duty_max_motor", pwm_duty_max_motor);
  pwm_duty_range_motor_ = pwm_duty_max_motor - pwm_duty_min_motor_;

  ROS_INFO("Servos {%d,%d}: PWM duty [%d,%d]", servo1_channel_, servo2_channel_, pwm_duty_min_servo_,
           pwm_duty_max_servo);
  ROS_INFO("Motors {%d,%d}: PWM duty [%d,%d]", upper_motor_channel_, lower_motor_channel_, pwm_duty_min_motor_,
           pwm_duty_max_motor);
}

void RockparaActuators::setupSubscribers(ros::NodeHandle& node_handle)
{
  sub_manual_commands_ = node_handle.subscribe("manual_commands", 1, &RockparaActuators::callbackManualCommands, this);
  sub_arm_ = node_handle.subscribe("arm", 5, &RockparaActuators::callbackArm, this);
  sub_disarm_ = node_handle.subscribe("disarm", 1, &RockparaActuators::callbackDisarm, this);
  sub_manual_override_ = node_handle.subscribe("manual_override", 1, &RockparaActuators::callbackManualOverride, this);
  sub_auto_commands_ = node_handle.subscribe("auto_commands", 1, &RockparaActuators::callbackAutonomousCommands, this);
}

int RockparaActuators::convertToPWM(const float& val, const int& min_out, const int& out_range,
                                    const float& in_min = 0.0, const float& in_max = 1.0, const bool& inverted = false)
{
  if (val < in_min || val > in_max)
  {
    // invalid data
    ROS_WARN("Invalid value: %f (range allowed %f to %f)", val, in_min, in_max);
    return min_out;
  }

  float fraction = (val - in_min) / (in_max - in_min);
  if (inverted)
    fraction = 1.0 - fraction;

  float length = fraction * (float)out_range;
  int duty = (int)length + min_out;
  // ROS_INFO("Setting val [%f], to PWM [%d]", val, duty);
  return duty;
}

void RockparaActuators::setServo(const int& channel, const float& val)
{
  // fixme
  bool inverted = false;
  if (channel == servo1_channel_)
    inverted = true;
  int duty = convertToPWM(val, pwm_duty_min_servo_, pwm_duty_range_servo_, -1.0, 1.0, inverted);
  driver_pca9685_.setPWM(channel, duty);
}

void RockparaActuators::setMotor(const int& channel, const float& val)
{
  int duty = convertToPWM(val, pwm_duty_min_motor_, pwm_duty_range_motor_);
  driver_pca9685_.setPWM(channel, duty);
}

void RockparaActuators::setUpperLowerMotors(const float& upper, const float& lower)
{
  setMotor(upper_motor_channel_, upper);
  setMotor(lower_motor_channel_, lower);
}

void RockparaActuators::callbackManualCommands(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  float front = msg->data[0];
  float side = msg->data[1];
  float throttle_total = msg->data[2];
  float throttle_upper = msg->data[3];
  float throttle_lower = msg->data[4];

  capped_throttle_ = throttle_total;

  // TODO: clamp ranges and warn if out-of-range
  // front = std::clamp(front, 0.0, 1.0);

  if (throttle_total < MIN_ARMED_FRACTION)
  {
    throttle_upper = MIN_ARMED_FRACTION;
    throttle_lower = MIN_ARMED_FRACTION;
  }

  // ROS_INFO("Msg received [%f]", throttle_upper);
  // ROS_INFO("Setting val [%f], to PWM [%d]", val, duty);

  if (manual_override_)
  {
    // servos
    setServo(servo1_channel_, front);
    setServo(servo2_channel_, side);

    // motors
    if (is_armed_)
    {
      // ROS_INFO("Armed, set %f, %f", throttle_upper, throttle_lower);
      setUpperLowerMotors(throttle_upper, throttle_lower);
    }
  }
}

void RockparaActuators::callbackArm(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data)
  {
    setUpperLowerMotors(MIN_ARMED_FRACTION, MIN_ARMED_FRACTION);
    is_armed_ = true;
    if (!is_armed_)
      ROS_INFO("Armed");
  }
}

void RockparaActuators::callbackDisarm(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data)
  {
    setUpperLowerMotors(0.0, 0.0);
    is_armed_ = false;
    if (is_armed_)
      ROS_INFO("Disarmed");
  }
}

void RockparaActuators::callbackManualOverride(const std_msgs::Bool::ConstPtr& msg)
{
  manual_override_ = msg->data;
}

void RockparaActuators::callbackAutonomousCommands(const crs_msgs::rocket_input::ConstPtr& msg)
{
  if (manual_override_)
    return;

  // use capped throttle value
  const float front = std::max(std::min(msg->servo_angle_1 / M_PI_2, 1.0), -1.0);
  const float side = std::max(std::min(msg->servo_angle_2 / M_PI_2, 1.0), -1.0);
  const float B_thrust_x = msg->thrust_magnitude;
  const float B_torque_x = msg->torque;
  const std::array<double, 2> main_pwm_commands = convertThrustAndTorqueToPWMCommands(B_thrust_x, B_torque_x);

  float throttle_upper = main_pwm_commands[0];
  float throttle_lower = main_pwm_commands[1];

  float throttle_total = (throttle_upper + throttle_lower) / 2.0;

  // servos
  setServo(servo1_channel_, front);
  setServo(servo2_channel_, side);
  if (is_armed_)
  {
    if (throttle_total > capped_throttle_)
    {
      float ratio = capped_throttle_ / throttle_total;
      // ROS_INFO("Cap %f to %f", throttle_total, capped_throttle_);
      throttle_total = capped_throttle_;
      throttle_upper *= ratio;
      throttle_lower *= ratio;
    }
    setUpperLowerMotors(throttle_upper, throttle_lower);
  }
  std_msgs::Float32MultiArray msg_exec;
  msg_exec.data.push_back(front);
  msg_exec.data.push_back(side);
  msg_exec.data.push_back(is_armed_ ? throttle_total : 0.0);
  msg_exec.data.push_back(is_armed_ ? throttle_upper : 0.0);
  msg_exec.data.push_back(is_armed_ ? throttle_lower : 0.0);

  pub_executed_.publish(msg_exec);
}

const std::array<double, 2> RockparaActuators::convertThrustAndTorqueToPWMCommands(const double B_thrust_x,
                                                                                   const double B_torque_x) const
{
  const Eigen::Matrix<double, 10, 1> features =
      constructThrustMappingFeatures(B_thrust_x, B_torque_x * actuator_parameters_.top_propeller_spin_direction);

  double pwm_1 = features.dot(actuator_parameters_.coefficients_pwm_1);
  double pwm_2 = features.dot(actuator_parameters_.coefficients_pwm_2);

  if (pwm_1 > 1.0 || pwm_1 < 0.0)
  {
    pwm_1 = std::max(std::min(pwm_1, 1.0), 0.0);
    ROS_WARN_STREAM_THROTTLE(1, "PWM 1 is outside the allowed bounds. limiting to range [0, 1]");
  }
  if (pwm_2 > 1.0 || pwm_2 < 0.0)
  {
    pwm_2 = std::max(std::min(pwm_2, 1.0), 0.0);
    ROS_WARN_STREAM_THROTTLE(1, "PWM 2 is outside the allowed bounds. limiting to range [0, 1]");
  }

  return { pwm_1, pwm_2 };
}

const Eigen::Matrix<double, 10, 1> RockparaActuators::constructThrustMappingFeatures(const double thrust,
                                                                                     const double torque) const
{
  const double thrust_sq = thrust * thrust;
  const double torque_sq = torque * torque;
  Eigen::Matrix<double, 10, 1> features;
  features << 1.0, thrust, torque, thrust_sq, thrust * torque, torque_sq, thrust_sq * thrust, thrust_sq * torque,
      thrust * torque_sq, torque_sq * torque;
  return features;
}
