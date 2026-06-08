#include "ros_simulator/ros_rocket_simulator.h"
#include <geometry_msgs/TransformStamped.h>
#include <rocket_6_dof_sensor_model/full_state_sensor_model.h>
#include <rocket_6_dof_sensor_model/imu_sensor_model.h>
#include <rocket_6_dof_sensor_model/mocap_pose_sensor_model.h>
#include <rocket_6_dof_sensor_model/mocap_twist_sensor_model.h>
#include <ros/console.h>
#include <ros_crs_utils/parameter_io.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>

namespace ros_simulator
{

RocketSimulator::RocketSimulator(ros::NodeHandle nh, ros::NodeHandle nh_private) : nh_(nh), nh_private_(nh_private)
{
  // Load model parameters from rosparameters (e.g. model_params.yaml  in rocket_6_dof_model)
  crs_models::rocket_6_dof_model::rocket_6_dof_params params;
  parameter_io::getModelParams<crs_models::rocket_6_dof_model::rocket_6_dof_params>(
      ros::NodeHandle(nh_, "model/model_params/"), params);

  Eigen::Matrix<double, crs_models::rocket_6_dof_model::rocket_6_dof_state::NX,
                crs_models::rocket_6_dof_model::rocket_6_dof_state::NX>
      Q = Eigen::Matrix<double, crs_models::rocket_6_dof_model::rocket_6_dof_state::NX,
                        crs_models::rocket_6_dof_model::rocket_6_dof_state::NX>::Identity();
  if (!parameter_io::getMatrixFromParams<crs_models::rocket_6_dof_model::rocket_6_dof_state::NX,
                                         crs_models::rocket_6_dof_model::rocket_6_dof_state::NX>(
          ros::NodeHandle(nh_, "model/Q"), Q))  // Load Q from rosparameters
                                                // (e.g. model_params.yaml  in
                                                // rocket_6_dof_model)
  {
    ROS_WARN_STREAM("Could not load Q Matrix for provided Model. Using to Identity Matrix");
  }
  // Create a rocket model
  model_ = std::make_unique<crs_models::rocket_6_dof_model::DiscreteRocket6DofModel>(params, Q);

  gt_state_pub = nh_private_.advertise<crs_msgs::rocket_state>("gt_state", 10);
  control_input_sub_ = nh_private_.subscribe("control_input", 10, &RocketSimulator::inputCallback, this);

  std::vector<double> initial_input;
  if (nh_private_.getParam("initial_input", initial_input))  // Load initial input from rosparameters (e.g.
                                                             // rocket_simulator.yaml)
  {
    last_input_.thrust_magnitude = initial_input[0];
    last_input_.torque = initial_input[1];
    last_input_.servo_angle_1 = initial_input[2];
    last_input_.servo_angle_1 = initial_input[3];
  }
  else
  {
    ROS_WARN_STREAM("No initial input set. Setting thrust vector to 0");
    last_input_.thrust_magnitude = 0.0;
    last_input_.torque = 0.0;
    last_input_.servo_angle_1 = 0.0;
    last_input_.servo_angle_1 = 0.0;
  }

  std::vector<double> initial_state;
  if (nh_private_.getParam("initial_state", initial_state))  // Load initial state from rosparameters (e.g.
                                                             // rocket_simulator.yaml)
  {
    current_state_.position_x = initial_state[0];
    current_state_.position_y = initial_state[1];
    current_state_.position_z = initial_state[2];
    current_state_.velocity_x = initial_state[3];
    current_state_.velocity_y = initial_state[4];
    current_state_.velocity_z = initial_state[5];
    current_state_.quaternion_x = initial_state[6];
    current_state_.quaternion_y = initial_state[7];
    current_state_.quaternion_z = initial_state[8];
    current_state_.quaternion_w = initial_state[9];
    current_state_.angular_velocity_x = initial_state[10];
    current_state_.angular_velocity_y = initial_state[11];
    current_state_.angular_velocity_z = initial_state[12];
    current_state_.thrust_magnitude = initial_state[13];
    current_state_.torque_x = initial_state[14];
    current_state_.servo_angle_1 = initial_state[15];
    current_state_.servo_angle_2 = initial_state[16];
  }
  else
  {
    ROS_WARN_STREAM("No initial state set. Using 0 and unit quaternion.");
    current_state_.position_x = 0.0;
    current_state_.position_y = 0.0;
    current_state_.position_z = 0.0;
    current_state_.velocity_x = 0.0;
    current_state_.velocity_y = 0.0;
    current_state_.velocity_z = 0.0;
    current_state_.quaternion_x = 0.0;
    current_state_.quaternion_y = 0.0;
    current_state_.quaternion_z = 0.0;
    current_state_.quaternion_w = 1.0;
    current_state_.angular_velocity_x = 0.0;
    current_state_.angular_velocity_y = 0.0;
    current_state_.angular_velocity_z = 0.0;
    current_state_.thrust_magnitude = 0.0;
    current_state_.torque_x = 0.0;
    current_state_.servo_angle_1 = 0.0;
    current_state_.servo_angle_2 = 0.0;
  }
}

void RocketSimulator::printConfig()
{
  ROS_INFO_STREAM("Started Rocket Simulator!\n"
                  << "Initial State:\n"
                  << current_state_ << "\nInitial Input " << last_input_ << "\n\nUsing Noise:\n"
                  << (noise_model_ ? "Yes" : "No") << "\n\nProcess Noise Cov Matrix:\n"
                  << model_->getQ());

  ROS_INFO_STREAM("Registered Sensor Models:\n");
  for (const auto sensor_model : sensor_models_)
  {
    ROS_INFO_STREAM("Key: " << sensor_model->getKey() << "\n R:\n" << sensor_model->getR());
  }
}

/**
 * @brief Add process noise model to the simulator
 *
 * @param noise_model
 */
void RocketSimulator::registerNoiseModel(std::shared_ptr<NoiseModel> noise_model)
{
  noise_model_ = noise_model;
}

/**
 * @brief Propagates model in simulation (adds process noise if needed)
 *
 * @param timestep time for which model is propagated
 */
void RocketSimulator::advanceState(double timestep)
{
  if (!got_init_input)
    return;

  current_state_ = model_->applyModel(current_state_, last_input_, timestep);
  if (noise_model_)
  {
    // Sample noise. Note Q is defined in noise per time
    Eigen::MatrixXd noise = noise_model_->sampleNoiseFromCovMatrix(timestep * model_->getQ());
    current_state_.position_x += noise(0);
    current_state_.position_y += noise(1);
    current_state_.position_y += noise(2);
    current_state_.velocity_x += noise(3);
    current_state_.velocity_y += noise(4);
    current_state_.velocity_z += noise(5);
    // TODO(@naefjo): The following four lines dont make much sense. We should consider
    // adding the noise in a way which respects the geometry of the state manifold.
    current_state_.quaternion_x += noise(6);
    current_state_.quaternion_y += noise(7);
    current_state_.quaternion_z += noise(8);
    current_state_.quaternion_w += noise(9);
    current_state_.angular_velocity_x += noise(10);
    current_state_.angular_velocity_y += noise(11);
    current_state_.angular_velocity_z += noise(12);
    current_state_.thrust_magnitude += noise(13);
    current_state_.torque_x += noise(14);
    current_state_.servo_angle_1 += noise(15);
    current_state_.servo_angle_2 += noise(16);
  }
}

/**
 * @brief sets inputs received from ros message
 *
 * @param input
 */
void RocketSimulator::inputCallback(crs_msgs::rocket_inputConstPtr input)
{
  last_input_.thrust_magnitude = input->thrust_magnitude;
  last_input_.torque = input->torque;
  last_input_.servo_angle_1 = input->servo_angle_1;
  last_input_.servo_angle_2 = input->servo_angle_2;
  got_init_input = true;
}

/**
 * @brief publish ground truth state of simulation model
 *
 */
void RocketSimulator::publishStates()
{
  gt_state_pub.publish(message_conversion::convertStateToRosMsg<crs_msgs::rocket_state>(current_state_, last_input_));
}

/**
 * @brief Add sensor model to list of sensor models
 *
 * @param sensor_model
 */
void RocketSimulator::registerSensorModel(
    std::shared_ptr<crs_sensor_models::SensorModel<crs_models::rocket_6_dof_model::rocket_6_dof_state,
                                                   crs_models::rocket_6_dof_model::rocket_6_dof_input>>
        sensor_model,
    double delay)
{
  sensor_models_.push_back(sensor_model);  // append sensor_model to list of all sensor models
  std::string key = sensor_model->getKey();

  if (key == crs_sensor_models::rocket_6_dof_sensor_models::FullStateSensorModel::SENSOR_KEY)
  {
    // TODO currently, there can only be e.g. 1 IMU as they would publish on the same key. Change this
    // publish sensor measurements to topic "key"
    auto full_state_pub = nh_private_.advertise<crs_msgs::rocket_state>(key, 10);
    sensor_models_pub_.push_back(std::move(DelayedPublisher(nh_private_, full_state_pub, delay)));
  }
  else if (key == crs_sensor_models::rocket_6_dof_sensor_models::ImuSensorModel::SENSOR_KEY)
  {
    // TODO currently, there can only be e.g. 1 IMU as they would publish on the same key. Change this
    // publish sensor measurements to topic "key"
    auto imu_pub = nh_private_.advertise<sensor_msgs::Imu>(key, 10);
    sensor_models_pub_.push_back(std::move(DelayedPublisher(nh_private_, imu_pub, delay)));
  }
  else if (key == crs_sensor_models::rocket_6_dof_sensor_models::MocapPoseSensorModel::SENSOR_KEY)
  {
    // TODO currently, there can only be e.g. 1 IMU as they would publish on the same key. Change this
    // publish sensor measurements to topic "key"
    auto mocap_pose_pub = nh_private_.advertise<geometry_msgs::TransformStamped>(key, 10);
    sensor_models_pub_.push_back(std::move(DelayedPublisher(nh_private_, mocap_pose_pub, delay)));
  }
  else if (key == crs_sensor_models::rocket_6_dof_sensor_models::MocapTwistSensorModel::SENSOR_KEY)
  {
    // TODO currently, there can only be e.g. 1 IMU as they would publish on the same key. Change this
    // publish sensor measurements to topic "key"
    auto mocap_twist_pub = nh_private_.advertise<geometry_msgs::TwistStamped>(key, 10);
    sensor_models_pub_.push_back(std::move(DelayedPublisher(nh_private_, mocap_twist_pub, delay)));
  }
  else
  {
    ROS_WARN_STREAM("Unknown sensor key " << key);
  }
}

/**
 * @brief publish sensor measurements
 *
 * @param key key of measurement that should be published
 */
void RocketSimulator::publishMeasurement(const std::string& key)
{
  for (int sensor_idx = 0; sensor_idx < sensor_models_.size(); sensor_idx++)  // iterate over all publishers
  {
    auto sensor_model = sensor_models_[sensor_idx];
    if (sensor_model->getKey() != key)  // Check if we want to publish this sensor
      continue;

    auto publisher = &sensor_models_pub_[sensor_idx];
    // Parse measurement (Eigen Vector) to custom ros message to publish it
    Eigen::MatrixXd measurement = sensor_model->applyModel(current_state_, last_input_);

    Eigen::MatrixXd noise;
    // sensor_name_to_noise_model_ of the form: {'mocap': GaussianNoise, 'imu': GaussianNoise}
    auto sensor_iter = sensor_name_to_noise_model_.find(key);
    if (sensor_iter != sensor_name_to_noise_model_.end())                           // Key was found
      noise = sensor_iter->second->sampleNoiseFromCovMatrix(sensor_model->getR());  // get R from noise model
    else
      noise.setZero(sensor_model->getR().rows(), 1);  // no noise

    // ========== FullState ==========
    if (sensor_model->getKey() == crs_sensor_models::rocket_6_dof_sensor_models::FullStateSensorModel::SENSOR_KEY)
    {
      crs_msgs::rocket_state msg;

      measurement += noise;
      msg.pos_x = measurement(0);
      msg.pos_y = measurement(1);
      msg.pos_z = measurement(2);
      msg.vel_x = measurement(3);
      msg.vel_y = measurement(4);
      msg.vel_z = measurement(5);
      msg.quat_x = measurement(6);
      msg.quat_y = measurement(7);
      msg.quat_z = measurement(8);
      msg.quat_w = measurement(9);
      msg.ang_vel_x = measurement(10);
      msg.ang_vel_y = measurement(11);
      msg.ang_vel_z = measurement(12);
      msg.thrust_magnitude = measurement(13);
      msg.torque_x = measurement(14);
      msg.servo_angle_1 = measurement(15);
      msg.servo_angle_2 = measurement(16);

      msg.header.stamp = ros::Time::now();
      publisher->publish(msg);
    }
    else if (sensor_model->getKey() == crs_sensor_models::rocket_6_dof_sensor_models::ImuSensorModel::SENSOR_KEY)
    {
      sensor_msgs::Imu msg;
      msg.linear_acceleration.x = measurement(0) + noise(0);
      msg.linear_acceleration.y = measurement(1) + noise(1);
      msg.linear_acceleration.z = measurement(2) + noise(2);
      msg.angular_velocity.x = measurement(3) + noise(3);
      msg.angular_velocity.y = measurement(4) + noise(4);
      msg.angular_velocity.z = measurement(5) + noise(5);

      msg.header.stamp = ros::Time::now();
      publisher->publish(msg);
    }
    else if (sensor_model->getKey() == crs_sensor_models::rocket_6_dof_sensor_models::MocapPoseSensorModel::SENSOR_KEY)
    {
      geometry_msgs::TransformStamped msg;

      msg.transform.translation.x = measurement(0) + noise(0);
      msg.transform.translation.y = measurement(1) + noise(1);
      msg.transform.translation.z = measurement(2) + noise(2);
      msg.transform.rotation.x = measurement(3) + noise(3);
      msg.transform.rotation.y = measurement(4) + noise(4);
      msg.transform.rotation.z = measurement(5) + noise(5);
      msg.transform.rotation.w = measurement(6) + noise(6);

      msg.header.stamp = ros::Time::now();
      publisher->publish(msg);
    }
    else if (sensor_model->getKey() == crs_sensor_models::rocket_6_dof_sensor_models::MocapTwistSensorModel::SENSOR_KEY)
    {
      geometry_msgs::TwistStamped msg;

      msg.twist.linear.x = measurement(0) + noise(0);
      msg.twist.linear.y = measurement(1) + noise(1);
      msg.twist.linear.z = measurement(2) + noise(2);
      msg.twist.angular.x = measurement(3) + noise(3);
      msg.twist.angular.y = measurement(4) + noise(4);
      msg.twist.angular.z = measurement(5) + noise(5);

      msg.header.stamp = ros::Time::now();
      publisher->publish(msg);
    }
    else
    {
      ROS_WARN_STREAM("Unknown sensor model " << sensor_model->getKey());
    }
  }
}
};  // namespace ros_simulator
