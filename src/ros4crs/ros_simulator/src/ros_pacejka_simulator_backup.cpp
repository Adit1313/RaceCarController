#include "ros_simulator/ros_pacejka_simulator.h"
#include <geometry_msgs/TransformStamped.h>
#include <pacejka_sensor_model/imu_sensor_model.h>
#include <pacejka_sensor_model/imu_yaw_rate_sensor_model.h>
#include <pacejka_sensor_model/mocap_sensor_model.h>
#include <pacejka_sensor_model/wheel_encoder_sensor_model.h>
#include <pacejka_sensor_model/lighthouse_sensor_model.h>
#include <ros/console.h>
#include <ros_crs_utils/parameter_io.h>
#include <crs_msgs/car_wheel_speed.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <crs_msgs/lighthouse_sweep.h>

#include <kalman_estimator/car_kalman_parameters.h>

#include <kalman_estimator/car_kalman_parameters.h>

namespace ros_simulator
{
PacejkaSimulator::PacejkaSimulator(ros::NodeHandle nh, ros::NodeHandle nh_private) : nh_(nh), nh_private_(nh_private)
{
  // Load model parameters from rosparameters (e.g. model_params.yaml  in pacejka_model)
  crs_models::pacejka_model::pacejka_params params;
  parameter_io::getModelParams<crs_models::pacejka_model::pacejka_params>(ros::NodeHandle(nh_, "model/model_params/"),
                                                                          params);

  Eigen::Matrix<double, crs_models::pacejka_model::pacejka_car_state::NX,
                crs_models::pacejka_model::pacejka_car_state::NX>
      Q = Eigen::Matrix<double, crs_models::pacejka_model::pacejka_car_state::NX,
                        crs_models::pacejka_model::pacejka_car_state::NX>::Identity();
  if (!parameter_io::getMatrixFromParams<crs_models::pacejka_model::pacejka_car_state::NX,
                                         crs_models::pacejka_model::pacejka_car_state::NX>(
          ros::NodeHandle(nh_, "model/Q"), Q))  // Load Q from rosparameters (e.g. model_params.yaml  in pacejka_model)
  {
    ROS_WARN_STREAM("Could not load Q Matrix for provided Model. Using to Identity Matrix");
  }
  // Create a pacejka model
  model_ = std::make_unique<crs_models::pacejka_model::DiscretePacejkaModel>(params, Q);

  gt_state_pub = nh_private_.advertise<crs_msgs::car_state_cart>("gt_state", 10);
  control_input_sub_ = nh_private_.subscribe("control_input", 10, &PacejkaSimulator::inputCallback, this);

  std::vector<double> initial_input;
  if (nh_private_.getParam("initial_input", initial_input))  // Load initial input from rosparameters (e.g.
                                                             // pacejka_car_simulator.yaml)
  {
    last_input_.steer = initial_input[0];
    last_input_.torque = initial_input[1];
  }
  else
  {
    ROS_WARN_STREAM("No initial input set. Using steer: 0, torque: 0.4");
    last_input_.steer = 0.0;
    last_input_.torque = 0.4;
  }

  std::vector<double> initial_state;
  if (nh_private_.getParam("initial_state", initial_state))  // Load initial state from rosparameters (e.g.
                                                             // pacejka_car_simulator.yaml)
  {
    current_state_.pos_x = initial_state[0];
    current_state_.pos_y = initial_state[1];
    current_state_.yaw = initial_state[2];
    current_state_.vel_x = initial_state[3];
    current_state_.vel_y = initial_state[4];
    current_state_.yaw_rate = initial_state[5];
  }
  else
  {
    ROS_WARN_STREAM("No initial state set. Using [0, 0, 0, 0.5, 0, 0]");
    current_state_.pos_x = 0;
    current_state_.pos_y = 0;
    current_state_.yaw = 0;
    current_state_.vel_x = 0.5;
    current_state_.vel_y = 0;
    current_state_.yaw_rate = 0;
  }

  nh_private_.getParam("do_collision_checks", simulate_track_collision_);
  if (simulate_track_collision_)
  {
    static_track_trajectory_ = parameter_io::loadTrackDescriptionFromParams(ros::NodeHandle(nh, "track"));
  }
}

void PacejkaSimulator::printConfig()
{
  ROS_INFO_STREAM("Started Pacejka Car Simulator!\n"
                  << "Initial State:\n"
                  << current_state_ << "\nInitial Input " << last_input_ << "\n\nUsing Noise:\n"
                  << "\n\nProcess Noise Cov Matrix:\n"
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
void PacejkaSimulator::registerNoiseModel(std::shared_ptr<NoiseModel> noise_model)
{
  noise_model_ = noise_model;
}

/**
 * @brief Propagates model in simulation (adds process noise if needed)
 *
 * @param timestep time for which model is propagated
 */
void PacejkaSimulator::advanceState(double timestep)
{
  if (simulate_track_collision_)
  {
    bool collided = static_track_trajectory_->getTrackError(Eigen::Vector2d(current_state_.pos_x, current_state_.pos_y))
                        .lateral_error > static_track_trajectory_->getWidth() / 2;

    if (collided)
    {
      ROS_WARN_THROTTLE(1, "Track collision in simulator detected!");

      if (!collision_detected_)  // State swapped from no collision - collision)
      {
        if (current_state_.vel_x > 0)    // Hit the track driving forward (normal situation)
          current_state_.vel_x = -0.05;  // Can not be zero, otherwise we get nan issues
        else                             // Hit the track driving backward
          current_state_.vel_x = 0.05;   // Can not be zero, otherwise we get nan issues
      }

      current_state_.vel_y = 0;
      current_state_.yaw_rate = 0;
    }
    collision_detected_ = collided;
  }

  if (!got_init_input)
    return;

  current_state_ = model_->applyModel(current_state_, last_input_, timestep);
  if (noise_model_)
  {
    // Sample noise. Note Q is defined in noise per time
    Eigen::MatrixXd noise = noise_model_->sampleNoiseFromCovMatrix(timestep * model_->getQ());
    current_state_.pos_x += noise(0);
    current_state_.pos_y += noise(1);
    current_state_.yaw += noise(2);
    current_state_.vel_x += noise(3);
    current_state_.vel_y += noise(4);
    current_state_.yaw_rate += noise(5);
  }
}

/**
 * @brief sets inputs received from ros message
 *
 * @param input
 */
void PacejkaSimulator::inputCallback(crs_msgs::car_inputConstPtr input)
{
  last_input_.steer = input->steer;
  last_input_.torque = input->torque;
  got_init_input = true;
}

/**
 * @brief publish ground truth state of simulation model
 *
 */
void PacejkaSimulator::publishStates()
{
  gt_state_pub.publish(message_conversion::convertStateToRosMsg<crs_msgs::car_state_cart>(current_state_, last_input_));

  // Publish car as tf frame.
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "crs_frame";
  transformStamped.child_frame_id = "car1";  // Todo, load from param
  transformStamped.transform.translation.x = current_state_.pos_x;
  transformStamped.transform.translation.y = current_state_.pos_y;
  transformStamped.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, current_state_.yaw);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();
  Simulator::transform_broadcaster_.sendTransform(transformStamped);
}

/**
 * @brief Add sensor model to list of sensor models
 *
 * @param sensor_model
 */
void PacejkaSimulator::registerSensorModel(
    std::shared_ptr<crs_sensor_models::SensorModel<crs_models::pacejka_model::pacejka_car_state,
                                                   crs_models::pacejka_model::pacejka_car_input>>
        sensor_model,
    double delay, std::string name)
{
  sensor_models_.push_back(sensor_model);  // append sensor_model to list of all sensor models
  std::string key = sensor_model->getKey();
  if (name == "")
    name = key;

  sensor_names_.push_back(name);  // append name of sensor to list of all sensor names

  if (key == crs_sensor_models::pacejka_sensor_models::MocapSensorModel::SENSOR_KEY)
  {
    // TODO currently, there can only be e.g. 1 IMU as they would publish on the same key. Change this
    // publish sensor measurements to topic "key"
    auto mocap_pub = nh_private_.advertise<geometry_msgs::TransformStamped>(name, 10);
    sensor_models_pub_.push_back(std::move(DelayedPublisher(nh_private_, mocap_pub, delay)));
  }
  else if (key == crs_sensor_models::pacejka_sensor_models::ImuSensorModel::SENSOR_KEY)
  {
    auto pub = nh_private_.advertise<sensor_msgs::Imu>(name, 10);
    sensor_models_pub_.push_back(std::move(DelayedPublisher(nh_private_, pub, delay)));
  }
  else if (key == crs_sensor_models::pacejka_sensor_models::ImuYawSensorModel::SENSOR_KEY)
  {
    auto pub = nh_private_.advertise<sensor_msgs::Imu>(name, 10);
    sensor_models_pub_.push_back(std::move(DelayedPublisher(nh_private_, pub, delay)));
  }
  else if (key == crs_sensor_models::pacejka_sensor_models::WheelEncoderSensorModel::SENSOR_KEY)
  {
    auto pub = nh_private_.advertise<crs_msgs::car_wheel_speed>(name, 10);
    sensor_models_pub_.push_back(std::move(DelayedPublisher(nh_private_, pub, delay)));
  }
  else if (key == crs_sensor_models::pacejka_sensor_models::LighthouseSensorModel::SENSOR_KEY)
  {
    // For Lighthouse, we have two sensor models with different names (lighthouse_1, lighthouse_2)
    // that publish to the same topic, lighthouse. We use the key here instead of the name.
    auto pub = nh_private_.advertise<crs_msgs::lighthouse_sweep>(key, 10);
    sensor_models_pub_.push_back(std::move(DelayedPublisher(nh_private_, pub, delay)));
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
void PacejkaSimulator::publishMeasurement(const std::string& key)
{
  for (int sensor_idx = 0; sensor_idx < sensor_models_.size(); sensor_idx++)  // iterate over all publishers
  {
    auto sensor_model = sensor_models_[sensor_idx];
    if (sensor_model->getKey() != key)  // Check if we want to publish this sensor
      continue;

    auto publisher = &sensor_models_pub_[sensor_idx];

    std::string sensor_name = sensor_names_[sensor_idx];
    // Parse measurement (Eigen Vector) to custom ros message to publish it
    Eigen::MatrixXd measurement = sensor_model->applyModel(current_state_, last_input_);

    Eigen::MatrixXd noise;
    // sensor_name_to_noise_model_ of the form: {'mocap': GaussianNoise, 'imu': GaussianNoise}
    auto sensor_iter = sensor_name_to_noise_model_.find(key);
    if (sensor_iter != sensor_name_to_noise_model_.end())                           // Key was found
      noise = sensor_iter->second->sampleNoiseFromCovMatrix(sensor_model->getR());  // get R from noise model
    else
    {
      noise.setZero(sensor_model->getR().rows(), 1);  // no noise
      std::cout << "no noise model set" << std::endl;
    }
    // ========== MOCAP ==========

    if (sensor_model->getKey() == crs_sensor_models::pacejka_sensor_models::MocapSensorModel::SENSOR_KEY)
    {
      geometry_msgs::TransformStamped msg;

      // Parse mocap measurements to ros messagex to publish it
      msg.transform.translation.x = measurement(0) + noise(0);  // x positiion
      msg.transform.translation.y = measurement(1) + noise(1);  // y position

      // yaw angle, ros msg wants quaternion not euler angles
      tf2::Quaternion myQuaternion;
      myQuaternion.setRPY(0, 0, measurement(2) + noise(2));
      msg.transform.rotation.x = myQuaternion.getX();
      msg.transform.rotation.y = myQuaternion.getY();
      msg.transform.rotation.z = myQuaternion.getZ();
      msg.transform.rotation.w = myQuaternion.getW();

      msg.header.stamp = ros::Time::now();
      publisher->publish(msg);
    }
    // ========== IMU ==========

    else if (sensor_model->getKey() == crs_sensor_models::pacejka_sensor_models::ImuSensorModel::SENSOR_KEY)
    {
      sensor_msgs::Imu msg;
      msg.linear_acceleration.x = measurement(0) + noise(0);
      msg.linear_acceleration.y = measurement(1) + noise(1);
      msg.angular_velocity.z = measurement(2) + noise(2);
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = "car1";
      publisher->publish(msg);
    }
    // ========== IMU YAW RATE ==========

    else if (sensor_model->getKey() == crs_sensor_models::pacejka_sensor_models::ImuYawSensorModel::SENSOR_KEY)
    {
      sensor_msgs::Imu msg;
      msg.angular_velocity.z = measurement(0) + noise(0);
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = "car1";
      publisher->publish(msg);
    }
    // ========== Wheel Encoder ==========

    else if (sensor_model->getKey() == crs_sensor_models::pacejka_sensor_models::WheelEncoderSensorModel::SENSOR_KEY)
    {
      crs_msgs::car_wheel_speed speed;
      speed.front_left = measurement(0) + noise(0);
      speed.front_right = measurement(1) + noise(1);
      speed.rear_left = measurement(2) + noise(2);
      speed.rear_right = measurement(3) + noise(3);
      speed.header.stamp = ros::Time::now();
      publisher->publish(speed);
    }
    // ========== LIGHTHOUSE ==========

    else if (sensor_model->getKey() == crs_sensor_models::pacejka_sensor_models::LighthouseSensorModel::SENSOR_KEY)
    {
      // Lighthouse is a special case as it publishes two sweeps per simulated base station.
      // The two sweeps have different sensor models (lighthouse_1, lighthouse_2), but publish on the same topic.
      // To simulate this behaviour, a small delay is introduced between the two sweeps.
      // The first_timestamp and sync_timestamp are currently not simulated.

      // Derive base station ID and sensor ID from the sensor name. The sensor name is "lighthouse_X_Y" where X is the
      // base station ID and Y is the sensor ID.
      int sensor_id = int(sensor_name.back()) - '0';  // get last char of sensor_name and convert it to int
      sensor_name.pop_back();                         // pop the sensor ID
      sensor_name.pop_back();                         // pop the underscore

      // TODO: this won't work for base station IDs > 9
      int base_station_id = int(sensor_name.back()) - '0';  // get last char of sensor_name and convert it to int

      bool first_sweep = sensor_id == 1;

      if (!first_sweep)
      {
        // This is the second sweep of the sensor. We simulate a short delay between the two sweeps.
        ros::Duration(0.0005).sleep();  // wait 1ms between the two sweeps
      }

      crs_msgs::lighthouse_sweep msg;
      msg.first_sweep = first_sweep;
      msg.polynomial = base_station_id << 1;  // base station ID = polynomial >> 1

      // Parse Lighthouse measurement to ROS message
      msg.angle_0 = measurement(0) + noise(0);
      msg.angle_1 = measurement(1) + noise(1);
      msg.angle_2 = measurement(2) + noise(2);
      msg.angle_3 = measurement(3) + noise(3);

      msg.header.stamp = ros::Time::now();
      publisher->publish(msg);
    }
  }
}
};  // namespace ros_simulator
