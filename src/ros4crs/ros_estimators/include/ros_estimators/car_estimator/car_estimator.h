#ifndef SRC_ROS_ROS_ESTIMATORS_INCLUDE_ROS_ESTIMATORS_CAR_ESTIMATOR_CAR_ESTIMATOR
#define SRC_ROS_ROS_ESTIMATORS_INCLUDE_ROS_ESTIMATORS_CAR_ESTIMATOR_CAR_ESTIMATOR

#include <ros/ros.h>
#include <crs_msgs/double_array_stamped.h>
#include <crs_msgs/car_state_cart.h>
#include <crs_msgs/car_wheel_speed.h>
#include <crs_msgs/car_input.h>
#include <crs_msgs/lighthouse_sweep.h>

#include <ros_crs_utils/parameter_io.h>
#include <ros_crs_utils/state_message_conversion.h>

#include "ros_estimators/data_converter.h"
#include "ros_estimators/state_estimator_ros.h"
#include "ros_estimators/visualizers/base_visualizer.h"

#include <estimators/base_estimator.h>
#include <estimators/model_based_estimator.h>
#include <crs_msgs/double_array_stamped.h>
#include <kalman_estimator/kalman_estimator.h>

#include "ros_estimators/visualizers/base_visualizer.h"
#include <estimators/base_estimator.h>

// Tf publishers
#include <tf/transform_broadcaster.h>

namespace ros_estimators
{
template <typename StateType, typename InputType, typename ParamType = parameter_io::empty_params>
class RosCarEstimator : public RosStateEstimator
{
public:
  std::string car_frame_name = "est_frame";
  ros::Publisher estimate_pub;
  ros::Publisher reference_pub;
  ros::Time last_publish_time;
  // Construction.
  RosCarEstimator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
                  std::unique_ptr<BaseEstimatorVisualizer<StateType>> visualizer,
                  const std::vector<std::string> measurement_keys,
                  std::shared_ptr<crs_estimators::BaseEstimator<StateType>> base_estimator)
    : base_estimator(base_estimator), nh_(nh), nh_private_(nh_private)
  {
    estimate_pub = nh_.advertise<visualization_msgs::Marker>("estimated_state_trajectory", 200);
    reference_pub = nh_.advertise<visualization_msgs::Marker>("reference_state_trajectory", 200);

    if (!nh_private.getParam("car_frame_name", car_frame_name))
    {
      ROS_WARN_STREAM("Did not load parameter for car frame name. Defaulting to: " << car_frame_name);
      ROS_WARN_STREAM("NAMESPACE: " << nh_private.getNamespace());
    }

    // Check if base estimator is model based
    // Try to convert it to parent type
    auto model_based_est =
        std::dynamic_pointer_cast<crs_estimators::ModelBasedEstimator<StateType, InputType>>(base_estimator);
    if (model_based_est)
    {
      // old was safely casted to NewType
      model_based_estimator = model_based_est;
    }
    else
    {
      model_based_estimator = nullptr;
    }

    // Load config
    if (!nh_private.getParam("max_callback_rate", max_callback_rate_))
    {
      ROS_WARN_STREAM(
          "Did not load parameter for max_callback_rate. Defaulting to processing all sensor measurements.");
      enforce_sensor_rate_limit_ = false;
    }
    else if (max_callback_rate_ <= 0)
    {
      ROS_INFO_STREAM("max_callback_rate is set to " << max_callback_rate_ << ". Disabling sensor rate limiting.");
      enforce_sensor_rate_limit_ = false;
    }
    if (!nh_private.getParam("measurement_timeout_threshold", measurement_timeout_threshold_))
      ROS_WARN_STREAM("Did not load parameter for measurement_timeout_threshold. Defaulting to: "
                      << measurement_timeout_threshold_);
    if (!nh_private.getParam("world_frame", mocap_converter.world_frame))
      ROS_WARN_STREAM("Did not load parameter for world frame. Defaulting to: " << mocap_converter.world_frame);
    if (!nh_private.getParam("track_frame", mocap_converter.track_frame))
      ROS_WARN_STREAM("Did not load parameter for track frame. Defaulting to: " << mocap_converter.track_frame);
    if (!nh_private.getParam("update_track_transform", mocap_converter.update_track_transform))
      ROS_WARN_STREAM("Did not load parameter for update_track_transform. Defaulting to: "
                      << mocap_converter.update_track_transform);

    // Setup ros publishers
    state_estimate_pub_ = nh_private_.advertise<crs_msgs::car_state_cart>("best_state", 10);

    // Register callbacks for keys
    for (const auto& key : measurement_keys)
    {
      if (key == "mocap")
      {
        ROS_INFO("subscribing to mocap");
        measurement_subs_.push_back(nh_.subscribe(key, 1, &RosCarEstimator::mocapMeasurementCallback, this));
      }
      else if (key == "wheel_encoders")
      {
        ROS_INFO("subscribing to wheel_encoders");
        measurement_subs_.push_back(nh_.subscribe(key, 1, &RosCarEstimator::wheelEncoderMeasurementCallback, this));
      }
      else if (key == "imu")
      {
        ROS_INFO("subscribing to imu");
        measurement_subs_.push_back(nh_.subscribe(key, 1, &RosCarEstimator::imuMeasurementCallback, this));
      }
      else if (key == "imu_yaw_rate")
      {
        ROS_INFO("subscribing to imu_yaw_rate");
        measurement_subs_.push_back(nh_.subscribe(key, 1, &RosCarEstimator::imuYawMeasurementCallback, this));
      }
      else if (key == "lighthouse")
      {
        measurement_subs_.push_back(nh_.subscribe(key, 10, &RosCarEstimator::lighthouseMeasurementCallback, this));
      }
      else
      {
        ROS_WARN_STREAM("Masurement key "
                        << key << " has no known ROS conversion registered in estimator. Key will be dropped!");
        continue;
      }
      sensor_last_timestamp_[key] = 0;  // Initialize sensor timestamp to zero.
    }

    assert(!measurement_subs_.empty() && "No Measurement topic provided for Estimator. Aborting!");

    // Set state to running. TODO: Toggle this externally using ros services. Probably different PR.
    is_running_ = true;

    visualizer_ = std::move(visualizer);
  };

  RosCarEstimator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
                  std::unique_ptr<BaseEstimatorVisualizer<StateType>> visualizer,
                  const std::vector<std::string> measurement_keys,
                  std::shared_ptr<crs_estimators::ModelBasedEstimator<StateType, InputType>> model_based_estimator)
    : RosCarEstimator(nh, nh_private, std::move(visualizer), measurement_keys,
                      std::static_pointer_cast<crs_estimators::BaseEstimator<StateType>>(model_based_estimator))
  {
    // Regiser control input subscription if estimator is model based
    control_input_sub_ = nh_.subscribe("control_input", 1, &RosCarEstimator::controlInputCallback, this);

    // Only need to load model parameters for e.g. kinematic model. The Pacejka model does not need this.
    // These model parameters are needed for message conversions between crs and ros.

    parameter_io::getModelParams<ParamType>(ros::NodeHandle(nh, "model/model_params/"), model_params);
    // Then overwrite specific parameters from local config (private nodehandle)
    parameter_io::getModelParams<ParamType>(ros::NodeHandle(nh_private, "model/model_params/"), model_params, false);
  }

  void checkMissingMeasurements(const long current_time)
  {
    if (startup_time == 0)
      startup_time = current_time;

    // Check for all measurement keys if data is received.
    for (const auto& sensor_entry : sensor_last_timestamp_)
    {
      const std::string sensor_name = sensor_entry.first;
      const float last_sensor_ts = sensor_entry.second;

      if (last_sensor_ts == 0)
      {
        sensor_last_timestamp_[sensor_name] = startup_time;
        continue;
      }

      if (current_time - last_sensor_ts > measurement_timeout_threshold_)
      {  // never received a measurement
         // or the latest is too old

        if (sensor_entry.second == startup_time)
        {
          ROS_WARN_STREAM_THROTTLE(5, "No measurement received from: " << sensor_name);
        }
        else
        {
          ROS_WARN_STREAM_THROTTLE(5, "Missing measurements from: " << sensor_name << ". Last message received: "
                                                                    << (current_time - last_sensor_ts)
                                                                    << " seconds ago.");
        }
      }
    }
  }

  /**
   * @brief Checks if a given sensor callback should be executed.
   *
   * The check uses the current state of the estimator (is it even running?) and enforces rate limits
   * on a per-sensor basis.
   *
   * @return True if the callback should continue, false if no measurement update should be performed right now.
   */
  bool checkSensorFrequency(const std::string sensor_name, const double timestamp)
  {
    if (!is_running_)
    {
      return false;
    }

    if (enforce_sensor_rate_limit_ && (timestamp - sensor_last_timestamp_[sensor_name] < 1 / max_callback_rate_))
    {
      return false;
    }
    sensor_last_timestamp_[sensor_name] = timestamp;

    return true;
  }

  void controlInputCallback(const crs_msgs::car_input::ConstPtr input_msg)
  {
    if (!is_running_)
      return;

    // Check if some measurements are ignored. This is here since we can not guarante that the measuremnt callbacks
    // are executed.
    checkMissingMeasurements(input_msg->header.stamp.toSec());

    // We have a valid input now.
    has_valid_input_ = true;

    last_input_ = *input_msg;
    std::dynamic_pointer_cast<crs_estimators::ModelBasedEstimator<StateType, InputType>>(base_estimator)
        ->controlInputCallback(message_conversion::convertToCrsInput<crs_msgs::car_input, InputType>(*input_msg),
                               input_msg->header.stamp.toSec());
  };

  void wheelEncoderMeasurementCallback(const crs_msgs::car_wheel_speedConstPtr msg)
  {
    // Only start estimator if we have a valid input. (Wait for first control input)
    if (!checkSensorFrequency("wheel_encoders", msg->header.stamp.toSec()) || !has_valid_input_)
      return;

    base_estimator->measurementCallback(parseWheelEncoder(msg));
  }

  void mocapMeasurementCallback(const geometry_msgs::TransformStamped::ConstPtr msg)
  {
    // If there is no valid input, we set initial position and yaw to the mocap measurement.
    if (!has_valid_input_)
    {
      StateType current_state = base_estimator->getStateEstimate();
      // Override position and yaw
      try
      {
        auto measurement = mocap_converter.parseData2D(msg);
        current_state.pos_x = measurement.measurement_data(0);
        current_state.pos_y = measurement.measurement_data(1);
        current_state.yaw = measurement.measurement_data(2);

        base_estimator->resetStateEstimate(current_state);
      }
      catch (tf::TransformException ex)
      {
        ROS_WARN_STREAM("Could not find transform for mocap. Error: " << ex.what());
        return;
      }
    }

    if (!checkSensorFrequency("mocap", msg->header.stamp.toSec()) || !has_valid_input_)
      return;

    try
    {
      base_estimator->measurementCallback(mocap_converter.parseData2D(msg));
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
    }
  };

  void imuMeasurementCallback(const sensor_msgs::Imu::ConstPtr msg)
  {
    if (!checkSensorFrequency("imu", msg->header.stamp.toSec()) || !has_valid_input_)
      return;

    base_estimator->measurementCallback(parseImuData2D(msg));
  };

  void imuYawMeasurementCallback(const sensor_msgs::Imu::ConstPtr msg)
  {
    if (!checkSensorFrequency("imu_yaw_rate", msg->header.stamp.toSec()) || !has_valid_input_)
      return;

    base_estimator->measurementCallback(parseImuYawData2D(msg));
  };

  void lighthouseMeasurementCallback(const crs_msgs::lighthouse_sweep::ConstPtr msg)
  {
    if (sensor_last_timestamp_["lighthouse"] == 0 && !msg->first_sweep)
    {
      return;  // Make sure we start with the first sweep on startup.
    }

    sensor_last_timestamp_["lighthouse"] = msg->header.stamp.toSec();

    if (!has_valid_input_)
    {
      StateType current_state = base_estimator->getStateEstimate();
      auto kalman_est =
          std::dynamic_pointer_cast<crs_estimators::kalman::KalmanEstimator<StateType, InputType>>(base_estimator);

      if (!kalman_est)
      {
        ROS_WARN_THROTTLE(1, "Could not cast base estimator to Kalman estimator. Initial state will not be set to "
                             "lighthouse measurement.");
      }
      else
      {
        // Override position and yaw
        try
        {
          kalman_est->measurementUpdate(parseLighthouseSweep(msg));
        }
        catch (const std::invalid_argument& ex)
        {
          ROS_WARN("Lighthouse measurement update failed. Probably a wrong base station ID was received.");
          ROS_WARN("%s", ex.what());
          return;
        }
        auto post_state = kalman_est->getStateEstimate();  // this will return the posterior state, since we only
                                                           // did a measurement update and no predict step.
        // Update Postion to posterior state
        current_state.pos_x = post_state.pos_x;
        current_state.pos_y = post_state.pos_y;
        // current_state.yaw = post_state.yaw;

        // Reset state estimate to pose computed by measurement update and velocities & yaw rate of initial state
        base_estimator->resetStateEstimate(current_state);
      }
      return;
    }

    // Parse the Lighthouse sweep data into a measurement
    auto lighthouse_measurement = parseLighthouseSweep(msg);

    // Sometimes, a malformed sweep can contain an invalid base station id.
    // In that case, the sensor key is unknown and we discard the measurement.
    try
    {
      base_estimator->measurementCallback(lighthouse_measurement);
    }
    catch (const std::invalid_argument& ex)
    {
      ROS_WARN("Lighthouse measurement update failed. Probably a wrong base station ID was received.");
      ROS_WARN("%s", ex.what());
    }
  };

  void publishState() override;

  void publishDiagnosticData()
  {
    for (auto const [key, value] : base_estimator->getDiagnosticData())
    {
      const std::string topic = "diagnostics/" + key;

      if (diagnostic_data_pubs_.find(topic) == diagnostic_data_pubs_.end())
      {
        // No publisher for this topic found (first time publishing it)
        diagnostic_data_pubs_[topic] = nh_private_.advertise<crs_msgs::double_array_stamped>(topic, 10);
        ROS_INFO_STREAM("Created publisher for diagnostic data: " << topic);
      }

      // Convert diagnostic data to ros message and publish it
      crs_msgs::double_array_stamped msg;
      msg.header.stamp = ros::Time::now();
      msg.data = std::vector<double>(value.begin(), value.end());
      diagnostic_data_pubs_[topic].publish(msg);

      // Throttle current rejection stats if available.
      // This is saved under the key "<sensor_name>/rejection_stats" and
      // contains the number of rejected measurements and the total number of measurements.
      const std::size_t idx = key.find("/");
      if (idx == std::string::npos)
      {
        ROS_WARN_STREAM("Could not find sensor name. Diagnostic data will not be published.");
        continue;
      }
      const std::string sensor_name = key.substr(0, idx);                  // e.g. "lighthouse_0_1" or "MHE"
      const std::string type_of_data = key.substr(idx + 1, key.length());  // "rejection_stats" or "solver"

      int n_rejected = int(value[0]);
      int n_msgs = int(value[1]);
      int n_rejected_prev = 0;

      // number_of_previously_rejected_meas is a map that contains the number of previously rejected measurements for
      // each sensor_name
      if (number_of_previously_rejected_meas.find(sensor_name) == number_of_previously_rejected_meas.end())
      {
        // Sensor has NOT been seen before -> 0 rejected measurements
        n_rejected_prev = 0;
        number_of_previously_rejected_meas.insert(std::pair<std::string, int>(sensor_name, n_rejected_prev));
      }
      else
      {
        // Sensor has been seen before
        n_rejected_prev = number_of_previously_rejected_meas.at(sensor_name);
      }

      // Every 2Hzoutput the rejection rate for that sensor, if measurements were rejected.
      if (type_of_data == "rejection_stats" && (n_rejected > n_rejected_prev))
      {
        ROS_INFO_THROTTLE(2, "%s: Total Rejected: %d (%.4f %%)", sensor_name.c_str(), n_rejected,
                          float(n_rejected) / (n_msgs + 0.0001) * 100);
        number_of_previously_rejected_meas.at(sensor_name) = n_rejected;
      }
      // Every 2Hzoutput the rejection rate for that sensor, if measurements were rejected.
      if (type_of_data == "solver" && (n_rejected > n_rejected_prev))
      {
        ROS_INFO_THROTTLE(2, "%s: Total Solver Errors: %d (%.4f %%)", sensor_name.c_str(), n_rejected,
                          float(n_rejected) / (n_msgs + 0.0001) * 100);
        number_of_previously_rejected_meas.at(sensor_name) = n_rejected;
      }
    }
  };

  // Ugly. TODO, use getter and setter or similar
  ParamType model_params;

private:
  /**
   * @brief Rate limiting on sensor (measurement) callback functions.
   *
   * This sets the maximum rate [Hz] at which a sensor callback is executed. Samples that arrive faster are dropped.
   * Applies to each sensor key, individually.
   *
   * @note The rate limiting is only enforced if @ref enforce_sensor_rate_limit_ is set to @c true.
   */
  double max_callback_rate_ = 200;              // Hz
  double measurement_timeout_threshold_ = 2.0;  // s

  /** If true, rate limiting on sensor callbacks is enabled. @see max_callback_rate_ */
  bool enforce_sensor_rate_limit_ = true;

  std::map<std::string, double> sensor_last_timestamp_;

  // Keep track of missing calls.
  double startup_time = 0;

  // Keep track of number of rejected measurements per sensor
  std::map<std::string, int> number_of_previously_rejected_meas;

  // Flag to set state of estimator
  bool is_running_ = false;
  bool has_valid_input_ = false;

  // tf publisher
  tf::TransformBroadcaster tf_broadcaster;

  // Node handles.
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // The following needs to be initialized by child class

  // Publisher
  ros::Publisher state_estimate_pub_;

  // List with publishers for diagnostic data
  std::unordered_map<std::string, ros::Publisher> diagnostic_data_pubs_;

  // Subscriptions
  ros::Subscriber control_input_sub_;
  std::vector<ros::Subscriber> measurement_subs_;

  MocapConverter mocap_converter;
  std::shared_ptr<crs_estimators::BaseEstimator<StateType>> base_estimator;

  // Visualizer for estimaotr
  std::unique_ptr<BaseEstimatorVisualizer<StateType>> visualizer_;

  // Pointer to the model based estimator.
  // If the current estimator does not support models (i.e. predict) this will be set to nullptr
  std::shared_ptr<crs_estimators::ModelBasedEstimator<StateType, InputType>> model_based_estimator;

  crs_msgs::car_input last_input_;
};
}  // namespace ros_estimators

#endif /* SRC_ROS_ROS_ESTIMATORS_INCLUDE_ROS_ESTIMATORS_CAR_ESTIMATOR_CAR_ESTIMATOR */
