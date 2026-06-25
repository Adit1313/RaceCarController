#ifndef SRC_ROS_ROS_ESTIMATORS_INCLUDE_ROS_ESTIMATORS_VISUALIZERS_CAR_ESTIMATOR_VISUALIZER
#define SRC_ROS_ROS_ESTIMATORS_INCLUDE_ROS_ESTIMATORS_VISUALIZERS_CAR_ESTIMATOR_VISUALIZER
#include <ros/ros.h>

#include <estimators/base_estimator.h>
#include <ros_crs_utils/state_message_conversion.h>
#include <visualization_msgs/Marker.h>

#include "ros_estimators/visualizers/base_visualizer.h"
#include <estimators/model_based_estimator.h>
#include <ros_crs_utils/validation.h>

namespace ros_estimators
{
template <typename StateType>
class CarEstimatorVisualizer : public BaseEstimatorVisualizer<StateType>
{
protected:
  std::string frame_id = "crs_frame";
  std::string ns = "";

private:
  // Use arrow to also show estimated yaw angle
  bool use_arrows = false;
  // Configurable parameters
  double est_size_x = 0.05;
  double est_size_y = 0.05;
  double est_size_z = 0.05;

  double est_color_r = 0;
  double est_color_g = 0;
  double est_color_b = 0;
  double est_color_a = 1;

  visualization_msgs::Marker state_estimate;

public:
  CarEstimatorVisualizer(ros::NodeHandle nh_private,
                         std::shared_ptr<crs_estimators::BaseEstimator<StateType>> estimator)
    : BaseEstimatorVisualizer<StateType>(nh_private, estimator)
  {
    nh_private.getParam("use_arrows", use_arrows);

    nh_private.getParam("frame_id", frame_id);

    nh_private.getParam("est_r", est_color_r);
    nh_private.getParam("est_g", est_color_g);
    nh_private.getParam("est_b", est_color_b);
    nh_private.getParam("est_a", est_color_a);
    nh_private.getParam("size_x", est_size_x);
    nh_private.getParam("size_y", est_size_y);
    nh_private.getParam("size_z", est_size_z);

    nh_private.getParam("namespace", ns);

    // State estimate
    state_estimate.header.frame_id = frame_id;
    state_estimate.ns = ns + "_state_estimate";
    state_estimate.id = 0;
    state_estimate.scale.x = est_size_x;
    state_estimate.scale.y = est_size_y;
    state_estimate.scale.z = est_size_z;
    // State estimate has static color
    state_estimate.color.r = est_color_r;
    state_estimate.color.g = est_color_g;
    state_estimate.color.b = est_color_b;
    state_estimate.color.a = est_color_a;
    state_estimate.action = visualization_msgs::Marker::ADD;
    state_estimate.type = use_arrows ? visualization_msgs::Marker::ARROW : visualization_msgs::Marker::POINTS;
  };

  void visualizationCallback(const ros::TimerEvent& event) override
  {
    state_estimate.header.stamp = ros::Time::now();
    state_estimate.points.clear();

    geometry_msgs::Point temp_point;
    auto car_estimator_ptr = std::dynamic_pointer_cast<crs_estimators::BaseEstimator<StateType>>(
        BaseEstimatorVisualizer<StateType>::estimator_);

    StateType estimate = car_estimator_ptr->getStateEstimate();

    if (!use_arrows)
    {
      // Position
      temp_point.x = estimate.pos_x;  // x_position
      temp_point.y = estimate.pos_y;  // y_position
      temp_point.z = 0;
      state_estimate.points.push_back(temp_point);
    }
    else
    {
      // Estimated position
      state_estimate.pose.position.x = estimate.pos_x;
      state_estimate.pose.position.y = estimate.pos_y;
      state_estimate.pose.position.z = 0;
      // Estimated orientation (quaternion from only yaw)
      state_estimate.pose.orientation.w = std::cos(estimate.yaw * 0.5);
      state_estimate.pose.orientation.x = 0;
      state_estimate.pose.orientation.y = 0;
      state_estimate.pose.orientation.z = std::sin(estimate.yaw * 0.5);
    }
    if (is_valid_marker(state_estimate))
      BaseEstimatorVisualizer<StateType>::est_visualization_publisher_.publish(state_estimate);
  }
};
};  // namespace ros_estimators

#endif /* SRC_ROS_ROS_ESTIMATORS_INCLUDE_ROS_ESTIMATORS_VISUALIZERS_CAR_ESTIMATOR_VISUALIZER */
