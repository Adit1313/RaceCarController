#ifndef SRC_ROS_ROS_ESTIMATORS_INCLUDE_ROS_ESTIMATORS_VISUALIZERS_CAR_EKF_VISUALIZER
#define SRC_ROS_ROS_ESTIMATORS_INCLUDE_ROS_ESTIMATORS_VISUALIZERS_CAR_EKF_VISUALIZER
#include <ros/ros.h>

#include <estimators/base_estimator.h>
#include <estimators/model_based_estimator.h>
#include <kalman_estimator/kalman_estimator.h>

#include <ros_crs_utils/state_message_conversion.h>
#include <visualization_msgs/Marker.h>

#include "ros_estimators/visualizers/base_visualizer.h"
#include "ros_estimators/visualizers/car_estimator_visualizer.h"
#include <ros_crs_utils/validation.h>

namespace ros_estimators
{
/**
 * @brief Visualizer that shows the state estimate and covariance of a discrete EKF.
 */
template <typename StateType, typename InputType>
class CarEkfVisualizer : public CarEstimatorVisualizer<StateType>
{
private:
  // Configurable parameters

  double cov_color_r = 0;
  double cov_color_g = 0;
  double cov_color_b = 0;
  double cov_color_a = 1;

  double covariance_scale = 1.0;

  visualization_msgs::Marker estimate_covariance;

protected:
  ros::Publisher cov_visualization_publisher_;

public:
  CarEkfVisualizer(ros::NodeHandle nh_private,
                   std::shared_ptr<crs_estimators::kalman::KalmanEstimator<StateType, InputType>> estimator)
    : CarEstimatorVisualizer<StateType>(nh_private,
                                        std::dynamic_pointer_cast<crs_estimators::BaseEstimator<StateType>>(estimator))

  {
    cov_visualization_publisher_ = nh_private.advertise<visualization_msgs::Marker>("cov_estimate_visualization", 100);

    nh_private.getParam("cov_r", cov_color_r);
    nh_private.getParam("cov_g", cov_color_g);
    nh_private.getParam("cov_b", cov_color_b);
    nh_private.getParam("cov_a", cov_color_a);

    nh_private.getParam("cov_scale", covariance_scale);

    // State estimate covariance
    estimate_covariance.header.frame_id = CarEstimatorVisualizer<StateType>::frame_id;
    estimate_covariance.ns = CarEstimatorVisualizer<StateType>::ns + "_state_estimate";
    estimate_covariance.id = 0;
    // State estimate covariance has static color
    estimate_covariance.color.r = cov_color_r;
    estimate_covariance.color.g = cov_color_g;
    estimate_covariance.color.b = cov_color_b;
    estimate_covariance.color.a = cov_color_a;
    estimate_covariance.action = visualization_msgs::Marker::ADD;
    estimate_covariance.type = visualization_msgs::Marker::CYLINDER;
  };

  void visualizationCallback(const ros::TimerEvent& event) override
  {
    // Call visualizationCallback of the parent class which is overwritten here
    // This will publish a marker for the state estimate
    CarEstimatorVisualizer<StateType>::visualizationCallback(event);

    // Everything below is for the covariance visualization of the position estimate of the EKF
    estimate_covariance.header.stamp = ros::Time::now();
    estimate_covariance.points.clear();

    geometry_msgs::Point temp_point;
    auto car_kalman_ptr = std::dynamic_pointer_cast<crs_estimators::kalman::KalmanEstimator<StateType, InputType>>(
        BaseEstimatorVisualizer<StateType>::estimator_);

    StateType estimate = car_kalman_ptr->getStateEstimate();

    estimate_covariance.pose.position.x = estimate.pos_x;
    estimate_covariance.pose.position.y = estimate.pos_y;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, StateType::NX, StateType::NX>> eig(
        car_kalman_ptr->getPosteriorCovariance());
    const Eigen::Matrix<double, StateType::NX, 1>& eigValues(eig.eigenvalues());
    const Eigen::Matrix<double, StateType::NX, StateType::NX>& eigVectors(eig.eigenvectors());
    double lengthMajor = sqrt(eigValues[0]);  // x position
    double lengthMinor = sqrt(eigValues[1]);  // y position

    estimate_covariance.scale.x = lengthMajor * covariance_scale;
    estimate_covariance.scale.y = lengthMinor * covariance_scale;
    estimate_covariance.scale.z = 0.01;

    estimate_covariance.pose.orientation.w = std::cos(estimate.yaw * 0.5);
    estimate_covariance.pose.orientation.x = 0;
    estimate_covariance.pose.orientation.y = 0;
    estimate_covariance.pose.orientation.z = std::sin(estimate.yaw * 0.5);
    if (is_valid_marker(estimate_covariance))
      cov_visualization_publisher_.publish(estimate_covariance);
  }
};
};  // namespace ros_estimators

#endif /* SRC_ROS_ROS_ESTIMATORS_INCLUDE_ROS_ESTIMATORS_VISUALIZERS_CAR_EKF_VISUALIZER */
