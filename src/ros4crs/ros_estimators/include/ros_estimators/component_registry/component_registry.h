#ifndef SRC_ROS_ROS_ESTIMATORS_INCLUDE_ROS_ESTIMATORS_COMPONENT_REGISTRY_COPY
#define SRC_ROS_ROS_ESTIMATORS_INCLUDE_ROS_ESTIMATORS_COMPONENT_REGISTRY_COPY
#include "ros_estimators/state_estimator_ros.h"
#include "ros_estimators/car_estimator/car_estimator.h"
#include <ros/ros.h>

namespace registry
{
namespace estimators
{

ros_estimators::RosStateEstimator* parseEstimator(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

}  // namespace estimators
}  // namespace registry
#endif /* SRC_ROS_ROS_ESTIMATORS_INCLUDE_ROS_ESTIMATORS_COMPONENT_REGISTRY_COPY */
