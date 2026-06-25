#ifndef SRC_ROS_ROS_ESTIMATORS_INCLUDE_ROS_ESTIMATORS_COMPONENT_REGISTRY_RESOLVE_SENSOR_MODELS
#define SRC_ROS_ROS_ESTIMATORS_INCLUDE_ROS_ESTIMATORS_COMPONENT_REGISTRY_RESOLVE_SENSOR_MODELS

#include <sensor_models/sensor_model.h>
#include <estimators/model_based_estimator.h>
#include <ros/ros.h>
#include <ros_crs_utils/parameter_io.h>

namespace registry
{
namespace estimators
{
template <typename SensorModelType, typename ParamType = parameter_io::empty_params>
std::shared_ptr<SensorModelType> loadSensorModel(const ros::NodeHandle& nh_private, const std::string sensor_name,
                                                 const ParamType params);

template <typename StateType, typename InputType>
std::vector<std::string>
parseSensorModels(ros::NodeHandle& nh, ros::NodeHandle& nh_private,
                  std::shared_ptr<crs_estimators::ModelBasedEstimator<StateType, InputType>> estimator);

}  // namespace estimators
}  // namespace registry
#endif /* SRC_ROS_ROS_ESTIMATORS_INCLUDE_ROS_ESTIMATORS_COMPONENT_REGISTRY_RESOLVE_SENSOR_MODELS */
