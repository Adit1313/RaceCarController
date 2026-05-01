#ifndef SRC_ROS_ROS_ESTIMATORS_INCLUDE_ROS_ESTIMATORS_COMPONENT_REGISTRY_RESOLVE_ESTIMATOR
#define SRC_ROS_ROS_ESTIMATORS_INCLUDE_ROS_ESTIMATORS_COMPONENT_REGISTRY_RESOLVE_ESTIMATOR

#include "ros_estimators/state_estimator_ros.h"
#include "ros_estimators/car_estimator/car_estimator.h"
#include <ros/ros.h>

#include "ros_estimators/component_registry/resolve_sensor_models.h"
#include "ros_estimators/component_registry/resolve_visualizers.h"

namespace registry
{
namespace estimators
{
/**
 * @brief Loads the CRS estimator given the desired EstimatorType.
 *
 * This function will load the  CRS estimator using the parameters specified in the parameter server.
 *
 * It requires two nodehandles, one for the global namespace and one for the private namespace.
 * The global namespace is used to load the model parameters. It may be overwritten by the private namespace, but this
 * is not required. The private namespace is used to load all estimator specific parameters, such as the initial state,
 * covariance matrix, etc.
 *
 * @tparam EstimatorType The type of the CRS estimator to load.
 *
 * @param nh The global nodehandle (i.e. /carX for cars.).
 * @param nh_private The private nodehandle. (i.e. /car_X/estimation_node for cars.)
 */
template <typename EstimatorType>
std::shared_ptr<EstimatorType> loadCRSEstimator(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

/**
 * @brief Loads the RosCarEstimator for the specified StateType, InputType, and StateDimension.
 *
 * This function returns a POINTER to the loaded RosCarEstimator. Make sure to delete it when you are done with it.
 *
 * The RosCarEstimator wraps around a CRS estimator and provides a ROS interface to it.
 *
 * This function will first load the internal CRS estimator for the specified StateType, InputType, and StateDimension.
 * If the estimator is a model based estimator, it will try to load sensor models for the estimator that are specified
 * in the parameter server.
 *
 * Finally, it will try to load visualizers for the estimator that are specified in the parameter server.
 *
 * @tparam StateType The type of the state.
 * @tparam InputType The type of the input.
 * @tparam StateDimension The dimension of the state.
 * @tparam EstimatorType The type of the CRS estimator to load.
 * @tparam ParamsType The type of the model that is needed to convert a crs state message to a ros car state message.
 *  For the pacejka model, this is empty, since the pacejka state can be directly parsed to a ros car state.
 *  For the kinematic model, this is crs_models::kinematic_model::kinematic_params since the kinematic state needs to be
 * converted to a ros car state and needs parameter information to do so.
 *
 * @return ros_estimators::RosCarEstimator<StateType, InputType, StateDimension, ParamsType>* A POINTER to the loaded
 * RosCarEstimator.
 */
template <typename StateType, typename InputType, typename EstimatorType,
          typename ParamType = parameter_io::empty_params>
ros_estimators::RosCarEstimator<StateType, InputType, ParamType>* loadCarEstimator(ros::NodeHandle& nh,
                                                                                   ros::NodeHandle& nh_private)
{
  // LOAD Estimator
  std::shared_ptr<EstimatorType> estimator = loadCRSEstimator<EstimatorType>(nh, nh_private);

  // Downcast estimator to modelbased estimator type
  auto model_based_estimator =
      std::dynamic_pointer_cast<crs_estimators::ModelBasedEstimator<StateType, InputType>>(estimator);

  std::vector<std::string> loaded_sensors;
  if (model_based_estimator)  // Only model based estimators support sensor models
  {
    // LOAD SENSOR MODELS
    loaded_sensors = parseSensorModels<StateType, InputType>(nh, nh_private, model_based_estimator);
    // Downcast modelbased estimator to base estimator type used to load visualizer
    auto base_estimator = std::dynamic_pointer_cast<crs_estimators::BaseEstimator<StateType>>(model_based_estimator);
    // LOAD VISUALIZER
    std::unique_ptr<ros_estimators::BaseEstimatorVisualizer<StateType>> visualizer_ptr =
        visualizers::parseVisualizers<StateType>(nh_private, base_estimator);

    return new ros_estimators::RosCarEstimator<StateType, InputType, ParamType>(
        nh, nh_private, std::move(visualizer_ptr), loaded_sensors, model_based_estimator);
  }

  // Downcast modelbased estimator to base estimator type used to load visualizer
  auto base_estimator = std::dynamic_pointer_cast<crs_estimators::BaseEstimator<StateType>>(estimator);
  // LOAD VISUALIZER
  std::unique_ptr<ros_estimators::BaseEstimatorVisualizer<StateType>> visualizer_ptr =
      visualizers::parseVisualizers<StateType>(nh_private, base_estimator);

  return new ros_estimators::RosCarEstimator<StateType, InputType, ParamType>(nh, nh_private, std::move(visualizer_ptr),
                                                                              loaded_sensors, base_estimator);
};

}  // namespace estimators
}  // namespace registry
#endif /* SRC_ROS_ROS_ESTIMATORS_INCLUDE_ROS_ESTIMATORS_COMPONENT_REGISTRY_RESOLVE_ESTIMATOR */
