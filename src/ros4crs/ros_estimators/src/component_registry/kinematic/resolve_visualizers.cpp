#include "ros_estimators/component_registry/component_registry.h"
#include "ros_estimators/data_converter.h"
#include <ros/ros.h>

#include <ros_crs_utils/parameter_io.h>
#include <ros_crs_utils/state_message_conversion.h>
#include <ros_estimators/car_estimator/car_estimator.h>

#include "ros_estimators/visualizers/base_visualizer.h"
#include "ros_estimators/visualizers/car_estimator_visualizer.h"
#include "ros_estimators/visualizers/car_ekf_visualizer.h"

#include <kinematic_model/kinematic_car_input.h>
#include <kinematic_model/kinematic_car_state.h>

#include <estimators/base_estimator.h>
#include <kalman_estimator/kalman_estimator.h>

#include "ros_estimators/component_registry/resolve_visualizers.h"

// Kinematic Model
typedef crs_models::kinematic_model::kinematic_car_state kinematic_car_state;
typedef crs_models::kinematic_model::kinematic_car_input kinematic_car_input;

// Estimator
typedef crs_estimators::BaseEstimator<kinematic_car_state> BaseEstimatorType;
typedef crs_estimators::kalman::KalmanEstimator<kinematic_car_state, kinematic_car_input> KalmanEstimatorType;

// Visualizer
typedef ros_estimators::CarEstimatorVisualizer<kinematic_car_state> CarEstimatorVisualizerType;
typedef ros_estimators::BaseEstimatorVisualizer<kinematic_car_state> BaseEstimatorVisualizerType;
typedef ros_estimators::CarEkfVisualizer<kinematic_car_state, kinematic_car_input> CarEkfVisualizerType;

namespace registry
{
namespace estimators
{
namespace visualizers
{

/**
 * @brief Load car visualizer for estimator. This only visualizes the state estimate.
 */
template <>
std::unique_ptr<BaseEstimatorVisualizerType>
loadEstimatorVisualizer<kinematic_car_state, kinematic_car_input, CarEstimatorVisualizerType>(
    const ros::NodeHandle& nh, std::shared_ptr<BaseEstimatorType> estimator)
{
  return std::make_unique<CarEstimatorVisualizerType>(nh, estimator);
}

/**
 * @brief Load EKF car visualizer for estimator. This visualizes the state estimate and covariance.
 */
template <>
std::unique_ptr<BaseEstimatorVisualizerType>
loadEstimatorVisualizer<kinematic_car_state, kinematic_car_input, CarEkfVisualizerType>(
    const ros::NodeHandle& nh, std::shared_ptr<BaseEstimatorType> estimator)
{
  auto kalman_ptr = std::dynamic_pointer_cast<KalmanEstimatorType>(estimator);
  if (!kalman_ptr)
  {
    ROS_ERROR_STREAM("Could not cast estimator to Kalman Estimator. Running with car_estimator_visualizer instead");
    return loadEstimatorVisualizer<kinematic_car_state, kinematic_car_input, CarEstimatorVisualizerType>(nh, estimator);
  }
  return std::make_unique<CarEkfVisualizerType>(nh, kalman_ptr);
}

template <>
std::unique_ptr<BaseEstimatorVisualizerType>
parseVisualizers<kinematic_car_state>(const ros::NodeHandle& nh_private,
                                      std::shared_ptr<BaseEstimatorType> base_estimator)
{
  std::unique_ptr<BaseEstimatorVisualizerType> visualizer_ptr;
  std::string type;
  if (!ros::NodeHandle(nh_private, "visualizer").getParam("type", type))
  {
    ROS_WARN("No Type specified for visualizer! No visualizer loaded for estimator. Available type options are: "
             "car_estimator_visualizer, car_ekf_visualizer");
    return std::unique_ptr<BaseEstimatorVisualizerType>(nullptr);
  }
  if (type == "car_estimator_visualizer")
  {
    std::unique_ptr<BaseEstimatorVisualizerType> visualizer_ptr =
        visualizers::loadEstimatorVisualizer<kinematic_car_state, kinematic_car_input, CarEstimatorVisualizerType>(
            ros::NodeHandle(nh_private, "visualizer"), base_estimator);
    return visualizer_ptr;
  }
  if (type == "car_ekf_visualizer")
  {
    std::unique_ptr<BaseEstimatorVisualizerType> visualizer_ptr =
        visualizers::loadEstimatorVisualizer<kinematic_car_state, kinematic_car_input, CarEkfVisualizerType>(
            ros::NodeHandle(nh_private, "visualizer"), base_estimator);
    return visualizer_ptr;
  }

  ROS_WARN_STREAM("Unknown Visualizer type specified! No visualizer loaded for estimator type: "
                  << type
                  << "Available type options are: "
                     "car_estimator_visualizer, car_ekf_visualizer");
  return nullptr;
}

}  // namespace visualizers
}  // namespace estimators
}  // namespace registry
