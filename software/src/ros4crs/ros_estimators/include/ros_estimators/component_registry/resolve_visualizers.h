#ifndef SRC_ROS_ROS_ESTIMATORS_INCLUDE_ROS_ESTIMATORS_COMPONENT_REGISTRY_RESOLVE_VISUALIZERS
#define SRC_ROS_ROS_ESTIMATORS_INCLUDE_ROS_ESTIMATORS_COMPONENT_REGISTRY_RESOLVE_VISUALIZERS

#include <ros/ros.h>
#include "ros_estimators/visualizers/base_visualizer.h"
#include <estimators/base_estimator.h>
#include <estimators/model_based_estimator.h>

namespace registry
{
namespace estimators
{
namespace visualizers
{
/**
 * @brief Loads the visualizer for the specified StateType, InputType, and VisualizerType. Using parameters from the
 * parameter server stored under nh if needed.
 *
 * @tparam StateType The type of the state.
 * @tparam InputType The type of the input.
 * @tparam VisualizerType The type of the visualizer to load (e.g. ros_estimators::CarVisualizer,
 * ros_estimator::CarEkfVisualizer).
 *
 * @param nh The nodehandle to load the visualizer from (e.g. for cars: /carX/ros_estimators/visualizer).
 *
 * @return std::unique_ptr<ros_estimators::BaseEstimatorVisualizer<StateType>> A unique pointer to the loaded
 * visualizer.
 */
template <typename StateType, typename InputType, typename VisualizerType>
std::unique_ptr<ros_estimators::BaseEstimatorVisualizer<StateType>>
loadEstimatorVisualizer(const ros::NodeHandle& nh, std::shared_ptr<crs_estimators::BaseEstimator<StateType>> estimator);

/**
 * @brief Parses the visualizers for the specified StateType.
 *
 * This function will parse the visualizers for the specified StateType from the parameter server.
 *
 * More precisely, it will first try to load the "type" field from the private nodehandle.
 * Given the type of visualizer and specified state type, it will call the corresponding (specialized)
 * loadEstimatorVisualizer function.
 *
 * @tparam StateType The type of the state.
 *
 * @param nh_private The private nodehandle to load the visualizers from (e.g. for cars:
 * /carX/ros_estimators/visualizer).
 * @param estimator The estimator to load the visualizers for.
 *
 * @return std::unique_ptr<ros_estimators::BaseEstimatorVisualizer<StateType>> A unique pointer to the loaded
 * visualizer.
 */
template <typename StateType>
std::unique_ptr<ros_estimators::BaseEstimatorVisualizer<StateType>>
parseVisualizers(const ros::NodeHandle& nh_private,
                 std::shared_ptr<crs_estimators::BaseEstimator<StateType>> estimator);

}  // namespace visualizers
}  // namespace estimators
}  // namespace registry
#endif /* SRC_ROS_ROS_ESTIMATORS_INCLUDE_ROS_ESTIMATORS_COMPONENT_REGISTRY_RESOLVE_VISUALIZERS */
