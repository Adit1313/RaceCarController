#ifndef ROS_ESTIMATORS_VISUALIZERS_BASE_VISUALIZER_H
#define ROS_ESTIMATORS_VISUALIZERS_BASE_VISUALIZER_H

#include <estimators/base_estimator.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

namespace ros_estimators
{

/**
 * @brief Visualizer that shows estimated states.
 * This creates a timer that calls the visualizationCallback at a specified rate internally.
 *
 * This is an abstract class. The method visualizationCallback needs to be implemented in a derived class.
 *
 * @tparam StateType
 */
template <typename StateType>
class BaseEstimatorVisualizer
{
private:
  double visualization_rate = 10;

protected:
  ros::Publisher est_visualization_publisher_;
  ros::Timer visualization_timer_;
  std::shared_ptr<crs_estimators::BaseEstimator<StateType>> estimator_;

public:
  BaseEstimatorVisualizer(ros::NodeHandle nh_private,
                          std::shared_ptr<crs_estimators::BaseEstimator<StateType>> estimator)
    : estimator_(estimator)
  {
    est_visualization_publisher_ =
        nh_private.advertise<visualization_msgs::Marker>("state_estimate_visualization", 100);

    if (!nh_private.getParam("rate", visualization_rate))
    {
      visualization_rate = 10.0;
      ROS_WARN_STREAM("No visualization rate specified for estimator visualizer! Checked namespace:"
                      << nh_private.getNamespace() << " Defaulting to " << visualization_rate << " Hz");
    }

    if (visualization_rate != 0.0)
      visualization_timer_ = nh_private.createTimer(ros::Duration(1 / visualization_rate),
                                                    &BaseEstimatorVisualizer::visualizationCallback, this);
  };

  virtual void visualizationCallback(const ros::TimerEvent& event) = 0;
};
}  // namespace ros_estimators

#endif  // ROS_ESTIMATORS_VISUALIZERS_BASE_VISUALIZER_H
