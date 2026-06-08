#ifndef ROCKET_VISUALIZER_ROCKETVISUALIZER_H
#define ROCKET_VISUALIZER_ROCKETVISUALIZER_H

#include <crs_msgs/rocket_state.h>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_broadcaster.h>

/**  RocketVisualizer
 *
 * @brief Subscribes to the state of the rocket and updates a marker based on
 * the state of the rocket. The marker is published to visualization_marker.
 */

class RocketVisualizer
{
public:
  RocketVisualizer(ros::NodeHandle& nh, ros::NodeHandle& nh_private) : nh_(nh), nh_private_(nh_private)
  {
    loadParameters();
    subscribeToState();
    setupPublisher();
    setupMarker();
  }

  /**
   * @brief subscribes to /rocket_state and calls stateCallback.
   */
  void subscribeToState();

  /**
   * @brief Callback for subscriber
   * @param n crs_msgs::rocket_state const ptr&
   * @see subscribeToState()
   */
  void stateCallback(const boost::shared_ptr<crs_msgs::rocket_state const> msg, bool is_estimation);

  /**
   * @brief Creates a new topic "/visualization_marker". rviz can display
   * markers published to this topic
   */
  void setupPublisher();

  /**
   * @brief sets up the marker, which represents the rocket for rviz
   */
  void setupMarker();

  /**
   * @brief published the marker onto /viusalization_marker
   */
  void publishMarker();

  /**
   * @brief updates and publishes the marker
   * @see publishMarker()
   *
   * The way this visualizer works is that the visualization marker of the rocket is
   * fixed in the rocket frame and we simply update the transform between the world frame
   * and the rocket frame at every iteration.
   */
  void run();

  /**
   * @brief loads parameters from param server
   */
  void loadParameters();

  /**
   * @brief returns the node rate to be used in the main function to set the node rate
   */
  float getNodeRate();

  /**
   * @brief publish the transform from world to rocket frame
   */
  void publishTransform();

protected:
  ros::NodeHandle& nh_;
  ros::NodeHandle& nh_private_;
  float node_rate_;

  bool got_rocket_state_ = false;
  crs_msgs::rocket_state rocket_state_;
  bool got_est_rocket_state_ = false;
  crs_msgs::rocket_state rocket_state_estimated_;

  ros::Publisher pub_;
  ros::Subscriber sub_gt_;
  ros::Subscriber sub_est_;
  visualization_msgs::Marker gt_rocket_marker_;
  visualization_msgs::Marker est_rocket_marker_;
  visualization_msgs::Marker boundary_;
  const char FRAME_WORLD_[20] = "rocket_world_frame";
  const char FRAME_ROCKET_[20] = "rocket_frame";

  std::string rocket_namespace_ = "";

  bool show_past_gt_trajectory_ = false;
  bool show_past_est_trajectory_ = false;
  int number_of_past_samples_ = -1;

  float min_velocity_ = -5.0;
  float max_velocity_ = 5.0;

  visualization_msgs::Marker gt_trajectory_;
  visualization_msgs::Marker est_trajectory_;
  // cache for rate limitation of callbacks
  double last_gt_callback_ = 0;
  double last_est_callback_ = 0;

  const float ROCKET_SCALE_ = 0.001;
  const float ROCKET_TRAJECTORY_SCALE_ = 0.02;
  const float ORANGE_[4] = { 1.0, 0.5, 0.0, 1.0 };
  const float BLACK_[4] = { 0.0, 0.0, 0.0, 1.0 };
  float COLOR_ROCKET_EST_[4] = { 1.0, 0.0, 0.0, 1.0 };
  float COLOR_ROCKET_GT_[4] = { 0.0, 1.0, 0.0, 0.7 };

  tf2_ros::TransformBroadcaster rocket_tf_broadcaster_;
};
#endif /* ROCKET_VISUALIZER_ROCKETVISUALIZER_H */
