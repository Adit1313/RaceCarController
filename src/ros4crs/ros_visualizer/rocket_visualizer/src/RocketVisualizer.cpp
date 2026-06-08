#include "rocket_visualizer/RocketVisualizer.h"

#include <cmath>

#include <Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros_crs_utils/parameter_io.h>

/**
 * @brief subscribes to /rocket_state and calls stateCallback.
 */
void RocketVisualizer::subscribeToState()
{
  sub_gt_ = nh_private_.subscribe<crs_msgs::rocket_state>(
      "rocket_state_gt", 10, boost::bind(&RocketVisualizer::stateCallback, this, _1, false));
  sub_est_ = nh_private_.subscribe<crs_msgs::rocket_state>(
      "rocket_state_est", 10, boost::bind(&RocketVisualizer::stateCallback, this, _1, true));
}

/**
 * @brief Callback for subscriber
 * @param n crs_msgs::rocket_state const ptr&
 * @see subscribeToState()
 */
void RocketVisualizer::stateCallback(const boost::shared_ptr<crs_msgs::rocket_state const> msg, bool is_estimation)
{
  auto& stamp = is_estimation ? last_est_callback_ : last_gt_callback_;
  if (msg->header.stamp.toSec() - stamp < 1 / getNodeRate())
    return;

  stamp = msg->header.stamp.toSec();

  if (is_estimation)
  {
    got_est_rocket_state_ = true;
    rocket_state_estimated_ = *msg;
  }
  else
  {
    got_rocket_state_ = true;
    rocket_state_ = *msg;
  }

  auto& trajectory_markers = is_estimation ? est_trajectory_ : gt_trajectory_;
  if ((show_past_est_trajectory_ && is_estimation) || (show_past_gt_trajectory_ && !is_estimation))
  {
    if (number_of_past_samples_ != -1 && trajectory_markers.points.size() > number_of_past_samples_)
    {
      trajectory_markers.points.erase(trajectory_markers.points.begin(),
                                      trajectory_markers.points.begin() +
                                          int(0.1 * number_of_past_samples_));  // remove first 10%
      trajectory_markers.colors.erase(trajectory_markers.colors.begin(),
                                      trajectory_markers.colors.begin() +
                                          int(0.1 * number_of_past_samples_));  // remove first 10%
    }
    geometry_msgs::Point p;
    p.x = msg->pos_x;
    p.y = msg->pos_y;
    p.z = msg->pos_z;
    trajectory_markers.points.push_back(p);

    std_msgs::ColorRGBA color;
    double normalized_velocity =
        std::max(0.0, std::min(1.0, (msg->vel_x - min_velocity_) / (max_velocity_ - min_velocity_)));
    color.r = normalized_velocity < 0 ? normalized_velocity : 1 - normalized_velocity;
    color.g = normalized_velocity < 0 ? 0 : normalized_velocity;
    color.b = normalized_velocity < 0 ? 1 - normalized_velocity : 0;
    trajectory_markers.colors.push_back(color);
    trajectory_markers.header.stamp = ros::Time(0);
  }
}

/**
 * @brief Creates a new topic "/visualization_marker". rviz can display
 * markers published to this topic
 */
void RocketVisualizer::setupPublisher()
{
  pub_ = nh_.advertise<visualization_msgs::Marker>("rocket_info", 200);
}

/**
 * @brief sets up the marker, which represents the rocket for rviz
 */
void RocketVisualizer::setupMarker()
{
  uint32_t shape = visualization_msgs::Marker::CUBE;

  gt_rocket_marker_.header.frame_id = FRAME_ROCKET_;
  gt_rocket_marker_.header.stamp = ros::Time(0);
  gt_rocket_marker_.ns = rocket_namespace_ + "_groundtruth";
  gt_rocket_marker_.id = 0;
  gt_rocket_marker_.lifetime = ros::Duration(5.0);

  // Set the marker type.
  gt_rocket_marker_.type = visualization_msgs::Marker::MESH_RESOURCE;
  gt_rocket_marker_.mesh_resource = "package://rocket_visualizer/config/emborocketh_v17.stl";
  gt_rocket_marker_.action = visualization_msgs::Marker::ADD;
  gt_rocket_marker_.pose.position.x = 0;
  gt_rocket_marker_.pose.position.y = 0;
  gt_rocket_marker_.pose.position.z = 0;
  gt_rocket_marker_.pose.orientation.x = 0.0;
  gt_rocket_marker_.pose.orientation.y = 0.7071;
  gt_rocket_marker_.pose.orientation.z = 0.0;
  gt_rocket_marker_.pose.orientation.w = 0.7071;

  // Set the scale of the marker (side lenghts of cube)
  gt_rocket_marker_.scale.x = ROCKET_SCALE_;
  gt_rocket_marker_.scale.y = ROCKET_SCALE_;
  gt_rocket_marker_.scale.z = ROCKET_SCALE_;

  // Set the color -- be sure to set alpha to something non-zero!
  gt_rocket_marker_.color.r = COLOR_ROCKET_GT_[0];
  gt_rocket_marker_.color.g = COLOR_ROCKET_GT_[1];
  gt_rocket_marker_.color.b = COLOR_ROCKET_GT_[2];
  gt_rocket_marker_.color.a = COLOR_ROCKET_GT_[3];

  // TODO(@naefjo): once we have an estimator and can actually make use of this marker,
  // we need to change the frame_id to the correct frame.
  est_rocket_marker_.header.frame_id = FRAME_ROCKET_;
  est_rocket_marker_.header.stamp = ros::Time(0);
  est_rocket_marker_.ns = rocket_namespace_ + "_estimated";
  est_rocket_marker_.id = 0;
  est_rocket_marker_.lifetime = ros::Duration(5.0);

  // Set the marker type.
  est_rocket_marker_.type = visualization_msgs::Marker::MESH_RESOURCE;
  est_rocket_marker_.mesh_resource = "package://rocket_visualizer/config/emborocketh_v17.stl";
  est_rocket_marker_.action = visualization_msgs::Marker::ADD;
  est_rocket_marker_.pose.position.x = 0;
  est_rocket_marker_.pose.position.y = 0;
  est_rocket_marker_.pose.position.z = 0;
  est_rocket_marker_.pose.orientation.x = 0.0;
  est_rocket_marker_.pose.orientation.y = 0.7071;
  est_rocket_marker_.pose.orientation.z = 0.0;
  est_rocket_marker_.pose.orientation.w = 0.7071;

  // Set the scale of the marker (side lenghts of cube)
  est_rocket_marker_.scale.x = ROCKET_SCALE_;
  est_rocket_marker_.scale.y = ROCKET_SCALE_;
  est_rocket_marker_.scale.z = ROCKET_SCALE_;

  // Set the color -- be sure to set alpha to something non-zero!
  est_rocket_marker_.color.r = COLOR_ROCKET_EST_[0];
  est_rocket_marker_.color.g = COLOR_ROCKET_EST_[1];
  est_rocket_marker_.color.b = COLOR_ROCKET_EST_[2];
  est_rocket_marker_.color.a = COLOR_ROCKET_EST_[3];

  // ===============================================
  //============== Trajectory history ==============
  // ===============================================

  est_trajectory_.header.frame_id = FRAME_WORLD_;
  est_trajectory_.header.stamp = ros::Time(0);
  est_trajectory_.ns = rocket_namespace_ + "_estimated_trajectory";
  est_trajectory_.id = 0;
  est_trajectory_.lifetime = ros::Duration(5.0);

  // Set the trajectory marker type.
  est_trajectory_.type = visualization_msgs::Marker::LINE_STRIP;
  est_trajectory_.action = visualization_msgs::Marker::ADD;
  est_trajectory_.pose.orientation.x = 0.0;
  est_trajectory_.pose.orientation.y = 0.0;
  est_trajectory_.pose.orientation.z = 0.0;
  est_trajectory_.pose.orientation.w = 1.0;

  // Set the scale of the marker (side lenghts of cube)
  est_trajectory_.scale.x = ROCKET_TRAJECTORY_SCALE_;
  est_trajectory_.scale.y = ROCKET_TRAJECTORY_SCALE_;
  est_trajectory_.scale.z = ROCKET_TRAJECTORY_SCALE_;

  // Set the color -- be sure to set alpha to something non-zero!
  est_trajectory_.color.r = 0;
  est_trajectory_.color.g = 1;
  est_trajectory_.color.b = 0;
  est_trajectory_.color.a = 1;

  gt_trajectory_.header.frame_id = FRAME_WORLD_;
  gt_trajectory_.header.stamp = ros::Time(0);
  gt_trajectory_.ns = rocket_namespace_ + "_groundtruth_trajectory";
  gt_trajectory_.id = 0;
  gt_trajectory_.lifetime = ros::Duration(5.0);

  // Set the trajectory marker type.
  gt_trajectory_.type = visualization_msgs::Marker::LINE_STRIP;
  gt_trajectory_.action = visualization_msgs::Marker::ADD;
  gt_trajectory_.pose.orientation.x = 0.0;
  gt_trajectory_.pose.orientation.y = 0.0;
  gt_trajectory_.pose.orientation.z = 0.0;
  gt_trajectory_.pose.orientation.w = 1.0;

  // Set the scale of the marker (side lenghts of cube)
  gt_trajectory_.scale.x = ROCKET_TRAJECTORY_SCALE_;
  gt_trajectory_.scale.y = ROCKET_TRAJECTORY_SCALE_;
  gt_trajectory_.scale.z = ROCKET_TRAJECTORY_SCALE_;

  // Set the color -- be sure to set alpha to something non-zero!
  gt_trajectory_.color.r = 0;
  gt_trajectory_.color.g = 1;
  gt_trajectory_.color.b = 0;
  gt_trajectory_.color.a = 1;
}

/**
 * @brief published the marker onto /viusalization_marker
 */
void RocketVisualizer::publishMarker()
{
  if (got_rocket_state_)
  {
    pub_.publish(gt_rocket_marker_);
  }
  if (got_est_rocket_state_)
  {
    pub_.publish(est_rocket_marker_);
  }

  if (show_past_gt_trajectory_)
  {
    pub_.publish(gt_trajectory_);
  }
  if (show_past_est_trajectory_)
  {
    pub_.publish(est_trajectory_);
  }
}

/**
 * @brief updates and publishes the marker
 * @see publishMarker()
 *
 * The way this visualizer works is that the visualization marker of the rocket is
 * fixed in the rocket frame and we simply update the transform between the world frame
 * and the rocket frame at every iteration.
 */
void RocketVisualizer::run()
{
  publishTransform();

  publishMarker();
}

/**
 * @brief loads parameters from param server
 */
void RocketVisualizer::loadParameters()
{
  ROS_INFO("Visualizer: loading visualizer parameters");
  // load node parameters
  if (!nh_private_.getParam("node_rate", node_rate_))
    ROS_WARN_STREAM("Visualizer: did not load visualizer node_rate.");

  // load node parameters
  nh_private_.getParam("trajectory/show_past_gt_trajectory", show_past_gt_trajectory_);
  nh_private_.getParam("trajectory/show_past_est_trajectory", show_past_est_trajectory_);
  nh_private_.getParam("trajectory/number_of_past_samples", number_of_past_samples_);

  nh_private_.getParam("estimate/color/r", COLOR_ROCKET_EST_[0]);
  nh_private_.getParam("estimate/color/g", COLOR_ROCKET_EST_[1]);
  nh_private_.getParam("estimate/color/b", COLOR_ROCKET_EST_[2]);
  nh_private_.getParam("estimate/color/a", COLOR_ROCKET_EST_[3]);

  nh_private_.getParam("gt/color/r", COLOR_ROCKET_GT_[0]);
  nh_private_.getParam("gt/color/g", COLOR_ROCKET_GT_[1]);
  nh_private_.getParam("gt/color/b", COLOR_ROCKET_GT_[2]);
  nh_private_.getParam("gt/color/a", COLOR_ROCKET_GT_[3]);

  nh_private_.getParam("rocket_namespace", rocket_namespace_);
}

/**
 * @brief returns the node rate to be used in the main function to set the node rate
 */
float RocketVisualizer::getNodeRate()
{
  return node_rate_;
}

/**
 * @brief publish the transform from world to rocket frame
 */
void RocketVisualizer::publishTransform()
{
  geometry_msgs::TransformStamped transform;
  transform.transform.translation.x = rocket_state_.pos_x;
  transform.transform.translation.y = rocket_state_.pos_y;
  transform.transform.translation.z = rocket_state_.pos_z;
  transform.transform.rotation.w = rocket_state_.quat_w;
  transform.transform.rotation.x = rocket_state_.quat_x;
  transform.transform.rotation.y = rocket_state_.quat_y;
  transform.transform.rotation.z = rocket_state_.quat_z;

  transform.header.stamp = ros::Time::now();
  transform.header.frame_id = FRAME_WORLD_;
  transform.child_frame_id = FRAME_ROCKET_;

  rocket_tf_broadcaster_.sendTransform(transform);
}
