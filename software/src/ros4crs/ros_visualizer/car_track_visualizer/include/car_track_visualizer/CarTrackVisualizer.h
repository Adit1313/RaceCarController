#ifndef CAR_TRACK_VISUALIZER_CARTRACKVISUALIZER_H
#define CAR_TRACK_VISUALIZER_CARTRACKVISUALIZER_H

#include <commons/static_track_trajectory.h>
#include <crs_msgs/car_state_cart.h>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

//!  CarTrackVisualizer
/*!
 *
  Subscribes to the state of the vehicle and updates a marker based on
  * the state of the vehicle. The marker is published to visualization_marker.
  * Loads and publishes the track.
*/
class CarTrackVisualizer
{
public:
  //!  constructor
  /*!
   * \param n ros::NodeHandle.
   */
  CarTrackVisualizer(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

  //!
  /*!
   * subscribes to /car_state and calls stateCallback.
   */
  void subscribeToState();

  //!
  /*!
   * Callback for subscriber
   * \param n crs_msgs::car_state_cart::ConstPtr&
   * \see subscribeToState()
   */
  void stateCallback(const boost::shared_ptr<crs_msgs::car_state_cart const> msg, bool is_estimation);

  //!
  /*!
   * Creates a new topic "/visualization_marker". rviz can display
   * markers published to this topic
   *
   */
  void setupPublisher();

  //!
  /*!
   * sets up the marker, which represents the vehicle for rviz
   *
   */
  void setupMarker();

  //!
  /*!
   * based on car_state_ the position and orientation of the marker
   * is updated
   *
   */
  void updateMarker();

  //!
  /*!
   * published the marker onto /viusalization_marker
   *
   */
  void publishMarker();

  /** Publish the track center line and boundaries as a marker array to @ref static_track_pub. */
  void publishTrack();

  //!
  /*!
   * updates and publishes the marker
   * \see updateMarker()
   * \see publishMarker()
   *
   */
  void run();

  //!
  /*!
   * loads parameters from param server
   *
   */
  void loadParameters();

  //!
  /*!
   * sets up the data structure for the center line of the track and the track boundaries
   *
   */
  void setupTrack();

  //!
  /*!
   * returns the node rate to be used in the main function to set the node rate
   *
   */
  float getNodeRate();

private:
  ros::NodeHandle& nh_;
  ros::NodeHandle& nh_private_;
  float node_rate_;

  bool got_car_state_ = false;
  crs_msgs::car_state_cart car_state_;
  bool got_est_car_state_ = false;
  crs_msgs::car_state_cart car_state_estimated_;

  ros::Publisher pub_;

  /**
   * @brief A "static" publisher on the /track topic
   *
   * Since the track does not update often, the topic is latched, and displays
   * the center line and boundaries.
   */
  ros::Publisher static_track_pub_;

  ros::Subscriber sub_gt_;
  ros::Subscriber sub_est_;
  visualization_msgs::Marker gt_car_marker_;
  visualization_msgs::Marker est_car_marker_;
  tf2::Quaternion q_;

  /** Pre-filled message for the track. */
  visualization_msgs::MarkerArray track_msg_;

  std::string frame_name_ = "crs_frame";

  std::shared_ptr<crs_controls::StaticTrackTrajectory> static_track_trajectory_;

  std::string car_namespace_ = "";

  int point_downsampling_factor_ = 1;
  bool show_track_angle_ = false;

  bool show_past_gt_trajectory_ = false;
  bool show_past_est_trajectory_ = false;
  bool always_publish_track_ = false;
  int number_of_past_samples_ = -1;

  bool publish_track_ = true;

  float min_velocity_ = 1.5;
  float max_velocity_ = 2.5;

  bool lloyd_flag_ = false;

  visualization_msgs::Marker gt_trajectory_;
  visualization_msgs::Marker est_trajectory_;
  // cache for rate limitation of callbacks
  double last_gt_callback_ = 0;
  double last_est_callback_ = 0;

  const float TRACK_SCALE_ = 0.03;
  const float CAR_SCALE_ = 0.0015;
  const float ORANGE_[4] = { 1.0, 0.5, 0.0, 1.0 };
  const float BLACK_[4] = { 0.0, 0.0, 0.0, 1.0 };
  float COLOR_CAR_EST_[4] = { 1.0, 0.0, 0.0, 1.0 };
  float COLOR_CAR_GT_[4] = { 0.0, 1.0, 0.0, 0.7 };
};
#endif
