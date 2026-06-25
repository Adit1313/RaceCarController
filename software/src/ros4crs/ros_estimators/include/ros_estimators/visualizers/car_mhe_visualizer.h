#ifndef SRC_ROS_ROS_ESTIMATORS_INCLUDE_ROS_ESTIMATORS_VISUALIZERS_CAR_MHE_VISUALIZER
#define SRC_ROS_ROS_ESTIMATORS_INCLUDE_ROS_ESTIMATORS_VISUALIZERS_CAR_MHE_VISUALIZER
#include <ros/ros.h>

#include <estimators/base_estimator.h>
#include <estimators/model_based_estimator.h>
#include <mh_estimator/pacejka_mhe.h>

#include <ros_crs_utils/state_message_conversion.h>
#include <visualization_msgs/Marker.h>

#include "ros_estimators/visualizers/base_visualizer.h"
#include "ros_estimators/visualizers/car_estimator_visualizer.h"
#include <ros_crs_utils/validation.h>

namespace ros_estimators
{
/**
 * @brief Visualizer that shows the state estimate and covariance of a MHE.
 */
template <typename StateType>
class CarMheVisualizer : public CarEstimatorVisualizer<StateType>
{
private:
  // Use arrow to also show mhe solution and reference yaw angle
  bool use_arrows = false;
  // Configurable parameters
  std::string frame_id = "crs_frame";

  double traj_est_size_x = 0.05;
  double traj_est_size_y = 0.05;
  double traj_est_size_z = 0.05;

  double traj_ref_size_x = 0.05;
  double traj_ref_size_y = 0.05;
  double traj_ref_size_z = 0.05;

  double traj_ref_color_r = 0;
  double traj_ref_color_g = 0;
  double traj_ref_color_b = 0;
  double traj_ref_color_a = 0;

  double min_velocity = 1.5;
  double max_velocity = 2.5;

  std::string ns = "";

  visualization_msgs::Marker trajectory_reference;
  visualization_msgs::Marker trajectory_estimate;

protected:
  ros::Publisher mhe_traj_ref_visualization_publisher_;
  ros::Publisher mhe_traj_est_visualization_publisher_;

public:
  CarMheVisualizer(ros::NodeHandle nh_private, std::shared_ptr<crs_estimators::mhe::Pacejka_MHE> estimator)
    : CarEstimatorVisualizer<StateType>(nh_private,
                                        std::dynamic_pointer_cast<crs_estimators::BaseEstimator<StateType>>(estimator))
  {
    mhe_traj_ref_visualization_publisher_ =
        nh_private.advertise<visualization_msgs::Marker>("traj_ref_visualization", 100);
    mhe_traj_est_visualization_publisher_ =
        nh_private.advertise<visualization_msgs::Marker>("traj_est_visualization", 100);

    nh_private.getParam("use_arrows", use_arrows);

    nh_private.getParam("frame_id", frame_id);

    nh_private.getParam("min_velocity", min_velocity);
    nh_private.getParam("max_velocity", max_velocity);

    nh_private.getParam("planned/size_x", traj_est_size_x);
    nh_private.getParam("planned/size_y", traj_est_size_y);
    nh_private.getParam("planned/size_z", traj_est_size_z);

    nh_private.getParam("reference/r", traj_ref_color_r);
    nh_private.getParam("reference/g", traj_ref_color_g);
    nh_private.getParam("reference/b", traj_ref_color_b);
    nh_private.getParam("reference/a", traj_ref_color_a);
    nh_private.getParam("reference/size_x", traj_ref_size_x);
    nh_private.getParam("reference/size_y", traj_ref_size_y);
    nh_private.getParam("reference/size_z", traj_ref_size_z);

    nh_private.getParam("namespace", ns);

    // Reference on track
    trajectory_reference.header.frame_id = frame_id;
    trajectory_reference.ns = ns + "_reference_trajectory";
    trajectory_reference.id = 1;
    trajectory_reference.scale.x = traj_ref_size_x;
    trajectory_reference.scale.y = traj_ref_size_y;
    trajectory_reference.scale.z = traj_ref_size_z;
    // Reference has static color as there is no target velocity on the reference
    trajectory_reference.color.r = traj_ref_color_r;
    trajectory_reference.color.g = traj_ref_color_g;
    trajectory_reference.color.b = traj_ref_color_b;
    trajectory_reference.color.a = traj_ref_color_a;
    trajectory_reference.action = visualization_msgs::Marker::ADD;
    trajectory_reference.type = use_arrows ? visualization_msgs::Marker::ARROW : visualization_msgs::Marker::POINTS;

    // Planned trajectory
    trajectory_estimate.header.frame_id = frame_id;
    trajectory_estimate.ns = ns + "_planned_trajectory";
    trajectory_estimate.id = 0;
    trajectory_estimate.scale.x = traj_est_size_x;
    trajectory_estimate.scale.y = traj_est_size_y;
    trajectory_estimate.scale.z = traj_est_size_z;
    trajectory_estimate.action = visualization_msgs::Marker::ADD;
    trajectory_estimate.type = use_arrows ? visualization_msgs::Marker::ARROW : visualization_msgs::Marker::POINTS;
  };

  void visualizationCallback(const ros::TimerEvent& event) override
  {
    // Call visualizationCallback of the parent class which is overwritten here
    // This will publish a marker for the state estimate
    CarEstimatorVisualizer<StateType>::visualizationCallback(event);

    trajectory_reference.header.stamp = ros::Time::now();
    trajectory_estimate.header.stamp = ros::Time::now();
    trajectory_estimate.points.clear();
    trajectory_estimate.colors.clear();
    trajectory_reference.points.clear();

    geometry_msgs::Point temp_point;

    auto mhe_estimator_ptr =
        std::dynamic_pointer_cast<crs_estimators::mhe::Pacejka_MHE>(BaseEstimatorVisualizer<StateType>::estimator_);

    int id_cnter = 0;  // Creates unique IDs for the markers. Only used in arrow mode
    std::vector<std::vector<double>> trajectory = mhe_estimator_ptr->getPlannedTrajectory();
    for (int i = 0; i < trajectory.size(); i++)
    {
      // Create color gradient from blue (lowest) to red (highest) depending on velocity
      std_msgs::ColorRGBA color;
      double normalized_velocity =
          std::max(0.0, std::min(1.0, (trajectory[i][2] - min_velocity) / (max_velocity - min_velocity)));
      color.r = normalized_velocity < 0.5 ? normalized_velocity : 1 - normalized_velocity;
      color.g = normalized_velocity < 0.5 ? 0 : normalized_velocity;
      color.b = normalized_velocity < 0.5 ? 1 - normalized_velocity : 0;
      color.a = 1;

      // ========= Do not use arrows (-> no yaw visualization)
      if (!use_arrows)
      {
        // Position
        temp_point.x = trajectory[i][0];  // x_position
        temp_point.y = trajectory[i][1];  // y_position
        temp_point.z = 0;
        trajectory_estimate.points.push_back(temp_point);
        trajectory_estimate.colors.push_back(color);
        if (trajectory[i][7] == 1.0)
        {  // Only add reference if it was a valid measurement
          // Refrence
          temp_point.x = trajectory[i][4];  // x_position on track
          temp_point.y = trajectory[i][5];  // y_position on track
          temp_point.z = 0.01;
          trajectory_reference.points.push_back(temp_point);
        }
      }
      else
      {
        // Planned position
        trajectory_estimate.pose.position.x = trajectory[i][0];
        trajectory_estimate.pose.position.y = trajectory[i][1];
        trajectory_estimate.pose.position.z = 0;
        // Planned orientation (quaternion from only yaw)
        trajectory_estimate.pose.orientation.w = std::cos(trajectory[i][3] * 0.5);
        trajectory_estimate.pose.orientation.x = 0;
        trajectory_estimate.pose.orientation.y = 0;
        trajectory_estimate.pose.orientation.z = std::sin(trajectory[i][3] * 0.5);

        trajectory_estimate.color.r = color.r;
        trajectory_estimate.color.g = color.g;
        trajectory_estimate.color.b = color.b;
        trajectory_estimate.color.a = color.a;

        trajectory_estimate.id = id_cnter++;

        if (trajectory[i][7] == 1.0)
        {  // Only add reference if it was a valid measurement
          // Reference position
          trajectory_reference.pose.position.x = trajectory[i][4];
          trajectory_reference.pose.position.y = trajectory[i][5];
          trajectory_reference.pose.position.z = 0;

          // Reference orientation (quaternion from only yaw)
          trajectory_reference.pose.orientation.w = std::cos(trajectory[i][6] * 0.5);
          trajectory_reference.pose.orientation.x = 0;
          trajectory_reference.pose.orientation.y = 0;
          trajectory_reference.pose.orientation.z = std::sin(trajectory[i][6] * 0.5);

          trajectory_reference.id = id_cnter++;
          if (is_valid_marker(trajectory_reference))
            mhe_traj_ref_visualization_publisher_.publish(trajectory_reference);
        }

        if (is_valid_marker(trajectory_estimate))
          mhe_traj_est_visualization_publisher_.publish(trajectory_estimate);
      }
    }

    if (!use_arrows)
    {
      // If we do not use arrows, only need to publish one marker containing all points.
      if (is_valid_marker(trajectory_reference))
        mhe_traj_ref_visualization_publisher_.publish(trajectory_reference);

      if (is_valid_marker(trajectory_estimate))
        mhe_traj_est_visualization_publisher_.publish(trajectory_estimate);
    }
  }
};
};  // namespace ros_estimators

#endif /* SRC_ROS_ROS_ESTIMATORS_INCLUDE_ROS_ESTIMATORS_VISUALIZERS_CAR_MHE_VISUALIZER */
