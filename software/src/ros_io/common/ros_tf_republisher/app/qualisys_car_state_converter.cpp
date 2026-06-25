/**
 * @file    qualisys_car_state_converter.cpp
 * @author  Joshua Näf
 * @brief   Node that transforms qualisys data to car state msgs.
 */

#include <ros/ros.h>

#include <cmath>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Header.h>
#include <crs_msgs/car_state_cart.h>

#include "commons/filter.h"

#include <eigen_conversions/eigen_msg.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

double normalizeAngle(double angle)
{
  angle = fmod(angle + M_PI, 2 * M_PI);  // Convert to [0, 2*pi)
  if (angle < 0)
    angle += 2 * M_PI;  // Adjust negative angles
  return angle - M_PI;  // Shift range to [-pi, pi)
}

double smallestAngleDifference(double angle1, double angle2)
{
  // Normalize angles to [-pi, pi)
  angle1 = normalizeAngle(angle1);
  angle2 = normalizeAngle(angle2);

  // Compute the difference between angles
  return std::atan2(std::sin(angle2 - angle1), std::cos(angle2 - angle1));
}

int main(int argc, char** argv)
{
  geometry_msgs::PoseStamped current_pose;
  geometry_msgs::PoseStamped track_pose;
  bool received_initial_pose = false;
  bool received_track_pose = false;

  int loop_counter_ = 0;

  double prev_yaw = 0;

  ros::init(argc, argv, "qualisys_car_state_converter");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private = ros::NodeHandle("~");
  ros::Publisher state_publisher = nh_private.advertise<crs_msgs::car_state_cart>("output_topic", 1);

  std::vector<double> b_num;
  std::vector<double> a_denom;

  if (!(nh_private.getParam("b_num", b_num) && nh_private.getParam("a_denom", a_denom)))
  {
    ROS_ERROR_STREAM("[Republisher Node]: Could not load filter params!");
  }
  Filter FilterOmegaX = Filter(b_num, a_denom);
  Filter FilterOmegaY = Filter(b_num, a_denom);
  Filter FilterOmegaZ = Filter(b_num, a_denom);

  ros::Subscriber pose_subscriber = nh_private.subscribe<geometry_msgs::PoseStamped>(
      "car_pose", 1, [&current_pose, &received_initial_pose](const geometry_msgs::PoseStamped::ConstPtr msg) {
        current_pose = *msg;
        received_initial_pose = true;
      });

  ros::Subscriber track_subscriber = nh_private.subscribe<geometry_msgs::PoseStamped>(
      "track_pose", 1, [&track_pose, &received_track_pose](const geometry_msgs::PoseStamped::ConstPtr msg) {
        track_pose = *msg;
        received_track_pose = true;
      });

  ros::Subscriber twist_subscriber = nh_private.subscribe<geometry_msgs::TwistStamped>(
      "car_twist", 1, [&](const geometry_msgs::TwistStamped::ConstPtr msg) {
        // Pose msg: I_p, q_IB
        // Twist msg: I_v, I_omega_IB

        if (!received_initial_pose || !received_track_pose)
        {
          return;
        }

        tf2::Transform transform_T_world_track, transform_T_world_car;
        tf2::fromMsg(track_pose.pose, transform_T_world_track);
        tf2::fromMsg(current_pose.pose, transform_T_world_car);

        // Compute the pose T_AB (pose of frame B relative to frame A)
        tf2::Transform transform_T_track_car = transform_T_world_track.inverseTimes(transform_T_world_car);

        // Convert tf2::Transform back to geometry_msgs::PoseStamped
        geometry_msgs::PoseStamped T_track_car;
        tf2::toMsg(transform_T_track_car, T_track_car.pose);

        crs_msgs::car_state_cart out_msg;
        out_msg.header = msg->header;

        out_msg.x = T_track_car.pose.position.x;
        out_msg.y = T_track_car.pose.position.y;
        out_msg.z = T_track_car.pose.position.z;

        tf2::Matrix3x3 car_as_rot_mat;
        car_as_rot_mat.setRotation(transform_T_track_car.getRotation());

        // Extract yaw
        tf2Scalar curr_yaw, pitch, roll;
        car_as_rot_mat.getRPY(roll, pitch, curr_yaw);

        double yaw_raw_diff = prev_yaw - curr_yaw;
        prev_yaw = curr_yaw;
        if (yaw_raw_diff >= M_PI)
          loop_counter_++;
        else if (yaw_raw_diff <= -M_PI)
          loop_counter_--;

        if (std::abs(loop_counter_) <= 1)
          out_msg.yaw = loop_counter_ * M_PI + ((0 < loop_counter_) - (loop_counter_ < 0)) * M_PI + curr_yaw;
        else
          out_msg.yaw = loop_counter_ * 2 * M_PI + curr_yaw;

        Eigen::Quaterniond q_IB;
        tf::quaternionMsgToEigen(T_track_car.pose.orientation, q_IB);

        Eigen::Vector3d I_omega_IB;
        tf::vectorMsgToEigen(msg->twist.angular, I_omega_IB);
        Eigen::Vector3d B_omega_IB = q_IB.inverse() * I_omega_IB;

        out_msg.dyaw = B_omega_IB.z();

        Eigen::Vector3d I_v;
        tf::vectorMsgToEigen(msg->twist.linear, I_v);
        Eigen::Vector3d B_v = q_IB.inverse() * I_v;

        out_msg.vx_b = B_v.x();
        out_msg.vy_b = B_v.y();

        state_publisher.publish(out_msg);
      });

  ros::spin();

  return 0;
}
