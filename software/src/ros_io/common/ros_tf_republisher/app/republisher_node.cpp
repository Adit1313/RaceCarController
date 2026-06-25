#include <ros/ros.h>
#include <ros_tf_republisher/republisher.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ros_tf_republisher");
  ros::NodeHandle nh = ros::NodeHandle("");           // /<NAMESPACE>/*
  ros::NodeHandle nh_private = ros::NodeHandle("~");  // /<NAMESPACE>/ros_tf_republisher/*

  int num_threads = nh.param("num_threads", 0);
  int queue_size = nh.param("queue_size", 20);

  ros_io::Republisher republisher_(nh, queue_size);

  ROS_INFO("Starting republisher with %d threads", num_threads);
  ros::MultiThreadedSpinner spinner(num_threads);
  spinner.spin();

  return 0;
}
