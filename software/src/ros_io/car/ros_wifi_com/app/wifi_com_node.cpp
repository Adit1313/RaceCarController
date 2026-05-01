/**
 * @file    wifi_com_node.cpp
 * @author  Lukas Vogel
 * @brief   Node that establishes WiFi communication through a WifiCom object.
 */

#include <ros/ros.h>
#include <wifi_com/wifi_com.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ros_wifi_com_node");

  ros::NodeHandle nh = ros::NodeHandle("");           // /<NAMESPACE>/*
  ros::NodeHandle nh_private = ros::NodeHandle("~");  // /<NAMESPACE>/ros_wifi_com/*

  WiFiCom com(nh);

  // Use 1 thread, spin() will not return until the node shuts down
  ros::MultiThreadedSpinner spinner(1);
  spinner.spin();

  return 0;
}
