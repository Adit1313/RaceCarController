#include <memory>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "crs_msgs/rocket_state.h"

#include "rocket_visualizer/RocketVisualizer.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rocket_visualizer");

  ros::NodeHandle nh = ros::NodeHandle("");           // /<NAMESPACE>/*
  ros::NodeHandle nh_private = ros::NodeHandle("~");  // /<NAMESPACE>/rocket_visualizer/*

  std::shared_ptr<RocketVisualizer> visualizer = std::make_shared<RocketVisualizer>(nh, nh_private);

  while (ros::ok())
  {
    ros::spinOnce();
    visualizer->run();
    ros::Duration(1 / visualizer->getNodeRate()).sleep();
  }
  return 0;
}
