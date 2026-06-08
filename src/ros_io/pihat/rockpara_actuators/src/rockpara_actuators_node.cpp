#include "ros/ros.h"
#include "rockpara_actuators/rockpara_actuators.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rockpara_actuators");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private = ros::NodeHandle("~");

  ns_rockpara_actuators::RockparaActuators node(nh_private);
  ros::spin();

  return 0;
}
