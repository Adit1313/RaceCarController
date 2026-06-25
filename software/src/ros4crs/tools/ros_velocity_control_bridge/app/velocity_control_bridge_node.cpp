#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <crs_msgs/car_input.h>

ros::Publisher car_input_pub;
float steer_value;

void jointTrajectoryCallback(const trajectory_msgs::JointTrajectory::ConstPtr& trajectory_msg)
{
  // Extract the velocity from the JointTrajectory message
  double velocity = trajectory_msg->points[0].velocities[0];

  // TODO: Get the steer value from the car_input message
  crs_msgs::car_input car_input_msg;
  car_input_msg.steer = steer_value;
  car_input_msg.velocity = velocity;
  car_input_msg.torque = NAN;

  car_input_pub.publish(car_input_msg);
}

void carInputCallback(const crs_msgs::car_input::ConstPtr& car_input_msg)
{
  steer_value = car_input_msg->steer;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "velocity_control_bridge_node");
  ros::NodeHandle nh;

  // Subscribe to the JointTrajectory topic
  ros::Subscriber joint_trajectory_sub = nh.subscribe("trajectory", 1, jointTrajectoryCallback);

  // Subscribe to the car_input topic
  ros::Subscriber car_input_sub = nh.subscribe("car_input", 1, carInputCallback);

  // TODO: Create a publisher for the new car_input message
  car_input_pub = nh.advertise<crs_msgs::car_input>("velocity_input", 1);

  ros::MultiThreadedSpinner spinner(1);  // Use 1 threads
  spinner.spin();                        // spin() will not return until the node has been shutdown

  return 0;
}
