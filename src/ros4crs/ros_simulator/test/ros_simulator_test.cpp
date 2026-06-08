#include <gtest/gtest.h>
#include <ros/ros.h>

#include <crs_msgs/car_input.h>
#include <crs_msgs/car_state_cart.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

std::shared_ptr<ros::NodeHandle> nh;

ros::Publisher input_publisher;
ros::Publisher measurement_publisher;

void publish_input_msg(float steer, float torque)
{
  crs_msgs::car_input input;
  input.header.stamp = ros::Time::now();
  input.steer = steer;
  input.torque = torque;
  input_publisher.publish(input);
}

/**
 * @brief initial input is just torque. Test if the state changes if no input is specified.
 *
 */
TEST(KalmanTestSuite, testSimulatorForwardInput)
{
  publish_input_msg(0.0, 0.3);
  ros::spinOnce();
  ros::Duration(0.2).sleep();  // sleep for 0.2s
  ros::spinOnce();
  // Current state estimate
  auto state_msg = ros::topic::waitForMessage<crs_msgs::car_state_cart>("/ros_simulator/gt_state");
  publish_input_msg(0.0, 0.3);
  ros::spinOnce();

  ros::Duration(1).sleep();  // sleep for 0.2s
  auto new_state_msg = ros::topic::waitForMessage<crs_msgs::car_state_cart>("/ros_simulator/gt_state");
  EXPECT_NE(new_state_msg->x, state_msg->x);
}

/**
 * @brief Test if state changes when a steer input is applied.
 *
 */
TEST(KalmanTestSuite, testSimulatorSteerInput)
{
  // Current state estimate
  auto state_msg = ros::topic::waitForMessage<crs_msgs::car_state_cart>("/ros_simulator/gt_state");
  // publish input
  publish_input_msg(0.3, 0.3);
  ros::Duration(0.2).sleep();  // sleep for 0.2s
  ros::spinOnce();
  auto new_state_msg = ros::topic::waitForMessage<crs_msgs::car_state_cart>("/ros_simulator/gt_state");
  EXPECT_NE(new_state_msg->x, state_msg->x);

  // Make sure yaw rate is not zero after steer input
  EXPECT_GT(std::abs(new_state_msg->dyaw), 0);
}

/**
 * @brief checks if the estimated state and measured state are similar
 *
 * @param state
 * @param mocap
 * @param imu
 */
void measurementCallback(const crs_msgs::car_state_cart::ConstPtr& state,
                         const geometry_msgs::TransformStamped::ConstPtr& mocap, const sensor_msgs::Imu::ConstPtr& imu)
{
  float epsilon = 0.05;

  // Mocap checks
  EXPECT_NEAR(state->x, mocap->transform.translation.x, epsilon);
  EXPECT_NEAR(state->y, mocap->transform.translation.y, epsilon);
  // IMU checks
  EXPECT_NEAR(state->dyaw, imu->angular_velocity.z, epsilon);
}

/**
 * @brief
 *
 */
TEST(KalmanTestSuite, testMeasruementsNoNoise)
{
  message_filters::Subscriber<crs_msgs::car_state_cart> state_sub(*nh, "/ros_simulator/gt_state", 1);
  message_filters::Subscriber<geometry_msgs::TransformStamped> mocap_sub(*nh, "/ros_simulator/mocap", 1);
  message_filters::Subscriber<sensor_msgs::Imu> imu_sub(*nh, "/ros_simulator/imu", 1);

  message_filters::TimeSynchronizer<crs_msgs::car_state_cart, geometry_msgs::TransformStamped, sensor_msgs::Imu> sync(
      state_sub, mocap_sub, imu_sub, 10);  // only use messages that come in at the same time

  sync.registerCallback(boost::bind(&measurementCallback, _1, _2, _3));  // call measurementCallback with the 3 messages
                                                                         // above.
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kalman_test");
  nh.reset(new ros::NodeHandle);
  testing::InitGoogleTest(&argc, argv);

  ros::Duration(5).sleep();  // Wait for ekf to start....
  input_publisher = nh->advertise<crs_msgs::car_input>("/ros_simulator/control_input", 10);
  measurement_publisher = nh->advertise<geometry_msgs::TransformStamped>("/ros_simulator/mocap", 10);

  return RUN_ALL_TESTS();
}
