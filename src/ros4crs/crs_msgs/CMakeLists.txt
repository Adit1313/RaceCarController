cmake_minimum_required(VERSION 3.0.2)
project(crs_msgs)

find_package(catkin REQUIRED COMPONENTS std_msgs message_generation)

add_message_files(
  DIRECTORY
  msg/car/core_messages
  FILES
  car_com.msg
  car_state_cart.msg
  car_with_imu_bias_state_cart.msg
  car_input.msg
  car_steer_state.msg
  car_wheel_speed.msg)

add_message_files(
  DIRECTORY
  msg/car/additional_messages
  FILES
  lighthouse_frame.msg
  lighthouse_sweep.msg
  car_actuator_inputs.msg
  car_clock.msg
  penalty_and_lap_count.msg)

add_message_files(DIRECTORY msg/rocket/core_messages FILES rocket_state.msg
                  rocket_input.msg)

add_message_files(DIRECTORY msg/rocket/additional_messages FILES
                  rocket_actuator_input.msg)

add_message_files(DIRECTORY msg/common FILES bool_array_stamped.msg
                  double_array_stamped.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs message_runtime)

include_directories(${catkin_INCLUDE_DIRS})
