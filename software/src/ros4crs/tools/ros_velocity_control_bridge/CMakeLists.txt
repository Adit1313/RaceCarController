cmake_minimum_required(VERSION 3.0.2)
project(ros_velocity_control_bridge)

# Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin_simple REQUIRED)

catkin_simple()
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

cs_add_executable(${PROJECT_NAME}_node app/velocity_control_bridge_node.cpp)

cs_install()
cs_export()
