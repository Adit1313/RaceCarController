cmake_minimum_required(VERSION 3.0.2)

# EDIT: name of package
project(rospy_controllers)

# EDIT: list all <build_depend> and <depend> package dependencies
find_package(catkin REQUIRED COMPONENTS rospy std_msgs)

catkin_python_setup()

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

install(PROGRAMS app/* DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
