cmake_minimum_required(VERSION 3.0.2)
project(rockpara_actuators)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslaunch
  crs_msgs
  eigen_catkin
)

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES rockpara_actuators
  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# if pigpio has not been installed, run the installation script
add_custom_target(pigpio_install ALL
  COMMAND sudo /bin/sh ${CMAKE_CURRENT_SOURCE_DIR}/lib/install_pigpio.sh  ${CMAKE_CURRENT_SOURCE_DIR}/lib
)

## Declare a C++ library
add_library(pca9685 src/pca9685.cpp)

add_dependencies(pca9685 pigpio_install)

target_link_libraries(pca9685
  ${catkin_LIBRARIES} pigpio rt
)
add_library(${PROJECT_NAME}_lib src/rockpara_actuators.cpp)
target_link_libraries(${PROJECT_NAME}_lib pca9685)

add_executable(${PROJECT_NAME}_node src/rockpara_actuators_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)
