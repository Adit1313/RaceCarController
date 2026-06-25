cmake_minimum_required(VERSION 3.0.2)
project(filter)

find_package(catkin REQUIRED COMPONENTS roscpp rospy dynamic_reconfigure)
# add dynamic reconfigure api find_package(catkin REQUIRED dynamic_reconfigure)
generate_dynamic_reconfigure_options(
  src/cfg/lowpass_filter.cfg src/cfg/butterworth_filter.cfg
  src/cfg/median_filter.cfg)

catkin_package(CATKIN_DEPENDS rospy message_runtime dynamic_reconfigure)

catkin_install_python(PROGRAMS src/filter_node.py DESTINATION
                      ${CATKIN_PACKAGE_BIN_DESTINATION})
# Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  # Set manually because Eigen sets a non standard INCLUDE DIR
)

# make sure configure headers are built before any node using them
# add_dependencies(example_node ${PROJECT_NAME}_gencfg)
# ##############################################################################
# Install ##
# ##############################################################################

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
