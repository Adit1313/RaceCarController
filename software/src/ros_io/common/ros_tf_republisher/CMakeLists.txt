cmake_minimum_required(VERSION 3.0.2)
project(ros_tf_republisher)

set(CMAKE_CXX_STANDARD 20)
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

catkin_package()

# ##############################################################################
# Libraries #
# ##############################################################################

include_directories(include ${catkin_INCLUDE_DIRS})

# ##############################################################################
# Libraries #
# ##############################################################################

# cs_add_library(${PROJECT_NAME} src/republisher.cpp )

# target_link_libraries(${PROJECT_NAME})

# ##############################################################################
# Tests #
# ##############################################################################

# ##############################################################################
# Executables #
# ##############################################################################

cs_add_executable(republisher_node app/republisher_node.cpp)

cs_add_executable(car_state_converter app/qualisys_car_state_converter.cpp)

# ##############################################################################
# Export #
# ##############################################################################

cs_install()
cs_export()
