cmake_minimum_required(VERSION 3.0.2)
project(ros_lighthouse_calibration)

set(CMAKE_CXX_STANDARD 20)
find_package(catkin_simple REQUIRED)
find_package(casadi)
catkin_simple(ALL_DEPS_REQUIRED)

catkin_package()

# ##############################################################################
# Libraries #
# ##############################################################################

include_directories(include ${catkin_INCLUDE_DIRS})

# ##############################################################################
# Tests #
# ##############################################################################

# ##############################################################################
# Executables #
# ##############################################################################

cs_add_executable(
  lighthouse_calibration_node
  app/calibration_node.cpp
  src/lighthouse_calibrator.cpp
  src/calibration_visualization.cpp
  src/lighthouse_calibration_collector.cpp
  src/lighthouse_calibration_io.cpp
  src/ootx_decoder.cpp)

cs_add_executable(lighthouse_calibration_solver_node app/solver_node.cpp
                  src/lighthouse_calibration_solver.cpp)

target_link_libraries(lighthouse_calibration_node casadi)
target_link_libraries(lighthouse_calibration_solver_node casadi)

# ##############################################################################
# Export #
# ##############################################################################

cs_install()
cs_export()
