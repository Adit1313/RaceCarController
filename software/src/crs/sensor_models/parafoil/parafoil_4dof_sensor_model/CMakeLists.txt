cmake_minimum_required(VERSION 3.0.2)
project(parafoil_4dof_sensor_model)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic -Werror)
find_package(catkin_simple REQUIRED)
find_package(casadi)
catkin_simple(ALL_DEPS_REQUIRED)
catkin_package()

# ##############################################################################
# Libraries #
# ##############################################################################
include_directories(include ${catkin_INCLUDE_DIRS})

cs_add_library(${PROJECT_NAME} src/gps_sensor_model.cpp
               src/imu_sensor_model.cpp)

target_link_libraries(${PROJECT_NAME})

# ##############################################################################
# Tests #
# ##############################################################################

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(sensor_tests test/sensor_tests.cpp WORKING_DIRECTORY
                   ${PROJECT_SOURCE_DIR}/test/)
  target_link_libraries(sensor_tests ${catkin_LIBRARIES} ${PROJECT_NAME} casadi)
endif()

# ##############################################################################
# Export #
# ##############################################################################

cs_install()
cs_export()
