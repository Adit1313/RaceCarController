cmake_minimum_required(VERSION 3.0.2)
project(kalman_estimator)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic -Werror)
find_package(catkin_simple REQUIRED)
find_package(casadi)
catkin_simple(ALL_DEPS_REQUIRED)
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
# ##############################################################################
# Libraries #
# ##############################################################################

# ##############################################################################
# Tests #
# ##############################################################################

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(ekf_test test/kalman_tests.cpp WORKING_DIRECTORY
                   ${PROJECT_SOURCE_DIR}/test/)
  target_link_libraries(ekf_test ${catkin_LIBRARIES} casadi stdc++fs)
endif()

# ##############################################################################
# Export #
# ##############################################################################

cs_install()
cs_export()
