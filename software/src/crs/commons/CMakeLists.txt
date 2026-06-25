cmake_minimum_required(VERSION 3.0.2)
project(commons)

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

cs_add_library(
  ${PROJECT_NAME}
  src/filter.cpp
  src/trajectory.cpp
  src/static_track_trajectory.cpp
  src/dynamic_point_trajectory.cpp
  src/casadi_utils.cpp
  src/curvilinear_track_trajectory.cpp)

target_link_libraries(${PROJECT_NAME} casadi)

# ##############################################################################
# Tests #
# ##############################################################################

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(track_tests test/track_tests.cpp)
  catkin_add_gtest(casadi_utils_test test/casadi_utils_test.cpp)
  target_link_libraries(track_tests ${catkin_LIBRARIES} ${PROJECT_NAME} casadi)
  target_link_libraries(casadi_utils_test ${catkin_LIBRARIES} ${PROJECT_NAME}
                        casadi)
endif()

# ##############################################################################
# Export #
# ##############################################################################

cs_install()
cs_export(DEPENDS casadi)
