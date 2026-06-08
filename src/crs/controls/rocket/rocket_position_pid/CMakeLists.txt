cmake_minimum_required(VERSION 3.0.2)
project(rocket_position_pid)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic -Werror)
find_package(catkin_simple REQUIRED)

find_package(Boost REQUIRED COMPONENTS atomic)

catkin_simple()

# ##############################################################################
# Libraries #
# ##############################################################################

cs_add_library(${PROJECT_NAME} src/rocket_high_level_pid_controller.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# ##############################################################################
# Tests #
# ##############################################################################

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(rocket_controller_tests test/rocket_high_level_pid_test.cpp)
  target_link_libraries(rocket_controller_tests ${catkin_LIBRARIES}
                        ${PROJECT_NAME})
endif()

# ##############################################################################
# Export #
# ##############################################################################

cs_install()
cs_export()
