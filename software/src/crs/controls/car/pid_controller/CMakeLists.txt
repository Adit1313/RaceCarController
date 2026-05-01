cmake_minimum_required(VERSION 3.0.2)
project(pid_controller)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic -Werror)
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

# ##############################################################################
# Libraries #
# ##############################################################################

cs_add_library(${PROJECT_NAME} src/pacejka_pid_controller.cpp
               src/pacejka_pid_const_ref_controller.cpp)

target_link_libraries(${PROJECT_NAME})

# ##############################################################################
# Tests #
# ##############################################################################

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(pid_tests test/pacejka_pid_test.cpp)
  target_link_libraries(pid_tests ${catkin_LIBRARIES} ${PROJECT_NAME})
endif()

# ##############################################################################
# Export #
# ##############################################################################

cs_install()
cs_export()
