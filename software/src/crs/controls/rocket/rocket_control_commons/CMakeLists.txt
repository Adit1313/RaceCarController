cmake_minimum_required(VERSION 3.0.2)
project(rocket_control_commons)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic -Werror)
find_package(catkin_simple REQUIRED)

find_package(Boost REQUIRED COMPONENTS atomic)

catkin_simple(ALL_DEPS_REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})

# ##############################################################################
# Libraries #
# ##############################################################################
cs_add_library(${PROJECT_NAME} src/rocket_attitude_controller.cpp
               src/rocket_6_dof_allocation.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# ##############################################################################
# Tests #
# ##############################################################################

# ##############################################################################
# Export #
# ##############################################################################

cs_install()
cs_export(DEPENDS Boost)
