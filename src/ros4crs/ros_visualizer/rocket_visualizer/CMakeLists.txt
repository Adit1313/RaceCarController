cmake_minimum_required(VERSION 2.8.3)
project(rocket_visualizer)

set(CMAKE_CXX_STANDARD 17)
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
catkin_package()

# ##############################################################################
# Libraries #
# ##############################################################################

include_directories(include ${catkin_INCLUDE_DIRS})
cs_add_library(${PROJECT_NAME} src/RocketVisualizer.cpp)

# ##############################################################################
# Tests #
# ##############################################################################

# ##############################################################################
# Executables #
# ##############################################################################

add_executable(rocket_visualizer_node src/rocket_visualizer_node.cpp)

target_link_libraries(rocket_visualizer_node ${PROJECT_NAME}
                      ${catkin_LIBRARIES})
add_dependencies(rocket_visualizer_node crs_msgs_generate_messages_cpp)

# ##############################################################################
# Export #
# ##############################################################################

cs_install()
cs_export()
