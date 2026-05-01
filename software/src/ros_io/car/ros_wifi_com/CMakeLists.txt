cmake_minimum_required(VERSION 3.0.2)
project(ros_wifi_com)

set(CMAKE_CXX_STANDARD 17)
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Protobuf REQUIRED)

# Generate Protobuf messages
file(GLOB proto_files msgs/proto/*.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${proto_files})
message(STATUS "Finished generating protobuf")
set_source_files_properties(${PROTO_SRCS} PROPERTIES GENERATED TRUE)

catkin_package()

# ##############################################################################
# Build ##
# ##############################################################################

# Specify additional locations of header files Your package locations should be
# listed before other locations
include_directories(
  include ${catkin_INCLUDE_DIRS} ${roscpp_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR} # this is not nice
)

# Declare a C++ library
add_library(${PROJECT_NAME}_proto_msgs ${PROTO_SRCS})

# ##############################################################################
# Libraries #
# ##############################################################################

cs_add_library(${PROJECT_NAME} src/udp_server.cpp src/wifi_com.cpp)
target_link_libraries(
  ${PROJECT_NAME} ${catkin_LIBRARIES} ${PROJECT_NAME}_proto_msgs
  ${PROTOBUF_LIBRARY} ${Boost_LIBRARIES})

# ##############################################################################
# Executables #
# ##############################################################################

cs_add_executable(wifi_com_node app/wifi_com_node.cpp)
target_link_libraries(wifi_com_node ${PROJECT_NAME})

# add_dependencies(${PROJECT_NAME}_node # ${${PROJECT_NAME}_EXPORTED_TARGETS} #
# ${catkin_EXPORTED_TARGETS} crs_msgs_generate_messages_cpp )

# ##############################################################################
# Export #
# ##############################################################################

cs_install()
cs_export()
