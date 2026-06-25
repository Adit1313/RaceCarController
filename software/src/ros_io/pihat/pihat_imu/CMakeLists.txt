cmake_minimum_required(VERSION 3.16.3)
project(pihat_imu)

include(ExternalProject)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-fPIC)

find_package(catkin_simple REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

# ##############################################################################
# Libraries #
# ##############################################################################

set(EXTERNAL_PROJECT_DIR ${CMAKE_CURRENT_LIST_DIR}/lib)
file(MAKE_DIRECTORY ${EXTERNAL_PROJECT_DIR})

set(BOSCH_IMU_SOURCE_FILES ${EXTERNAL_PROJECT_DIR}/bmi08a.c
                           ${EXTERNAL_PROJECT_DIR}/bmi08g.c)

add_custom_command(
  OUTPUT ${BOSCH_IMU_SOURCE_FILES}
  COMMAND git clone -q --depth 1 --branch bmi08x_v1.5.8
          https://github.com/boschsensortec/BMI08x_SensorAPI.git . 2>/dev/null
  WORKING_DIRECTORY ${EXTERNAL_PROJECT_DIR})

add_library(bosch_imu_code OBJECT ${BOSCH_IMU_SOURCE_FILES})

cs_add_library(${PROJECT_NAME} src/bmi088.c)

target_include_directories(${PROJECT_NAME} PUBLIC ${EXTERNAL_PROJECT_DIR})
target_link_libraries(${PROJECT_NAME} bosch_imu_code)

#
# Tests #
#

# ##############################################################################
# Executables #
# ##############################################################################
cs_add_executable(pihat_imu_node src/pihat_imu.cpp)

target_link_libraries(pihat_imu_node ${PROJECT_NAME})

# ##############################################################################
# Export #
# ##############################################################################

cs_install()
cs_export()
