cmake_minimum_required(VERSION 3.12)
project(kinematic_tracking_mpc)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-fPIC)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# ##############################################################################
# Libraries #
# ##############################################################################
cs_add_library(${PROJECT_NAME} src/tracking_mpc_kinematic_controller.cpp)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic -Werror)

generate_acados_solver(
  kinematic_tracking_mpc_solver
  MODEL_NAME
  kinematic_model_tracking_mpc
  CONFIG_PATH
  config/solver_tracking_mpc.yaml
  LIB_DIR
  ${CMAKE_CURRENT_BINARY_DIR}/lib
  OCP_MODULE_DIR
  script
  SOURCES
  src/solvers/acados_kinematic_tracking_mpc_solver.cpp)

# ##############################################################################
# Tests #
# ##############################################################################

# ##############################################################################
# Export #
# ##############################################################################

cs_install()
cs_export(INCLUDE_DIRS ${${PROJECT_NAME}_SOLVER_INCLUDE_DIRS})
