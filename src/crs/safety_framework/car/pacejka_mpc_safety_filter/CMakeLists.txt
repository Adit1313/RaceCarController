cmake_minimum_required(VERSION 3.12)
project(pacejka_mpc_safety_filter)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-fPIC)

find_package(catkin_simple REQUIRED)
find_package(casadi)
catkin_simple(ALL_DEPS_REQUIRED)

# ##############################################################################
# Libraries #
# ##############################################################################

cs_add_library(${PROJECT_NAME} src/pacejka_mpc_safety_filter.cpp)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic -Werror)

generate_acados_solver(
  pacejka_safety_solver
  MODEL_NAME
  safety_dynamic_model
  CONFIG_PATH
  config/solver.yaml
  LIB_DIR
  ${CMAKE_CURRENT_BINARY_DIR}/lib
  OCP_MODULE_DIR
  script
  SOURCES
  src/solvers/acados_pacejka_safety_model_solver.cpp)

# ##############################################################################
# Tests #
# ##############################################################################

# ##############################################################################
# Export #
# ##############################################################################

cs_install()
cs_export(INCLUDE_DIRS ${${PROJECT_NAME}_SOLVER_INCLUDE_DIRS})
