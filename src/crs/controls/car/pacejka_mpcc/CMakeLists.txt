cmake_minimum_required(VERSION 3.12)
project(pacejka_mpcc)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-fPIC)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

option(ACADOS_MPCC_SOLVER "Whether to build the Acados MPCC solver" ON)
option(ACADOS_CURVILINEAR_MPCC_SOLVER
       "Whether to build the Acados Curvilinear MPCC solver" ON)
option(FORCES_MPCC_SOLVER "Whether to build the Acados MPCC solver" OFF)
option(CAR_LINEAR_TRACK_CONSTRAINT
       "Whether to use the linear or nonlinear track constraint" OFF)

# ##############################################################################
# Libraries #
# ##############################################################################

cs_add_library(${PROJECT_NAME} src/mpcc_pacejka_controller.cpp)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic -Werror)

if(NOT ACADOS_MPCC_SOLVER
   AND NOT FORCES_MPCC_SOLVER
   AND NOT ACADOS_CURVILINEAR_MPCC_SOLVER)
  message(FATAL_ERROR "No MPCC solver enabled. You must enable at least one.")
endif()

set(PACEJKA_MPCC_ADDITIONAL_ARGS "")
if(CAR_LINEAR_TRACK_CONSTRAINT)
  set(PACEJKA_MPCC_ADDITIONAL_ARGS
      "${PACEJKA_MPCC_ADDITIONAL_ARGS} linear_track_constraint")
endif()

if(ACADOS_MPCC_SOLVER)
  add_definitions(-DBUILD_ACADOS_SOLVER)

  generate_acados_solver(
    pacejka_mpcc
    MODEL_NAME
    pacejka_model
    CONFIG_PATH
    config/solver_acados.yaml
    LIB_DIR
    ${CMAKE_CURRENT_BINARY_DIR}/lib_mpcc
    OCP_MODULE_DIR
    script/acados
    SOURCES
    src/solvers/acados_pacejka_mpcc_solver.cpp
    EXTRA_GEN_ARGS
    ${PACEJKA_MPCC_ADDITIONAL_ARGS})

  # Mark headers in lib folder for export
  list(APPEND ADDITIONAL_INCLUDE_DIRS ${SOLVER_LIB_DIR})
endif()

if(ACADOS_CURVILINEAR_MPCC_SOLVER)
  add_definitions(-DBUILD_ACADOS_CURVILINEAR_SOLVER)

  generate_acados_solver(
    pacejka_curvilinear_mpcc
    MODEL_NAME
    pacejka_curvilinear_model
    CONFIG_PATH
    config/solver_acados_curvilinear.yaml
    LIB_DIR
    ${CMAKE_CURRENT_BINARY_DIR}/lib_curv
    OCP_MODULE_DIR
    script/acados_curvilinear
    SOURCES
    src/solvers/acados_pacejka_curvilinear_mpcc_solver.cpp
    EXTRA_GEN_ARGS
    ${PACEJKA_MPCC_ADDITIONAL_ARGS})

  target_sources(${PROJECT_NAME}
                 PRIVATE src/mpcc_curvilinear_pacejka_controller.cpp)

endif()

if(FORCES_MPCC_SOLVER)
  add_definitions(-DBUILD_FORCES_SOLVER)
  # TODO: make same build flow as acados (also check that this actually works)
  # solver but i need a license for that:/
  add_library(
    ${PROJECT_NAME}_forces_solver
    src/solvers/forces_pacejka_mpcc_solver.cpp
    src/c_generated_code/lib/libFORCESNLPsolver.so
    src/c_generated_code/FORCESNLPsolver_interface.c
    src/c_generated_code/FORCESNLPsolver_model.c
    src/c_generated_code/include/FORCESNLPsolver_info.cpp)

  target_include_directories(${PROJECT_NAME}forces_solver
                             INTERFACE src/c_generated_code/include)

  target_link_libraries(
    ${PROJECT_NAME}_forces_solver
    ${CMAKE_CURRENT_SOURCE_DIR}/src/c_generated_code/lib/libFORCESNLPsolver.so)

  target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_forces_solver)
endif()

# ##############################################################################
# Tests #
# ##############################################################################
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}_test test/test_mpcc_pacejka_controller.cpp
                   WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test/)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME}
                        ${catkin_LIBRARIES} casadi stdc++fs)
endif()

# ##############################################################################
# Export #
# ##############################################################################

cs_install()
cs_export(INCLUDE_DIRS ${${PROJECT_NAME}_SOLVER_INCLUDE_DIRS} CFG_EXTRAS
          ${PROJECT_NAME}-extras.cmake)
