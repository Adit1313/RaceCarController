cmake_minimum_required(VERSION 3.0.2)
project(mpc_controller)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic -Werror)
find_package(catkin_simple REQUIRED)
find_package(casadi)

catkin_simple()

if(${acados_pacejka_mpcc_solver_FOUND})
   add_definitions(-Dacados_pacejka_mpcc_solver_FOUND)
else()
   message("Acados Pacejka MPCC Solver not found. Building without AcadosMPCC support")
endif()

if(${acados_pacejka_tracking_mpc_solver_FOUND})
   add_definitions(-Dacados_pacejka_tracking_mpc_solver_FOUND)
else()
   message("Acados Pacejka tracking MPC Solver not found. Building without AcadosTrackingMPC support")
endif()

if(${acados_kinematic_tracking_mpc_solver_FOUND})
   add_definitions(-Dacados_kinematic_tracking_mpc_solver_FOUND)
else()
   message("Acados Kinematic tracking MPC Solver not found. Building without AcadosTrackingMPC support")
endif()

if(${forces_pacejka_mpcc_solver_FOUND})
   add_definitions(-Dforces_pacejka_mpcc_solver_FOUND)
else()
   message("Forces Pacejka tracking MPC Solver not found. Building without Forces support")
endif()

#############
# Libraries #
#############

cs_add_library(${PROJECT_NAME}
  src/pacejka_controller/mpcc_pacejka_controller.cpp
  src/pacejka_controller/tracking_mpc_pacejka_controller.cpp
  src/kinematic_controller/tracking_mpc_kinematic_controller.cpp)

##########
# Tests #
##########

if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(controller_test test/test_mpcc_pacejka_controller.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test/)
  target_link_libraries(controller_test ${catkin_LIBRARIES} ${PROJECT_NAME} casadi stdc++fs)
endif()

##########
# Export #
##########

cs_install()
cs_export()
