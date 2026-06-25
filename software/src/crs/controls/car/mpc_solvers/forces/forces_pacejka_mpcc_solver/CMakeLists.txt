cmake_minimum_required(VERSION 3.0.2)
project(forces_pacejka_mpcc_solver)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic -Werror)
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  src/c_generated_code/include
)

#############
# Libraries #
#############

cs_add_library(${PROJECT_NAME}
        src/forces_pacejka_mpcc_solver.cpp # Main solver file used to interface with MPC controller
        src/c_generated_code/lib/libFORCESNLPsolver.so
        src/c_generated_code/FORCESNLPsolver_interface.c
        src/c_generated_code/FORCESNLPsolver_model.c
        src/c_generated_code/include/FORCESNLPsolver_info.cpp

)
target_link_libraries(${PROJECT_NAME}
  ${CMAKE_CURRENT_SOURCE_DIR}/src/c_generated_code/lib/libFORCESNLPsolver.so
)


##########
# Tests #
##########

##########
# Export #
##########

cs_install()
cs_export(LIBRARIES ${PROJECT_NAME})
