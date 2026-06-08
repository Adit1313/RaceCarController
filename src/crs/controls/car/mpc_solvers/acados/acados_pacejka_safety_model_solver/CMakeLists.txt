cmake_minimum_required(VERSION 3.12)
project(acados_pacejka_safety_model_solver)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-fPIC)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# DEFINE CATKIN WORKSPACE DIRECTORY
execute_process(COMMAND catkin locate
                OUTPUT_VARIABLE CATKIN_WORKSPACE_DIR
                OUTPUT_STRIP_TRAILING_WHITESPACE)

#############
# Libraries #
#############

# NOTE(@naefjo): Create lib directory at build system generation time.
# If this dir isn't present, the cs_export fails.
set(SOLVER_LIB_DIR ${CMAKE_CURRENT_LIST_DIR}/lib)
file(MAKE_DIRECTORY ${SOLVER_LIB_DIR})
# Run solver creation once at build generation time so we can track the
# solver files.
execute_process(
  COMMAND bash ${CATKIN_WORKSPACE_DIR}/scripts/create_solver_watch.sh ${CMAKE_CURRENT_LIST_DIR}
  WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
)

# Define the solver source files.
set(MODEL_NAME safety_dynamic_model)
file(GLOB OCP_COST_SRC "${SOLVER_LIB_DIR}/${MODEL_NAME}_cost/*.c")
file(GLOB OCP_CONSTR_SRC "${SOLVER_LIB_DIR}/${MODEL_NAME}_constraints/*.c")

set(OCP_SRC
  ${OCP_COST_SRC}
  ${OCP_CONSTR_SRC}
  ${SOLVER_LIB_DIR}/acados_solver_${MODEL_NAME}.c
)
file(GLOB MODEL_SRC "${SOLVER_LIB_DIR}/${MODEL_NAME}_model/*.c")


# Use a custom target so solver script is invoked at every build of the project and changes in
# the script show up in the code.
# NOTE(@naefjo): if for some reason the script should produce different files than during the
# initial call in execute_process, the file list in OCP_SRC and MODEL_SRC will be wrong as these
# are generated during the cmake/generate step and this custom target is invoked during the build
# step. in that case simply run `catkin build --force-cmake package-name` to reinitialize the
# MODEL_SRC and OCP_SRC variables.
add_custom_target(${PROJECT_NAME}_c_solver_lib
  COMMAND bash ${CATKIN_WORKSPACE_DIR}/scripts/create_solver_watch.sh ${CMAKE_CURRENT_LIST_DIR} update
  WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
  BYPRODUCTS ${MODEL_SRC} ${OCP_SRC}
  COMMENT "Generate acados C solvers and create lib/ directory"
)

# Create an object library from the generated acados source files.
# Since we do not want compile time warnings for auto generated code, we need to compile
# the acados code as a separate library. Since we also do not need to export the acados solver
# code directly but only our "mpc wrapper", we use an object library which is not a standard
# library in the sense that a lib<target>.so is created but instead its a "temporary collection
# of object files".
add_library(${PROJECT_NAME}_acados_solver OBJECT ${MODEL_SRC} ${OCP_SRC})
add_dependencies(${PROJECT_NAME}_acados_solver ${PROJECT_NAME}_c_solver_lib)
# NOTE(@naefjo): Interface option declares to dependent targets that there are headers for this
# target to be found in this folder.
target_include_directories(${PROJECT_NAME}_acados_solver INTERFACE ${SOLVER_LIB_DIR})

# The actual solver library we want to export.
cs_add_library(${PROJECT_NAME} SHARED src/acados_pacjeka_safety_model_solver.cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_c_solver_lib ${PROJECT_NAME}_acados_solver)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_acados_solver)

##########
# Tests #
##########

##########
# Export #
##########

cs_install()
cs_export(INCLUDE_DIRS ${SOLVER_LIB_DIR})
