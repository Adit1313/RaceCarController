cmake_minimum_required(VERSION 3.12)
project(acados_pacejka_mhe_solver)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-fPIC)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# ##############################################################################
# Libraries #
# ##############################################################################

# NOTE(@naefjo): Create lib directory at build system generation time. If this
# dir isn't present, the cs_export fails.
set(SOLVER_LIB_DIR ${CMAKE_CURRENT_LIST_DIR}/lib)
file(MAKE_DIRECTORY ${SOLVER_LIB_DIR})

set(SOLVER_GENERATION_DIRS "script:config")
set(SOLVER_GENERATION_CMD
    "python3 script/generate_acados_solver.py --config solver.yaml")
# Run solver creation once at build generation time so we can track the solver
# files.
execute_process(
	COMMAND bash create_solver_watch_new.sh -d "${SOLVER_GENERATION_DIRS}" -s
          "${SOLVER_GENERATION_CMD}"
  WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})

# Define the solver source files.
set(MODEL_NAME pacejka_model)
file(GLOB OCP_COST_SRC "${SOLVER_LIB_DIR}/${MODEL_NAME}_cost/*.c")
file(GLOB OCP_CONSTR_SRC "${SOLVER_LIB_DIR}/${MODEL_NAME}_constraints/*.c")

set(OCP_SRC ${OCP_COST_SRC} ${OCP_CONSTR_SRC}
            ${SOLVER_LIB_DIR}/acados_solver_${MODEL_NAME}.c)
file(GLOB MODEL_SRC "${SOLVER_LIB_DIR}/${MODEL_NAME}_model/*.c")

add_custom_target(
  ${PROJECT_NAME}_c_solver_lib
  COMMAND bash create_solver_watch_new.sh -d "${SOLVER_GENERATION_DIRS}" -u -s
          "${SOLVER_GENERATION_CMD}"
  WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
  BYPRODUCTS ${MODEL_SRC} ${OCP_SRC}
  COMMENT "Generate acados C solvers and create lib/ directory")

add_library(${PROJECT_NAME}_acados_solver OBJECT ${MODEL_SRC} ${OCP_SRC})
add_dependencies(${PROJECT_NAME}_acados_solver ${PROJECT_NAME}_c_solver_lib)
target_include_directories(${PROJECT_NAME}_acados_solver
                           INTERFACE ${SOLVER_LIB_DIR})

cs_add_library(${PROJECT_NAME} SHARED src/acados_pacejka_mhe_solver.cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_c_solver_lib
                 ${PROJECT_NAME}_acados_solver)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_acados_solver)

# ##############################################################################
# Tests #
# ##############################################################################

# ##############################################################################
# Export #
# ##############################################################################

cs_install()
cs_export(INCLUDE_DIRS ${SOLVER_LIB_DIR})
