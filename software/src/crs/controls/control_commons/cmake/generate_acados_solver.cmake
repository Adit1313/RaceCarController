# ~~~
# Generates acados solver code given the config
#
# Params:
# _solver_name: Used to create unique target names (does not have any meaningful
#   effect you can set anything you want as long as it is unique within the CMakeLists)
# MODEL_NAME: The name of the model specified in the solver generation scripts
# CONFIG_PATH: Path (relative from current ros package root) to the solver config file
# LIB_DIR: Directory where the generated code should be put
# OCP_MODULE_DIR: script dir where the solver generation pyhton files can be found
# SOURCES: cpp files needed to build our solver wrapper
# EXTRA_GEN_ARGS: additional args that can be passed to the solver scripts
# EXTRA_HASH_DIRS: Additional dirs to watch when creating the solver hash
#
# "Return":
# ${PROJECT_NAME}_SOLVER_INCLUDE_DIRS is populated with the include dirs necessary
# for the solvers.
# ~~~
macro(generate_acados_solver _solver_name)
  set(SOLVER_NAME ${_solver_name})

  cmake_parse_arguments(
    generate_acados_solver "" "MODEL_NAME;CONFIG_PATH;LIB_DIR;OCP_MODULE_DIR"
    "SOURCES;EXTRA_GEN_ARGS;EXTRA_HASH_DIRS" ${ARGN})

  set(${SOLVER_NAME}_MODEL_NAME "${generate_acados_solver_MODEL_NAME}")
  set(${SOLVER_NAME}_CONFIG_PATH "${generate_acados_solver_CONFIG_PATH}")
  set(${SOLVER_NAME}_LIB_DIR "${generate_acados_solver_LIB_DIR}")
  set(${SOLVER_NAME}_OCP_MODULE_DIR "${generate_acados_solver_OCP_MODULE_DIR}")

  set(${SOLVER_NAME}_SOURCES "${generate_acados_solver_SOURCES}")
  set(${SOLVER_NAME}_EXTRA_GEN_ARGS "${generate_acados_solver_EXTRA_GEN_ARGS}")
  set(${SOLVER_NAME}_EXTRA_HASH_DIRS
      "${generate_acados_solver_EXTRA_HASH_DIRS}")

  # The build of the solver is done in multiple steps. First, the auto generated
  # files are compiled as a standalone library, then we compile our solver
  # wrapper and link it against the auto generated code. Finally, we link the
  # mpcc_controller to the solver code.

  file(MAKE_DIRECTORY ${${SOLVER_NAME}_LIB_DIR})

  message(STATUS "Generating solver with the following settings:")
  message(STATUS "Model Name: ${${SOLVER_NAME}_MODEL_NAME}")
  message(STATUS "Config Path: ${${SOLVER_NAME}_CONFIG_PATH}")
  message(STATUS "Library Dir: ${${SOLVER_NAME}_LIB_DIR}")
  message(STATUS "OCP Module Dir: ${${SOLVER_NAME}_OCP_MODULE_DIR}")
  message(STATUS "Sources: ${${SOLVER_NAME}_SOURCES}")
  message(STATUS "extra gen args: ${${SOLVER_NAME}_EXTRA_GEN_ARGS}")
  message(STATUS "extra hash files: ${${SOLVER_NAME}_EXTRA_HASH_DIRS}")

  # Run solver creation once at build generation time so we can track the solver
  # files.
  execute_process(
    COMMAND
      create_solver.py --config ${${SOLVER_NAME}_CONFIG_PATH} --lib_dir
      ${${SOLVER_NAME}_LIB_DIR} --script_dir ${${SOLVER_NAME}_OCP_MODULE_DIR}
      --dirs_to_hash ${${SOLVER_NAME}_CONFIG_PATH}
      ${${SOLVER_NAME}_OCP_MODULE_DIR} ${${SOLVER_NAME}_EXTRA_HASH_DIRS}
      --additional_args ${${SOLVER_NAME}_EXTRA_GEN_ARGS}
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})

  # Define the solver source files.
  file(GLOB ${SOLVER_NAME}_OCP_COST_SRC
       "${${SOLVER_NAME}_LIB_DIR}/${${SOLVER_NAME}_MODEL_NAME}_cost/*.c")
  file(GLOB ${SOLVER_NAME}_OCP_CONSTR_SRC
       "${${SOLVER_NAME}_LIB_DIR}/${${SOLVER_NAME}_MODEL_NAME}_constraints/*.c")

  set(${SOLVER_NAME}_OCP_SRC
      ${${SOLVER_NAME}_OCP_COST_SRC} ${${SOLVER_NAME}_OCP_CONSTR_SRC}
      ${${SOLVER_NAME}_LIB_DIR}/acados_solver_${${SOLVER_NAME}_MODEL_NAME}.c)
  file(GLOB ${SOLVER_NAME}_MODEL_SRC
       "${${SOLVER_NAME}_LIB_DIR}/${${SOLVER_NAME}_MODEL_NAME}_model/*.c")

  # Use a custom target so solver script is invoked at every build of the
  # project and changes in the script show up in the code.
  # NOTE(@naefjo): if for some reason the script should produce different files
  # than during the initial call in execute_process, the file list in OCP_SRC
  # and MODEL_SRC will be wrong as these are generated during the cmake/generate
  # step and this custom target is invoked during the build step. in that case
  # simply run `catkin build --force-cmake package-name` to reinitialize the
  # MODEL_SRC and OCP_SRC variables.
  add_custom_target(
    ${PROJECT_NAME}_${SOLVER_NAME}_c_solver_lib_generator
    COMMAND
      create_solver.py --config ${${SOLVER_NAME}_CONFIG_PATH} --lib_dir
      ${${SOLVER_NAME}_LIB_DIR} --script_dir ${${SOLVER_NAME}_OCP_MODULE_DIR}
      --dirs_to_hash ${${SOLVER_NAME}_CONFIG_PATH}
      ${${SOLVER_NAME}_OCP_MODULE_DIR} ${${SOLVER_NAME}_EXTRA_HASH_DIRS}
      --additional_args ${${SOLVER_NAME}_EXTRA_GEN_ARGS}
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
    BYPRODUCTS ${${SOLVER_NAME}_MODEL_SRC} ${${SOLVER_NAME}_OCP_SRC}
    COMMENT "Generate acados C solver code")

  # Create an object library from the generated acados source files. Since we do
  # not want compile time warnings for auto generated code, we need to compile
  # the acados code as a separate library. Since we also do not need to export
  # the acados solver code directly but only our "mpc wrapper", we use an object
  # library which is not a standard library in the sense that a lib<target>.so
  # is created but instead its a "temporary collection of object files".
  add_library(${PROJECT_NAME}_${SOLVER_NAME}_acados_solver_generated_code OBJECT
              ${${SOLVER_NAME}_MODEL_SRC} ${${SOLVER_NAME}_OCP_SRC})
  add_dependencies(${PROJECT_NAME}_${SOLVER_NAME}_acados_solver_generated_code
                   ${PROJECT_NAME}_${SOLVER_NAME}_c_solver_lib_generator)

  # NOTE(@naefjo): Interface option declares to dependent targets that there are
  # headers for this target to be found in this folder.
  target_include_directories(
    ${PROJECT_NAME}_${SOLVER_NAME}_acados_solver_generated_code
    INTERFACE ${${SOLVER_NAME}_LIB_DIR})

  add_library(${PROJECT_NAME}_${SOLVER_NAME}_acados_solver SHARED
              ${${SOLVER_NAME}_SOURCES})
  target_compile_options(${PROJECT_NAME}_${SOLVER_NAME}_acados_solver
                         PRIVATE -Wall -Wextra -Wpedantic -Werror)
  add_dependencies(
    ${PROJECT_NAME}_${SOLVER_NAME}_acados_solver
    ${PROJECT_NAME}_${SOLVER_NAME}_c_solver_lib_generator
    ${PROJECT_NAME}_${SOLVER_NAME}_acados_solver_generated_code)

  target_link_libraries(
    ${PROJECT_NAME}_${SOLVER_NAME}_acados_solver
    ${PROJECT_NAME}_${SOLVER_NAME}_acados_solver_generated_code
    ${catkin_LIBRARIES})

  target_link_libraries(${PROJECT_NAME}
                        ${PROJECT_NAME}_${SOLVER_NAME}_acados_solver)

  # Populate a variable with header locations for cs_export
  list(APPEND ${PROJECT_NAME}_SOLVER_INCLUDE_DIRS ${${SOLVER_NAME}_LIB_DIR})
endmacro()
