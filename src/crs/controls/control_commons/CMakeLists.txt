cmake_minimum_required(VERSION 3.0.2)
project(control_commons)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic -Werror)
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
catkin_package()

# ##############################################################################
# Libraries #
# ##############################################################################

# ##############################################################################
# Tests #
# ##############################################################################

# ##############################################################################
# Export #
# ##############################################################################

cs_install()
cs_export(CFG_EXTRAS generate_acados_solver.cmake)
