cmake_minimum_required(VERSION 2.8.3)
project(mhe_solvers)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic -Werror)
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

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
cs_export(INCLUDE_DIRS)
