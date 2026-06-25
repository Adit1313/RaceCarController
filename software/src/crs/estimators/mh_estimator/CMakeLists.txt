cmake_minimum_required(VERSION 2.8.3)
project(mh_estimator)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic -Werror)
find_package(catkin_simple REQUIRED)
find_package(casadi)
catkin_simple(ALL_DEPS_REQUIRED)
catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  $ENV{ACADOS_SOURCE_DIR}/include
  $ENV{ACADOS_SOURCE_DIR}/include/acados
  $ENV{ACADOS_SOURCE_DIR}/include/blasfeo/include
  $ENV{ACADOS_SOURCE_DIR}/include/hpipm/include)
# ##############################################################################
# Libraries #
# ##############################################################################

cs_add_library(${PROJECT_NAME} src/pacejka_mhe.cpp src/utils/mhe_utils.cpp)

# ##############################################################################
# Tests #
# ##############################################################################

if(CATKIN_ENABLE_TESTING)

endif()

# ##############################################################################
# Export #
# ##############################################################################

cs_install()
cs_export(INCLUDE_DIRS)
