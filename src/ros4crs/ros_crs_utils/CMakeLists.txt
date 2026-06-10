cmake_minimum_required(VERSION 3.0.2)
project(ros_crs_utils)

set(CMAKE_CXX_STANDARD 17)
find_package(catkin_simple REQUIRED)

catkin_simple()
catkin_package()

# ##############################################################################
# Libraries #
# ##############################################################################
include_directories(include ${catkin_INCLUDE_DIRS})

list(APPEND CPP_SRC_FILES src/parameter_io.cpp
     src/trajectory_message_conversion.cpp)

if(${mh_estimator_FOUND})
  add_definitions(-Dmh_estimator_FOUND)
  list(APPEND CPP_SRC_FILES src/parameter_io_mhe.cpp)
else()
  message(
    "Moving Horizon Estimator not found. Building without Moving Horizon Estimator support"
  )
endif()

if(${stacked_model_FOUND} AND ${imu_bias_FOUND})
  add_definitions(-Dpacejka_model_FOUND)
  list(APPEND CPP_SRC_FILES src/parameter_io_pacejka_with_imu_bias.cpp)
  list(APPEND CPP_SRC_FILES
       src/state_message_conversion_pacejka_with_imu_bias.cpp)
else()
  message(
    "Stacked model or IMU bias model not found. Building without stacked model/imu bias support"
  )
endif()

if(${lowpass_estimator_FOUND})
  add_definitions(-Dlowpass_estimator_FOUND)
  list(APPEND CPP_SRC_FILES src/parameter_io_lowpass.cpp)
else()
  message(
    "Lowpass Estimator not found. Building without Lowpass Estimator support")
endif()

if(${kalman_estimator_FOUND})
  add_definitions(-Dkalman_estimator_FOUND)
  list(APPEND CPP_SRC_FILES src/parameter_io_kalman.cpp)
else()
  message(
    "Kalman Estimator not found. Building without Kalman Estimator support")
endif()

if(${pid_controller_FOUND})
  add_definitions(-Dpid_controller_FOUND)
  list(APPEND CPP_SRC_FILES src/parameter_io_pid.cpp)
else()
  message("PID Controller not found. Building without PID controller support")
endif()

if(${ff_fb_controller_FOUND})
  add_definitions(-Dff_fb_controller_FOUND)
  list(APPEND CPP_SRC_FILES src/parameter_io_ff_fb.cpp)
else()
  message(
    "FF FB Controller not found. Building without FF FB controller support")
endif()

if(${pacejka_mpcc_FOUND})
  add_definitions(-DPACEJKA_MPCC_FOUND)
  list(APPEND CPP_SRC_FILES src/parameter_io_pacejka_mpcc.cpp)
else()
  message("Pacejka MPCC not found. Building without Pacejka MPCC support")
endif()

if(${pacejka_tracking_mpc_FOUND})
  add_definitions(-DPACEJKA_TRACKING_MPC_FOUND)
  list(APPEND CPP_SRC_FILES src/parameter_io_pacejka_tracking_mpc.cpp)
else()
  message(
    "Pacejka Tracking MPC not found. Building without Pacejka Tracking MPC support"
  )
endif()

if(${kinematic_tracking_mpc_FOUND})
  add_definitions(-DKINEMATIC_TRACKING_MPC_FOUND)
  list(APPEND CPP_SRC_FILES src/parameter_io_kinematic_tracking_mpc.cpp)
else()
  message(
    "Kinematic Tracking MPC not found. Building without Kinematic Tracking MPC support"
  )
endif()

if(${pacejka_model_FOUND})
  add_definitions(-Dpacejka_model_FOUND)
  list(APPEND CPP_SRC_FILES src/parameter_io_pacejka.cpp)
  list(APPEND CPP_SRC_FILES src/state_message_conversion_pacejka.cpp)
else()
  message("Pacejka Model not found. Building without Pacejka model support")
endif()

if(${kinematic_model_FOUND})
  add_definitions(-Dkinematic_model_FOUND)
  list(APPEND CPP_SRC_FILES src/parameter_io_kinematic.cpp)
  list(APPEND CPP_SRC_FILES src/state_message_conversion_kinematic.cpp)
else()
  message("Kinematic Model not found. Building without kinematic model support")
endif()

if(${rocket_6_dof_model_FOUND})
  add_definitions(-Drocket_6_dof_model_FOUND)
  list(APPEND CPP_SRC_FILES src/parameter_io_rocket.cpp)
  list(APPEND CPP_SRC_FILES src/state_message_conversion_rocket.cpp)
else()
  message("Rocket Model not found. Building without rocket model support")
endif()

if(${rocket_control_commons_FOUND})
  add_definitions(-Drocket_control_commons_FOUND)
  list(APPEND CPP_SRC_FILES src/parameter_io_rocket_control_commons.cpp)
else()
  message(
    "Rocket Control Commons not found. Building without rocket controller support"
  )
endif()

if(${rocket_position_pid_FOUND})
  add_definitions(-Drocket_position_pid_FOUND)
  list(APPEND CPP_SRC_FILES src/parameter_io_rocket_position_pid.cpp)
else()
  message(
    "Rocket Position PID not found. Building without rocket controller support")
endif()

if(${pacejka_mpc_safety_filter_FOUND})
  add_definitions(-Dpacejka_mpc_safety_filter_FOUND)
  list(APPEND CPP_SRC_FILES src/parameter_io_pacejka_mpc_safety.cpp)
else()
  message(
    "Pacejka MPC Safety not found. Building without Pacejka MPC Safety support")
endif()

cs_add_library(${PROJECT_NAME} ${CPP_SRC_FILES})

target_link_libraries(${PROJECT_NAME})

# ##############################################################################
# Tests #
# ##############################################################################

# ##############################################################################
# Executables #
# ##############################################################################

# ##############################################################################
# Export #
# ##############################################################################

cs_install()
cs_export()
