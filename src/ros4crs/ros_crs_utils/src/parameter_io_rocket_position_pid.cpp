#include <ros_crs_utils/parameter_io.h>

#include <rocket_position_pid/rocket_high_level_pid_controller_config.h>
#include <rocket_position_pid/rocket_controller_specializations.h>

namespace parameter_io
{

template <>
crs_controls::rocket_high_level_pid_controller_config
getConfig<crs_controls::rocket_high_level_pid_controller_config>(const ros::NodeHandle& nh)
{
  crs_controls::rocket_high_level_pid_controller_config params;
  double position_p_gain_x = 1.0;
  if (!nh.getParam("position_p_gain_x", position_p_gain_x))
    ROS_WARN_STREAM(
        " getConfig<crs_controls::rocket_high_level_pid_controller_config>: did not load position_p_gain_x");
  double position_p_gain_y = 1.0;
  if (!nh.getParam("position_p_gain_y", position_p_gain_y))
    ROS_WARN_STREAM(
        " getConfig<crs_controls::rocket_high_level_pid_controller_config>: did not load position_p_gain_z");
  double position_p_gain_z = 1.0;
  if (!nh.getParam("position_p_gain_z", position_p_gain_z))
    ROS_WARN_STREAM(
        " getConfig<crs_controls::rocket_high_level_pid_controller_config>: did not load position_p_gain_z");
  params.position_p_gain << position_p_gain_x, position_p_gain_y, position_p_gain_z;

  double velocity_p_gain_x = 1.0;
  if (!nh.getParam("velocity_p_gain_x", velocity_p_gain_x))
    ROS_WARN_STREAM(
        " getConfig<crs_controls::rocket_high_level_pid_controller_config>: did not load velocity_p_gain_x");
  double velocity_p_gain_y = 1.0;
  if (!nh.getParam("velocity_p_gain_y", velocity_p_gain_y))
    ROS_WARN_STREAM(
        " getConfig<crs_controls::rocket_high_level_pid_controller_config>: did not load velocity_p_gain_y");
  double velocity_p_gain_z = 1.0;
  if (!nh.getParam("velocity_p_gain_z", velocity_p_gain_z))
    ROS_WARN_STREAM(
        " getConfig<crs_controls::rocket_high_level_pid_controller_config>: did not load velocity_p_gain_z");
  params.velocity_p_gain << velocity_p_gain_x, velocity_p_gain_y, velocity_p_gain_z;

  double velocity_i_gain_x = 0.0;
  if (!nh.getParam("velocity_i_gain_x", velocity_i_gain_x))
    ROS_WARN_STREAM(
        " getConfig<crs_controls::rocket_high_level_pid_controller_config>: did not load velocity_i_gain_x");
  double velocity_i_gain_y = 0.0;
  if (!nh.getParam("velocity_i_gain_y", velocity_i_gain_y))
    ROS_WARN_STREAM(
        " getConfig<crs_controls::rocket_high_level_pid_controller_config>: did not load velocity_i_gain_y");
  double velocity_i_gain_z = 0.0;
  if (!nh.getParam("velocity_i_gain_z", velocity_i_gain_z))
    ROS_WARN_STREAM(
        " getConfig<crs_controls::rocket_high_level_pid_controller_config>: did not load velocity_i_gain_z");
  params.velocity_i_gain << velocity_i_gain_x, velocity_i_gain_y, velocity_i_gain_z;

  double velocity_d_gain_x = 0.0;
  if (!nh.getParam("velocity_d_gain_x", velocity_d_gain_x))
    ROS_WARN_STREAM(
        " getConfig<crs_controls::rocket_high_level_pid_controller_config>: did not load velocity_d_gain_x");
  double velocity_d_gain_y = 0.0;
  if (!nh.getParam("velocity_d_gain_y", velocity_d_gain_y))
    ROS_WARN_STREAM(
        " getConfig<crs_controls::rocket_high_level_pid_controller_config>: did not load velocity_d_gain_y");
  double velocity_d_gain_z = 0.0;
  if (!nh.getParam("velocity_d_gain_z", velocity_d_gain_z))
    ROS_WARN_STREAM(
        " getConfig<crs_controls::rocket_high_level_pid_controller_config>: did not load velocity_d_gain_z");
  params.velocity_d_gain << velocity_d_gain_x, velocity_d_gain_y, velocity_d_gain_z;

  double velocity_i_limit_x = 2.0;
  if (!nh.getParam("velocity_i_limits_x", velocity_i_limit_x))
    ROS_WARN_STREAM(
        " getConfig<crs_controls::rocket_high_level_pid_controller_config>: did not load velocity_i_limits_x");
  double velocity_i_limit_y = 1.0;
  if (!nh.getParam("velocity_i_limits_y", velocity_i_limit_y))
    ROS_WARN_STREAM(
        " getConfig<crs_controls::rocket_high_level_pid_controller_config>: did not load velocity_i_limits_y");
  double velocity_i_limit_z = 1.0;
  if (!nh.getParam("velocity_i_limits_z", velocity_i_limit_z))
    ROS_WARN_STREAM(
        " getConfig<crs_controls::rocket_high_level_pid_controller_config>: did not load velocity_i_limits_z");
  params.velocity_i_limits << velocity_i_limit_x, velocity_i_limit_y, velocity_i_limit_z;

  double loop_rate = 50.0;
  if (!nh.getParam("loop_rate", loop_rate))
    ROS_WARN_STREAM(" getConfig<crs_controls::rocket_high_level_pid_controller_config>: did not load loop_rate");
  params.loop_rate = loop_rate;

  double max_angle = 20.0;
  if (!nh.getParam("max_angle", max_angle))
    ROS_WARN_STREAM(" getConfig<crs_controls::rocket_high_level_pid_controller_config>: did not load max_angle");
  params.max_angle = max_angle / 180.0 * M_PI;
  return params;
}

template <>
crs_controls::rocket_controller_config<crs_controls::RocketHighLevelPidController,
                                       crs_controls::RocketAttitudeController, crs_controls::Rocket6DofAllocation>
getConfig<crs_controls::rocket_controller_config<crs_controls::RocketHighLevelPidController,
                                                 crs_controls::RocketAttitudeController,
                                                 crs_controls::Rocket6DofAllocation>>(const ros::NodeHandle& nh)
{
  crs_controls::rocket_controller_config<crs_controls::RocketHighLevelPidController,
                                         crs_controls::RocketAttitudeController, crs_controls::Rocket6DofAllocation>
      config;

  config.high_level_controller_config =
      getConfig<crs_controls::rocket_high_level_pid_controller_config>(ros::NodeHandle(nh, "high_level_controller"));
  config.low_level_controller_config =
      getConfig<crs_controls::rocket_attitude_controller_config>(ros::NodeHandle(nh, "low_level_controller"));

  config.allocation_config =
      getConfig<crs_controls::rocket_6_dof_allocation_config>(ros::NodeHandle(nh, "allocator_params"));
  return config;
}

}  // namespace parameter_io
