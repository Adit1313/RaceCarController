#include <ros_crs_utils/parameter_io.h>

#include "rocket_control_commons/rocket_attitude_controller_config.h"
#include "rocket_control_commons/rocket_6_dof_allocation.h"
#include "rocket_position_pid/rocket_high_level_pid_controller_config.h"

namespace parameter_io
{

template <>
crs_controls::rocket_attitude_controller_config
getConfig<crs_controls::rocket_attitude_controller_config>(const ros::NodeHandle& nh)
{
  crs_controls::rocket_attitude_controller_config params;
  double attitude_p_gain_x = 1.0;
  if (!nh.getParam("attitude_p_gain_x", attitude_p_gain_x))
    ROS_WARN_STREAM(" getConfig<crs_controls::rocket_attitude_controller_config>: did not load attitude_p_gain_x");
  double attitude_p_gain_y = 1.0;
  if (!nh.getParam("attitude_p_gain_y", attitude_p_gain_y))
    ROS_WARN_STREAM(" getConfig<crs_controls::rocket_attitude_controller_config>: did not load attitude_p_gain_y");
  double attitude_p_gain_z = 1.0;
  if (!nh.getParam("attitude_p_gain_z", attitude_p_gain_z))
    ROS_WARN_STREAM(" getConfig<crs_controls::rocket_attitude_controller_config>: did not load attitude_p_gain_z");
  params.attitude_p_gain << attitude_p_gain_x, attitude_p_gain_y, attitude_p_gain_z;

  double rate_p_gain_x = 1.0;
  if (!nh.getParam("rate_p_gain_x", rate_p_gain_x))
    ROS_WARN_STREAM(" getConfig<crs_controls::rocket_attitude_controller_config>: did not load rate_p_gain_x");
  double rate_p_gain_y = 1.0;
  if (!nh.getParam("rate_p_gain_y", rate_p_gain_y))
    ROS_WARN_STREAM(" getConfig<crs_controls::rocket_attitude_controller_config>: did not load rate_p_gain_y");
  double rate_p_gain_z = 1.0;
  if (!nh.getParam("rate_p_gain_z", rate_p_gain_z))
    ROS_WARN_STREAM(" getConfig<crs_controls::rocket_rate_rocket_attitude_controller_configcontroller_config>: did not "
                    "load rate_p_gain_z");
  params.rate_p_gain << rate_p_gain_x, rate_p_gain_y, rate_p_gain_z;

  double rate_i_gain_x = 0.0;
  if (!nh.getParam("rate_i_gain_x", rate_i_gain_x))
    ROS_WARN_STREAM(" getConfig<crs_controls::rocket_attitude_controller_config>: did not load rate_i_gain_x");
  double rate_i_gain_y = 0.0;
  if (!nh.getParam("rate_i_gain_y", rate_i_gain_y))
    ROS_WARN_STREAM(" getConfig<crs_controls::rocket_attitude_controller_config>: did not load rate_i_gain_y");
  double rate_i_gain_z = 0.0;
  if (!nh.getParam("rate_i_gain_z", rate_i_gain_z))
    ROS_WARN_STREAM(" getConfig<crs_controls::rocket_attitude_controller_config>: did not load rate_i_gain_z");
  params.rate_i_gain << rate_i_gain_x, rate_i_gain_y, rate_i_gain_z;

  double rate_d_gain_x = 0.0;
  if (!nh.getParam("rate_d_gain_x", rate_d_gain_x))
    ROS_WARN_STREAM(" getConfig<crs_controls::rocket_attitude_controller_config>: did not load rate_d_gain_x");
  double rate_d_gain_y = 0.0;
  if (!nh.getParam("rate_d_gain_y", rate_d_gain_y))
    ROS_WARN_STREAM(" getConfig<crs_controls::rocket_attitude_controller_config>: did not load rate_d_gain_y");
  double rate_d_gain_z = 0.0;
  if (!nh.getParam("rate_d_gain_z", rate_d_gain_z))
    ROS_WARN_STREAM(" getConfig<crs_controls::rocket_attitude_controller_config>: did not load rate_d_gain_z");
  params.rate_d_gain << rate_d_gain_x, rate_d_gain_y, rate_d_gain_z;

  double rate_i_limit_x = 0.1;
  if (!nh.getParam("rate_i_limits_x", rate_i_limit_x))
    ROS_WARN_STREAM(" getConfig<crs_controls::rocket_high_level_pid_controller_config>: did not load rate_i_limit_x");
  double rate_i_limit_y = 0.1;
  if (!nh.getParam("rate_i_limits_y", rate_i_limit_y))
    ROS_WARN_STREAM(" getConfig<crs_controls::rocket_high_level_pid_controller_config>: did not load rate_i_limit_y");
  double rate_i_limit_z = 0.1;
  if (!nh.getParam("rate_i_limits_z", rate_i_limit_z))
    ROS_WARN_STREAM(" getConfig<crs_controls::rocket_high_level_pid_controller_config>: did not load rate_i_limit_z");
  params.rate_i_limits << rate_i_limit_x, rate_i_limit_y, rate_i_limit_z;

  double loop_rate = 1000.0;
  if (!nh.getParam("loop_rate", loop_rate))
    ROS_WARN_STREAM(" getConfig<crs_controls::rocket_attitude_controller_config>: did not load loop_rate");
  params.loop_rate = loop_rate;

  std::vector<double> filter_b = { 1, 0 };
  if (!nh.getParam("derivative_filter_numerator", filter_b))
  {
    ROS_WARN_STREAM(
        " getConfig<crs_controls::rocket_attitude_controller_config>: did not load derivative_filter_numerator");
  }
  params.derivative_filter_b = filter_b;

  std::vector<double> filter_a = { 1, 0 };
  if (!nh.getParam("derivative_filter_denominator", filter_a))
  {
    ROS_WARN_STREAM(
        " getConfig<crs_controls::rocket_attitude_controller_config>: did not load derivative_filter_denominator");
  }
  params.derivative_filter_a = filter_a;
  return params;
}

template <>
crs_controls::rocket_6_dof_allocation_config
getConfig<crs_controls::rocket_6_dof_allocation_config>(const ros::NodeHandle& nh)
{
  crs_controls::rocket_6_dof_allocation_config config;
  std::vector<double> constraint_params;
  if (!nh.getParam("constraint_coefficients", constraint_params))
  {
    ROS_WARN_STREAM(" getConfig<crs_controls::rocket_6_dof_allocation_config>: did not load constraint_coefficients");
  }

  std::cout << constraint_params.size() << std::endl;
  config.constraint_coefficients = Eigen::Matrix<double, 4, 2, Eigen::RowMajor>(constraint_params.data());
  return config;
}
}  // namespace parameter_io
