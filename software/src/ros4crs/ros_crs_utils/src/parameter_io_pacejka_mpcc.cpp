#include "ros_crs_utils/parameter_io.h"

#include "pacejka_mpcc/mpcc_pacejka_config.h"

namespace parameter_io
{
template <>
crs_controls::pacejka_mpcc::mpcc_pacejka_config
getConfig<crs_controls::pacejka_mpcc::mpcc_pacejka_config>(const ros::NodeHandle& nh)
{
  crs_controls::pacejka_mpcc::mpcc_pacejka_config params;

  if (!nh.getParam("Q1", params.Q1))
    ROS_WARN_STREAM(" getConfig<crs_controls::pacejka_mpcc::mpcc_pacejka_config>: did not load Q1");
  if (!nh.getParam("Q2", params.Q2))
    ROS_WARN_STREAM(" getConfig<crs_controls::pacejka_mpcc::mpcc_pacejka_config>: did not load Q2");
  if (!nh.getParam("R1", params.R1))
    ROS_WARN_STREAM(" getConfig<crs_controls::pacejka_mpcc::mpcc_pacejka_config>: did not load R1");
  if (!nh.getParam("R2", params.R2))
    ROS_WARN_STREAM(" getConfig<crs_controls::pacejka_mpcc::mpcc_pacejka_config>: did not load R2");
  if (!nh.getParam("R3", params.R3))
    ROS_WARN_STREAM(" getConfig<crs_controls::pacejka_mpcc::mpcc_pacejka_config>: did not load R3");
  if (!nh.getParam("q", params.q))
    ROS_WARN_STREAM(" getConfig<crs_controls::pacejka_mpcc::mpcc_pacejka_config>: did not load q");
  if (!nh.getParam("lag_compensation_time", params.lag_compensation_time))
    ROS_WARN_STREAM(" getConfig<crs_controls::pacejka_mpcc::mpcc_pacejka_config>: did not load lag_compensation_time");
  if (!nh.getParam("solver_type", params.solver_type))
    ROS_WARN_STREAM(" getConfig<crs_controls::pacejka_mpcc::mpcc_pacejka_config>: did not load solver_type");

  int max_sqp_iter = 1;
  if (!nh.getParam("max_sqp_iterations", max_sqp_iter))
    ROS_WARN_STREAM(" getConfig<crs_controls::pacejka_mpcc::mpcc_pacejka_config>: did not load max_sqp_iterations");
  params.max_sqp_iterations = static_cast<unsigned int>(max_sqp_iter);

  nh.getParam("warmstart_iterations", params.warmstart_iterations);  // Optional Param. No warning raised if not set.

  return params;
}
}  // namespace parameter_io
