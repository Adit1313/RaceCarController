#include "ros_crs_utils/parameter_io.h"

#include "pacejka_tracking_mpc/tracking_mpc_pacejka_config.h"

namespace parameter_io
{
template <>
crs_controls::pacejka_tracking_mpc::tracking_mpc_pacejka_config
getConfig<crs_controls::pacejka_tracking_mpc::tracking_mpc_pacejka_config>(const ros::NodeHandle& nh)
{
  crs_controls::pacejka_tracking_mpc::tracking_mpc_pacejka_config params;

  if (!nh.getParam("Q1", params.Q1))
    ROS_WARN_STREAM(" getConfig<crs_controls::pacejka_tracking_mpc::tracking_mpc_pacejka_config>: did not load Q1");
  if (!nh.getParam("Q2", params.Q2))
    ROS_WARN_STREAM(" getConfig<crs_controls::pacejka_tracking_mpc::tracking_mpc_pacejka_config>: did not load Q2");
  if (!nh.getParam("R1", params.R1))
    ROS_WARN_STREAM(" getConfig<crs_controls::pacejka_tracking_mpc::tracking_mpc_pacejka_config>: did not load R1");
  if (!nh.getParam("R2", params.R2))
    ROS_WARN_STREAM(" getConfig<crs_controls::pacejka_tracking_mpc::tracking_mpc_pacejka_config>: did not load R2");
  if (!nh.getParam("lag_compensation_time", params.lag_compensation_time))
    ROS_WARN_STREAM(" getConfig<crs_controls::pacejka_tracking_mpc::tracking_mpc_pacejka_config>: did not load "
                    "lag_compensation_time");
  if (!nh.getParam("solver_type", params.solver_type))
    ROS_WARN_STREAM(
        " getConfig<crs_controls::pacejka_tracking_mpc::tracking_mpc_pacejka_config>: did not load solver_type");

  return params;
}
}  // namespace parameter_io
