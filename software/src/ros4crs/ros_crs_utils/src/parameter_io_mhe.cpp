#include <ros_crs_utils/parameter_io.h>

#include <mh_estimator/pacejka_mhe_config.h>

namespace parameter_io
{
template <>
crs_estimators::pacejka_mhe_config getConfig<crs_estimators::pacejka_mhe_config>(const ros::NodeHandle& nh)
{
  crs_estimators::pacejka_mhe_config params;

  if (!getMatrixFromParams<crs_models::pacejka_model::pacejka_car_state::NX,
                           crs_models::pacejka_model::pacejka_car_state::NX>(ros::NodeHandle(nh, "P"), params.P))
    ROS_WARN_STREAM(" getConfig<crs_estimators::pacejka_mhe_config>: did not load P");

  if (!getMatrixFromParams<crs_models::pacejka_model::pacejka_car_state::NX,
                           crs_models::pacejka_model::pacejka_car_state::NX>(ros::NodeHandle(nh, "Q"), params.Q))
    ROS_WARN_STREAM(" getConfig<crs_estimators::pacejka_mhe_config>: did not load Q");

  if (!getMatrixFromParams<3, 3>(ros::NodeHandle(nh, "R_mocap"), params.R_mocap))
    ROS_WARN_STREAM(" getConfig<crs_estimators::pacejka_mhe_config>: did not load R_mocap");

  if (!getMatrixFromParams<3, 3>(ros::NodeHandle(nh, "R_imu"), params.R_imu))
    ROS_WARN_STREAM(" getConfig<crs_estimators::pacejka_mhe_config>: did not load R_imu");

  if (!getMatrixFromParams<1, 1>(ros::NodeHandle(nh, "R_imu_yaw_rate"), params.R_imu_yaw_rate))
    ROS_WARN_STREAM(" getConfig<crs_estimators::pacejka_mhe_config>: did not load R_imu_yaw_rate");

  if (!getMatrixFromParams<4, 4>(ros::NodeHandle(nh, "R_wheel_encoders"), params.R_wheel_encoders))
    ROS_WARN_STREAM(" getConfig<crs_estimators::pacejka_mhe_config>: did not load R_wheel_encoders");

  if (!getMatrixFromParams<4, 4>(ros::NodeHandle(nh, "R_lighthouse"), params.R_lighthouse))
    ROS_WARN_STREAM(" getConfig<crs_estimators::pacejka_mhe_config>: did not load R_lighthouse");

  if (!nh.getParam("solver_type", params.solver_type))
    ROS_WARN_STREAM(" getConfig<crs_estimators::pacejka_mhe_config>: did not load solver type");

  if (!nh.getParam("start_delay", params.start_delay))
    ROS_WARN_STREAM(" getConfig<crs_estimators::pacejka_mhe_config>: did not load start_delay. Defaulting to 0.0");

  if (!nh.getParam("max_buffer_size", params.max_buffer_size))
    ROS_WARN_STREAM(" getConfig<crs_estimators::pacejka_mhe_config>: did not load max_buffer_size. Defaulting to 400");

  if (!nh.getParam("lag_compensation_time", params.lag_compensation_time))
    ROS_WARN_STREAM(
        " getConfig<crs_estimators::pacejka_mhe_config>: did not load lag_compensation_time. Defaulting to 0.02");

  if (!nh.getParam("warmstart_iterations", params.warmstart_iterations))
    ROS_WARN_STREAM(
        " getConfig<crs_estimators::pacejka_mhe_config>: did not load warmstart_iterations. Defaulting to 5");

  if (!nh.getParam("use_internal_estimator", params.use_internal_estimator))
    ROS_WARN_STREAM(
        "getConfig<crs_estimators::pacejka_mhe_config>: did not load use_internal_estimator. Defaulting to false");

  if (!nh.getParam("recover_internal_estimate_if_solver_failure", params.recover_internal_estimate_if_solver_failure))
    ROS_WARN_STREAM("getConfig<crs_estimators::pacejka_mhe_config>: did not load "
                    "recover_internal_estimate_if_solver_failure. Defaulting to false");

  if (!nh.getParam("use_internal_filter", params.use_internal_filter))
    ROS_WARN_STREAM(
        "getConfig<crs_estimators::pacejka_mhe_config>: did not load use_internal_filter. Defaulting to false");

  if (!nh.getParam("internal_filter_type", params.internal_filter_type))
    ROS_WARN_STREAM("getConfig<crs_estimators::pacejka_mhe_config>: did not load internal_filter_type.");

  if (!nh.getParam("eta", params.eta))
    ROS_WARN_STREAM("getConfig<crs_estimators::pacejka_mhe_config>: did not load eta. Defaulting to 0.9");

  if (!nh.getParam("print_solve_time", params.print_solve_time))
    ROS_WARN_STREAM(
        "getConfig<crs_estimators::pacejka_mhe_config>: did not load print_solve_time. Defaulting to false");

  return params;
}

template <>
crs_estimators::pacejka_mhe_config getOutlierParams<crs_estimators::pacejka_mhe_config>(const ros::NodeHandle& nh)
{
  crs_estimators::pacejka_mhe_config params;

  if (!nh.getParam("use_outlier_rejection", params.use_outlier_rejection))
    ROS_WARN_STREAM(
        "getConfig<crs_estimators::pacejka_mhe_config>: did not load use_outlier_rejection. Defaulting to false");

  if (!nh.getParam("outlier_threshold", params.outlier_threshold))
    ROS_WARN_STREAM("getConfig<crs_estimators::pacejka_mhe_config>: did not load outlier_threshold.");

  return params;
}

}  // namespace parameter_io
