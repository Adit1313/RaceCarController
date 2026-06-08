#include <ros_crs_utils/parameter_io.h>
#include <kalman_estimator/car_kalman_parameters.h>

namespace parameter_io
{
template <>
crs_estimators::kalman::car_kalman_parameters
getConfig<crs_estimators::kalman::car_kalman_parameters>(const ros::NodeHandle& nh)
{
  crs_estimators::kalman::car_kalman_parameters params;

  if (!nh.getParam("use_outlier_rejection", params.use_outlier_rejection))
    ROS_WARN_STREAM(" getConfig<crs_estimators::kalman::car_kalman_parameters>: did not load use_outlier_rejection");
  if (params.use_outlier_rejection)
  {
    ROS_INFO_STREAM("use_outlier_rejection is enabled.");
    if (!nh.getParam("outlier_rejection_type", params.outlier_rejection_type))
      ROS_WARN_STREAM("getConfig<crs_estimators::kalman::car_kalman_parameters>: did not load outlier_rejection_type");
    if (!nh.getParam("outlier_threshold", params.outlier_threshold))
      ROS_WARN_STREAM(" getConfig<crs_estimators::kalman::car_kalman_parameters>: did not load outlier_threshold");
    if (!nh.getParam("max_consecutive_outliers", params.max_consecutive_outliers))
      ROS_WARN_STREAM(
          " getConfig<crs_estimators::kalman::car_kalman_parameters>: did not load max_consecutive_outliers");
  }

  return params;
}

}  // namespace parameter_io
