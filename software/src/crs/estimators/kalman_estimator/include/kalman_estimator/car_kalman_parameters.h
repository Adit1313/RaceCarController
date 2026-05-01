#ifndef KALMAN_ESTIMATOR_CAR_KALMAN_PARAMETERS_H
#define KALMAN_ESTIMATOR_CAR_KALMAN_PARAMETERS_H
#include <vector>
#include <iostream>
#include <yaml-cpp/yaml.h>

namespace crs_estimators
{
namespace kalman
{
struct car_kalman_parameters
{
  /**
   * @brief Te string corresponds to a sensor name, the vector contains [use_outlier_rejection, outlier_rejection_type,
   * outlier_threshold]
   *
   */
  // std::vector<std::pair<std::string, std::vector<float>>> outlier_rejection_params;

  /**
   * @brief Bool if outlier rejection should be used
   *
   */
  bool use_outlier_rejection;

  /**
   * @brief String describing type of outlier rejection to use
   *
   */
  std::string outlier_rejection_type;

  /**
   * @brief Threshold for outlier rejection
   *
   */
  double outlier_threshold;

  /**
   * @brief Max number of consecutive outliers before measurement is accepted, even if classified as outlier
   *
   */
  int max_consecutive_outliers = 0;
};

}  // namespace kalman
}  // namespace crs_estimators

#endif  // KALMAN_ESTIMATOR_CAR_KALMAN_PARAMETERS_H
