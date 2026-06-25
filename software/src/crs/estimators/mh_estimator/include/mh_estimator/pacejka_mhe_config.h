#ifndef MH_ESTIMATOR_MH_ESTIMATOR_PACEJKA_MHE_CONFIG_H
#define MH_ESTIMATOR_MH_ESTIMATOR_PACEJKA_MHE_CONFIG_H

#include <Eigen/Dense>

#include "pacejka_model/pacejka_car_state.h"

namespace crs_estimators
{
struct pacejka_mhe_config
{
  /**
   * @brief EKF State Covariance Matrix
   *
   */
  Eigen::Matrix<double, crs_models::pacejka_model::pacejka_car_state::NX,
                crs_models::pacejka_model::pacejka_car_state::NX>
      P;

  /**
   * @brief Process Noise Matrix
   *
   */
  Eigen::Matrix<double, crs_models::pacejka_model::pacejka_car_state::NX,
                crs_models::pacejka_model::pacejka_car_state::NX>
      Q;

  /**
   * @brief Mocap Measurement Noise Matrix
   *
   */
  Eigen::Matrix3d R_mocap;

  /**
   * @brief IMU Measurement Noise Matrix
   *
   */
  Eigen::Matrix3d R_imu;

  /**
   * @brief IMU Measurement Noise Matrix
   *
   */
  Eigen::Matrix<double, 1, 1> R_imu_yaw_rate;

  /**
   * @brief Wheel encoder Measurement Noise Matrix
   *
   */
  Eigen::Matrix4d R_wheel_encoders;

  /**
   * @brief Lighthouse Measurement Noise Matrix
   *
   */
  Eigen::Matrix<double, 4, 4> R_lighthouse;

  /**
   * @brief Solver Type
   *
   */
  std::string solver_type;

  /**
   * @brief Start Delay in seconds
   */
  double start_delay = 0.0;

  /**
   * @brief  Maximum buffer size for the data buffers of the measurements, state estimates and inputs
   */
  int max_buffer_size = 400;

  /**
   * @brief The time in seconds to compensate for the delay introduced by solving the mhe problem and potential
   * measurement delays.
   *
   */
  double lag_compensation_time = 0.02;

  /**
   * @brief The number of iterations to warmstart the solver with
   *
   */
  int warmstart_iterations = 5;

  /**
   * @brief Discount factor for the MHE. This determines how heavily older measurements influence the estimate. 0 < Eta
   * < 1, a value of 0.99 means all measurements are weighted approximatly equally.
   *
   */
  double eta = 0.9;

  /**
   * @brief Use internal estimator. If this is set to true, the EKF estimate will be used in the cost function of the
   * MHE
   *
   */
  bool use_internal_estimator = false;

  /**
   * @brief Recover internal estimate if solver failure. If this is set to true, in the event that the MHE can not find
   * a solution, the internal estimator will be used as the estimate.
   *
   */
  bool recover_internal_estimate_if_solver_failure = false;

  /**
   * @brief If set to True, the solve time of the MHE will be printed.
   *
   */
  bool print_solve_time = false;

  /**
   * @brief Use outlier rejection. If this is set to true, the MHE will use the outlier rejection parameters to reject
   * measurements
   *
   */
  bool use_outlier_rejection = false;

  /**
   * @brief Threshold for outlier rejection
   *
   */
  double outlier_threshold;

  /**
   * @brief Use internal filter. If this is set to true, the MHE will use an internal filter to filter the measurements
   *
   */
  bool use_internal_filter = false;

  /**
   * @brief Internal filter type. The type of filter to use for the internal filter
   *
   */
  std::string internal_filter_type = "median";
};
}  // namespace crs_estimators

#endif  // MH_ESTIMATOR_MH_ESTIMATOR_PACEJKA_MHE_CONFIG_H
