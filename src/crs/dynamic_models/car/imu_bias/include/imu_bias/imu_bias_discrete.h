#ifndef SRC_CRS_DYNAMIC_MODELS_IMU_BIAS_INCLUDE_IMU_BIASL_IMU_BIAS_DISCRETE
#define SRC_CRS_DYNAMIC_MODELS_IMU_BIAS_INCLUDE_IMU_BIASL_IMU_BIAS_DISCRETE

#include "imu_bias_state.h"
#include "imu_bias_input.h"
#include <algorithm>
#include <dynamic_models/continuous_dynamic_model.h>
#include <dynamic_models/discrete_dynamic_model_wrapper.h>
#include <iostream>
#include <iostream>

namespace crs_models
{
namespace imu_bias
{

class DiscreteImuBias : public DiscreteDynamicModelWrapper<imu_bias_state, imu_bias_input>
{
public:
  /**
   * @brief Construct a new Discrete Pacejka Model object.
   * matrix
   *
   * @param params Process noise covariance matrix Q. Unit 1/s
   * @param params kinematic parameters
   * @param integration_method integration method. Supported Integrators - see casadi documentation:  cvodes, idas,
   * collocation, oldcollocation, rk
   */
  DiscreteImuBias(Eigen::Matrix<double, 3, 3> Q, std::string integration_method = "rk");

  /**
   * @brief Construct a new Discrete Pacejka Model object. Process noise covariance matrix Q will default to the unit
   * matrix
   *
   * @param params kinematic parameters
   * @param integration_method integration method. Supported Integrators - see casadi documentation:  cvodes, idas,
   * collocation, oldcollocation, rk
   */
  DiscreteImuBias(std::string integration_method = "rk")
    : DiscreteImuBias(Eigen::Matrix<double, 3, 3>::Identity(), integration_method)
  {
    std::cout << "[WARNING] No Q Matrix specified for DiscreteImuBias. Using identity Matrix! " << std::endl;
  }

  /**
   * @brief integrates the state for a given integration_time
   *
   * @param state
   * @param control_input
   * @param integration_time
   * @return pacejka_car_state returns the integrated state
   */
  imu_bias_state applyModel(const imu_bias_state state, const imu_bias_input control_input, double integration_time);
};

}  // namespace imu_bias

}  // namespace crs_models
#endif /* IMU_BIAS_DISCRETE */
