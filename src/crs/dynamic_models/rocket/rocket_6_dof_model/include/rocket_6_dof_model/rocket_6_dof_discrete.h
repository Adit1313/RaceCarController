#ifndef ROCKET_6_DOF_MODEL_ROCKET_6_DOF_DISCRETE_H
#define ROCKET_6_DOF_MODEL_ROCKET_6_DOF_DISCRETE_H

#include <Eigen/Core>
#include <algorithm>
#include <casadi/casadi.hpp>
#include <iostream>

#include "rocket_6_dof_model/rocket_6_dof_continuous.h"
#include <dynamic_models/discrete_dynamic_model_wrapper.h>
#include <dynamic_models/utils/data_conversion.h>

namespace crs_models
{
namespace rocket_6_dof_model
{
class DiscreteRocket6DofModel : public DiscreteDynamicModelWrapper<rocket_6_dof_state, rocket_6_dof_input>
{
public:
  /**
   * @brief Construct a new Discrete Rocket 6-DoF Model object.
   * matrix
   *
   * @param params Process noise covariance matrix Q. Unit 1/s
   * @param params Rocket 6-DoF parameters
   * @param integration_method integration method. Supported Integrators - see casadi documentation:  cvodes, idas,
   * collocation, oldcollocation, rk
   */
  DiscreteRocket6DofModel(rocket_6_dof_params params,
                          Eigen::Matrix<double, DiscreteRocket6DofModel::NX, DiscreteRocket6DofModel::NX> Q,
                          std::string integration_method = "rk");

  /**
   * @brief Construct a new Discrete Rocket 6-DoF Model object. Process noise covariance matrix Q will default to the
   * unit matrix
   *
   * @param params Rocket 6-DoF parameters
   * @param integration_method integration method. Supported Integrators - see casadi documentation:  cvodes, idas,
   * collocation, oldcollocation, rk
   */
  DiscreteRocket6DofModel(rocket_6_dof_params params, std::string integration_method = "rk")
    : DiscreteRocket6DofModel(
          params, Eigen::Matrix<double, DiscreteRocket6DofModel::NX, DiscreteRocket6DofModel::NX>::Identity(),
          integration_method)
  {
    std::cout << "[WARNING] No Q Matrix specified for DiscreteRocket6DofModel. Using identity Matrix! " << std::endl;
  }

  /**
   * @brief integrates the state for a given integration_time
   *
   * @param state
   * @param control_input
   * @param integration_time
   * @return rocket_6_dof_state returns the integrated state
   */
  rocket_6_dof_state applyModel(const rocket_6_dof_state state, const rocket_6_dof_input control_input,
                                double integration_time);
};

}  // namespace rocket_6_dof_model

}  // namespace crs_models
#endif
