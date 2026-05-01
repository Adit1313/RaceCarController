#ifndef PARAFOIL_4DOF_MODEL_PARAFOIL_4DOF_DISCRETE_H
#define PARAFOIL_4DOF_MODEL_PARAFOIL_4DOF_DISCRETE_H

#include <Eigen/Core>
#include <algorithm>
#include <casadi/casadi.hpp>
#include <iostream>

#include "parafoil_4dof_model/parafoil_4dof_continuous.h"
#include <dynamic_models/discrete_dynamic_model_wrapper.h>
#include <dynamic_models/utils/data_conversion.h>

namespace crs_models
{
namespace parafoil_4dof_model
{
class DiscreteParafoil4dofModel : public DiscreteDynamicModelWrapper<parafoil_4dof_state, parafoil_4dof_input>
{
public:
  /**
   * @brief Construct a new Discrete 4-DoF Parafoil Model object.
   *
   * @param params kinematic parameters
   * @param Q Process noise covariance matrix Q. Unit 1/s
   */
  DiscreteParafoil4dofModel(parafoil_4dof_params params,
                            Eigen::Matrix<double, DiscreteParafoil4dofModel::NX, DiscreteParafoil4dofModel::NX> Q);

  /**
   * @brief Construct a new Discrete 4-DoF Parafoil Model object. Process noise covariance matrix Q will default to the
   * unit matrix
   *
   * @param params kinematic parameters
   */
  DiscreteParafoil4dofModel(parafoil_4dof_params params)
    : DiscreteParafoil4dofModel(
          params, Eigen::Matrix<double, DiscreteParafoil4dofModel::NX, DiscreteParafoil4dofModel::NX>::Identity())
  {
    std::cout << "[WARNING] No Q Matrix specified for DiscreteParafoil4dofModel. Using identity Matrix! " << std::endl;
  }

  /**
   * @brief integrates the state for a given integration_time
   *
   * @param state
   * @param control_input
   * @param integration_time
   * @return parafoil_4dof_state returns the integrated state
   */
  parafoil_4dof_state applyModel(const parafoil_4dof_state state, const parafoil_4dof_input control_input,
                                 double integration_time);
};

}  // namespace parafoil_4dof_model

}  // namespace crs_models
#endif
