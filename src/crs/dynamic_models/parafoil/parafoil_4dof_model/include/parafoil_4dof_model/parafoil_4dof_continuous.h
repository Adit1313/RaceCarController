#ifndef PARAFOIL_4DOF_MODEL_PARAFOIL_4DOF_CONTINUOUS_H
#define PARAFOIL_4DOF_MODEL_PARAFOIL_4DOF_CONTINUOUS_H

#include <Eigen/Core>
#include <algorithm>
#include <casadi/casadi.hpp>
#include <iostream>

#include "parafoil_4dof_model/parafoil_4dof_input.h"
#include "parafoil_4dof_model/parafoil_4dof_params.h"
#include "parafoil_4dof_model/parafoil_4dof_state.h"
#include <dynamic_models/continuous_dynamic_model.h>

namespace crs_models
{
namespace parafoil_4dof_model
{
class ContinuousParafoil4dofModel : public ContinuousDynamicModel<parafoil_4dof_state, parafoil_4dof_input>
{
public:
  ContinuousParafoil4dofModel(parafoil_4dof_params params);  // Constructor

  /**
   * @brief applies the model for a given control input
   *
   */
  parafoil_4dof_state applyModel(const parafoil_4dof_state state, const parafoil_4dof_input control_input);

  /**
   * @brief Returns the casadi function x_dot = f(x,u)
   *
   * @param state the state x
   * @param control_input  the input u
   * @return
   */
  std::vector<casadi::MX> getContinuousDynamics(const std::vector<casadi::MX> state,
                                                const std::vector<casadi::MX> control_input);

private:
  parafoil_4dof_params params;
};

}  // namespace parafoil_4dof_model

}  // namespace crs_models
#endif
