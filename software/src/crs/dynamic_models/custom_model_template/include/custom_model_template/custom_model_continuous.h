#ifndef CUSTOM_MODEL_TEMPLATE_CUSTOM_MODEL_CONTINUOUS_H
#define CUSTOM_MODEL_TEMPLATE_CUSTOM_MODEL_CONTINUOUS_H

#include <Eigen/Core>
#include <algorithm>
#include <casadi/casadi.hpp>
#include <iostream>

#include "custom_model_template/custom_input.h"
#include "custom_model_template/custom_params.h"
#include "custom_model_template/custom_state.h"
#include <dynamic_models/continuous_dynamic_model.h>

namespace crs_models
{
namespace custom_model
{
class ContinuousCustomModel : public ContinuousDynamicModel<custom_state, custom_input>
{
public:
  ContinuousCustomModel(custom_params params);  // Constructor

  /**
   * @brief applies the model for a given control input
   *
   */
  custom_state applyModel(const custom_state state, const custom_input control_input);

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
  custom_params params;
};

}  // namespace custom_model

}  // namespace crs_models
#endif
