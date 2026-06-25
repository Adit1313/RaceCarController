#ifndef PACEJKA_MODEL_PACEJKA_CONTINUOUS_H
#define PACEJKA_MODEL_PACEJKA_CONTINUOUS_H

#include "pacejka_car_input.h"
#include "pacejka_car_state.h"
#include "pacejka_params.h"
#include "pacejka_params.h"
#include <algorithm>
#include <dynamic_models/continuous_dynamic_model.h>
#include <iostream>
#include <iostream>

namespace crs_models
{
namespace pacejka_model
{

class ContinuousPacejkaModel : public ContinuousDynamicModel<pacejka_car_state, pacejka_car_input>
{
public:
  ContinuousPacejkaModel(pacejka_params params);  // Constructor

  /**
   * @brief applies Pacejka model for a given control input
   *
   */
  pacejka_car_state applyModel(const pacejka_car_state state, const pacejka_car_input control_input);

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
  pacejka_params params;
};

}  // namespace pacejka_model

}  // namespace crs_models
#endif
