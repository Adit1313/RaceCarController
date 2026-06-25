#ifndef ROCKET_6_DOF_MODEL_ROCKET_6_DOF_CONTINUOUS_H
#define ROCKET_6_DOF_MODEL_ROCKET_6_DOF_CONTINUOUS_H

#include <Eigen/Core>
#include <algorithm>
#include <casadi/casadi.hpp>
#include <iostream>

#include "rocket_6_dof_model/rocket_6_dof_input.h"
#include "rocket_6_dof_model/rocket_6_dof_params.h"
#include "rocket_6_dof_model/rocket_6_dof_state.h"
#include <dynamic_models/continuous_dynamic_model.h>

namespace crs_models
{
namespace rocket_6_dof_model
{

class ContinuousRocket6DofModel : public ContinuousDynamicModel<rocket_6_dof_state, rocket_6_dof_input>
{
public:
  ContinuousRocket6DofModel(rocket_6_dof_params params);  // Constructor

  /**
   * @brief applies the model for a given control input
   *
   */
  rocket_6_dof_state applyModel(const rocket_6_dof_state state, const rocket_6_dof_input control_input);

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
  /**
   * @brief Compute the gimbal angle given the servo angle.
   *
   * @param servo_angle is the servo angle in radians
   * @param tilt_axis_angle is the previous gimbal angle. I.e. if we are computing gimbal angle 2
   * then tilt_axis_angle = gimal_angle_1 and if we are computing gimbal angle 1 then it is simply 0.
   */
  casadi::MX computeGimbalAngle(casadi::MX& servo_angle, casadi::MX& tilt_axis_angle);

  rocket_6_dof_params params;
};

}  // namespace rocket_6_dof_model

}  // namespace crs_models
#endif
