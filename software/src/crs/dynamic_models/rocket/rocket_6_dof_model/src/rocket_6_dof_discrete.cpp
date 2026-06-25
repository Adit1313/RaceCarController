
#include <Eigen/Core>
#include <algorithm>
#include <casadi/casadi.hpp>
#include <iostream>

#include "rocket_6_dof_model/rocket_6_dof_continuous.h"
#include "rocket_6_dof_model/rocket_6_dof_discrete.h"
#include <dynamic_models/discrete_dynamic_model_wrapper.h>
#include <dynamic_models/utils/data_conversion.h>

namespace crs_models
{
namespace rocket_6_dof_model
{

DiscreteRocket6DofModel::DiscreteRocket6DofModel(rocket_6_dof_params params, Eigen::Matrix<double, 17, 17> Q,
                                                 std::string integration_method /* rk (default) */)
  : DiscreteDynamicModelWrapper(Q)
{
  cont_model = std::make_unique<crs_models::rocket_6_dof_model::ContinuousRocket6DofModel>(params);

  // Setup integrator
  using namespace casadi;  // somehow this is needed to use vertcat(), casadi::vertcat() does not work

  std::vector<casadi::MX> state_mx = { casadi::MX::sym("x"),
                                       casadi::MX::sym("y"),
                                       casadi::MX::sym("z"),
                                       casadi::MX::sym("v_x"),
                                       casadi::MX::sym("v_y"),
                                       casadi::MX::sym("v_z"),
                                       casadi::MX::sym("quat_x"),
                                       casadi::MX::sym("quat_y"),
                                       casadi::MX::sym("quat_z"),
                                       casadi::MX::sym("quat_w"),
                                       casadi::MX::sym("angular_vx"),
                                       casadi::MX::sym("angular_vy"),
                                       casadi::MX::sym("angular_vz"),
                                       casadi::MX::sym("thrust_magnitude"),
                                       casadi::MX::sym("torque_x"),
                                       casadi::MX::sym("servo_angle_1"),
                                       casadi::MX::sym("servo_angle_2") };
  std::vector<casadi::MX> input_mx = { casadi::MX::sym("thrust_magnitude"), casadi::MX::sym("torque"),
                                       casadi::MX::sym("servo_angle_1"), casadi::MX::sym("servo_angle_2") };

  std::vector<casadi::MX> state_dot_mx = cont_model->getContinuousDynamics(state_mx, input_mx);

  input_mx.push_back(casadi::MX::sym("Ts"));
  auto x = vertcat(state_mx);
  auto p = vertcat(input_mx);
  auto ode = vertcat(state_dot_mx);

  casadi::MXDict dae = { // list of lists
                         { "x", x },
                         { "p", p },
                         { "ode", ode * input_mx[4] }
  };  // The main problem we want to solve
  casadi::Dict opts = { { "tf", 1 } };

  // Create integrator function
  integrator_ = casadi::integrator("cont_dynamics_integrator", integration_method, dae, opts);
}

/**
 * @brief integrates the state for a given integration_time
 *
 * @param state
 * @param control_input
 * @param integration_time
 * @return rocket_state returns the integrated state
 */
rocket_6_dof_state DiscreteRocket6DofModel::applyModel(const rocket_6_dof_state state,
                                                       const rocket_6_dof_input control_input, double integration_time)
{
  assert(integration_time >= 0);  // Model can not go backwards in time

  if (integration_time == 0)
  {
    return state;
  }

  // This solves the differential equation dx/dt from t = 0 to t = ts using x(0) = x0

  casadi::Function::MapArg arg = { { "x0",
                                     { state.position_x, state.position_y, state.position_z, state.velocity_x,
                                       state.velocity_y, state.velocity_z, state.quaternion_x, state.quaternion_y,
                                       state.quaternion_z, state.quaternion_w, state.angular_velocity_x,
                                       state.angular_velocity_y, state.angular_velocity_z, state.thrust_magnitude,
                                       state.torque_x, state.servo_angle_1, state.servo_angle_2 } },
                                   { "p",
                                     { control_input.thrust_magnitude, control_input.torque,
                                       control_input.servo_angle_1, control_input.servo_angle_2, integration_time } } };
  rocket_6_dof_state output_state;

  auto vec = commons::convertToVector(output_state);
  // TODO(@zrene), this is needed since the casadi integrator returns "6" entries (first one being the state of
  // dimension 4, everything else gets discarded) This is an ugly fix to make sure that states smaller than 6 can be
  // passed to the integrator. Maybe use casadi::Function::MapRes or something similar to populate output
  while (vec.size() < static_cast<size_t>(integrator_.n_out()))
  {
    vec.push_back(nullptr);
  }

  integrator_(integrator_.buf_in(arg), vec);

  return output_state;
}

}  // namespace rocket_6_dof_model

}  // namespace crs_models
