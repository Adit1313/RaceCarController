
#include <Eigen/Core>
#include <algorithm>
#include <casadi/casadi.hpp>
#include <iostream>

#include "parafoil_4dof_model/parafoil_4dof_continuous.h"
#include "parafoil_4dof_model/parafoil_4dof_discrete.h"
#include <dynamic_models/discrete_dynamic_model_wrapper.h>
#include <dynamic_models/utils/data_conversion.h>

namespace crs_models
{
namespace parafoil_4dof_model
{

/**
 * @brief Construct a new Discrete 4-DoF Parafoil Model object.
 *
 * @param params kinematic parameters
 * @param Q Process noise covariance matrix Q. Unit 1/s
 */
DiscreteParafoil4dofModel::DiscreteParafoil4dofModel(parafoil_4dof_params params, Eigen::Matrix<double, 9, 9> Q)
  : DiscreteDynamicModelWrapper(Q)
{
  cont_model = std::make_unique<crs_models::parafoil_4dof_model::ContinuousParafoil4dofModel>(params);

  // Setup integrator
  using namespace casadi;  // somehow this is needed to use vertcat(), casadi::vertcat() does not work

  // Define symbolic mx variables
  std::vector<casadi::MX> state_mx = { casadi::MX::sym("x"),   casadi::MX::sym("y"),     casadi::MX::sym("z"),
                                       casadi::MX::sym("phi"), casadi::MX::sym("theta"), casadi::MX::sym("psi"),
                                       casadi::MX::sym("u"),   casadi::MX::sym("v"),     casadi::MX::sym("w") };
  std::vector<casadi::MX> input_mx = { casadi::MX::sym("deflection_symmetric"),
                                       casadi::MX::sym("deflection_asymmetric"), casadi::MX::sym("Ts") };
  // ==================== END TODO ====================

  std::vector<casadi::MX> state_dot_mx = cont_model->getContinuousDynamics(state_mx, input_mx);

  auto x = vertcat(state_mx);
  auto p = vertcat(input_mx);
  auto ode = vertcat(state_dot_mx);

  casadi::MXDict dae = { // list of lists
                         { "x", x },
                         { "p", p },
                         { "ode", ode * input_mx[2] }
  };  // The main problem we want to solve
  casadi::Dict opts = { { "tf", 1 } };

  // Create integrator function, using runge kutta for now
  // TODO(sabodmer) create discretization_params struct with field "intergation_method" and pass it to model in
  // constructor to remove hardcoded rk
  integrator_ = casadi::integrator("cont_dynamics_integrator", "rk", dae, opts);
}

/**
 * @brief integrates the state for a given integration_time
 *
 * @param state
 * @param control_input
 * @param integration_time
 * @return parafoil_4dof_state returns the integrated state
 */
parafoil_4dof_state DiscreteParafoil4dofModel::applyModel(const parafoil_4dof_state state,
                                                          const parafoil_4dof_input control_input,
                                                          double integration_time)
{
  assert(integration_time >= 0);  // Model can not go backwards in time

  if (integration_time == 0)
  {
    return state;
  }

  // This solves the differential equation dx/dt from t = 0 to t = ts using x(0) = x0

  casadi::Function::MapArg arg = {
    { "x0",
      { state.pos_x, state.pos_y, state.pos_z, state.phi, state.theta, state.psi, state.vel_u, state.vel_v,
        state.vel_w } },
    { "p", { control_input.deflection_symmetric, control_input.deflection_asymmetric, integration_time } }
  };
  parafoil_4dof_state output_state;

  auto vec = commons::convertToVector(output_state);
  // TODO(@zrene), this is needed since the casadi integrator returns "9" entries (first one being the state of
  // dimension 4, everything else gets discarded) This is an ugly fix to make sure that states smaller than 9 can be
  // passed to the integrator. Maybe use casadi::Function::MapRes or something similar to populate output
  while (vec.size() < static_cast<size_t>(integrator_.n_out()))
  {
    vec.push_back(nullptr);
  }

  integrator_(integrator_.buf_in(arg), vec);

  return output_state;
}

}  // namespace parafoil_4dof_model

}  // namespace crs_models
