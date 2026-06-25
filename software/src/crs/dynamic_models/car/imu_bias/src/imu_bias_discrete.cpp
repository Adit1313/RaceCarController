
#include <Eigen/Core>
#include <algorithm>
#include <casadi/casadi.hpp>
#include <iostream>

#include "imu_bias/imu_bias_state.h"
#include "imu_bias/imu_bias_continuous.h"
#include "imu_bias/imu_bias_discrete.h"
#include <dynamic_models/discrete_dynamic_model_wrapper.h>
#include <dynamic_models/utils/data_conversion.h>

namespace crs_models
{
namespace imu_bias
{

// Option 1: Create an object and specify the process noise covariance matrix Q yourself.
DiscreteImuBias::DiscreteImuBias(Eigen::Matrix<double, DiscreteImuBias::NX, DiscreteImuBias::NX> Q,
                                 std::string integration_method /* rk (default) */)
  : DiscreteDynamicModelWrapper(Q)
{
  auto cont_model = std::make_unique<crs_models::imu_bias::ContinuousImuBias>();

  // Setup integrator
  using namespace casadi;  // somehow this is needed to use vertcat(), casadi::vertcat() does not work
  std::vector<casadi::MX> state_mx = { casadi::MX::sym("bias_ax"), casadi::MX::sym("bias_ay"),
                                       casadi::MX::sym("bias_dyaw") };
  std::vector<casadi::MX> input_mx = { casadi::MX::sym("Ts") };
  std::vector<casadi::MX> state_dot_mx = cont_model->getContinuousDynamics(state_mx, input_mx);

  auto x = vertcat(state_mx);
  auto p = vertcat(input_mx);
  auto ode = vertcat(state_dot_mx);

  casadi::MXDict dae = { // list of lists
                         { "x", x },
                         { "p", p },
                         { "ode", ode * input_mx[0] }
  };  // The main problem we want to solve
  casadi::Dict opts = {};
  const float t0 = 0;
  const float tf = 1;

  integrator_ = casadi::integrator("cont_dynamics_integrator", integration_method, dae, t0, tf, opts);
}

/**
 * @brief integrates the state for a given integration_time
 *
 * @param state
 * @param control_input
 * @param integration_time
 * @return imu_bias_state returns the integrated state
 */
imu_bias_state DiscreteImuBias::applyModel(const imu_bias_state state,
                                           const imu_bias_input control_input [[maybe_unused]], double integration_time)
{
  assert(integration_time >= 0);  // Model can not go backwards in time

  if (integration_time == 0)
  {
    return state;
  }

  // This solves the differential equation dx/dt from t = 0 to t = ts using x(0) = x0
  casadi::Function::MapArg arg = { { "x0", { state.bias_ax, state.bias_ay, state.bias_dyaw } },
                                   { "p", { integration_time } } };
  imu_bias_state output_state;

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
}  // namespace imu_bias
}  // namespace crs_models
