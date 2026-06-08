#include <casadi/casadi.hpp>

#include "parafoil_4dof_model/parafoil_4dof_continuous.h"

namespace crs_models
{
namespace parafoil_4dof_model
{
ContinuousParafoil4dofModel::ContinuousParafoil4dofModel(parafoil_4dof_params params) : params(params)  // Constructor
{
  // model inputs and state variable names
  std::vector<casadi::MX> state_mx = { casadi::MX::sym("x"),   casadi::MX::sym("y"),     casadi::MX::sym("z"),
                                       casadi::MX::sym("phi"), casadi::MX::sym("theta"), casadi::MX::sym("psi"),
                                       casadi::MX::sym("u"),   casadi::MX::sym("v"),     casadi::MX::sym("w") };
  std::vector<casadi::MX> input_mx = { casadi::MX::sym("deflection_symmetric"),
                                       casadi::MX::sym("deflection_asymmetric") };

  std::vector<casadi::MX> state_dot_mx = getContinuousDynamics(state_mx, input_mx);
  // append control inputs to state vector (casadi functions only take one input)
  for (const auto input : input_mx)
    state_mx.push_back(input);

  dynamics_f_ = casadi::Function("f_dot", state_mx, state_dot_mx);
}

/**
 * @brief evaluated state_dot (= f(x,u)) at current state and control input
 *
 * @param state
 * @param input
 * @return parafoil_4dof_state returns state_dot evaluated at current state and control input
 */
parafoil_4dof_state ContinuousParafoil4dofModel::applyModel(const parafoil_4dof_state state,
                                                            const parafoil_4dof_input input)
{
  parafoil_4dof_state state_dot_numerical;
  // Convert casadi symbols (mx) to casadi function to be evaluated at current state & control input
  dynamics_f_(commons::convertToConstVector(state, input), commons::convertToVector(state_dot_numerical));
  return state_dot_numerical;
}

/**
 * @brief Get the analytical state equations f(state, input) = state_dot
 *
 * @param state
 * @param control_input
 * @return std::vector<casadi::MX> returns the analytical state equations f(state, input) = state_dot
 */
std::vector<casadi::MX> ContinuousParafoil4dofModel::getContinuousDynamics(const std::vector<casadi::MX> state,
                                                                           const std::vector<casadi::MX> control_input)
{
  using namespace casadi;

  // Just for better readability, function inputs (states and control inputs)
  auto x = state[0];  // Make sure to match these with states MX defined in constructor
  auto y = state[1];
  auto z = state[2];
  auto phi = state[3];
  auto theta = state[4];
  auto psi = state[5];
  auto u = state[6];
  auto v = state[7];
  auto w = state[8];
  auto deflection_symmetric = control_input[0];
  auto deflection_asymmetric = control_input[1];

  // Intermediate function values
  auto V_a = sqrt(pow(u, 2) + pow(w, 2));
  auto alpha = atan2(w, u);
  auto q = params.rho * pow(V_a, 2) / 2;
  auto L = q * params.S * (params.C_L0 + params.C_Ldelta_s * deflection_symmetric);
  auto D = q * params.S * (params.C_D0 + params.C_Ddelta_s * deflection_symmetric);

  Eigen::Matrix<casadi::MX, 3, 3> R_bn;
  R_bn(0, 0) = cos(psi) * cos(theta);
  R_bn(0, 1) = sin(psi) * cos(theta);
  R_bn(0, 2) = -sin(theta);

  R_bn(1, 0) = cos(psi) * sin(theta) * sin(phi) - sin(psi) * cos(phi);
  R_bn(1, 1) = sin(psi) * sin(theta) * sin(phi) + cos(psi) * cos(phi);
  R_bn(1, 2) = cos(theta) * sin(phi);

  R_bn(2, 0) = cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi);
  R_bn(2, 1) = sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi);
  R_bn(2, 2) = cos(theta) * cos(phi);

  Eigen::Matrix<casadi::MX, 3, 1> body_rate(u, v, w);
  Eigen::Matrix<casadi::MX, 3, 1> ned_rate = R_bn.transpose() * body_rate;

  auto phi_rate = (-phi + params.K_phi * deflection_asymmetric) / params.T_phi;
  auto psi_rate = 9.81 * tan(phi) / u + w * phi_rate / (u * cos(phi));

  // Output of f(state, control)
  auto x_dot = ned_rate(0);
  auto y_dot = ned_rate(1);
  auto z_dot = ned_rate(2);
  auto phi_dot = phi_rate;
  auto theta_dot = 0.0;
  auto psi_dot = psi_rate;
  auto u_dot = (L * sin(alpha) - D * cos(alpha)) / params.m - w * psi_rate * sin(phi);
  auto v_dot = 0.0;
  auto w_dot = (-L * cos(alpha) - D * sin(alpha)) / params.m + 9.81 * cos(phi) + u * psi_rate * sin(phi);

  std::vector<casadi::MX> state_dot = { x_dot, y_dot, z_dot, phi_dot, theta_dot, psi_dot, u_dot, v_dot, w_dot };
  return state_dot;
}

}  // namespace parafoil_4dof_model
}  // namespace crs_models
