#include "pacejka_model/pacejka_continuous.h"
#include <dynamic_models/utils/data_conversion.h>

#include <casadi/casadi.hpp>
namespace crs_models
{
namespace pacejka_model
{
ContinuousPacejkaModel::ContinuousPacejkaModel(pacejka_params params) : params(params)  // Constructor
{
  std::vector<casadi::MX> state_mx = { casadi::MX::sym("x"),   casadi::MX::sym("y"),   casadi::MX::sym("yaw"),
                                       casadi::MX::sym("v_x"), casadi::MX::sym("v_y"), casadi::MX::sym("yaw_rate") };
  std::vector<casadi::MX> input_mx = { casadi::MX::sym("torque"), casadi::MX::sym("steer") };

  std::vector<casadi::MX> state_dot_mx = getContinuousDynamics(state_mx, input_mx);
  // append control inputs to state vector (casadi functions only take one input)
  state_mx.push_back(input_mx[0]);
  state_mx.push_back(input_mx[1]);
  dynamics_f_ = casadi::Function("f_pacejka_cont", state_mx, state_dot_mx);
}

/**
 * @brief evaluated state_dot (= f(x,u)) at current state and control input
 *
 * @param state
 * @param input
 * @return pacejka_car_state returns state_dot evaluated at current state and control input
 */
pacejka_car_state ContinuousPacejkaModel::applyModel(const pacejka_car_state state, const pacejka_car_input input)
{
  pacejka_car_state state_dot_numerical;
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
std::vector<casadi::MX> ContinuousPacejkaModel::getContinuousDynamics(const std::vector<casadi::MX> state,
                                                                      const std::vector<casadi::MX> control_input)
{
  using namespace casadi;
  assert(state.size() == 6);
  assert(control_input.size() == 2);

  // Just for better readability, function inputs (states and control inputs)
  auto x = state[0];
  auto y = state[1];
  auto yaw = state[2];
  auto v_x = state[3];  // max(state[3], 0.0001)
  auto v_y = state[4];
  auto yaw_rate = state[5];
  auto torque = control_input[0];
  auto steer = control_input[1];

  // Slip angles
  /*
  Approximation of arctan(w/x) with a polynomial of degree 3
  for the discontinuity at x=0. This is needed to avoid
  numerical problems with the atan function. The approximation
  is applied for x < eps and has the same value and derivative at
  x=eps as the atan function.

  The polynomial is given by:
  g(x) = b*x + b*x*x*x;
  b = 1/2 * w/(eps^2+w^2) + 3/(2*eps)*arctan(w/eps)
  c = -1/(2*eps^3)* arctan(w/eps) - w/(2eps^2) * 1/(w*w + eps*eps)
  */

  auto w_r = (-v_y + params.lr * yaw_rate);
  auto b_r = 0.5 * w_r / (params.eps * params.eps + w_r * w_r) + 3 / (2 * params.eps) * atan(w_r / params.eps);
  auto c_r = -1 / (2 * params.eps * params.eps * params.eps) * atan(w_r / params.eps) -
             w_r / (2 * params.eps * params.eps) * 1 / (w_r * w_r + params.eps * params.eps);
  auto g_r = b_r * v_x + c_r * v_x * v_x * v_x;

  auto w_f = (-v_y - params.lf * yaw_rate);
  auto b_f = 0.5 * w_f / (params.eps * params.eps + w_f * w_f) + 3 / (2 * params.eps) * atan(w_f / params.eps);
  auto c_f = -1 / (2 * params.eps * params.eps * params.eps) * atan(w_f / params.eps) -
             w_f / (2 * params.eps * params.eps) * 1 / (w_f * w_f + params.eps * params.eps);
  auto g_f = b_f * v_x + c_f * v_x * v_x * v_x;

  auto ar = if_else(v_x < params.eps, g_r, atan2(-v_y + params.lr * yaw_rate, v_x));
  auto af = if_else(v_x < params.eps, g_f, steer + atan2(-v_y - params.lf * yaw_rate, v_x));

  // TODO @(sabodmer) this is not so nice. Somehow casadi::sin() does not work but using namespace casadi, sin() works?
  // Intermediate function values

  // Motor force and friction force expansion
  auto Fm = (params.Cm1 - params.Cm2 * v_x) * torque;
  auto Ffriction = sign(v_x) * (-params.Cd0 - params.Cd1 * v_x - params.Cd2 * v_x * v_x);

  // Split the longitudinal force between front and rear wheels.
  // - For all-wheel drive, the force is split (gamma = 0.5)
  // - for rear-wheel drive, it's on the back wheels (gamma = 1)
  auto Fx_f = (1 - params.gamma) * Fm;
  auto Fx_r = params.gamma * Fm;

  auto Fy_f = params.Df * sin(params.Cf * atan(params.Bf * af));
  auto Fy_r = params.Dr * sin(params.Cr * atan(params.Br * ar));

  // Dyamics including apparent forces
  auto Fx = Fx_r + Fx_f * cos(steer) - Fy_f * sin(steer) + params.m * v_y * yaw_rate + Ffriction;
  auto Fy = Fy_r + Fx_f * sin(steer) + Fy_f * cos(steer) - params.m * v_x * yaw_rate;
  auto Mz = Fy_f * params.lf * cos(steer) + Fx_f * params.lf * sin(steer) - Fy_r * params.lr;

  // Output of f(state, control)
  auto x_dot = v_x * cos(yaw) - v_y * sin(yaw);
  auto y_dot = v_x * sin(yaw) + v_y * cos(yaw);
  auto yaw_dot = yaw_rate;
  auto v_x_dot = Fx / params.m;
  auto v_y_dot = Fy / params.m;
  auto yaw_rate_dot = Mz / params.I;

  auto state_dot = { x_dot, y_dot, yaw_dot, v_x_dot, v_y_dot, yaw_rate_dot };
  return state_dot;
}

}  // namespace pacejka_model
}  // namespace crs_models
