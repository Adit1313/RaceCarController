#include "imu_bias/imu_bias_continuous.h"
#include <dynamic_models/utils/data_conversion.h>

#include <casadi/casadi.hpp>
namespace crs_models
{
namespace imu_bias
{
ContinuousImuBias::ContinuousImuBias()  // Constructor
{
  std::vector<casadi::MX> state_mx = { casadi::MX::sym("bias_ax"), casadi::MX::sym("bias_ay"),
                                       casadi::MX::sym("bias_dyaw") };
  std::vector<casadi::MX> input_mx = {};

  std::vector<casadi::MX> state_dot_mx = getContinuousDynamics(state_mx, input_mx);
  // append control inputs to state vector (casadi functions only take one input)

  dynamics_f_ = casadi::Function("f_bias_cont", state_mx, state_dot_mx);
}

/**
 * @brief evaluated state_dot (= f(x,u)) at current state and control input
 *
 * @param state
 * @param input
 * @return imu_bias_state returns state_dot evaluated at current state and control input
 */
imu_bias_state ContinuousImuBias::applyModel(const imu_bias_state state, const imu_bias_input input [[maybe_unused]])
{
  imu_bias_state state_dot_numerical;
  // Convert casadi symbols (mx) to casadi function to be evaluated at current state & control input
  dynamics_f_(commons::convertToConstVector(state), commons::convertToVector(state_dot_numerical));
  return state_dot_numerical;
}

/**
 * @brief Get the analytical state equations f(state, input) = state_dot
 *
 * @param state
 * @param control_input
 * @return std::vector<casadi::MX> returns the analytical state equations f(state, input) = state_dot
 */
std::vector<casadi::MX> ContinuousImuBias::getContinuousDynamics(const std::vector<casadi::MX> state [[maybe_unused]],
                                                                 const std::vector<casadi::MX> control_input
                                                                 [[maybe_unused]])
{
  using namespace casadi;

  std::vector<casadi::MX> state_dot = { 0, 0, 0 };  // Constant IMU bias

  return state_dot;
}

}  // namespace imu_bias
}  // namespace crs_models
