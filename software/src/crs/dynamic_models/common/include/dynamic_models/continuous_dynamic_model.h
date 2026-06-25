#ifndef DYNAMIC_MODELS_CONTINUOUS_DYNAMIC_MODEL_H
#define DYNAMIC_MODELS_CONTINUOUS_DYNAMIC_MODEL_H

#include "utils/data_conversion.h"
#include "commons/casadi_utils.h"
#include <Eigen/Core>
#include <casadi/casadi.hpp>
#include <iostream>

namespace crs_models
{
template <typename StateType, typename InputType>
class ContinuousDynamicModel
{
public:
  typedef Eigen::Matrix<double, StateType::NX, StateType::NX> StateMatrix;
  typedef Eigen::Matrix<double, StateType::NX, InputType::NU> InputMatrix;

  constexpr static int NX = StateType::NX;  ///< Number of dynamic model states
  constexpr static int NU = InputType::NU;  ///< Number of dynamic model inputs

  /**
   * @brief Evaluate the model dynamic function at a given state and control input.
   *
   * @param state Current state
   * @param control_input Applied control input
   * @return StateType The state derivative for a continuous model, i.e. x_dot = f(x,u).
   */
  virtual StateType applyModel(const StateType state, const InputType control_input) = 0;

  /**
   * @brief Get the Numerical Jacobian for a given state and control input
   *
   * @param state current state
   * @param control_input control input
   * @param A the state jacobian df/dx
   * @param B the input jacobian df/du
   */
  void getNumericalJacobian(const StateType& state, const InputType& control_input,
                            Eigen::Matrix<double, StateType::NX, StateType::NX>& A,
                            Eigen::Matrix<double, StateType::NX, InputType::NU>& B)
  {
    Eigen::MatrixXd jacobian = dynamics_f_.evaluateJacobian(commons::convertToConstVector(state, control_input));
    A = jacobian.block(0, 0, StateType::NX, StateType::NX);              // df/dx
    B = jacobian.block(0, StateType::NX, StateType::NX, InputType::NU);  // df/du
  }

  /**
   * @brief Get the Symbolic Jacobian as casadi function.
   *
   * Note, this functions requires #States + #Inputs + #States inputs and gives #States x (#Inputs + #States) outputs.
   *
   * Example for a state of dimension 6 (x,y,yaw, vx, vy, dyaw) and input of dimension 2 (u0, u1):
   *        jacobian(x,y,yaw,vx,vy,dyaw,u0,u1, x_d,y_d,yaw_d,vx_d,vy_d,dyaw_d) -> (dx_d/dx, dx_d/dy, dx_d/dyaw ...)
   *        Where x_d, y_d, yaw_d, ... is the output of f(x,y,yaw,vx,vy,dyaw,u0,u1) and can be ignored if the function
   *        is defined explicitely
   *
   * @return casadi::Function
   */
  casadi::Function getSymbolicJacobian()
  {
    return dynamics_f_.getSymbolicJacobian();
  }

  /**
   * @brief Returns the casadi function x_dot = f(x,u)
   *
   * @param state the state x
   * @param control_input  the input u
   * @return vector with dyanmics defined as casadi::MX vector
   */
  virtual std::vector<casadi::MX> getContinuousDynamics(const std::vector<casadi::MX> state,
                                                        const std::vector<casadi::MX> control_input) = 0;

  void setQ(const Eigen::Matrix<double, StateType::NX, StateType::NX>& Q)
  {
    Q_ = Q;
  }

  const Eigen::Matrix<double, StateType::NX, StateType::NX> getQ()
  {
    return Q_;
  }

protected:
  Eigen::Matrix<double, StateType::NX, StateType::NX> Q_;
  commons::CasadiFunction dynamics_f_;  // x_dot = f(x,u)
};
}  // namespace crs_models
#endif
