#ifndef STACKED_MODEL_STACKED_TWO_MODELS_CONTINUOUS_H
#define STACKED_MODEL_STACKED_TWO_MODELS_CONTINUOUS_H

#include <algorithm>
#include <dynamic_models/continuous_dynamic_model.h>
#include <iostream>
#include <variant>

namespace crs_models
{
namespace stacked_model
{

template <typename StateType, typename InputType, typename ModelType1, typename ModelType2>
class ContinuousTwoStackedModels : public ContinuousDynamicModel<StateType, InputType>
{
  typedef typename ContinuousDynamicModel<StateType, InputType>::StateMatrix StateMatrix;
  typedef typename ContinuousDynamicModel<StateType, InputType>::InputMatrix InputMatrix;

public:
  ContinuousTwoStackedModels(std::shared_ptr<ModelType1> model_to_concat1,
                             std::shared_ptr<ModelType2> model_to_concat2)  // Constructor
  {
    // Stack the Q matrix together for the entire system
    StateMatrix Q_temp = StateMatrix::Zero();

    model_to_concat1_ = model_to_concat1;
    model_to_concat2_ = model_to_concat2;

    // Model 1
    Q_temp.block(0, 0, ModelType1::NX, ModelType1::NX) = model_to_concat1_->getQ();

    // Model 2
    Q_temp.block(ModelType1::NX, ModelType1::NX, ModelType2::NX, ModelType2::NX) = model_to_concat2_->getQ();

    this->setQ(Q_temp);
  }

  /**
   * @brief applies Stacked model for a given control input
   *
   */
  StateType applyModel(const StateType state, const InputType control_input)
  {
    StateType state_out;

    // Model 1
    state_out = model_to_concat1_->applyModel(state, control_input);

    // Model 2
    state_out = model_to_concat2_->applyModel(state, control_input);

    return state_out;
  }

  /**
   * @brief Get the Numerical Jacobian for a given state and control input (evaluate symbolic jacobian)
   *
   * @param state current state
   * @param control_input control input
   * @param A the state jacobian df/dx
   * @param B the input jacobian df/du
   */
  void getNumericalJacobian(const StateType& state, const InputType& control_input, StateMatrix& A, InputMatrix& B)
  {
    StateMatrix A_out = StateMatrix::Zero();
    InputMatrix B_out = InputMatrix::Zero();

    // Model 1
    Eigen::Matrix<double, ModelType1::NX, ModelType1::NX> A_temp1;
    Eigen::Matrix<double, ModelType1::NX, ModelType1::NU> B_temp1;

    model_to_concat1_->getNumericalJacobian(state, control_input, A_temp1, B_temp1);
    A_out.block(0, 0, ModelType1::NX, ModelType1::NX) << A_temp1;
    B_out.block(0, 0, ModelType1::NX, ModelType1::NU) << B_temp1;

    // Model 2
    Eigen::Matrix<double, ModelType2::NX, ModelType2::NX> A_temp2;
    Eigen::Matrix<double, ModelType2::NX, ModelType2::NU> B_temp2;
    model_to_concat2_->getNumericalJacobian(state, control_input, A_temp2, B_temp2);

    A_out.block(ModelType1::NX, ModelType1::NX, ModelType2::NX, ModelType2::NX) << A_temp2;
    B_out.block(ModelType1::NX, ModelType1::NU, ModelType2::NX, ModelType2::NU) << B_temp2;
    A << A_out;
    B << B_out;
  }

  /**
   * @brief Returns the casadi function x_dot = f(x,u)
   *
   * @param state the state x
   * @param control_input  the input u
   * @return
   */
  std::vector<casadi::MX> getContinuousDynamics(const std::vector<casadi::MX> state,
                                                const std::vector<casadi::MX> control_input)
  {
    std::vector<casadi::MX> cont_dyn;

    // Model 1
    std::vector<casadi::MX> state_temp1(state.begin(), state.begin() + ModelType1::NX);
    std::vector<casadi::MX> control_input_temp1(control_input.begin(), control_input.begin() + ModelType1::NU);
    std::vector<casadi::MX> cont_dyn_temp1 = model_to_concat1_->getContinuousDynamics(state_temp1, control_input_temp1);
    cont_dyn.insert(cont_dyn.end(), cont_dyn_temp1.begin(), cont_dyn_temp1.end());

    // Model 2
    std::vector<casadi::MX> state_temp2(state.begin() + ModelType1::NX,
                                        state.begin() + ModelType1::NX + ModelType2::NX);
    std::vector<casadi::MX> control_input_temp2(control_input.begin() + ModelType1::NU,
                                                control_input.begin() + ModelType1::NU + ModelType2::NU);
    std::vector<casadi::MX> cont_dyn_temp2 = model_to_concat2_->getContinuousDynamics(state_temp2, control_input_temp2);
    cont_dyn.insert(cont_dyn.end(), cont_dyn_temp2.begin(), cont_dyn_temp2.end());

    return cont_dyn;
  }

private:
  std::shared_ptr<ModelType1> model_to_concat1_;
  std::shared_ptr<ModelType2> model_to_concat2_;
};

}  // namespace stacked_model

}  // namespace crs_models
#endif
