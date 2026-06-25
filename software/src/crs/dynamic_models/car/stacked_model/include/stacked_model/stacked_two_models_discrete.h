#ifndef SRC_CRS_DYNAMIC_MODELS_STACKED_MODEL_STACKED_TWO_MODELS_DISCRETE
#define SRC_CRS_DYNAMIC_MODELS_STACKED_MODEL_STACKED_TWO_MODELS_DISCRETE

#include <algorithm>
#include <dynamic_models/continuous_dynamic_model.h>
#include <dynamic_models/discrete_dynamic_model_wrapper.h>
#include <stacked_model/stacked_two_models_continuous.h>
#include <iostream>

namespace crs_models
{
namespace stacked_model
{

template <typename StateType, typename InputType, typename ModelType1, typename ModelType2, typename ModelType1Cont,
          typename ModelType2Cont>
class DiscreteTwoStackedModels : public DiscreteDynamicModelWrapper<StateType, InputType>
{
  typedef typename DiscreteDynamicModel<StateType, InputType>::StateMatrix StateMatrix;
  typedef typename DiscreteDynamicModel<StateType, InputType>::InputMatrix InputMatrix;

private:
  std::shared_ptr<ModelType1> model_to_concat1_;
  std::shared_ptr<ModelType2> model_to_concat2_;

public:
  /**
   * @brief Construct a new Discrete Stacked Model object.
   *
   */
  DiscreteTwoStackedModels(std::shared_ptr<ModelType1> model_to_concat1, std::shared_ptr<ModelType2> model_to_concat2,
                           std::shared_ptr<ModelType1Cont> model_to_concat1_cont,
                           std::shared_ptr<ModelType2Cont> model_to_concat2_cont)
    : DiscreteDynamicModelWrapper<StateType, InputType>(Eigen::Matrix<double, StateType::NX, StateType::NX>::Identity())
  {
    // Stack the Q matrix together for the entire system
    StateMatrix Q_temp = Eigen::Matrix<double, StateType::NX, StateType::NX>::Zero();

    model_to_concat1_ = model_to_concat1;
    model_to_concat2_ = model_to_concat2;

    // Model 1
    Q_temp.block(0, 0, ModelType1Cont::NX, ModelType1Cont::NX) = model_to_concat1_->getQ();
    // Model 2
    Q_temp.block(ModelType1Cont::NX, ModelType1Cont::NX, ModelType2Cont::NX, ModelType2Cont::NX) =
        model_to_concat2_->getQ();

    this->setQ(Q_temp);

    this->cont_model = std::make_unique<
        crs_models::stacked_model::ContinuousTwoStackedModels<StateType, InputType, ModelType1Cont, ModelType2Cont>>(
        model_to_concat1_cont, model_to_concat2_cont);
  }

  /**
   * @brief integrates the state for a given integration_time
   *
   * @param state
   * @param control_input
   * @param integration_time
   * @return returns the integrated state
   */
  StateType applyModel(const StateType state, const InputType control_input, double integration_time)
  {
    StateType state_out;
    // Model 1.
    // Requires the "=" operator to be overloaded
    state_out = model_to_concat1_->applyModel(state, control_input, integration_time);
    // Model 2
    // Requires the "=" operator to be overloaded
    state_out = model_to_concat2_->applyModel(state, control_input, integration_time);

    return state_out;
  }
};

}  // namespace stacked_model

}  // namespace crs_models
#endif /* SRC_CRS_DYNAMIC_MODELS_STACKED_MODEL_DISCRETE */
