#ifndef SRC_CRS_SAFETY_FRAMEWORK_COMMON_INCLUDE_SAFETY_FRAMEWORK_MPC_BASED_SAFETY_FILTER
#define SRC_CRS_SAFETY_FRAMEWORK_COMMON_INCLUDE_SAFETY_FRAMEWORK_MPC_BASED_SAFETY_FILTER

#include "control_commons/mpc_solver.h"
#include "safety_framework/model_based_safety_filter.h"

namespace crs_safety
{
template <typename StateType, typename InputType, typename Model, typename MpcSolverType>
class MpcBasedSafetyFilter : public ModelBasedSafetyFilter<StateType, InputType, Model>
{
protected:
  MpcSolverType solver_;

public:
  using MpcSolution = typename MpcSolverType::MpcSolution;
  using MpcInitialGuess = typename MpcSolverType::MpcInitialGuess;
  using StateArray = typename MpcSolverType::StateArray;
  using MpcParameters = typename MpcSolverType::MpcParameters;

  MpcBasedSafetyFilter(std::shared_ptr<Model> model) : ModelBasedSafetyFilter<StateType, InputType, Model>(model){};

  virtual InputType getSafeControlInput(const StateType state, const InputType control_input) = 0;
};
}  // namespace crs_safety
#endif /* SRC_CRS_SAFETY_FRAMEWORK_COMMON_INCLUDE_SAFETY_FRAMEWORK_MPC_BASED_SAFETY_FILTER */
