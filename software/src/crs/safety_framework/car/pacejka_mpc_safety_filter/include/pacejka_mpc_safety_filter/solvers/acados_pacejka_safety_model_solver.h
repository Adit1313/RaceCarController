#pragma once

#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados_solver_safety_dynamic_model.h"

#include "pacejka_model/pacejka_params.h"
#include "control_commons/mpc_solver.h"

#include "pacejka_mpc_safety_filter/solvers/pacejka_safety_structures.h"
#include "pacejka_safety_structures.h"

namespace crs_safety::pacejka_mpc_safety_filter::solvers
{

struct AcadosPacejkaSafetySolverDims
{
  static constexpr int NX = SAFETY_DYNAMIC_MODEL_NX;
  static constexpr int NU = SAFETY_DYNAMIC_MODEL_NU;
  static constexpr int N = SAFETY_DYNAMIC_MODEL_N;
  static constexpr int NP = SAFETY_DYNAMIC_MODEL_NP;
};

class AcadosPacejkaSafetySolver
  : public crs_controls::control_commons::MpcSolver<AcadosPacejkaSafetySolver, AcadosPacejkaSafetySolverDims,
                                                    crs_models::pacejka_model::pacejka_params, ReferenceInput,
                                                    ReferenceOnTrack>
{
public:
  // TODO(@naefjo): This is kinda ugly. Can we get these dimensions from the template list?
  static constexpr int NX = AcadosPacejkaSafetySolverDims::NX;
  static constexpr int NU = AcadosPacejkaSafetySolverDims::NU;
  static constexpr int N = AcadosPacejkaSafetySolverDims::N;
  static constexpr int NP = AcadosPacejkaSafetySolverDims::NP;

public:
  AcadosPacejkaSafetySolver();

  /**
   * @brief Sets the Initial State Constraint.
   *
   * @param initial_state
   */
  void setInitialState(StateArray& initial_state);

  /**
   * @brief Sets an initial guess for the state at stage "stage" of the solver.
   * The provided array must have the same length as the state dimension
   *
   * @param stage the stage at which to set the initial guess
   * @param initial_guess the guess for x at the relevant stage
   */
  void setStateInitialGuess(int stage, StateArray& initial_guess);

  /**
   * @brief Sets an initial guess for the input at stage "stage" of the solver.
   * The provided array must have the same length as the input dimension
   *
   * @param stage the stage at which to set the initial guess
   * @param initial_guess the guess for u at the relevant stage
   */
  void setInputInitialGuess(int stage, InputArray& initial_guess);

  /**
   * @brief Updates the internal params for stage "stage"
   *
   * @param stage which stage to update the parameter [0,HorizonLength)
   * @param params the parameter struct
   */
  void updateParams(int stage, MpcStageParameters& params);
  /**
   * @brief Solves the optimization problems and stores the solution in x and u.
   *
   * @return MpcReturnType containing exit code and the predicted state and input trajectories
   */
  MpcReturnType solve();

  /**
   * @brief get the sample period (Ts) of the solver
   *
   * (shocking i know)
   */
  double getSamplePeriod();

  /**
   * @brief Return the amount of time in seconds it took to solve the optimization problem
   *
   * @return double
   */
  double getSolveTime();

private:
  MpcSolution getSolution();

  std::unique_ptr<safety_dynamic_model_solver_capsule> acados_ocp_capsule_;
  std::unique_ptr<ocp_nlp_config> nlp_config_;
  std::unique_ptr<ocp_nlp_dims> nlp_dims_;
  std::unique_ptr<ocp_nlp_in> nlp_in_;
  std::unique_ptr<ocp_nlp_out> nlp_out_;
  std::unique_ptr<ocp_nlp_solver> nlp_solver_;

  unsigned int solve_cnt_ = 0;
  double avg_elapsed_total_ = 0.0;
  double avg_elapsed_ = 0.0;
};

}  // namespace crs_safety::pacejka_mpc_safety_filter::solvers
