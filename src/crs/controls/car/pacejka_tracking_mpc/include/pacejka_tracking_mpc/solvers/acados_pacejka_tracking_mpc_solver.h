#pragma once

#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados_solver_pacejka_model_tracking_mpc.h"

#include "pacejka_model/pacejka_params.h"
#include "control_commons/mpc_solver.h"

#include "pacejka_tracking_mpc/solvers/pacejka_tracking_mpc_structures.h"

namespace crs_controls::pacejka_tracking_mpc::solvers
{
struct AcadosPacejkaTrackingMpcSolverDims
{
  static constexpr int NX = PACEJKA_MODEL_TRACKING_MPC_NX;
  static constexpr int NU = PACEJKA_MODEL_TRACKING_MPC_NU;
  static constexpr int N = PACEJKA_MODEL_TRACKING_MPC_N;
  static constexpr int NP = PACEJKA_MODEL_TRACKING_MPC_NP;
};

class AcadosPacejkaTrackingMpcSolver
  : public crs_controls::control_commons::MpcSolver<AcadosPacejkaTrackingMpcSolver, AcadosPacejkaTrackingMpcSolverDims,
                                                    crs_models::pacejka_model::pacejka_params, TrackingCosts,
                                                    TrajectoryTrackPoint>
{
public:
  // TODO(@naefjo): This is kinda ugly. Can we get these dimensions from the template list?
  static constexpr int NX = AcadosPacejkaTrackingMpcSolverDims::NX;
  static constexpr int NU = AcadosPacejkaTrackingMpcSolverDims::NU;
  static constexpr int N = AcadosPacejkaTrackingMpcSolverDims::N;
  static constexpr int NP = AcadosPacejkaTrackingMpcSolverDims::NP;

  AcadosPacejkaTrackingMpcSolver();

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

private:
  std::unique_ptr<pacejka_model_tracking_mpc_solver_capsule> acados_ocp_capsule_;
  std::unique_ptr<ocp_nlp_config> nlp_config_;
  std::unique_ptr<ocp_nlp_dims> nlp_dims_;
  std::unique_ptr<ocp_nlp_in> nlp_in_;
  std::unique_ptr<ocp_nlp_out> nlp_out_;
  std::unique_ptr<ocp_nlp_solver> nlp_solver_;

  /**
   * @brief Get the Last Solution
   *
   * @return MpcSolution containing the predicted x and u
   */
  MpcSolution getSolution();
};

}  // namespace crs_controls::pacejka_tracking_mpc::solvers
