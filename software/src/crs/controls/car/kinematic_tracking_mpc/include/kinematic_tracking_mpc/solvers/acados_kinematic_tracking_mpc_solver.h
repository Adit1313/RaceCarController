#pragma once
#include <array>

#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados_solver_kinematic_model_tracking_mpc.h"

#include "kinematic_model/kinematic_params.h"
#include "control_commons/mpc_solver.h"

#include "kinematic_tracking_mpc/solvers/kinematic_tracking_mpc_structures.h"

namespace crs_controls::kinematic_tracking_mpc::solvers
{

struct AcadosKinematicTrackingMpcSolverDims
{
  static constexpr int NX = KINEMATIC_MODEL_TRACKING_MPC_NX;
  static constexpr int NU = KINEMATIC_MODEL_TRACKING_MPC_NU;
  static constexpr int N = KINEMATIC_MODEL_TRACKING_MPC_N;
  static constexpr int NP = KINEMATIC_MODEL_TRACKING_MPC_NP;
};

class AcadosKinematicTrackingMpcSolver
  : public crs_controls::control_commons::MpcSolver<
        AcadosKinematicTrackingMpcSolver, AcadosKinematicTrackingMpcSolverDims,
        crs_models::kinematic_model::kinematic_params, crs_controls::kinematic_tracking_mpc::solvers::TrackingCosts,
        crs_controls::kinematic_tracking_mpc::solvers::TrajectoryTrackPoint>
{
public:
  static constexpr int NX = AcadosKinematicTrackingMpcSolverDims::NX;
  static constexpr int NU = AcadosKinematicTrackingMpcSolverDims::NU;
  static constexpr int N = AcadosKinematicTrackingMpcSolverDims::N;
  static constexpr int NP = AcadosKinematicTrackingMpcSolverDims::NP;

  AcadosKinematicTrackingMpcSolver();

  /**
   * @brief Set the Initial State Constraint. The provided array must have the same length as the state dimension
   *
   * @param initial_state
   */
  void setInitialState(AcadosKinematicTrackingMpcSolver::StateArray& initial_state);
  /**
   * @brief Sets an initial guess for the state at stage "stage" of the solver.
   * The provided array must have the same length as the state dimension
   *
   * @param guess state guess for the specific stage
   */
  void setStateInitialGuess(int stage, StateArray& guess);
  /**
   * @brief Sets an initial guess for the input at stage "stage" of the solver.
   * The provided array must have the same length as the input dimension
   *
   * @param guess input guess for the specific stage
   */
  void setInputInitialGuess(int stage, InputArray& guess);
  /**
   * @brief Updates the internal params for stage "stage"
   *
   * @param stage which stage to update the parameter [0,HorizonLength)
   * @param paramters the combined stage parameter struct
   */
  void updateParams(int stage, const MpcStageParameters& parameters);

  /**
   * @brief Solves the optimization problems and stores the solution in x and u.
   *
   * @return MpcReturnType containing exit code and solution if successful.
   */
  MpcReturnType solve();

  /**
   * @brief Returns the sample period in seconds
   *
   * @return double
   */
  double getSamplePeriod();

private:
  std::unique_ptr<kinematic_model_tracking_mpc_solver_capsule> acados_ocp_capsule_;
  std::unique_ptr<ocp_nlp_config> nlp_config_;
  std::unique_ptr<ocp_nlp_dims> nlp_dims_;
  std::unique_ptr<ocp_nlp_in> nlp_in_;
  std::unique_ptr<ocp_nlp_out> nlp_out_;
  std::unique_ptr<ocp_nlp_solver> nlp_solver_;

  /**
   * @brief Get the Last Solution
   *
   */
  MpcSolution getSolution();
};

}  // namespace crs_controls::kinematic_tracking_mpc::solvers
