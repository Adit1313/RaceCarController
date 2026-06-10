#pragma once

#include <array>

#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados_solver_pacejka_curvilinear_model.h"

#include "pacejka_model/pacejka_params.h"
#include "control_commons/mpc_solver.h"

#include "pacejka_mpcc/solvers/pacejka_mpcc_structures.h"
#include "pacejka_mpcc_structures.h"

namespace crs_controls::pacejka_mpcc::solvers::acados_solver
{
struct AcadosPacejkaCurvilinearMpccSolverDims
{
  static constexpr int NX = PACEJKA_CURVILINEAR_MODEL_NX;
  static constexpr int NU = PACEJKA_CURVILINEAR_MODEL_NU;
  static constexpr int N = PACEJKA_CURVILINEAR_MODEL_N;
  static constexpr int NP = PACEJKA_CURVILINEAR_MODEL_NP;
};

class AcadosPacejkaCurvilinearMpccSolver
  : public crs_controls::control_commons::MpcSolver<
        AcadosPacejkaCurvilinearMpccSolver, AcadosPacejkaCurvilinearMpccSolverDims,
        crs_models::pacejka_model::pacejka_params, crs_controls::pacejka_mpcc::solvers::TrackingCosts,
        crs_controls::pacejka_mpcc::solvers::TrajectoryReferenceCurvilinear>

{
public:
  // TODO(@naefjo): This is kinda ugly. Can we get these dimensions from the template list?
  static constexpr int NX = AcadosPacejkaCurvilinearMpccSolverDims::NX;
  static constexpr int NU = AcadosPacejkaCurvilinearMpccSolverDims::NU;
  static constexpr int N = AcadosPacejkaCurvilinearMpccSolverDims::N;
  static constexpr int NP = AcadosPacejkaCurvilinearMpccSolverDims::NP;
  using vars = crs_controls::pacejka_mpcc::solvers::vars_curvilinear;
  using inputs = crs_controls::pacejka_mpcc::solvers::inputs;

  AcadosPacejkaCurvilinearMpccSolver();

  /**
   * @brief Sets the Initial State Constraint. The provided array must have the same length as the state dimension
   *
   * @param constraint
   */
  void setInitialState(AcadosPacejkaCurvilinearMpccSolver::StateArray& initial_state);

  /**
   * @brief Sets an initial guess for the state at stage "stage" of the solver.
   * The provided array must have the same length as the state dimension
   *
   * @param constraint
   */
  void setStateInitialGuess(int stage, AcadosPacejkaCurvilinearMpccSolver::StateArray& initial_guess);

  /**
   * @brief Sets an initial guess for the input at stage "stage" of the solver.
   * The provided array must have the same length as the input dimension
   *
   * @param constraint
   */
  void setInputInitialGuess(int stage, AcadosPacejkaCurvilinearMpccSolver::InputArray& initial_guess);

  /**
   * @brief Updates the internal params for stage "stage"
   *
   * @param stage which stage to update the parameter [0,HorizonLength)
   * @param model_dynamics  the model dynamics
   * @param costs  the cost parameters
   * @param tracking_point tracking point
   */
  void updateParams(int stage, AcadosPacejkaCurvilinearMpccSolver::MpcStageParameters& params);
  /**
   * @brief Solves the optimization problems and stores the solution in x and u.
   *
   * @param x State array or point with size N*StateDimenstion
   * @param u Input array or point with size N*Inputdimension
   * @return const int, return code. If no error occurred, return code is zero
   */
  MpcReturnType solve();

  double getSamplePeriod();

private:
  std::unique_ptr<pacejka_curvilinear_model_solver_capsule> acados_ocp_capsule_;
  std::unique_ptr<ocp_nlp_config> nlp_config_;
  std::unique_ptr<ocp_nlp_dims> nlp_dims_;
  std::unique_ptr<ocp_nlp_in> nlp_in_;
  std::unique_ptr<ocp_nlp_out> nlp_out_;
  std::unique_ptr<ocp_nlp_solver> nlp_solver_;

  /**
   * @brief Get the Last Solution and stores it in x and u
   *
   * @param x states, size state dimension x horizon length
   * @param u input, size state dimension x horizon length
   */
  AcadosPacejkaCurvilinearMpccSolver::MpcSolution getSolution();
};

}  // namespace crs_controls::pacejka_mpcc::solvers::acados_solver
