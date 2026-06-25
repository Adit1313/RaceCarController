#pragma once

#include "mpc_solvers/pacejka_mpcc_solver.h"
#include <pacejka_model/pacejka_params.h>

#include "FORCESNLPsolver_memory.h"
#include <memory>

#ifdef _cplusplus
extern "C" {
#endif
#include "FORCESNLPsolver.h"
#ifdef _cplusplus
}
#endif

namespace crs_controls::pacejka_mpcc::solvers::forces_solver
{

// TODO: where can we load these?
struct ForcesPacejkMpccSolverDims
{
  static constexpr int NX = 9;
  static constexpr int NU = 3;
  static constexpr int N = 40;
  static constexpr int NP = 26;
};

class ForcesPacejkaMpccSolver
  : public crs_controls::mpc_solvers::commons::MpcSolver<
        ForcesPacejkaMpccSolver, ForcesPacejkMpccSolverDims, crs_models::pacejka_model::pacejka_params,
        crs_controls::mpc_solvers::commons::pacejka_mpcc::TrackingCosts,
        crs_controls::mpc_solvers::commons::pacejka_mpcc ::TrajectoryTrackPoint>

{
public:
  // TODO(@naefjo): This is kinda ugly. Can we get these dimensions from the template list?
  static constexpr int NX = ForcesPacejkMpccSolverDims::NX;
  static constexpr int NU = ForcesPacejkMpccSolverDims::NU;
  static constexpr int N = ForcesPacejkMpccSolverDims::N;
  static constexpr int NP = ForcesPacejkMpccSolverDims::NP;

  ForcesPacejkaMpccSolver();

  /**
   * @brief Set the Initial State Constraint. The provided array must have the same length as the state dimension
   *
   * @param init_state
   */
  void setInitialState(ForcesPacejkaMpccSolver::StateArray& init_state);
  /**
   * @brief Sets an initial guess for the state at stage "stage" of the solver.
   * The provided array must have the same length as the state dimension
   *
   * @param init_state
   */
  void setStateInitialGuess(int stage, ForcesPacejkaMpccSolver::StateArray& init_state);
  /**
   * @brief Sets an initial guess for the input at stage "stage" of the solver.
   * The provided array must have the same length as the input dimension
   *
   * @param init_input
   */
  void setInputInitialGuess(int stage, ForcesPacejkaMpccSolver::InputArray& init_input);
  /**
   * @brief Updates the internal params for stage "stage"
   *
   * @param stage which stage to update the parameter [0,HorizonLength)
   * @param model_dynamics  the model dynamics
   * @param costs  the cost parameters
   * @param tracking_point tracking point
   */
  void updateParams(int stage, ForcesPacejkaMpccSolver::MpcStageParameters& params) override;

  /**
   * @brief Solves the optimization problems and stores the solution in x and u.
   *
   * @param x State array or point with size N*StateDimenstion
   * @param u Input array or point with size N*Inputdimension
   * @return const int, return code. If no error occurred, return code is zero
   */
  MpcReturnType solve();

  /**
   * @brief Returns the sample period in seconds
   *
   * @return double
   */
  double getSamplePeriod();

private:
  // TODO, create interface to solver here

  FORCESNLPsolver_params params_;

  /* handle to the solver memory */
  FORCESNLPsolver_mem* mem_handle;
};

}  // namespace crs_controls::pacejka_mpcc::solvers::forces_solver
