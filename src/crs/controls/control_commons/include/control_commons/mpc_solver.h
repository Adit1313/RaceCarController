#ifndef MPC_SOLVERS_MPC_SOLVER_H
#define MPC_SOLVERS_MPC_SOLVER_H

#include <array>
#include <optional>

#include "mpc_solver_structures.h"

namespace crs_controls::control_commons
{
/**
 * @brief Base class for MPC solvers.
 *
 * This class implements a "template" outline of the mpc controllers but instead of using run time polymorphism, we
 * instead use build-time polymorphism for pure unadulterated _SPEED_. Specifically, all the relevant functions are
 * substituted at compile time (using the cool `static_cast<T*>(this)` calls, avoiding the use of slow vtable lookups.
 * This is achieved using CRTP using the template parameter `T`. Here, T is the inheriting specialization of the
 * specific solvers.
 */
template <typename T, typename MpcSolverDimensions, typename DynamicsParameterType, typename CostParameterType,
          typename AuxillaryParameterType>
class MpcSolver
{
public:
  using StateArray = std::array<double, MpcSolverDimensions::NX>;
  using InputArray = std::array<double, MpcSolverDimensions::NU>;
  struct MpcSolution
  {
    std::array<StateArray, MpcSolverDimensions::N + 1> x;
    std::array<InputArray, MpcSolverDimensions::N> u;
  };

  struct MpcStageParameters
  {
    DynamicsParameterType dynamics_param;
    CostParameterType cost_param;
    AuxillaryParameterType auxillary_param;
  };

  struct MpcReturnType
  {
    MpcExitCode exit_code;
    std::optional<MpcSolution> solution;
  };

  using MpcParameters = std::array<MpcStageParameters, MpcSolverDimensions::N + 1>;

  using MpcInitialGuess = MpcSolution;

  /**
   * @brief Returns the sample period in seconds
   *
   * @return double
   */
  double getSamplePeriod() const
  {
    return static_cast<T*>(this)->getSamplePeriod();
  }

  /**
   * @brief Solves the optimization problem.
   *
   * @param initial_condition the initial state x0 of the system for which to optimize
   * @param initial_guess initial guesses for x and u
   * @param params the parameters of the optimization problem
   * @return MpcReturnType consisting of the solver exit code and the optimal solution
   */
  MpcReturnType solve_problem(StateArray& initial_condition, MpcInitialGuess& initial_guess, MpcParameters& params)
  {
    T* derived_solver = static_cast<T*>(this);
    derived_solver->setInitialState(initial_condition);

    for (unsigned int stage = 0; stage < MpcSolverDimensions::N; ++stage)
    {
      derived_solver->setStateInitialGuess(stage, initial_guess.x[stage]);
      derived_solver->updateParams(stage, params[stage]);
      derived_solver->setInputInitialGuess(stage, initial_guess.u[stage]);
    }
    derived_solver->setStateInitialGuess(MpcSolverDimensions::N, initial_guess.x[MpcSolverDimensions::N]);
    derived_solver->updateParams(MpcSolverDimensions::N, params[MpcSolverDimensions::N]);

    return derived_solver->solve();
  }
};
}  // namespace crs_controls::control_commons
#endif  // MPC_SOLVERS_MPC_SOLVER_H
