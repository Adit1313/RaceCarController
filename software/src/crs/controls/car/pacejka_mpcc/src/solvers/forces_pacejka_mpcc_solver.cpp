#include "forces_pacejka_mpcc_solver/forces_pacejka_mpcc_solver.h"

#include "forces_pacejka_mpcc_solver/FORCESNLPsolver_info.h"

namespace crs_controls::mpc_controller::pacejka_controller::solvers::forces_solver
{
#ifdef __cplusplus
extern "C" {
#endif

/* AD tool to FORCESPRO interface */
extern solver_int32_default
FORCESNLPsolver_adtool2forces(FORCESNLPsolver_float* x,       /* primal vars                                         */
                              FORCESNLPsolver_float* y,       /* eq. constraint multiplers                           */
                              FORCESNLPsolver_float* l,       /* ineq. constraint multipliers                        */
                              FORCESNLPsolver_float* p,       /* parameters                                          */
                              FORCESNLPsolver_float* f,       /* objective function (scalar)                         */
                              FORCESNLPsolver_float* nabla_f, /* gradient of objective function                      */
                              FORCESNLPsolver_float* c,       /* dynamics                                            */
                              FORCESNLPsolver_float* nabla_c, /* Jacobian of the dynamics (column major)             */
                              FORCESNLPsolver_float* h,       /* inequality constraints                              */
                              FORCESNLPsolver_float* nabla_h, /* Jacobian of inequality constraints (column major)   */
                              FORCESNLPsolver_float* hess,    /* Hessian (column major)                              */
                              solver_int32_default stage,     /* stage number (0 indexed)                           */
                              solver_int32_default iteration, /* iteration number of solver                         */
                              solver_int32_default threadID /* Id of caller thread                                */);

#ifdef __cplusplus
}
#endif

/**
 * @brief Construct a new Forces Pacejka Mpcc Solver:: Forces Pacejka Mpcc Solver object
 *
 */
ForcesPacejkaMpccSolver::ForcesPacejkaMpccSolver()
{
  mem_handle = FORCESNLPsolver_internal_mem(0);
  for (int i = 0; i < (NU + NX) * N; i++)
  {
    params_.x0[i] = 0;
  }

  for (int i = 0; i < NP * N; i++)
  {
    params_.all_parameters[i] = 0;
  }
}

/**
 * @brief Set the Initial State Constraint. The provided array must have the same length as the state dimension
 *
 * @param constraint
 */
void ForcesPacejkaMpccSolver::setInitialState(ForcesPacejkaMpccSolver::StateArray& init_state)
{
  for (int i = 0; i < NX; i++)
  {
    params_.xinit[i] = init_state[i];
    params_.x0[i] = init_state[i];
  }
}
/**
 * @brief Sets an initial guess for the state at stage "stage" of the solver.
 * The provided array must have the same length as the state dimension
 *
 * @param init_state
 */
void ForcesPacejkaMpccSolver::setStateInitialGuess(int stage, ForcesPacejkaMpccSolver::StateArray& init_state)
{
  for (int i = 0; i < NX; i++)
    params_.x0[i + stage * (NX + NU)] = init_state[i];
}
/**
 * @brief Sets an initial guess for the input at stage "stage" of the solver.
 * The provided array must have the same length as the input dimension
 *
 * @param init_input
 */
void ForcesPacejkaMpccSolver::setInputInitialGuess(int stage, ForcesPacejkaMpccSolver::InputArray& init_input)
{
  for (int i = 0; i < NU; i++)
    params_.x0[NX + i + stage * (NX + NU)] = init_input[i];
}

/**
 * @brief Updates the internal params for stage "stage"
 *
 * @param stage which stage to update the parameter [0,HorizonLength)
 * @param model_dynamics  the model dynamics
 * @param costs  the cost parameters
 * @param tracking_point tracking point
 */
void ForcesPacejkaMpccSolver::updateParams(int stage, ForcesPacejkaMpccSolver::MpcStageParameters& parameters)
{
  using params = crs_controls::mpc_solvers::commons::pacejka_mpcc::params;

  params_.all_parameters[NP * stage + params::X_LIN] = parameters.auxillary_param.x;
  params_.all_parameters[NP * stage + params::Y_LIN] = parameters.auxillary_param.y;
  params_.all_parameters[NP * stage + params::GRAD_X_LIN] = parameters.auxillary_param.grad_x;
  params_.all_parameters[NP * stage + params::GRAD_Y_LIN] = parameters.auxillary_param.grad_y;
  params_.all_parameters[NP * stage + params::THETA_PRED] = parameters.auxillary_param.theta;
  params_.all_parameters[NP * stage + params::PHI_LIN] = parameters.auxillary_param.phi;
  // Costs
  params_.all_parameters[NP * stage + params::Q1] = parameters.cost_param.Q1;
  params_.all_parameters[NP * stage + params::Q2] = parameters.cost_param.Q2;
  params_.all_parameters[NP * stage + params::R1] = parameters.cost_param.R1;
  params_.all_parameters[NP * stage + params::R2] = parameters.cost_param.R2;
  params_.all_parameters[NP * stage + params::R3] = parameters.cost_param.R3;
  params_.all_parameters[NP * stage + params::q] = parameters.cost_param.q;
  // Dynamics
  params_.all_parameters[NP * stage + params::L_REAR] = parameters.dynamics_param.lr;
  params_.all_parameters[NP * stage + params::L_FRONT] = parameters.dynamics_param.lf;
  params_.all_parameters[NP * stage + params::m] = parameters.dynamics_param.m;
  params_.all_parameters[NP * stage + params::I] = parameters.dynamics_param.I;
  params_.all_parameters[NP * stage + params::Df] = parameters.dynamics_param.Df;
  params_.all_parameters[NP * stage + params::Cf] = parameters.dynamics_param.Cf;
  params_.all_parameters[NP * stage + params::Bf] = parameters.dynamics_param.Bf;
  params_.all_parameters[NP * stage + params::Dr] = parameters.dynamics_param.Dr;
  params_.all_parameters[NP * stage + params::Cr] = parameters.dynamics_param.Cr;
  params_.all_parameters[NP * stage + params::Br] = parameters.dynamics_param.Br;
  params_.all_parameters[NP * stage + params::Cm1] = parameters.dynamics_param.Cm1;
  params_.all_parameters[NP * stage + params::Cm2] = parameters.dynamics_param.Cm2;
  params_.all_parameters[NP * stage + params::Cd] = parameters.dynamics_param.Cd;
  params_.all_parameters[NP * stage + params::Croll] = parameters.dynamics_param.Croll;
}

/**
 * @brief Solves the optimization problems and stores the solution in x and u.
 *
 * @param x State array or point with size N*StateDimenstion
 * @param u Input array or point with size N*Inputdimension
 * @return const int, return code. If no error occurred, return code is zero
 */
ForcesPacejkaMpccSolver::MpcReturnType ForcesPacejkaMpccSolver::solve()
{
  using MpcExitCode = crs_controls::mpc_solvers::commons::MpcExitCode;

  // solve
  FORCESNLPsolver_output output;
  FORCESNLPsolver_info info;
  auto exitflag = FORCESNLPsolver_solve(&params_, &output, &info, mem_handle, stdout, &FORCESNLPsolver_adtool2forces);

  MpcSolution solution;
  for (int stage = 0; stage < N + 1; stage++)
  {
    for (int i = 0; i < NX; i++)
      solution.x[stage][i] = output.horizon[i + stage * (NX + NU)];

    if (stage == N)
    {
      break;
    }

    for (int i = 0; i < NU; i++)
      solution.u[stage][i] = output.horizon[NX + i + stage * (NX + NU)];
  }
  MpcExitCode exit_code;
  // TODO: make more fine grained
  switch (exitflag)
  {
    case 1:
      exit_code = MpcExitCode::SUCCEEDED;
      break;
    default:
      exit_code = MpcExitCode::FAILURE;
      break;
  }

  return { exit_code, solution };
}

double ForcesPacejkaMpccSolver::getSamplePeriod()
{
  return get_data_FORCESNLPsolver().sampling_time;
}

}  // namespace crs_controls::mpc_controller::pacejka_controller::solvers::forces_solver
