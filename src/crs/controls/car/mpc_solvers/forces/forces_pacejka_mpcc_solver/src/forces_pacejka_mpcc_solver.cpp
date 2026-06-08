#include "forces_pacejka_mpcc_solver/forces_pacejka_mpcc_solver.h"
#include "forces_pacejka_mpcc_solver/FORCESNLPsolver_info.h"

namespace mpc_solvers
{
namespace pacejka_solvers
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
  for (int i = 0; i < (getInputDimension() + getStateDimension()) * getHorizonLength(); i++)
  {
    params_.x0[i] = 0;
  }

  for (int i = 0; i < (np_)*getHorizonLength(); i++)
  {
    params_.all_parameters[i] = 0;
  }
}

/**
 * @brief Get the Horizon Length
 *
 * @return const int
 */
const int ForcesPacejkaMpccSolver::getHorizonLength() const
{
  // TODO, from solver
  return get_data_FORCESNLPsolver().N;
}

/**
 * @brief Set the Initial State Constraint. The provided array must have the same length as the state dimension
 *
 * @param constraint
 */
void ForcesPacejkaMpccSolver::setInitialState(double init_state[])
{
  for (int i = 0; i < getStateDimension(); i++)
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
void ForcesPacejkaMpccSolver::setStateInitialGuess(int stage, double init_state[])
{
  int n_states = getStateDimension();
  int n_inputs = getInputDimension();
  for (int i = 0; i < n_states; i++)
    params_.x0[i + stage * (n_states + n_inputs)] = init_state[i];
}
/**
 * @brief Sets an initial guess for the input at stage "stage" of the solver.
 * The provided array must have the same length as the input dimension
 *
 * @param init_input
 */
void ForcesPacejkaMpccSolver::setInputInitialGuess(int stage, double init_input[])
{
  int n_states = getStateDimension();
  int n_inputs = getInputDimension();
  for (int i = 0; i < n_inputs; i++)
    params_.x0[n_states + i + stage * (n_states + n_inputs)] = init_input[i];
}

/**
 * @brief Updates the internal params for stage "stage"
 *
 * @param stage which stage to update the parameter [0,HorizonLength)
 * @param model_dynamics  the model dynamics
 * @param costs  the cost parameters
 * @param tracking_point tracking point
 */
void ForcesPacejkaMpccSolver::updateParams(int stage, const crs_models::pacejka_model::pacejka_params& model_dynamics,
                                           const tracking_costs& costs, const trajectory_track_point& tracking_point)
{
  // double params[np_];
  params_.all_parameters[np_ * stage + params::X_LIN] = tracking_point.x;
  params_.all_parameters[np_ * stage + params::Y_LIN] = tracking_point.y;
  params_.all_parameters[np_ * stage + params::GRAD_X_LIN] = tracking_point.grad_x;
  params_.all_parameters[np_ * stage + params::GRAD_Y_LIN] = tracking_point.grad_y;
  params_.all_parameters[np_ * stage + params::THETA_PRED] = tracking_point.theta;
  params_.all_parameters[np_ * stage + params::PHI_LIN] = tracking_point.phi;
  // Costs
  params_.all_parameters[np_ * stage + params::Q1] = costs.Q1;
  params_.all_parameters[np_ * stage + params::Q2] = costs.Q2;
  params_.all_parameters[np_ * stage + params::R1] = costs.R1;
  params_.all_parameters[np_ * stage + params::R2] = costs.R2;
  params_.all_parameters[np_ * stage + params::R3] = costs.R3;
  params_.all_parameters[np_ * stage + params::q] = costs.q;
  // Dynamics
  params_.all_parameters[np_ * stage + params::L_REAR] = model_dynamics.lr;
  params_.all_parameters[np_ * stage + params::L_FRONT] = model_dynamics.lf;
  params_.all_parameters[np_ * stage + params::m] = model_dynamics.m;
  params_.all_parameters[np_ * stage + params::I] = model_dynamics.I;
  params_.all_parameters[np_ * stage + params::Df] = model_dynamics.Df;
  params_.all_parameters[np_ * stage + params::Cf] = model_dynamics.Cf;
  params_.all_parameters[np_ * stage + params::Bf] = model_dynamics.Bf;
  params_.all_parameters[np_ * stage + params::Dr] = model_dynamics.Dr;
  params_.all_parameters[np_ * stage + params::Cr] = model_dynamics.Cr;
  params_.all_parameters[np_ * stage + params::Br] = model_dynamics.Br;
  params_.all_parameters[np_ * stage + params::Cm1] = model_dynamics.Cm1;
  params_.all_parameters[np_ * stage + params::Cm2] = model_dynamics.Cm2;
  params_.all_parameters[np_ * stage + params::Cd] = model_dynamics.Cd;
  params_.all_parameters[np_ * stage + params::Croll] = model_dynamics.Croll;
}

/**
 * @brief Solves the optimization problems and stores the solution in x and u.
 *
 * @param x State array or point with size N*StateDimenstion
 * @param u Input array or point with size N*Inputdimension
 * @return const int, return code. If no error occurred, return code is zero
 */
int ForcesPacejkaMpccSolver::solve(double x[], double u[])
{
  int n_horizon = getHorizonLength();
  int n_states = getStateDimension();
  int n_inputs = getInputDimension();

  // solve
  auto exitflag = FORCESNLPsolver_solve(&params_, &output_, &info_, mem_handle, stdout, &FORCESNLPsolver_adtool2forces);

  for (int stage = 0; stage < n_horizon; stage++)
  {
    // Copy outputs to arrays

    for (int i = 0; i < n_states; i++)
      x[i + stage * n_states] = output_.horizon[i + stage * (n_states + n_inputs)];

    for (int i = 0; i < n_inputs; i++)
      u[i + stage * n_inputs] = output_.horizon[n_states + i + stage * (n_states + n_inputs)];
  }

  return exitflag;
}

double ForcesPacejkaMpccSolver::getSamplePeriod()
{
  // TODO
  return get_data_FORCESNLPsolver().sampling_time;
}
};  // namespace pacejka_solvers

}  // namespace mpc_solvers
