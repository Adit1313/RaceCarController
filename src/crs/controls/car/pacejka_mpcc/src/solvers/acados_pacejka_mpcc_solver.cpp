#include "pacejka_mpcc/solvers/acados_pacejka_mpcc_solver.h"

namespace crs_controls::pacejka_mpcc::solvers::acados_solver
{

AcadosPacejkaMpccSolver::AcadosPacejkaMpccSolver()
{
  // initialize acados solver
  acados_ocp_capsule_.reset(pacejka_model_acados_create_capsule());
  pacejka_model_acados_create(acados_ocp_capsule_.get());

  nlp_config_.reset(pacejka_model_acados_get_nlp_config(acados_ocp_capsule_.get()));
  nlp_dims_.reset(pacejka_model_acados_get_nlp_dims(acados_ocp_capsule_.get()));
  nlp_in_.reset(pacejka_model_acados_get_nlp_in(acados_ocp_capsule_.get()));
  nlp_out_.reset(pacejka_model_acados_get_nlp_out(acados_ocp_capsule_.get()));
  nlp_solver_.reset(pacejka_model_acados_get_nlp_solver(acados_ocp_capsule_.get()));
}

/**
 * @brief Set the Initial State Constraint. The provided array must have the same length as the state dimension
 *
 * @param constraint
 */
void AcadosPacejkaMpccSolver::setInitialState(AcadosPacejkaMpccSolver::StateArray& initial_state)
{
  ocp_nlp_constraints_model_set(nlp_config_.get(), nlp_dims_.get(), nlp_in_.get(), 0, "lbx", initial_state.data());
  ocp_nlp_constraints_model_set(nlp_config_.get(), nlp_dims_.get(), nlp_in_.get(), 0, "ubx", initial_state.data());
}

/**
 * @brief Sets an initial guess for the state at stage "stage" of the solver.
 * The provided array must have the same length as the state dimension
 *
 * @param constraint
 */
void AcadosPacejkaMpccSolver::setStateInitialGuess(int stage, AcadosPacejkaMpccSolver::StateArray& initial_guess)
{
  ocp_nlp_out_set(nlp_config_.get(), nlp_dims_.get(), nlp_out_.get(), stage, "x", initial_guess.data());
}

/**
 * @brief Sets an initial guess for the input at stage "stage" of the solver.
 * The provided array must have the same length as the input dimension
 *
 * @param constraint
 */
void AcadosPacejkaMpccSolver::setInputInitialGuess(int stage, AcadosPacejkaMpccSolver::InputArray& initial_guess)
{
  ocp_nlp_out_set(nlp_config_.get(), nlp_dims_.get(), nlp_out_.get(), stage, "u", initial_guess.data());
}

/**
 * @brief Updates the internal params for stage "stage"
 *
 * @param stage which stage to update the parameter [0,HorizonLength)
 * @param model_dynamics  the model dynamics
 * @param costs  the cost parameters
 * @param tracking_point tracking point
 */
void AcadosPacejkaMpccSolver::updateParams(int stage, AcadosPacejkaMpccSolver::MpcStageParameters& parameters)
{
  using params = crs_controls::pacejka_mpcc::solvers::params;

  std::array<double, NP> mpc_parameters;
  // Tracking
  mpc_parameters[static_cast<int>(params::X_LIN)] = parameters.auxillary_param.x;
  mpc_parameters[static_cast<int>(params::Y_LIN)] = parameters.auxillary_param.y;
  mpc_parameters[static_cast<int>(params::GRAD_X_LIN)] = parameters.auxillary_param.grad_x;
  mpc_parameters[static_cast<int>(params::GRAD_Y_LIN)] = parameters.auxillary_param.grad_y;
  mpc_parameters[static_cast<int>(params::THETA_PRED)] = parameters.auxillary_param.theta;
  mpc_parameters[static_cast<int>(params::PHI_LIN)] = parameters.auxillary_param.phi;
  // Costs
  mpc_parameters[static_cast<int>(params::Q1)] = parameters.cost_param.Q1;
  mpc_parameters[static_cast<int>(params::Q2)] = parameters.cost_param.Q2;
  mpc_parameters[static_cast<int>(params::R1)] = parameters.cost_param.R1;
  mpc_parameters[static_cast<int>(params::R2)] = parameters.cost_param.R2;
  mpc_parameters[static_cast<int>(params::R3)] = parameters.cost_param.R3;
  mpc_parameters[static_cast<int>(params::Q)] = parameters.cost_param.q;
  // Dynamics
  mpc_parameters[static_cast<int>(params::L_REAR)] = parameters.dynamics_param.lr;
  mpc_parameters[static_cast<int>(params::L_FRONT)] = parameters.dynamics_param.lf;
  mpc_parameters[static_cast<int>(params::M)] = parameters.dynamics_param.m;
  mpc_parameters[static_cast<int>(params::I)] = parameters.dynamics_param.I;
  mpc_parameters[static_cast<int>(params::DF)] = parameters.dynamics_param.Df;
  mpc_parameters[static_cast<int>(params::CF)] = parameters.dynamics_param.Cf;
  mpc_parameters[static_cast<int>(params::BF)] = parameters.dynamics_param.Bf;
  mpc_parameters[static_cast<int>(params::DR)] = parameters.dynamics_param.Dr;
  mpc_parameters[static_cast<int>(params::CR)] = parameters.dynamics_param.Cr;
  mpc_parameters[static_cast<int>(params::BR)] = parameters.dynamics_param.Br;
  mpc_parameters[static_cast<int>(params::CM1)] = parameters.dynamics_param.Cm1;
  mpc_parameters[static_cast<int>(params::CM2)] = parameters.dynamics_param.Cm2;
  mpc_parameters[static_cast<int>(params::CD0)] = parameters.dynamics_param.Cd0;
  mpc_parameters[static_cast<int>(params::CD1)] = parameters.dynamics_param.Cd1;
  mpc_parameters[static_cast<int>(params::CD2)] = parameters.dynamics_param.Cd2;

  mpc_parameters[static_cast<int>(params::GAMMA)] = parameters.dynamics_param.gamma;
  mpc_parameters[static_cast<int>(params::EPS)] = parameters.dynamics_param.eps;
  mpc_parameters[static_cast<int>(params::CAR_WIDTH)] = parameters.dynamics_param.car_width;
  mpc_parameters[static_cast<int>(params::WHEEL_RADIUS)] = parameters.dynamics_param.wheel_radius;

  pacejka_model_acados_update_params(acados_ocp_capsule_.get(), stage, mpc_parameters.data(), NP);
}

AcadosPacejkaMpccSolver::MpcReturnType AcadosPacejkaMpccSolver::solve()
{
  using MpcExitCode = crs_controls::control_commons::MpcExitCode;

  int status = pacejka_model_acados_solve(acados_ocp_capsule_.get());
  MpcExitCode exit_code;
  switch (status)
  {
    case 0:
      exit_code = MpcExitCode::SUCCEEDED;
      break;
    case 2:
      exit_code = MpcExitCode::SOLVER_TIMEOUT;
      break;
    default:
      exit_code = MpcExitCode::FAILURE;
      break;
  }
  if (exit_code != MpcExitCode::SUCCEEDED)
  {
    return { exit_code, {} };
  }
  return { exit_code, getSolution() };
}

double AcadosPacejkaMpccSolver::getSamplePeriod()
{
  return *(nlp_in_->Ts);
}

AcadosPacejkaMpccSolver::MpcSolution AcadosPacejkaMpccSolver::getSolution()
{
  AcadosPacejkaMpccSolver::MpcSolution solution;
  for (int n = 0; n < N; n++)
  {
    ocp_nlp_out_get(nlp_config_.get(), nlp_dims_.get(), nlp_out_.get(), n, "x", solution.x[n].data());
    ocp_nlp_out_get(nlp_config_.get(), nlp_dims_.get(), nlp_out_.get(), n, "u", solution.u[n].data());
  }
  ocp_nlp_out_get(nlp_config_.get(), nlp_dims_.get(), nlp_out_.get(), N, "x", solution.x[N].data());
  return solution;
}

}  // namespace crs_controls::pacejka_mpcc::solvers::acados_solver
