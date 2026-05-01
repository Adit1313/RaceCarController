#include "pacejka_tracking_mpc/solvers/acados_pacejka_tracking_mpc_solver.h"

#include <array>

namespace crs_controls::pacejka_tracking_mpc::solvers
{
AcadosPacejkaTrackingMpcSolver::AcadosPacejkaTrackingMpcSolver()
{
  // initialize acados solver
  acados_ocp_capsule_.reset(pacejka_model_tracking_mpc_acados_create_capsule());
  pacejka_model_tracking_mpc_acados_create(acados_ocp_capsule_.get());

  nlp_config_.reset(pacejka_model_tracking_mpc_acados_get_nlp_config(acados_ocp_capsule_.get()));
  nlp_dims_.reset(pacejka_model_tracking_mpc_acados_get_nlp_dims(acados_ocp_capsule_.get()));
  nlp_in_.reset(pacejka_model_tracking_mpc_acados_get_nlp_in(acados_ocp_capsule_.get()));
  nlp_out_.reset(pacejka_model_tracking_mpc_acados_get_nlp_out(acados_ocp_capsule_.get()));
  nlp_solver_.reset(pacejka_model_tracking_mpc_acados_get_nlp_solver(acados_ocp_capsule_.get()));
}

void AcadosPacejkaTrackingMpcSolver::setInitialState(AcadosPacejkaTrackingMpcSolver::StateArray& initial_state)
{
  ocp_nlp_constraints_model_set(nlp_config_.get(), nlp_dims_.get(), nlp_in_.get(), nlp_out_.get(), 0, "lbx", initial_state.data());
  ocp_nlp_constraints_model_set(nlp_config_.get(), nlp_dims_.get(), nlp_in_.get(), nlp_out_.get(), 0, "ubx", initial_state.data());
}

void AcadosPacejkaTrackingMpcSolver::setStateInitialGuess(int stage,
                                                          AcadosPacejkaTrackingMpcSolver::StateArray& initial_guess)
{
  ocp_nlp_out_set(nlp_config_.get(), nlp_dims_.get(), nlp_out_.get(), nlp_in_.get(), stage, "x", initial_guess.data());
}

void AcadosPacejkaTrackingMpcSolver::setInputInitialGuess(int stage,
                                                          AcadosPacejkaTrackingMpcSolver::InputArray& initial_guess)
{
  ocp_nlp_out_set(nlp_config_.get(), nlp_dims_.get(), nlp_out_.get(), nlp_in_.get(), stage, "u", initial_guess.data());
}

void AcadosPacejkaTrackingMpcSolver::updateParams(int stage,
                                                  AcadosPacejkaTrackingMpcSolver::MpcStageParameters& parameters)
{
  std::array<double, NP> mpc_parameters;

  // Tracking
  mpc_parameters[params::X_LIN] = parameters.auxillary_param.x;
  mpc_parameters[params::Y_LIN] = parameters.auxillary_param.y;

  // Costs
  mpc_parameters[params::Q1] = parameters.cost_param.Q1;
  mpc_parameters[params::Q2] = parameters.cost_param.Q2;
  mpc_parameters[params::R1] = parameters.cost_param.R1;
  mpc_parameters[params::R2] = parameters.cost_param.R2;

  // Dynamics
  mpc_parameters[params::L_REAR] = parameters.dynamics_param.lr;
  mpc_parameters[params::L_FRONT] = parameters.dynamics_param.lf;
  mpc_parameters[params::M] = parameters.dynamics_param.m;
  mpc_parameters[params::I] = parameters.dynamics_param.I;
  mpc_parameters[params::DF] = parameters.dynamics_param.Df;
  mpc_parameters[params::CF] = parameters.dynamics_param.Cf;
  mpc_parameters[params::BF] = parameters.dynamics_param.Bf;
  mpc_parameters[params::DR] = parameters.dynamics_param.Dr;
  mpc_parameters[params::CR] = parameters.dynamics_param.Cr;
  mpc_parameters[params::BR] = parameters.dynamics_param.Br;
  mpc_parameters[params::CM1] = parameters.dynamics_param.Cm1;
  mpc_parameters[params::CM2] = parameters.dynamics_param.Cm2;
  mpc_parameters[params::CD0] = parameters.dynamics_param.Cd0;
  mpc_parameters[params::CD1] = parameters.dynamics_param.Cd1;
  mpc_parameters[params::CD2] = parameters.dynamics_param.Cd2;

  mpc_parameters[params::GAMMA] = parameters.dynamics_param.gamma;
  mpc_parameters[params::EPS] = parameters.dynamics_param.eps;

  pacejka_model_tracking_mpc_acados_update_params(acados_ocp_capsule_.get(), stage, mpc_parameters.data(), NP);
}

AcadosPacejkaTrackingMpcSolver::MpcReturnType AcadosPacejkaTrackingMpcSolver::solve()
{
  using MpcExitCode = crs_controls::control_commons::MpcExitCode;

  int status = pacejka_model_tracking_mpc_acados_solve(acados_ocp_capsule_.get());
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

double AcadosPacejkaTrackingMpcSolver::getSamplePeriod()
{
  return *(nlp_in_->Ts);
}

AcadosPacejkaTrackingMpcSolver::MpcSolution AcadosPacejkaTrackingMpcSolver::getSolution()
{
  AcadosPacejkaTrackingMpcSolver::MpcSolution solution;
  for (int n = 0; n < N; n++)
  {
    ocp_nlp_out_get(nlp_config_.get(), nlp_dims_.get(), nlp_out_.get(), n, "x", solution.x[n].data());
    ocp_nlp_out_get(nlp_config_.get(), nlp_dims_.get(), nlp_out_.get(), n, "u", solution.u[n].data());
  }
  ocp_nlp_out_get(nlp_config_.get(), nlp_dims_.get(), nlp_out_.get(), N, "x", solution.x[N].data());
  return solution;
}

}  // namespace crs_controls::pacejka_tracking_mpc::solvers
