#include "kinematic_tracking_mpc/solvers/acados_kinematic_tracking_mpc_solver.h"

namespace crs_controls::kinematic_tracking_mpc::solvers
{
AcadosKinematicTrackingMpcSolver::AcadosKinematicTrackingMpcSolver()
{
  // initialize acados solver
  acados_ocp_capsule_.reset(kinematic_model_tracking_mpc_acados_create_capsule());
  kinematic_model_tracking_mpc_acados_create(acados_ocp_capsule_.get());

  nlp_config_.reset(kinematic_model_tracking_mpc_acados_get_nlp_config(acados_ocp_capsule_.get()));
  nlp_dims_.reset(kinematic_model_tracking_mpc_acados_get_nlp_dims(acados_ocp_capsule_.get()));
  nlp_in_.reset(kinematic_model_tracking_mpc_acados_get_nlp_in(acados_ocp_capsule_.get()));
  nlp_out_.reset(kinematic_model_tracking_mpc_acados_get_nlp_out(acados_ocp_capsule_.get()));
  nlp_solver_.reset(kinematic_model_tracking_mpc_acados_get_nlp_solver(acados_ocp_capsule_.get()));
}

void AcadosKinematicTrackingMpcSolver::setInitialState(AcadosKinematicTrackingMpcSolver::StateArray& initial_state)
{
  ocp_nlp_constraints_model_set(nlp_config_.get(), nlp_dims_.get(), nlp_in_.get(), 0, "lbx", initial_state.data());
  ocp_nlp_constraints_model_set(nlp_config_.get(), nlp_dims_.get(), nlp_in_.get(), 0, "ubx", initial_state.data());
}

void AcadosKinematicTrackingMpcSolver::setStateInitialGuess(int stage,
                                                            AcadosKinematicTrackingMpcSolver::StateArray& guess)
{
  ocp_nlp_out_set(nlp_config_.get(), nlp_dims_.get(), nlp_out_.get(), stage, "x", guess.data());
}

void AcadosKinematicTrackingMpcSolver::setInputInitialGuess(int stage,
                                                            AcadosKinematicTrackingMpcSolver::InputArray& guess)
{
  ocp_nlp_out_set(nlp_config_.get(), nlp_dims_.get(), nlp_out_.get(), stage, "u", guess.data());
}

void AcadosKinematicTrackingMpcSolver::updateParams(
    int stage, const AcadosKinematicTrackingMpcSolver::MpcStageParameters& parameters)
{
  // using params = crs_controls::kinematic_tracking_mpc::solvers::params;

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
  mpc_parameters[params::A] = parameters.dynamics_param.a;
  mpc_parameters[params::B] = parameters.dynamics_param.b;
  mpc_parameters[params::TAU] = parameters.dynamics_param.tau;

  kinematic_model_tracking_mpc_acados_update_params(acados_ocp_capsule_.get(), stage, mpc_parameters.data(), NP);
}

AcadosKinematicTrackingMpcSolver::MpcReturnType AcadosKinematicTrackingMpcSolver::solve()
{
  using MpcExitCode = crs_controls::control_commons::MpcExitCode;

  int status = kinematic_model_tracking_mpc_acados_solve(acados_ocp_capsule_.get());
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

double AcadosKinematicTrackingMpcSolver::getSamplePeriod()
{
  return *(nlp_in_->Ts);
}

AcadosKinematicTrackingMpcSolver::MpcSolution AcadosKinematicTrackingMpcSolver::getSolution()
{
  AcadosKinematicTrackingMpcSolver::MpcSolution solution;
  for (int n = 0; n < N; n++)
  {
    ocp_nlp_out_get(nlp_config_.get(), nlp_dims_.get(), nlp_out_.get(), n, "x", solution.x[n].data());
    ocp_nlp_out_get(nlp_config_.get(), nlp_dims_.get(), nlp_out_.get(), n, "u", solution.u[n].data());
  }
  ocp_nlp_out_get(nlp_config_.get(), nlp_dims_.get(), nlp_out_.get(), N, "x", solution.x[N].data());
  return solution;
}
}  // namespace crs_controls::kinematic_tracking_mpc::solvers
