#include "pacejka_mpc_safety_filter/solvers/acados_pacejka_safety_model_solver.h"

#include <array>

namespace crs_safety::pacejka_mpc_safety_filter::solvers
{

AcadosPacejkaSafetySolver::AcadosPacejkaSafetySolver()
{
  // initialize acados solver
  acados_ocp_capsule_.reset(safety_dynamic_model_acados_create_capsule());
  safety_dynamic_model_acados_create(acados_ocp_capsule_.get());

  nlp_config_.reset(safety_dynamic_model_acados_get_nlp_config(acados_ocp_capsule_.get()));
  nlp_dims_.reset(safety_dynamic_model_acados_get_nlp_dims(acados_ocp_capsule_.get()));
  nlp_in_.reset(safety_dynamic_model_acados_get_nlp_in(acados_ocp_capsule_.get()));
  nlp_out_.reset(safety_dynamic_model_acados_get_nlp_out(acados_ocp_capsule_.get()));
  nlp_solver_.reset(safety_dynamic_model_acados_get_nlp_solver(acados_ocp_capsule_.get()));
}

void AcadosPacejkaSafetySolver::setInitialState(AcadosPacejkaSafetySolver::StateArray& initial_state)
{
  ocp_nlp_constraints_model_set(nlp_config_.get(), nlp_dims_.get(), nlp_in_.get(), 0, "lbx", initial_state.data());
  ocp_nlp_constraints_model_set(nlp_config_.get(), nlp_dims_.get(), nlp_in_.get(), 0, "ubx", initial_state.data());
}

void AcadosPacejkaSafetySolver::setStateInitialGuess(int stage, AcadosPacejkaSafetySolver::StateArray& initial_guess)
{
  ocp_nlp_out_set(nlp_config_.get(), nlp_dims_.get(), nlp_out_.get(), stage, "x", initial_guess.data());
}

void AcadosPacejkaSafetySolver::setInputInitialGuess(int stage, AcadosPacejkaSafetySolver::InputArray& initial_guess)
{
  ocp_nlp_out_set(nlp_config_.get(), nlp_dims_.get(), nlp_out_.get(), stage, "u", initial_guess.data());
}

void AcadosPacejkaSafetySolver::updateParams(int stage, AcadosPacejkaSafetySolver::MpcStageParameters& parameters)
{
  if (stage == 1)
  {
    std::array<double, 4 * 4> W_safety = { 0.0 };
    W_safety[0 * 4] = parameters.cost_param.cost_torque;
    W_safety[1 * 4 + 1] = parameters.cost_param.cost_steer;
    W_safety[2 * 4 + 2] = parameters.cost_param.cost_delta_torque;
    W_safety[3 * 4 + 3] = parameters.cost_param.cost_delta_steer;

    ocp_nlp_cost_model_set(nlp_config_.get(), nlp_dims_.get(), nlp_in_.get(), stage, "W", W_safety.data());
  }
  else if (stage > 1)
  {
    std::array<double, 4 * 4> W_safety = { 0.0 };
    W_safety[2 * 4 + 2] = parameters.cost_param.cost_delta_torque;
    W_safety[3 * 4 + 3] = parameters.cost_param.cost_delta_steer;
    ocp_nlp_cost_model_set(nlp_config_.get(), nlp_dims_.get(), nlp_in_.get(), stage, "W", W_safety.data());
  }

  std::array<double, 4> y_ref = { parameters.cost_param.torque, parameters.cost_param.steer, 0.0, 0.0 };
  ocp_nlp_cost_model_set(nlp_config_.get(), nlp_dims_.get(), nlp_in_.get(), 1, "yref", y_ref.data());

  std::array<double, NP> mpc_params;
  // Dynamics
  mpc_params[parameters::L_REAR] = parameters.dynamics_param.lr;
  mpc_params[parameters::L_FRONT] = parameters.dynamics_param.lf;
  mpc_params[parameters::M] = parameters.dynamics_param.m;
  mpc_params[parameters::I] = parameters.dynamics_param.I;
  mpc_params[parameters::DF] = parameters.dynamics_param.Df;
  mpc_params[parameters::CF] = parameters.dynamics_param.Cf;
  mpc_params[parameters::BF] = parameters.dynamics_param.Bf;
  mpc_params[parameters::DR] = parameters.dynamics_param.Dr;
  mpc_params[parameters::CR] = parameters.dynamics_param.Cr;
  mpc_params[parameters::BR] = parameters.dynamics_param.Br;
  mpc_params[parameters::CM1] = parameters.dynamics_param.Cm1;
  mpc_params[parameters::CM2] = parameters.dynamics_param.Cm2;
  mpc_params[parameters::CD0] = parameters.dynamics_param.Cd0;
  mpc_params[parameters::CD1] = parameters.dynamics_param.Cd1;
  mpc_params[parameters::CD2] = parameters.dynamics_param.Cd2;
  mpc_params[parameters::GAMMA] = parameters.dynamics_param.gamma;
  mpc_params[parameters::EPS] = parameters.dynamics_param.eps;

  // TODO: Hardcoded!!!!!!
  mpc_params[parameters::CAR_WIDTH] = parameters.dynamics_param.car_width;
  mpc_params[parameters::CAR_OVERHANG] = 0.04;

  // Reference on track
  mpc_params[parameters::XP_TRACK] = parameters.auxillary_param.xp_track;
  mpc_params[parameters::YP_TRACK] = parameters.auxillary_param.yp_track;
  mpc_params[parameters::YAW_TRACK] = parameters.auxillary_param.yaw_track;

  // Final terminal point
  mpc_params[parameters::XP_E] = parameters.auxillary_param.xp_e;
  mpc_params[parameters::YP_E] = parameters.auxillary_param.yp_e;
  mpc_params[parameters::YAW_E] = parameters.auxillary_param.yaw_e;

  safety_dynamic_model_acados_update_params(acados_ocp_capsule_.get(), stage, mpc_params.data(), mpc_params.size());
}

AcadosPacejkaSafetySolver::MpcReturnType AcadosPacejkaSafetySolver::solve()
{
  using MpcExitCode = crs_controls::control_commons::MpcExitCode;

  int status = safety_dynamic_model_acados_solve(acados_ocp_capsule_.get());
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
double AcadosPacejkaSafetySolver::getSolveTime()
{
  double elapsed_time;
  ocp_nlp_get(nlp_solver_.get(), "time_tot", &elapsed_time);
  return elapsed_time;
}

double AcadosPacejkaSafetySolver::getSamplePeriod()
{
  return *(nlp_in_->Ts);
}

AcadosPacejkaSafetySolver::MpcSolution AcadosPacejkaSafetySolver::getSolution()
{
  AcadosPacejkaSafetySolver::MpcSolution solution;
  for (int n = 0; n < N; n++)
  {
    ocp_nlp_out_get(nlp_config_.get(), nlp_dims_.get(), nlp_out_.get(), n, "x", solution.x[n].data());
    ocp_nlp_out_get(nlp_config_.get(), nlp_dims_.get(), nlp_out_.get(), n, "u", solution.u[n].data());
  }
  ocp_nlp_out_get(nlp_config_.get(), nlp_dims_.get(), nlp_out_.get(), N, "x", solution.x[N].data());
  return solution;
}

}  // namespace crs_safety::pacejka_mpc_safety_filter::solvers
