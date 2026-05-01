#include "pacejka_tracking_mpc/tracking_mpc_pacejka_controller.h"

namespace crs_controls::pacejka_tracking_mpc
{

PacejkaTrackingMpcController::PacejkaTrackingMpcController(
    tracking_mpc_pacejka_config config, std::shared_ptr<crs_models::pacejka_model::DiscretePacejkaModel> model,
    std::shared_ptr<Trajectory> track)
  : MpcController(model, std::static_pointer_cast<Trajectory>(track))
{
  setConfig(config);
}

std::optional<std::vector<std::vector<double>>> PacejkaTrackingMpcController::getPlannedTrajectory()
{
  std::vector<std::vector<double>> traj;
  for (int i = 0; i < solver_.N; i++)
  {
    traj.push_back({
        last_solution_.x[i][pacejka_vars::X], last_solution_.x[i][pacejka_vars::Y],
        last_solution_.x[i][pacejka_vars::VX], last_solution_.x[i][pacejka_vars::VY],
        last_solution_.x[i][pacejka_vars::YAW], last_reference_on_track_[i].x, last_reference_on_track_[i].y,
        0.0,  // no planned yaw?
    });
  }

  return traj;
}

void PacejkaTrackingMpcController::initialize(crs_models::pacejka_model::pacejka_car_state state)
{
  auto model_dynamics = model_->getParams();
  int N_WARM_START = 50;

  solvers::TrackingCosts mpc_costs = { config_.Q1, config_.Q2, config_.R1, config_.R2 };

  // Initial inputs.
  last_input_.steer = 0;
  // Initial torque input.
  last_input_.torque = 0;

  StateArray x_0 = { state.pos_x, state.pos_y,    state.yaw,          std::max(0.5, state.vel_x),
                     state.vel_y, state.yaw_rate, last_input_.torque, last_input_.steer };

  // Setup parameter for initial solve
  for (int stage = 0; stage < solver_.N + 1; stage++)
  {
    // Set initial state
    last_solution_.x[stage] = x_0;
  }

  // Run solver
  for (int run = 0; run < N_WARM_START; run++)
  {
    MpcParameters parameters;
    for (int stage = 0; stage < solver_.N; stage++)
    {
      // Update tracking point
      solvers::TrajectoryTrackPoint track_point;
      auto next_track_point = (*trajectory_)[counter_loop_ + stage];

      track_point.x = next_track_point(0);
      track_point.y = next_track_point(1);

      parameters[stage] = { model_dynamics, mpc_costs, track_point };
    }
    auto solution = solver_.solve_problem(x_0, last_solution_, parameters);

    if (solution.exit_code == crs_controls::control_commons::MpcExitCode::SUCCEEDED)
    {
      last_solution_ = solution.solution.value();
    }
    else
    {
      uint64_t now_ms =
          std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
              .count();
      std::cout << "Solver [ " << std::fixed << uint64_t((now_ms) / 100.0) / 10.0
                << " ]: Tracking MPC exitflag: " << static_cast<int>(solution.exit_code) << std::endl;
    }
  }
}

crs_models::pacejka_model::pacejka_car_input PacejkaTrackingMpcController::getControlInput(
    crs_models::pacejka_model::pacejka_car_state state, double timestamp [[maybe_unused]])
{
  counter_loop_++;

  const auto model_params = model_->getParams();
  solvers::TrackingCosts mpc_costs = { config_.Q1, config_.Q2, config_.R1, config_.R2 };

  if (!is_initialized_)
  {
    initializing_ = true;
    initialize(state);
    initializing_ = false;
    is_initialized_ = true;
  }

  // initialize state to virtually advanced vehicle states and inputs
  // auto virtual_state = model_->applyModel(state, last_input_, config_.lag_compensation_time);

  double vel_x = std::max(0.2, state.vel_x);  // Should not be zero as otherwise the model gets NaN issues

  StateArray x0 = {
    state.pos_x, state.pos_y, state.yaw, vel_x, state.vel_y, state.yaw_rate, last_input_.torque, last_input_.steer,
  };

  crs_controls::control_commons::shiftSequence(last_solution_.x);
  crs_controls::control_commons::shiftSequence(last_solution_.u);

  // Run solver
  MpcParameters parameters;
  for (int current_stage = 0; current_stage < solver_.N + 1; current_stage++)
  {
    // Update tracking point based on predicted distance on track
    solvers::TrajectoryTrackPoint track_point;
    auto next_track_point = (*trajectory_)[counter_loop_ + current_stage];

    track_point.x = next_track_point(0);
    track_point.y = next_track_point(1);

    // Update reference visualization
    last_reference_on_track_[current_stage] = { track_point.x, track_point.y };

    parameters[current_stage] = { model_params, mpc_costs, track_point };
  }
  auto solution = solver_.solve_problem(x0, last_solution_, parameters);
  if (solution.exit_code == crs_controls::control_commons::MpcExitCode::SUCCEEDED)
  {
    last_solution_ = solution.solution.value();
  }
  else
  {
    uint64_t now_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
            .count();
    std::cout << "Solver [ " << std::fixed << uint64_t((now_ms) / 100.0) / 10.0
              << " ]: Tracking MPC exitflag: " << static_cast<int>(solution.exit_code) << std::endl;
  }

  last_input_.torque = last_solution_.x[1][pacejka_vars::TORQUE];
  last_input_.steer = last_solution_.x[1][pacejka_vars::STEER];

  return last_input_;
}

inline tracking_mpc_pacejka_config& PacejkaTrackingMpcController::getConfig()
{
  return config_;
}

inline void PacejkaTrackingMpcController::setConfig(tracking_mpc_pacejka_config config)
{
  config_ = config;
}
inline bool PacejkaTrackingMpcController::isInitializing()
{
  return initializing_;
}
}  // namespace crs_controls::pacejka_tracking_mpc
