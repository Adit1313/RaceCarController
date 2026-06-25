#pragma once

/**
 * Implementations of the template PacejkaMpccController class.
 */

namespace crs_controls::pacejka_mpcc
{
template <typename SolverType>
PacejkaMpccController<SolverType>::PacejkaMpccController(
    mpcc_pacejka_config config, std::shared_ptr<crs_models::pacejka_model::DiscretePacejkaModel> model,
    std::shared_ptr<StaticTrackTrajectory> track)
  : MpcController(model, std::static_pointer_cast<crs_controls::Trajectory>(track))
{
  setConfig(config);
}

template <typename SolverType>
std::optional<std::vector<std::vector<double>>> PacejkaMpccController<SolverType>::getPlannedTrajectory()
{
  std::vector<std::vector<double>> traj;
  for (int i = 0; i < solver_.N; i++)
  {
    const auto global_coords = convertToGlobalCoordinates(last_solution_.x[i]);
    traj.push_back({
        global_coords.pos_x,
        global_coords.pos_y,
        global_coords.vel_x,
        global_coords.vel_y,
        global_coords.yaw,
        last_reference_on_track_[i].x,
        last_reference_on_track_[i].y,
        last_reference_on_track_[i].yaw,
    });
  }

  return traj;
}

template <typename SolverType>
typename PacejkaMpccController<SolverType>::MpcParameters
PacejkaMpccController<SolverType>::generateCurrentSolverParameters()
{
  auto track = getStaticTrack();
  const double max_arc_length = track->getMaxArcLength();

  MpcParameters parameters;

  for (int stage = 0; stage < solver_.N; stage++)
  {
    double distance_on_track = last_solution_.x[stage][static_cast<int>(pacejka_vars::THETA)] - laps_ * max_arc_length;
    int reference_track_index = distance_on_track * track->getDensity();

    // Update tracking point based on predicted distance on track
    const auto reference_track_point = track->operator[](reference_track_index);
    const auto reference_track_rate = track->getRate(reference_track_index);
    crs_controls::pacejka_mpcc::solvers::TrajectoryTrackPoint track_point = {
      reference_track_point.x(),
      reference_track_point.y(),
      reference_track_rate.x(),
      reference_track_rate.y(),
      distance_on_track + laps_ * max_arc_length,
      track->getTrackAngle(reference_track_index) + laps_ * 2 * M_PI,
    };

    // Update reference visualization
    last_reference_on_track_[stage] = { track_point.x, track_point.y, track_point.phi };

    parameters[stage] = { model_->getParams(),
                          { config_.Q1, config_.Q2, config_.R1, config_.R2, config_.R3, config_.q },
                          track_point };
  }
  return parameters;
}

template <typename SolverType>
void PacejkaMpccController<SolverType>::initialize(crs_models::pacejka_model::pacejka_car_state state)
{
  auto track = getStaticTrack();
  // Initial inputs.
  last_input_.steer = 0;
  // Initial torque input. Lets use 50% throttle
  last_input_.torque = 0.5;
  // Set initial arc length
  theta_ = track->getArcLength(track->getClosestTrackPointIdx(Eigen::Vector2d(state.pos_x, state.pos_y)));

  state.vel_x = std::max(0.01, state.vel_x);

  StateArray x_0 = convertToLocalCoordinates(state, last_input_, theta_);

  // Setup the initial guess (last_solution) of the solver for solver initialization.
  // NOTE(@naefjo): The initial solution of the solver is set using the track reference.
  // The track points are chosen by getting the distance that the car would drive if
  // the forward velocity was ramped up linearly from the initial condition to 1.5m/s along
  // the horizon. This distance (which is the arc length if we assume the car just drives along the
  // center-line) is then transformed into the corresponding track center-line points which make
  // up the initial guess.
  const double ts = solver_.getSamplePeriod();
  const double initial_vel = x_0[3];
  double driven_arclength = theta_;
  auto x_init = x_0;

  for (int stage = 0; stage < solver_.N + 1; stage++)
  {
    const double curr_vel = (1.5 - initial_vel) * stage / (float)(solver_.N + 1) + initial_vel;
    driven_arclength += ts * curr_vel;
    const int reference_track_index = driven_arclength * track->getDensity();
    x_init = generateInitialization(reference_track_index, curr_vel);

    // Set initial state
    last_solution_.x[stage] = x_init;
    // Set initial input (nothing to do, its zero)
  }

  // Run solver
  // TODO(@naefjo): reduce cognitive complexity of this code
  for (int run = 0; run < config_.warmstart_iterations; run++)
  {
    MpcParameters parameters = generateCurrentSolverParameters();

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
                << " ]: MPCC exitflag: " << static_cast<int>(solution.exit_code) << " in warmstart iteration " << run
                << std::endl;
    }
  }
}

template <typename SolverType>
crs_models::pacejka_model::pacejka_car_input
PacejkaMpccController<SolverType>::getControlInput(crs_models::pacejka_model::pacejka_car_state state,
                                                   double timestamp [[maybe_unused]])
{
  const double max_arc_length = getStaticTrack()->getMaxArcLength();

  if (!is_initialized_)
  {
    initialize(state);
    is_initialized_ = true;
  }

  // initialize state to virtually advanced vehicle states and inputs
  auto virtual_state = model_->applyModel(state, last_input_, config_.lag_compensation_time);

  // theta also needs to be adjusted to account for the forward simulation. This is exact since dTheta is
  // piecewise constant.
  theta_ += config_.lag_compensation_time * last_solution_.u[0][pacejka_inputs::DTHETA];

  StateArray x_0 = convertToLocalCoordinates(virtual_state, last_input_, theta_);

  // if lap is done, increase lap counter
  if (theta_ > (laps_ + 1) * max_arc_length)
  {
    laps_++;
  }

  crs_controls::control_commons::shiftSequence(last_solution_.x);
  crs_controls::control_commons::shiftSequence(last_solution_.u);

  for (unsigned int iteration = 0; iteration < config_.max_sqp_iterations; ++iteration)
  {
    MpcParameters parameters = generateCurrentSolverParameters();

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
                << " ]: MPCC exitflag: " << static_cast<int>(solution.exit_code) << std::endl;
    }
  }

  last_input_.torque = last_solution_.x[1][static_cast<int>(pacejka_vars::TORQUE)];
  last_input_.steer = last_solution_.x[1][static_cast<int>(pacejka_vars::STEER)];
  theta_ = last_solution_.x[1][static_cast<int>(pacejka_vars::THETA)];

  return last_input_;
}

template <typename SolverType>
mpcc_pacejka_config& PacejkaMpccController<SolverType>::getConfig()
{
  return config_;
}

template <typename SolverType>
void PacejkaMpccController<SolverType>::setConfig(mpcc_pacejka_config config)
{
  config_ = config;
}

template <typename SolverType>
bool PacejkaMpccController<SolverType>::isInitializing()
{
  return false;
}

template <typename SolverType>
std::shared_ptr<StaticTrackTrajectory> PacejkaMpccController<SolverType>::getStaticTrack()
{
  return std::static_pointer_cast<crs_controls::StaticTrackTrajectory>(trajectory_);
}

template <typename SolverType>
typename PacejkaMpccController<SolverType>::StateArray PacejkaMpccController<SolverType>::convertToLocalCoordinates(
    const crs_models::pacejka_model::pacejka_car_state& global_state,
    const crs_models::pacejka_model::pacejka_car_input& input, double theta)
{
  return StateArray{ global_state.pos_x, global_state.pos_y, global_state.yaw,
                     global_state.vel_x, global_state.vel_y, global_state.yaw_rate,
                     input.torque,       input.steer,        theta };
}

template <typename SolverType>
crs_models::pacejka_model::pacejka_car_state PacejkaMpccController<SolverType>::convertToGlobalCoordinates(
    const typename PacejkaMpccController<SolverType>::StateArray& local_state)
{
  // NOTE(@naefjo): standard MPCC is: x, y, yaw, vx, vy, dyaw.
  return { local_state[0], local_state[1], local_state[2], local_state[3], local_state[4], local_state[5] };
}

template <typename SolverType>
typename PacejkaMpccController<SolverType>::StateArray
PacejkaMpccController<SolverType>::generateInitialization(const int reference_track_index,
                                                          const double current_velocity)
{
  auto track = getStaticTrack();
  const Eigen::Vector2d current_centerline_reference = track->operator[](reference_track_index);
  return {
    current_centerline_reference.x(),
    current_centerline_reference.y(),
    track->getTrackAngle(reference_track_index),
    current_velocity,                                               // v_x
    0.0,                                                            // v_y
    track->getCurvature(reference_track_index) * current_velocity,  // dyaw
    0.0,
    0.0,
    track->getArcLength(reference_track_index),
  };
}
}  // namespace crs_controls::pacejka_mpcc
