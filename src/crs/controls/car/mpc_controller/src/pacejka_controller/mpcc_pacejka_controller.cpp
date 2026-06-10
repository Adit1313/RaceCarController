#include <mpc_controller/pacejka_controller/mpcc_pacejka_controller.h>

#ifdef acados_pacejka_mpcc_solver_FOUND
#include "acados_pacejka_mpcc_solver/acados_pacejka_mpcc_solver.h"
#endif

#ifdef forces_pacejka_mpcc_solver_FOUND
#include "forces_pacejka_mpcc_solver/forces_pacejka_mpcc_solver.h"
#endif

namespace crs_controls
{
void PacejkaMpccController::loadMpcSolver()
{
  if (config_.solver_type == "ACADOS")
  {
#ifdef acados_pacejka_mpcc_solver_FOUND
    solver_ = std::make_shared<mpc_solvers::pacejka_solvers::AcadosPacejkaMpccSolver>();
#else
    assert(true && "Requested ACADOS solver for pacejka MPC. Solver not found!");
#endif
  }

#ifdef forces_pacejka_mpcc_solver_FOUND
  if (config_.solver_type == "FORCES")
  {
    solver_ = std::make_shared<mpc_solvers::pacejka_solvers::ForcesPacejkaMpccSolver>();
  }
#else
  assert(true && "Requested Forces solver for pacejka MPC. Solver not found!");
#endif
}

std::shared_ptr<StaticTrackTrajectory> PacejkaMpccController::getStaticTrack()
{
  return std::static_pointer_cast<StaticTrackTrajectory>(trajectory_);
}

// Constructor, Creates a MPC controller based on the config and a track description
PacejkaMpccController::PacejkaMpccController(mpcc_pacejka_config config,
                                             std::shared_ptr<crs_models::pacejka_model::DiscretePacejkaModel> model,
                                             std::shared_ptr<StaticTrackTrajectory> track)
  : MpcController(model, std::static_pointer_cast<Trajectory>(track))
{
  setConfig(config);
  loadMpcSolver();

  // Initialize last solutions with zeros
  last_solution.states_ = std::vector<double>(solver_->getStateDimension() * solver_->getHorizonLength(), 0.0);
  last_solution.inputs_ = std::vector<double>(solver_->getInputDimension() * solver_->getHorizonLength(), 0.0);

  // Initialize last references with zeros (only for visualization)
  for (int i = 0; i < solver_->getHorizonLength(); i++)
    last_solution.reference_on_track_.push_back(Eigen::Vector3d(0, 0, 0));
}

std::optional<std::vector<std::vector<double>>> PacejkaMpccController::getPlannedTrajectory() const
{
  std::vector<std::vector<double>> traj;
  for (int i = 0; i < solver_->getHorizonLength(); i++)
  {
    double x_pos = last_solution.states_[i * solver_->getStateDimension() + pacejka_vars::X];
    double y_pos = last_solution.states_[i * solver_->getStateDimension() + pacejka_vars::Y];
    double x_vel = last_solution.states_[i * solver_->getStateDimension() + pacejka_vars::VX];
    double y_vel = last_solution.states_[i * solver_->getStateDimension() + pacejka_vars::VY];

    // Append values to trajectory for visualization
    traj.push_back({
        x_pos,                                                                        // Planned x
        y_pos,                                                                        // Planned y
        x_vel,                                                                        // Planned longitudinal velocity
        y_vel,                                                                        // Planned lateral velocity
        last_solution.states_[i * solver_->getStateDimension() + pacejka_vars::YAW],  // Planned Yaw
        last_solution.reference_on_track_[i].x(),                                     // Reference x
        last_solution.reference_on_track_[i].y(),                                     // Reference y
        last_solution.reference_on_track_[i](2)                                       // Reference yaw
    });
  }

  return traj;
}

void PacejkaMpccController::initialize(crs_models::pacejka_model::pacejka_car_state state)
{
  mpc_solvers::pacejka_solvers::tracking_costs mpc_costs = { config_.Q1, config_.Q2, config_.R1,
                                                             config_.R2, config_.R3, config_.q };
  std::shared_ptr<crs_controls::StaticTrackTrajectory> track = getStaticTrack();

  // Initial inputs.
  last_input_.steer = 0;
  // Initial torque input. Lets use 50% throttle
  last_input_.torque = 0.5;
  // Set initial arc length
  theta_ = track->getArcLength(track->getClosestTrackPointIdx(Eigen::Vector2d(state.pos_x, state.pos_y)));

  double x_init[9];
  x_init[0] = state.pos_x;
  x_init[1] = state.pos_y;
  x_init[2] = state.yaw;
  x_init[3] = std::max(0.01, state.vel_x);  // Should not be zero as otherwise the model gets NaN issues
  x_init[4] = state.vel_y;
  x_init[5] = state.yaw_rate;
  x_init[6] = last_input_.torque;
  x_init[7] = last_input_.steer;
  x_init[8] = theta_;

  solver_->setInitialState(x_init);

  // Setup the initial guess (last_solution) of the solver for solver initialization.
  // NOTE(@naefjo): The initial solution of the solver is set using the track reference.
  // The track points are chosen by getting the distance that the car would drive if
  // the forward velocity was ramped up linearly from the initial condition to 1.5m/s along
  // the horizon. This distance (which is the arc length if we assume the car just drives along the
  // center-line) is then transformed into the corresponding track center-line points which make
  // up the initial guess.
  const double ts = solver_->getSamplePeriod();
  const double initial_vel = x_init[3];
  double driven_arclength = theta_;

  for (int stage = 0; stage < solver_->getHorizonLength(); stage++)
  {
    const double curr_vel = (1.5 - initial_vel) * stage / (float)(solver_->getHorizonLength()) + initial_vel;
    driven_arclength += ts * curr_vel;
    const int reference_track_index = driven_arclength * track->getDensity();
    const Eigen::Vector2d current_centerline_reference = track->operator[](reference_track_index);
    x_init[0] = current_centerline_reference.x();
    x_init[1] = current_centerline_reference.y();
    x_init[2] = track->getTrackAngle(reference_track_index);
    x_init[3] = curr_vel;                                               // v_x
    x_init[4] = 0.0;                                                    // v_y
    x_init[5] = track->getCurvature(reference_track_index) * curr_vel;  // dyaw
    x_init[8] = track->getArcLength(reference_track_index);

    // Set initial state
    for (int x_idx = 0; x_idx < solver_->getStateDimension(); x_idx++)
      last_solution.states_[stage * solver_->getStateDimension() + x_idx] = x_init[x_idx];
    // Set initial input (nothing to do, its zero)
  }

  // Run solver
  for (int run = 0; run < config_.warmstart_iterations; run++)
  {
    for (int stage = 0; stage < solver_->getHorizonLength(); stage++)
    {
      // Ugly indexing to get distance on track of predicted solution
      double distance_on_track = last_solution.states_[stage * solver_->getStateDimension() + pacejka_vars::THETA];
      // Convert it to track indice using density of track points which are regularly sampled
      int reference_track_index = distance_on_track * track->getDensity();

      // Update tracking point based on predicted distance on track
      const Eigen::Vector2d current_centerline_reference = track->operator[](reference_track_index);
      const Eigen::Vector2d current_rate_reference = track->getRate(reference_track_index);

      mpc_solvers::pacejka_solvers::trajectory_track_point track_point;
      track_point.x = current_centerline_reference.x();
      track_point.y = current_centerline_reference.y();
      track_point.grad_x = current_rate_reference.x();
      track_point.grad_y = current_rate_reference.y();
      track_point.theta = track->getArcLength(reference_track_index);
      track_point.phi = track->getTrackAngle(reference_track_index);

      // Update reference visualization
      last_solution.reference_on_track_[stage] = Eigen::Vector3d(track_point.x, track_point.y, track_point.phi);

      solver_->updateParams(stage,
                            model_->getParams(),  // Model Dynamics
                            mpc_costs,            // Costs
                            track_point);         // Reference point

      // Use previous solution as initial guess this time
      solver_->setStateInitialGuess(stage, &last_solution.states_[stage * solver_->getStateDimension()]);
      solver_->setInputInitialGuess(stage, &last_solution.inputs_[stage * solver_->getInputDimension()]);
    }
    int exit_flag = solver_->solve(&last_solution.states_[0], &last_solution.inputs_[0]);
    if ((exit_flag > 0 && config_.solver_type == "ACADOS") || (exit_flag < 1 && config_.solver_type == "FORCES"))
    {
      uint64_t now_ms =
          std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
              .count();
      std::cout << "Solver [ " << std::fixed << uint64_t((now_ms) / 100.0) / 10.0 << "] "
                << "]: MPCC exitflag: " << exit_flag << std::endl;
    }
  }
}

/**
 * @brief Executes MPC controller, returning a new input to apply
 *
 * @param state current measured state of the system
 * @param timestamp current timestamp, will be ignored
 * @return crs_models::pacejka_model::pacejka_car_input
 */
crs_models::pacejka_model::pacejka_car_input PacejkaMpccController::getControlInput(
    crs_models::pacejka_model::pacejka_car_state state, double timestamp [[maybe_unused]])
{
  if (initializing)
  {
    return { 0, 0 };
  }

  // max_arc_length is needed to detect when a lap is done. The full track has
  // been extended by a full lap, such that the horizon can predict past the
  // start
  double max_arc_length = getStaticTrack()->getMaxArcLength();
  mpc_solvers::pacejka_solvers::tracking_costs mpc_costs = { config_.Q1, config_.Q2, config_.R1,
                                                             config_.R2, config_.R3, config_.q };

  if (!is_initialized)
  {
    initializing = true;
    initialize(state);
    initializing = false;
    is_initialized = true;
  }

  // initialize state to virtually advanced vehicle states and inputs
  auto virtual_state = model_->applyModel(state, last_input_, config_.lag_compensation_time);

  double x0[] = {
    virtual_state.pos_x, virtual_state.pos_y, virtual_state.yaw,
    virtual_state.vel_x, virtual_state.vel_y, virtual_state.yaw_rate,
    last_input_.torque,  last_input_.steer,   theta_,
  };

  // if lap is done, increase lap counter
  if (theta_ > (laps_ + 1) * max_arc_length)
  {
    laps_++;
  }

  solver_->setInitialState(x0);
  // Run solver
  for (int current_stage = 0; current_stage < solver_->getHorizonLength(); current_stage++)
  {
    // next stage points to current_stage + 1. If current_stage is at end of horizon, next stage directy points to
    // current stage i.e. current_stage = 2 -> next_stage = 3, current_stage = 29 -> next_stage = 29, assuming
    // horizon of 30
    int next_stage = current_stage + (current_stage != solver_->getHorizonLength() - 1);
    // Ugly indexing to get distance on track of predicted solution
    double distance_on_track = last_solution.states_[next_stage * solver_->getStateDimension() + pacejka_vars::THETA] -
                               laps_ * getStaticTrack()->getMaxArcLength();
    // Convert it to track indice using density of track points which are regularly sampled
    int reference_track_index = distance_on_track * getStaticTrack()->getDensity();

    // Update tracking point based on predicted distance on track
    mpc_solvers::pacejka_solvers::trajectory_track_point track_point;
    track_point.x = getStaticTrack()->operator[](reference_track_index).x();
    track_point.y = getStaticTrack()->operator[](reference_track_index).y();
    track_point.grad_x = getStaticTrack()->getRate(reference_track_index).x();
    track_point.grad_y = getStaticTrack()->getRate(reference_track_index).y();
    track_point.theta = distance_on_track + laps_ * getStaticTrack()->getMaxArcLength();
    track_point.phi = getStaticTrack()->getTrackAngle(reference_track_index) + laps_ * 2 * M_PI;

    // Update reference visualization
    last_solution.reference_on_track_[current_stage] = Eigen::Vector3d(track_point.x, track_point.y, track_point.phi);

    solver_->updateParams(current_stage,
                          model_->getParams(),  // Model Dynamics
                          mpc_costs,            // Costs
                          track_point           // Tracking point
    );

    // Use previous solution as initial guess this time
    solver_->setStateInitialGuess(current_stage, &last_solution.states_[next_stage * solver_->getStateDimension()]);
    solver_->setInputInitialGuess(current_stage, &last_solution.inputs_[next_stage * solver_->getInputDimension()]);
  }
  int exit_flag = solver_->solve(&last_solution.states_[0], &last_solution.inputs_[0]);

  if ((exit_flag > 0 && config_.solver_type == "ACADOS") || (exit_flag < 1 && config_.solver_type == "FORCES"))
    std::cerr << "[MPCC SOLVER] - Error when getting solution. Exitcode: " << exit_flag << std::endl;

  last_input_.torque = last_solution.states_[1 * solver_->getStateDimension() + pacejka_vars::TORQUE];
  last_input_.steer = last_solution.states_[1 * solver_->getStateDimension() + pacejka_vars::STEER];
  theta_ = last_solution.states_[1 * solver_->getStateDimension() + pacejka_vars::THETA];

  return last_input_;
}

/**
 * @brief Returns the config
 *
 * @return mpc_config&
 */
mpcc_pacejka_config& PacejkaMpccController::getConfig()
{
  return config_;
}

void PacejkaMpccController::setConfig(mpcc_pacejka_config config)
{
  config_ = config;
}
}  // namespace crs_controls
