#ifndef MPC_CONTROLLER_KINEMATIC_CONTROLLER_TRACKING_MPC_KINEMATIC_CONTROLLER_H
#define MPC_CONTROLLER_KINEMATIC_CONTROLLER_TRACKING_MPC_KINEMATIC_CONTROLLER_H

#include <cmath>
#include <memory>
#include <thread>

#include "commons/filter.h"
#include "commons/trajectory.h"
#include "kinematic_model/kinematic_discrete.h"
#include "controls/mpc_controller.h"
#include "mpc_solvers/kinematic_tracking_mpc_solver.h"
#include "tracking_mpc_kinematic_config.h"

using kinematic_tracking_vars = mpc_solvers::kinematic_tracking_solvers::vars;
using kinematic_tracking_inputs = mpc_solvers::kinematic_tracking_solvers::inputs;

namespace crs_controls
{

#ifndef TRACKING_MPC_SOLUTION
#define TRACKING_MPC_SOLUTION
/**
 * @brief The predicted mpc solution
 *
 */
struct tracking_mpc_solution
{
  /**
   * @brief The predicted states over the full horizon.
   * @note Dimension is number_of_states * horizon_length
   *
   */
  std::vector<double> states_;

  /**
   * @brief The predicted input over the full horizon.
   * @note Dimension is number_of_inputs * horizon_length
   *
   * This is not the car input (these are part of the states vector), but rather the rate of change of the car inputs
   * as well as rate of change of the track distance
   */
  std::vector<double> inputs_;
  std::vector<Eigen::Vector2d> reference_;
};

#endif

class KinematicTrackingMpcController : public MpcController<crs_models::kinematic_model::DiscreteKinematicModel,
                                                            crs_models::kinematic_model::kinematic_car_state,
                                                            crs_models::kinematic_model::kinematic_car_input>
{
private:
  // The MPC config (cost values)
  tracking_mpc_kinematic_config config_;
  // Shared pointer to the solver for the MPC problem (requires changing)
  std::shared_ptr<mpc_solvers::kinematic_tracking_solvers::KinematicTrackingMpcSolver> solver_;
  // Current distance driven on the track (not used)

  bool is_initialized = false;
  bool initializing = false;

  int counter_loop = 0;

  // Reference to last mpc solution
  tracking_mpc_solution last_solution;

  // Pointer to the thread that currently solves the optimization problem
  std::thread* solver_thread_;

  // Last known input that was applied
  crs_models::kinematic_model::kinematic_car_input last_input_;

  void loadMpcSolver();

public:
  // Constructor, Creates a MPC controller based on the config and a track description (requires changing of pointer to
  // dynamic track)
  KinematicTrackingMpcController(tracking_mpc_kinematic_config config,
                                 std::shared_ptr<crs_models::kinematic_model::DiscreteKinematicModel> model,
                                 std::shared_ptr<Trajectory> reference_trajectory);
  /**
   * @brief Get the Planned Trajectory.
   * Returns a vector with 4 entries for each step of the horizon
   *
   * @return std::vector<Eigen::VectorXd> vector containing
   * [planned_x,
   *  planned_y,
   *  planned_velocity,
   *  planned_yaw,
   * ]
   */
  std::optional<std::vector<std::vector<double>>> getPlannedTrajectory() const override;

  /**
   * @brief Initializes the mpc solver
   *
   */
  void initialize(crs_models::kinematic_model::kinematic_car_state state);

  /**
   * @brief Executes the MPC controller, returning a new input to apply
   *
   * @param state current measured state of the system
   * @param timestamp current timestamp, will be ignored
   * @return crs_models::kinematic_model::kinematic_car_input
   */
  crs_models::kinematic_model::kinematic_car_input
  getControlInput(crs_models::kinematic_model::kinematic_car_state state,
                  double timestamp = 0 /* timestamp will be ignored */) override;

  /**
   * @brief Updates the mpc config
   *
   */
  void setConfig(tracking_mpc_kinematic_config config);

  /**
   * @brief Returns the config
   *
   * @return mpc_config&
   */
  tracking_mpc_kinematic_config& getConfig();

  /**
   * @brief Returns wether the controller is initialized
   *
   * @return true if controller is already initialized
   * @return false if controller is not initialized
   */
  bool isInitializing() override
  {
    return initializing;
  }
};

}  // namespace crs_controls

#endif
