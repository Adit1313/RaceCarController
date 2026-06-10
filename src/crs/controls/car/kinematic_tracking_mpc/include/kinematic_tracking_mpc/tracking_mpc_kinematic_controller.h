#pragma once

#include <array>

#include "commons/trajectory.h"
#include "kinematic_model/kinematic_discrete.h"
#include "control_commons/mpc_controller.h"

#include "kinematic_tracking_mpc/tracking_mpc_kinematic_config.h"
#include "kinematic_tracking_mpc/solvers/acados_kinematic_tracking_mpc_solver.h"

namespace crs_controls::kinematic_tracking_mpc
{

class KinematicTrackingMpcController : public MpcController<crs_models::kinematic_model::DiscreteKinematicModel,
                                                            crs_models::kinematic_model::kinematic_car_state,
                                                            crs_models::kinematic_model::kinematic_car_input>
{
  using MpcSolution = solvers::AcadosKinematicTrackingMpcSolver::MpcSolution;
  using StateArray = solvers::AcadosKinematicTrackingMpcSolver::StateArray;
  using MpcParameters = solvers::AcadosKinematicTrackingMpcSolver::MpcParameters;
  using kinematic_vars = solvers::vars;

  struct ReferenceOnTrack
  {
    double x;
    double y;
  };

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
  std::optional<std::vector<std::vector<double>>> getPlannedTrajectory() override;

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
    return initializing_;
  }

private:
  // The MPC config (cost values)
  tracking_mpc_kinematic_config config_;
  solvers::AcadosKinematicTrackingMpcSolver solver_;

  MpcSolution last_solution_;
  std::array<ReferenceOnTrack, solvers::AcadosKinematicTrackingMpcSolver::N> last_reference_on_track_;

  // Last known input that was applied
  crs_models::kinematic_model::kinematic_car_input last_input_;

  bool is_initialized_ = false;
  bool initializing_ = false;
  int counter_loop_ = 0;
};

}  // namespace crs_controls::kinematic_tracking_mpc
