#pragma once

#include <array>

#include "commons/trajectory.h"
#include "control_commons/mpc_controller.h"
#include "pacejka_model/pacejka_discrete.h"

#include "pacejka_tracking_mpc/tracking_mpc_pacejka_config.h"
#include "pacejka_tracking_mpc/solvers/pacejka_tracking_mpc_structures.h"
#include "pacejka_tracking_mpc/solvers/acados_pacejka_tracking_mpc_solver.h"

namespace crs_controls::pacejka_tracking_mpc
{

class PacejkaTrackingMpcController
  : public MpcController<crs_models::pacejka_model::DiscretePacejkaModel, crs_models::pacejka_model::pacejka_car_state,
                         crs_models::pacejka_model::pacejka_car_input>
{
  using MpcSolution = solvers::AcadosPacejkaTrackingMpcSolver::MpcSolution;
  using StateArray = solvers::AcadosPacejkaTrackingMpcSolver::StateArray;
  using MpcParameters = solvers::AcadosPacejkaTrackingMpcSolver::MpcParameters;
  using pacejka_vars = solvers::vars;
  struct ReferenceOnTrack
  {
    double x;
    double y;
  };

public:
  // Constructor, Creates a MPC controller based on the config and a track description (requires changing of pointer to
  // dynamic track)
  PacejkaTrackingMpcController(tracking_mpc_pacejka_config config,
                               std::shared_ptr<crs_models::pacejka_model::DiscretePacejkaModel> model,
                               std::shared_ptr<Trajectory> reference_trajectory);
  /**
   * @brief Get the Planned Trajectory.
   * Returns a vector with 4 entries for each step of the horizon
   *
   * @return std::vector<Eigen::VectorXd> vector containing
   * [planned_x,
   *  planned_y,
   *  planned_vx,
   *  planned_vy,
   *  planned_yaw,
   * ]
   */
  std::optional<std::vector<std::vector<double>>> getPlannedTrajectory() override;

  /**
   * @brief Initializes the mpc solver
   *
   */
  void initialize(crs_models::pacejka_model::pacejka_car_state state);

  /**
   * @brief Executes the MPC controller, returning a new input to apply
   *
   * @param state current measured state of the system
   * @param timestamp current timestamp, will be ignored
   * @return crs_models::pacejka_model::pacejka_car_input
   */
  crs_models::pacejka_model::pacejka_car_input
  getControlInput(crs_models::pacejka_model::pacejka_car_state state,
                  double timestamp = 0 /* timestamp will be ignored */) override;

  /**
   * @brief Updates the mpc config
   *
   */
  void setConfig(tracking_mpc_pacejka_config config);

  /**
   * @brief Returns the config
   *
   * @return mpc_config&
   */
  tracking_mpc_pacejka_config& getConfig();

  /**
   * @brief Returns wether the controller is initialized
   *
   * @return true if controller is already initialized
   * @return false if controller is not initialized
   */
  bool isInitializing() override;

private:
  tracking_mpc_pacejka_config config_;
  solvers::AcadosPacejkaTrackingMpcSolver solver_;

  MpcSolution last_solution_;
  std::array<ReferenceOnTrack, solvers::AcadosPacejkaTrackingMpcSolver::N> last_reference_on_track_;

  // Last known input that was applied
  crs_models::pacejka_model::pacejka_car_input last_input_;

  bool is_initialized_ = false;
  bool initializing_ = false;

  unsigned int counter_loop_ = 0;
};

}  // namespace crs_controls::pacejka_tracking_mpc
