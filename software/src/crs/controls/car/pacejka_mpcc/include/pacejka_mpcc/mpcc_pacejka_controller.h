#pragma once

#include <memory>

#include "commons/static_track_trajectory.h"
#include "control_commons/mpc_solver_structures.h"
#include "pacejka_model/pacejka_discrete.h"
#include "pacejka_model/pacejka_car_state.h"
#include "pacejka_model/pacejka_car_input.h"
#include "control_commons/mpc_controller.h"

#include "pacejka_mpcc/mpcc_pacejka_config.h"
#include "pacejka_mpcc/solvers/pacejka_mpcc_structures.h"
#ifdef BUILD_ACADOS_SOLVER
#include "pacejka_mpcc/solvers/acados_pacejka_mpcc_solver.h"
#endif
#ifdef BUILD_FORCES_SOLVER
#include "pacejka_mpcc/solvers/forces_pacejka_mpcc_solver.h"
#endif
#ifdef BUILD_ACADOS_CURVILINEAR_SOLVER
#include "pacejka_mpcc/solvers/acados_pacejka_curvilinear_mpcc_solver.h"
#endif

namespace crs_controls::pacejka_mpcc
{

/**
 * @brief Returns a downcast pointer containing the MPCC controller corresponding to the passed configuration.
 *
 * We put this here to not need to introduce additional complexity in the component registry.
 *
 * @param config specifies the contorller configuration
 * @param is the model with the required model parameters
 * @param track is the track configuration
 */
std::shared_ptr<crs_controls::MpcController<crs_models::pacejka_model::DiscretePacejkaModel,
                                            crs_models::pacejka_model::pacejka_car_state,
                                            crs_models::pacejka_model::pacejka_car_input>>
controllerFactory(mpcc_pacejka_config config, std::shared_ptr<crs_models::pacejka_model::DiscretePacejkaModel> model,
                  std::shared_ptr<StaticTrackTrajectory> track);

/**
 * MPCC for the Pacejka car model.
 *
 * NOTE(@naefjo): This header file only contains the declaration of the class functions. The definitions can be found
 * in the `impl/mpcc_pacejka_controller.tpp` file. We need to put them there since we are dealing with templates and
 * thus can't put the definitions into the cpp file as usually done.
 */
template <typename SolverType>
class PacejkaMpccController
  : public MpcController<crs_models::pacejka_model::DiscretePacejkaModel, crs_models::pacejka_model::pacejka_car_state,
                         crs_models::pacejka_model::pacejka_car_input>
{
  using MpcSolution = typename SolverType::MpcSolution;
  using StateArray = typename SolverType::StateArray;
  using MpcParameters = typename SolverType::MpcParameters;
  using pacejka_vars = typename SolverType::vars;
  using pacejka_inputs = typename SolverType::inputs;

  struct ReferenceOnTrack
  {
    double x;
    double y;
    double yaw;
  };

public:
  // Constructor, Creates a MPC controller based on the config and a track description
  PacejkaMpccController(mpcc_pacejka_config config,
                        std::shared_ptr<crs_models::pacejka_model::DiscretePacejkaModel> model,
                        std::shared_ptr<StaticTrackTrajectory> track);

  /**
   * @brief Get the Planned Trajectory.
   * Returns a vector with 7 entries for each step of the horizon
   *
   * @return std::vector<Eigen::VectorXd> vector containing
   * [planned_x,
   *  planned_y,
   *  planned_velocity,
   *  planned_yaw,
   *  ------------
   *  reference_x,
   *  reference_y.
   *  refernce_yaw
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
  crs_models::pacejka_model::pacejka_car_input getControlInput(crs_models::pacejka_model::pacejka_car_state state,
                                                               double timestamp [[maybe_unused]]) override;

  /**
   * @brief Updates the mpc config
   *
   */
  void setConfig(mpcc_pacejka_config config);

  /**
   * @brief Returns the config
   *
   * @return mpc_config&
   */
  mpcc_pacejka_config& getConfig();

  bool isInitializing() override;

private:
  std::shared_ptr<StaticTrackTrajectory> getStaticTrack();

  /**
   * @brief Convert to the appropriate coordinates the solver expects.
   */
  StateArray convertToLocalCoordinates(const crs_models::pacejka_model::pacejka_car_state& global_state,
                                       const crs_models::pacejka_model::pacejka_car_input& input, double theta);

  /**
   * @brief Convert from local solver coordinates to global coordinates.
   */
  crs_models::pacejka_model::pacejka_car_state convertToGlobalCoordinates(const StateArray& local_state);

  /**
   * @brief Generate the local representation of the initialization.
   */
  StateArray generateInitialization(const int reference_track_index, const double current_velocity);

  /**
   * @brief Generate the current parameter values for the solver.
   *
   * Computes centerline reference and populates the parameter struct
   * with the dynamics and cost parameters.
   */
  MpcParameters generateCurrentSolverParameters();

  mpcc_pacejka_config config_;

  SolverType solver_;

  MpcSolution last_solution_;

  std::array<ReferenceOnTrack, SolverType::N> last_reference_on_track_;

  // Last known input that was applied
  crs_models::pacejka_model::pacejka_car_input last_input_;

  // Current lap count
  int laps_ = 0;
  // Current distance driven on the track
  double theta_ = 0.0;

  bool is_initialized_ = false;
};
}  // namespace crs_controls::pacejka_mpcc

#ifdef BUILD_ACADOS_CURVILINEAR_SOLVER
#include "pacejka_mpcc/impl/mpcc_curvilinear_pacejka_controller.h"
#endif  // BUILD_ACADOS_CURVILINEAR_SOLVER
#include "pacejka_mpcc/impl/mpcc_pacejka_controller.tpp"
