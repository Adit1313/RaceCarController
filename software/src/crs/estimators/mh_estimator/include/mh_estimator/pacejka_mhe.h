#ifndef SRC_CRS_ESTIMATORS_PACEJKA_MH_ESTIMATOR_INCLUDE_MHE
#define SRC_CRS_ESTIMATORS_PACEJKA_MH_ESTIMATOR_INCLUDE_MHE

#include "mh_estimator/mhe.h"
#include <mhe_solvers/pacejka_mhe_solver.h>

#include <commons/data_buffer.h>

#include <thread>

#include <pacejka_model/pacejka_car_state.h>
#include <pacejka_model/pacejka_car_input.h>
#include "pacejka_mhe_config.h"
#include <pacejka_model/pacejka_discrete.h>

#include <kalman_estimator/discrete_ekf.h>
#include <estimators/base_estimator.h>
#include <estimators/model_based_estimator.h>
#include <boost/circular_buffer.hpp>

typedef mhe_solvers::pacejka_solvers::vars pacejka_vars;

namespace crs_estimators
{
namespace mhe
{
typedef crs_models::pacejka_model::DiscretePacejkaModel model_type;
typedef crs_models::pacejka_model::pacejka_car_state pacejka_state;
typedef crs_models::pacejka_model::pacejka_car_input pacejka_input;
typedef crs_estimators::ModelBasedEstimator<pacejka_state, pacejka_input> estimator_type;

/**
 * @brief The predicted mhe solution
 *
 */
struct mhe_solution
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
  std::vector<Eigen::Vector3d> trajectory_estimate_;
};

class Pacejka_MHE : public MHE<pacejka_state, pacejka_input>
{
  // map saves objects that have a key and a value (here key = string, e.g. mocap, value = sensor_model e.g.
  // mocap_sensor_model)
  std::map<std::string, std::shared_ptr<crs_sensor_models::SensorModel<pacejka_state, pacejka_input>>>
      key_to_sensor_model_;

private:
  pacejka_mhe_config config_;
  std::shared_ptr<mhe_solvers::pacejka_solvers::PacejkaMheSolver> solver_;

  bool is_initialized = false;
  bool initializing = false;

  // Flag to indicate if the solver is currently solving the optimization problem
  bool is_solving_ = false;

  std::string base_station_id_;
  const std::vector<std::pair<std::string, std::tuple<bool, double>>> outlier_rejection_params_;

  DataBuffer<pacejka_input> input_buffer;
  DataBuffer<pacejka_state> states_buffer;
  DataBuffer<Eigen::Vector3d> mocap_buffer;
  DataBuffer<Eigen::Vector3d> imu_buffer;
  DataBuffer<Eigen::Matrix<double, 1, 1>> imu_yaw_rate_buffer;
  DataBuffer<Eigen::Vector4d> wheel_encoder_buffer;
  DataBuffer<Eigen::Matrix<double, 4, 1>> lighthouse_sweep_1_buffer;
  DataBuffer<Eigen::Matrix<double, 4, 1>> lighthouse_sweep_2_buffer;

  DataBuffer<double> reference_timestamps;

  std::vector<pacejka_input> subsampled_inputs;
  std::vector<pacejka_state> subsampled_states;
  std::vector<Eigen::Vector3d> subsampled_mocap;
  std::vector<Eigen::Vector3d> subsampled_imu;
  std::vector<Eigen::Matrix<double, 1, 1>> subsampled_imu_yaw_rate;
  std::vector<Eigen::Vector4d> subsampled_wheel_encoders;
  std::vector<Eigen::Vector4d> subsampled_lighthouse_sweep_1;
  std::vector<Eigen::Vector4d> subsampled_lighthouse_sweep_2;

  std::vector<double> horizon_shooting_ts_;

  std::vector<bool> valid_mocap;
  std::vector<bool> valid_imu;
  std::vector<bool> valid_imu_yaw_rate;
  std::vector<bool> valid_wheel_encoders;
  std::vector<bool> valid_lighthouse_sweep_1;
  std::vector<bool> valid_lighthouse_sweep_2;

  // Pointer to the thread that currently solves the optimization problem
  std::thread* solver_thread_;

  void loadMheSolver();

  /**
   * @brief Helper function to get state from solver at certain stage. Converts from double array to pacejka_state
   *
   * @param stage
   * @return pacejka_state
   */
  pacejka_state getStateFromSolution(const int stage);

public:
  // Reference to last mhe solution
  mhe_solution last_solution;

  // Just used for visualization. Debugging
  std::vector<Eigen::Vector3d> mocap_reference;

  // Constructor
  Pacejka_MHE(pacejka_mhe_config config, std::shared_ptr<model_type> discrete_model, pacejka_state initial_state,
              pacejka_input initial_input, std::shared_ptr<estimator_type> estimator, std::string base_station_id,
              const std::vector<std::pair<std::string, std::tuple<bool, double>>> outlier_rejection_params = {},
              bool log_diagnostic_data = false);

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
  std::vector<std::vector<double>> getPlannedTrajectory() override;

  /**
   * @brief Adds a sensor model to the map (key and sensor_model)
   */
  void
  addSensorModel(std::string sensor_key,
                 std::shared_ptr<crs_sensor_models::SensorModel<pacejka_state, pacejka_input>> sensors_model) override;

  /**
   * @brief Function that gets called whenever a new input is applied.
   *
   * @param input e.g. control input
   * @param timestamp current time in s when this input was applied
   */
  void controlInputCallback(const pacejka_input input, const double timestamp) override;

  /**
   * @brief Function that gets called whenever a new measurement is received.
   *
   * @param measurement
   */
  void measurementCallback(const crs_sensor_models::measurement measurement) override;

  /**
   * @brief Function that returns the current best state estimate
   */
  pacejka_state getStateEstimate() override
  {  // Defaults to internal system time, if nothing is provided.
    auto system_time_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
    const double timestamp = system_time_ms.count() / 1000.0;
    return getStateEstimate(timestamp);
  }

  /**
   * @brief Function that returns the current best state estimate for a given timestamp (seconds).
   */
  pacejka_state getStateEstimate(const double timestamp) override;

  /**
   * @brief Initializes the mhe solver
   *
   */
  void initialize(pacejka_state state);

  /**
   * @brief Returns whether the controller is initialized
   *
   * @return true if controller is already initialized
   * @return false if controller is not initialized
   */
  bool isInitializing()
  {
    return initializing;
  }

  /**
   * @brief Updates the mhe config
   *
   */
  void setConfig(pacejka_mhe_config config);

  /**
   * @brief Returns the config
   *
   * @return mhe_config&
   */
  pacejka_mhe_config& getConfig();

  /**
   * @brief Solves MHE problem, returning a new estimate of the state, using the internally cached measurements.

   * @return crs_models::pacejka_model::pacejka_car_state
   */
  pacejka_state solveMHE(const double timestamp);
};
}  // namespace mhe
}  // namespace crs_estimators
#endif /* SRC_CRS_ESTIMATORS_PACEJKA_MH_ESTIMATOR_INCLUDE_MHE */
