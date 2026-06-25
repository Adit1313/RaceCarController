#include "mh_estimator/pacejka_mhe.h"

#include <chrono>
#include <acados_pacejka_mhe_solver/acados_pacejka_mhe_solver.h>
#include <cmath>
#include <pacejka_sensor_model/mocap_sensor_model.h>
#include <pacejka_sensor_model/imu_sensor_model.h>
#include <pacejka_sensor_model/imu_yaw_rate_sensor_model.h>
#include <pacejka_sensor_model/wheel_encoder_sensor_model.h>
#include <pacejka_sensor_model/lighthouse_sensor_model.h>
#include <dynamic_models/utils/data_conversion.h>

#include <mh_estimator/utils/mhe_utils.h>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>

namespace crs_estimators
{
namespace mhe
{
pacejka_state Pacejka_MHE::getStateFromSolution(const int stage)
{
  pacejka_state state;
  // solution is a vector of all solutions from the solver.
  state.pos_x = last_solution.states_[stage * solver_->getStateDimension() + mhe_solvers::pacejka_solvers::X];
  state.pos_y = last_solution.states_[stage * solver_->getStateDimension() + mhe_solvers::pacejka_solvers::Y];
  state.yaw = last_solution.states_[stage * solver_->getStateDimension() + mhe_solvers::pacejka_solvers::YAW];
  state.vel_x = last_solution.states_[stage * solver_->getStateDimension() + mhe_solvers::pacejka_solvers::VX];
  state.vel_y = last_solution.states_[stage * solver_->getStateDimension() + mhe_solvers::pacejka_solvers::VY];
  state.yaw_rate = last_solution.states_[stage * solver_->getStateDimension() + mhe_solvers::pacejka_solvers::dYAW];
  return state;
}

// Internal estimator used
Pacejka_MHE::Pacejka_MHE(pacejka_mhe_config config, std::shared_ptr<model_type> discrete_model,
                         pacejka_state initial_state, pacejka_input initial_input,
                         std::shared_ptr<estimator_type> estimator, std::string base_station_id,
                         const std::vector<std::pair<std::string, std::tuple<bool, double>>> outlier_rejection_params,
                         bool log_diagnostic_data)
  : MHE<pacejka_state, pacejka_input>(std::static_pointer_cast<MHE::discrete_model_type>(discrete_model), initial_state,
                                      initial_input, estimator, config.start_delay, log_diagnostic_data)
  , base_station_id_(base_station_id)
  , outlier_rejection_params_(outlier_rejection_params)
  , input_buffer(config.max_buffer_size)
  , states_buffer(config.max_buffer_size)
  , mocap_buffer(config.max_buffer_size)
  , imu_buffer(config.max_buffer_size)
  , imu_yaw_rate_buffer(config.max_buffer_size)
  , wheel_encoder_buffer(config.max_buffer_size)
  , lighthouse_sweep_1_buffer(config.max_buffer_size)
  , lighthouse_sweep_2_buffer(config.max_buffer_size)
  , reference_timestamps(config.max_buffer_size)
{
  loadMheSolver();
  setConfig(config);

  // Initialize last solutions with zeros
  last_solution.states_ = std::vector<double>(solver_->getStateDimension() * solver_->getHorizonLength(), 0.0);
  last_solution.inputs_ = std::vector<double>(solver_->getInputDimension() * solver_->getHorizonLength(), 0.0);

  // Initialize all measurements and state estimate with zeros (only for visualization, we "plot" all points at zero
  // while the MHE isn't running or warmstarting)
  for (int i = 0; i < solver_->getHorizonLength(); i++)
  {
    last_solution.trajectory_estimate_.push_back(Eigen::Vector3d(0, 0, 0));
    subsampled_inputs.push_back({});
    subsampled_states.push_back({});

    subsampled_mocap.push_back(Eigen::Vector3d(0, 0, 0));
    subsampled_imu.push_back(Eigen::Vector3d(0, 0, 0));
    subsampled_imu_yaw_rate.push_back(Eigen::Matrix<double, 1, 1>(0));
    subsampled_wheel_encoders.push_back(Eigen::Vector4d(0, 0, 0, 0));
    subsampled_lighthouse_sweep_1.push_back(Eigen::Vector4d(0, 0, 0, 0));
    subsampled_lighthouse_sweep_2.push_back(Eigen::Vector4d(0, 0, 0, 0));

    valid_mocap.push_back(false);
    valid_imu.push_back(false);
    valid_imu_yaw_rate.push_back(false);
    valid_wheel_encoders.push_back(false);
    valid_lighthouse_sweep_1.push_back(false);
    valid_lighthouse_sweep_2.push_back(false);

    horizon_shooting_ts_.push_back(solver_->getSamplePeriod());
  }
}

// Returns a vector with 7 entries for each step of the horizon
// [planned_x, planned_y, planned_velocity, planned_yaw, reference_x, reference_y, reference_yaw]
std::vector<std::vector<double>> Pacejka_MHE::getPlannedTrajectory()
{
  std::vector<std::vector<double>> traj;
  for (int i = 0; i < solver_->getHorizonLength(); i++)
  {
    double x_pos = last_solution.states_[i * solver_->getStateDimension() + pacejka_vars::X];
    double y_pos = last_solution.states_[i * solver_->getStateDimension() + pacejka_vars::Y];
    double vel = std::sqrt(std::pow(last_solution.states_[i * solver_->getStateDimension() + pacejka_vars::VX], 2) +
                           std::pow(last_solution.states_[i * solver_->getStateDimension() + pacejka_vars::VY], 2));

    // If using mocap, use mocap measurement to visualize reference trajectory
    if (valid_mocap[i])
    {
      // Append values to trajectory for visualization
      traj.push_back({ x_pos,                                                                        // Planned x
                       y_pos,                                                                        // Planned y
                       vel,                                                                          // Planned velocity
                       last_solution.states_[i * solver_->getStateDimension() + pacejka_vars::YAW],  // Planned Yaw
                       subsampled_mocap[i][0],                                                       // Reference x
                       subsampled_mocap[i][1],                                                       // Reference y
                       subsampled_mocap[i][2],                                                       // Reference yaw
                       double(valid_mocap[i]) });
    }

    // If using lighthouse, use ekf estimate to visualize reference trajectory
    if (valid_lighthouse_sweep_1[i] || valid_lighthouse_sweep_2[i])
    {
      bool valid = valid_lighthouse_sweep_1[i] || valid_lighthouse_sweep_2[i];
      // Append values to trajectory for visualization
      traj.push_back({ x_pos,                                                                        // Planned x
                       y_pos,                                                                        // Planned y
                       vel,                                                                          // Planned velocity
                       last_solution.states_[i * solver_->getStateDimension() + pacejka_vars::YAW],  // Planned yaw
                       subsampled_states[i].pos_x,                                                   // Reference x
                       subsampled_states[i].pos_y,                                                   // Reference y
                       subsampled_states[i].yaw,                                                     // Reference yaw
                       double(valid) });
    }
  }

  return traj;
}

void Pacejka_MHE::loadMheSolver()
{
  solver_ = std::make_shared<mhe_solvers::pacejka_solvers::AcadosPacejkaMheSolver>();
}

void Pacejka_MHE::controlInputCallback(const pacejka_input input, const double timestamp)
{
  MHE::previous_input_ = input;
  MHE::last_valid_ts_ = timestamp;
  bool full_buffer =
      (mocap_buffer.getTimespan() >= solver_->getHorizonLength() * solver_->getSamplePeriod()) ||
      (lighthouse_sweep_1_buffer.getTimespan() >= solver_->getHorizonLength() * solver_->getSamplePeriod()) ||
      (lighthouse_sweep_2_buffer.getTimespan() >= solver_->getHorizonLength() * solver_->getSamplePeriod());
  // If the MHE is NOT up and running, i.e. start up time is not reached or measurement buffer is not full we want to
  // use the internal estimator If use_internal_estimator flag is set, we always want to use the internal estimator
  if ((MHE::last_valid_ts_ < mhe_startup_time_) || !full_buffer || config_.use_internal_estimator)
  {
    estimator_->controlInputCallback(input, timestamp);
  }
  input_buffer.addData(input, timestamp);
}

pacejka_state Pacejka_MHE::getStateEstimate(const double timestamp)
{
  /**
   * If we are not running or if we have not yet reached the startup time of the mhe, return the ekf estimate
   */
  if (MHE::last_valid_ts_ < mhe_startup_time_)
  {
    return estimator_->getStateEstimate();
  }

  bool full_buffer =
      (mocap_buffer.getTimespan() >= solver_->getHorizonLength() * solver_->getSamplePeriod()) ||
      (lighthouse_sweep_1_buffer.getTimespan() >= solver_->getHorizonLength() * solver_->getSamplePeriod()) ||
      (lighthouse_sweep_2_buffer.getTimespan() >= solver_->getHorizonLength() * solver_->getSamplePeriod());

  // If we have enough measurements, solve the MHE problem
  if (full_buffer && !is_solving_)
  {
    is_solving_ = true;
    MHE::best_state_ = solveMHE(timestamp);
    is_solving_ = false;
  }
  else if (is_solving_)
  {
    return MHE::best_state_;
  }
  else
  {
    MHE::best_state_ = estimator_->getStateEstimate();
    std::cout << "waiting to fill buffer in mhe" << std::endl;
  }

  return MHE::best_state_;
}

void Pacejka_MHE::measurementCallback(const crs_sensor_models::measurement measurement)
{
  if (mhe_startup_time_ == 0.0)
  {
    mhe_startup_time_ = measurement.timestamp + MHE::mhe_delay_time_;
  }

  MHE::last_valid_ts_ = measurement.timestamp;

  bool full_buffer =
      (mocap_buffer.getTimespan() >= solver_->getHorizonLength() * solver_->getSamplePeriod()) ||
      (lighthouse_sweep_1_buffer.getTimespan() >= solver_->getHorizonLength() * solver_->getSamplePeriod()) ||
      (lighthouse_sweep_2_buffer.getTimespan() >= solver_->getHorizonLength() * solver_->getSamplePeriod());

  // If the MHE is NOT up and running, i.e. start up time is not reached or measurement buffer is not full we want to
  // use the internal estimator If use_internal_estimator flag is set, we always want to use the internal estimator
  if ((MHE::last_valid_ts_ < mhe_startup_time_) || !full_buffer || config_.use_internal_estimator)
  {
    estimator_->measurementCallback(measurement);  // updates best_state_
    states_buffer.addData(estimator_->getStateEstimate(), measurement.timestamp);
  }

  if (measurement.sensor_key == crs_sensor_models::pacejka_sensor_models::MocapSensorModel::SENSOR_KEY)
    mocap_buffer.addData(measurement.measurement_data, measurement.timestamp);
  if (measurement.sensor_key == crs_sensor_models::pacejka_sensor_models::ImuSensorModel::SENSOR_KEY)
    imu_buffer.addData(measurement.measurement_data, measurement.timestamp);
  if (measurement.sensor_key == crs_sensor_models::pacejka_sensor_models::ImuYawSensorModel::SENSOR_KEY)
    imu_yaw_rate_buffer.addData(measurement.measurement_data, measurement.timestamp);
  if (measurement.sensor_key == crs_sensor_models::pacejka_sensor_models::WheelEncoderSensorModel::SENSOR_KEY)
    wheel_encoder_buffer.addData(measurement.measurement_data, measurement.timestamp);
  if ((measurement.sensor_key) == "lighthouse_" + base_station_id_ + "_1")
  {
    lighthouse_sweep_1_buffer.addData(measurement.measurement_data, measurement.timestamp);
    reference_timestamps.addData(measurement.timestamp, measurement.timestamp);
  }
  if ((measurement.sensor_key) == "lighthouse_" + base_station_id_ + "_2")
  {
    lighthouse_sweep_2_buffer.addData(measurement.measurement_data, measurement.timestamp);
    reference_timestamps.addData(measurement.timestamp, measurement.timestamp);
  }
}  // namespace mhe

/**
 * @brief Initializes the mhe solver
 *
 */
void Pacejka_MHE::initialize(pacejka_state state)
{
  int N_ = solver_->getHorizonLength();
  Eigen::Matrix<double, pacejka_state::NX, pacejka_state::NX> P = config_.P;

  if (config_.use_internal_estimator)
  {
    // Try to cast internal estimator to ekf.
    // If this is possible we want to load the state estimate covariance matrix P from the ekf.
    // Otherwise we will load it from the config.
    auto ekf_ptr =
        std::dynamic_pointer_cast<crs_estimators::kalman::DiscreteEKF<pacejka_state, pacejka_input>>(estimator_);
    if (ekf_ptr)
    {
      P = ekf_ptr->getPosteriorCovariance();
    }
  }

  mhe_solvers::pacejka_solvers::cost_values mhe_costs = { P,
                                                          config_.Q,
                                                          config_.R_mocap,
                                                          config_.R_imu,
                                                          config_.R_imu_yaw_rate,
                                                          config_.R_wheel_encoders,
                                                          config_.R_lighthouse,
                                                          config_.eta };

  auto discrete_model = std::static_pointer_cast<crs_models::pacejka_model::DiscretePacejkaModel>(MHE::discrete_model);

  // Setup parameter for initial solve
  for (int stage = 0; stage < N_; stage++)
  {
    // Set initial guess to ekf estimation
    last_solution.states_[stage * solver_->getStateDimension() + 0] = subsampled_states[stage].pos_x;
    last_solution.states_[stage * solver_->getStateDimension() + 1] = subsampled_states[stage].pos_y;
    last_solution.states_[stage * solver_->getStateDimension() + 2] = subsampled_states[stage].yaw;
    last_solution.states_[stage * solver_->getStateDimension() + 3] = subsampled_states[stage].vel_x;
    last_solution.states_[stage * solver_->getStateDimension() + 4] = subsampled_states[stage].vel_y;
    last_solution.states_[stage * solver_->getStateDimension() + 5] = subsampled_states[stage].yaw_rate;

    // Set initial input
    // nothing to do, its zero
  }

  // warmstart solver
  for (int run = 0; run < config_.warmstart_iterations; run++)
  {
    std::cout << " WARMSTART: " << run << "/" << config_.warmstart_iterations << std::endl;

    for (int stage = 0; stage < N_; stage++)
    {
      // Define reference at this stage
      mhe_solvers::pacejka_solvers::references references = {
        state,                                                                             // internal estimator
        subsampled_mocap[stage],                                                           // mocap
        subsampled_imu[stage],                                                             // imu
        subsampled_imu_yaw_rate[stage],                                                    // imu yaw rate
        subsampled_wheel_encoders[stage],                                                  // wheel encoders
        subsampled_lighthouse_sweep_1[stage],                                              // lighthouse_sweep_1
        subsampled_lighthouse_sweep_2[stage],                                              // lighthouse_sweep_2
        Eigen::Vector2d(subsampled_inputs[stage].torque, subsampled_inputs[stage].steer),  // input
        valid_mocap[stage],                                                                // valid_mocap
        valid_imu[stage],                                                                  // valid_imu
        valid_imu_yaw_rate[stage],                                                         // valid_imu_yaw_rate
        valid_wheel_encoders[stage],                                                       // valid_wheel_encoders
        valid_lighthouse_sweep_1[stage],                                                   // valid_lighthouse_sweep_1
        valid_lighthouse_sweep_2[stage],                                                   // valid_lighthouse_sweep_2
      };

      if (valid_lighthouse_sweep_1[stage] && valid_lighthouse_sweep_2[stage])
      {
        auto sensor_model_1 = key_to_sensor_model_.find("lighthouse_" + base_station_id_ + "_1");
        auto sensor_model_2 = key_to_sensor_model_.find("lighthouse_" + base_station_id_ + "_2");
        auto lighthouse_sensor_model_1 =
            std::dynamic_pointer_cast<crs_sensor_models::pacejka_sensor_models::LighthouseSensorModel>(
                sensor_model_1->second);
        auto lighthouse_sensor_model_2 =
            std::dynamic_pointer_cast<crs_sensor_models::pacejka_sensor_models::LighthouseSensorModel>(
                sensor_model_2->second);

        std::tuple<Eigen::Matrix<double, 2, 4>, Eigen::Vector3d, Eigen::Matrix3d, double> lighthouse_params_1 =
            lighthouse_sensor_model_1->getLighthouseParams();

        std::tuple<Eigen::Matrix<double, 2, 4>, Eigen::Vector3d, Eigen::Matrix3d, double> lighthouse_params_2 =
            lighthouse_sensor_model_2->getLighthouseParams();

        solver_->updateParams(stage,
                              discrete_model->getParams(),  // Model Dynamics
                              mhe_costs,                    // Costs
                              references,                   // Tracking point
                              lighthouse_params_1, lighthouse_params_2);
      }
      else
      {
        solver_->updateParams(stage,
                              discrete_model->getParams(),  // Model Dynamics
                              mhe_costs,                    // Costs
                              references,                   // Tracking point
                              {}, {});
      }

      solver_->setStateInitialGuess(stage, &last_solution.states_[stage * solver_->getStateDimension()]);
      solver_->setInputInitialGuess(stage, &last_solution.inputs_[stage * solver_->getInputDimension()]);
    }

    try
    {
      int exit_flag = solver_->solve(&last_solution.states_[0], &last_solution.inputs_[0]);

      if ((exit_flag > 0 && config_.solver_type == "ACADOS") || (exit_flag < 1 && config_.solver_type == "FORCES"))
      {
        std::cout << "MHE exitflag: " << exit_flag << std::endl;
      }
    }
    catch (std::exception& e)
    {
      std::cerr << "Exception caught while solving MHE : " << e.what() << std::endl;
    }
  }
  std::cout << "WARMSTART DONE" << std::endl;
}  // namespace mhe

/**
 * @brief Adds a sensor model to the map (key and sensor_model)
 */
void Pacejka_MHE::addSensorModel(
    std::string sensor_key, std::shared_ptr<crs_sensor_models::SensorModel<pacejka_state, pacejka_input>> sensors_model)
{
  key_to_sensor_model_.insert(std::make_pair<>(sensor_key, sensors_model));
}

/**
 * @brief Solves MHE problem, returning a new estimate of the state
 *
 * @return crs_models::pacejka_model::pacejka_car_state
 */
pacejka_state Pacejka_MHE::solveMHE(const double timestamp)
{
  auto timestamp_now = std::chrono::system_clock::now();
  int N_ = solver_->getHorizonLength();

  double ref_ts = last_valid_ts_ + config_.lag_compensation_time;

  // ------------------------------- Subsample Data Buffers and do Outlier Rejection -------------------------------
  bool use_outlier_rejection = 0;
  float outlier_threshold = 0.0;

  // --------------------- INTERNAL FILTERING ---------------------
  if (config_.use_internal_filter)
  {
    std::cout << "Using internal filter with type: " << config_.internal_filter_type << std::endl;
    for (auto key_value : outlier_rejection_params_)  // outlier_rejection_params_ is a vector of pairs: <sensor_key,
                                                      // tuple<use_outlier_rejection, outlier_threshold>>
    {
      // MOCAP
      if (key_value.first == "mocap")
      {
        DataBuffer<Eigen::Vector3d> mocap_filtered_data(mocap_buffer.size());
        mhe_common::filter_mocap(mocap_buffer, mocap_filtered_data, config_.internal_filter_type);
        use_outlier_rejection = std::get<0>(key_value.second);
        if (use_outlier_rejection)
        {
          outlier_threshold = std::get<1>(key_value.second);
          valid_mocap =
              mocap_buffer.interpolateData(1.0 / solver_->getSamplePeriod(), ref_ts, N_, subsampled_mocap,
                                           mhe_common::outlier_check_fnc_mocap, outlier_threshold, "bilinear");
        }
        else
        {
          valid_mocap =
              mocap_filtered_data.interpolateData(1.0 / solver_->getSamplePeriod(), ref_ts, N_, subsampled_mocap);
        }
      }

      // IMU
      if (key_value.first == "imu")
      {
        DataBuffer<Eigen::Vector3d> imu_filtered_data(imu_buffer.size());
        mhe_common::filter_imu(imu_buffer, imu_filtered_data, config_.internal_filter_type);
        use_outlier_rejection = std::get<0>(key_value.second);
        if (use_outlier_rejection)
        {
          outlier_threshold = std::get<1>(key_value.second);
          valid_imu =
              imu_filtered_data.interpolateData(1.0 / solver_->getSamplePeriod(), ref_ts, N_, subsampled_imu,
                                                mhe_common::outlier_check_fnc_imu, outlier_threshold, "bilinear");
        }
        else
        {
          valid_imu = imu_filtered_data.interpolateData(1.0 / solver_->getSamplePeriod(), ref_ts, N_, subsampled_imu);
        }
      }

      // IMU YAW
      if (key_value.first == "imu_yaw_rate")
      {
        DataBuffer<Eigen::Matrix<double, 1, 1>> imu_yaw_rate_filtered_data(imu_yaw_rate_buffer.size());
        mhe_common::filter_imu_yaw_rate(imu_yaw_rate_buffer, imu_yaw_rate_filtered_data, config_.internal_filter_type);
        use_outlier_rejection = std::get<0>(key_value.second);
        if (use_outlier_rejection)
        {
          outlier_threshold = std::get<1>(key_value.second);
          valid_imu_yaw_rate =
              imu_yaw_rate_buffer.interpolateData(1.0 / solver_->getSamplePeriod(), ref_ts, N_, subsampled_imu_yaw_rate,
                                                  mhe_common::outlier_check_fnc_imu_yaw, outlier_threshold, "bilinear");
        }
        else
        {
          valid_imu_yaw_rate = imu_yaw_rate_filtered_data.interpolateData(1.0 / solver_->getSamplePeriod(), ref_ts, N_,
                                                                          subsampled_imu_yaw_rate);
        }
      }

      // WHEEL ENCODERS
      if (key_value.first == "wheel_encoders")
      {
        DataBuffer<Eigen::Vector4d> wheel_encoders_filtered_data(wheel_encoder_buffer.size());
        mhe_common::filter_wheel_encoders(wheel_encoder_buffer, wheel_encoders_filtered_data,
                                          config_.internal_filter_type);

        use_outlier_rejection = std::get<0>(key_value.second);
        if (use_outlier_rejection)
        {
          outlier_threshold = std::get<1>(key_value.second);
          valid_wheel_encoders = wheel_encoders_filtered_data.interpolateData(
              1.0 / solver_->getSamplePeriod(), ref_ts, N_, subsampled_wheel_encoders, mhe_common::outlier_check_fnc_we,
              outlier_threshold, "bilinear");
        }
        else
        {
          valid_wheel_encoders = wheel_encoders_filtered_data.interpolateData(1.0 / solver_->getSamplePeriod(), ref_ts,
                                                                              N_, subsampled_wheel_encoders);
        }
      }

      // LIGHTHOUSE 1
      if (key_value.first == "lighthouse_" + base_station_id_ + "_1")
      {
        DataBuffer<Eigen::Vector4d> lighthouse_sweep_1_filtered_data(lighthouse_sweep_1_buffer.size());
        mhe_common::filter_lighthouse(lighthouse_sweep_1_buffer, lighthouse_sweep_1_filtered_data,
                                      config_.internal_filter_type);
        use_outlier_rejection = std::get<0>(key_value.second);
        if (use_outlier_rejection)
        {
          outlier_threshold = std::get<1>(key_value.second);
          valid_lighthouse_sweep_1 = lighthouse_sweep_1_buffer.interpolateData(
              1.0 / solver_->getSamplePeriod(), ref_ts, N_, subsampled_lighthouse_sweep_1,
              mhe_common::outlier_check_fnc_lh, outlier_threshold, "bilinear");
        }
        else
        {
          valid_lighthouse_sweep_1 = lighthouse_sweep_1_filtered_data.interpolateData(
              1.0 / solver_->getSamplePeriod(), ref_ts, N_, subsampled_lighthouse_sweep_1);
        }
      }

      // LIGHTHOUSE 2
      if (key_value.first == "lighthouse_" + base_station_id_ + "_2")
      {
        DataBuffer<Eigen::Vector4d> lighthouse_sweep_2_filtered_data(lighthouse_sweep_2_buffer.size());
        mhe_common::filter_lighthouse(lighthouse_sweep_2_buffer, lighthouse_sweep_2_filtered_data,
                                      config_.internal_filter_type);
        use_outlier_rejection = std::get<0>(key_value.second);
        if (use_outlier_rejection)
        {
          outlier_threshold = std::get<1>(key_value.second);
          valid_lighthouse_sweep_2 = lighthouse_sweep_2_buffer.interpolateData(
              1.0 / solver_->getSamplePeriod(), ref_ts, N_, subsampled_lighthouse_sweep_2,
              mhe_common::outlier_check_fnc_lh, outlier_threshold, "bilinear");
        }
        else
        {
          valid_lighthouse_sweep_2 = lighthouse_sweep_2_filtered_data.interpolateData(
              1.0 / solver_->getSamplePeriod(), ref_ts, N_, subsampled_lighthouse_sweep_2);
        }
      }
    }
  }

  // --------------------- NO INTERNAL FILTERING ---------------------
  else
  {
    // Load parameters for outlier rejection out of map
    for (auto key_value : outlier_rejection_params_)  // outlier_rejection_params_ is a vector of pairs: <sensor_key,
                                                      // tuple<use_outlier_rejection, outlier_threshold>>
    {
      // MOCAP
      if (key_value.first == "mocap")
      {
        use_outlier_rejection = std::get<0>(key_value.second);
        if (use_outlier_rejection)
        {
          outlier_threshold = std::get<1>(key_value.second);
          valid_mocap =
              mocap_buffer.interpolateData(1.0 / solver_->getSamplePeriod(), ref_ts, N_, subsampled_mocap,
                                           mhe_common::outlier_check_fnc_mocap, outlier_threshold, "bilinear");
        }
        else
        {
          valid_mocap = mocap_buffer.interpolateData(1.0 / solver_->getSamplePeriod(), ref_ts, N_, subsampled_mocap);
        }
      }

      // IMU
      if (key_value.first == "imu")
      {
        use_outlier_rejection = std::get<0>(key_value.second);
        if (use_outlier_rejection)
        {
          outlier_threshold = std::get<1>(key_value.second);
          valid_imu = imu_buffer.interpolateData(1.0 / solver_->getSamplePeriod(), ref_ts, N_, subsampled_imu,
                                                 mhe_common::outlier_check_fnc_imu, outlier_threshold, "bilinear");
        }
        else
        {
          valid_imu = imu_buffer.interpolateData(1.0 / solver_->getSamplePeriod(), ref_ts, N_, subsampled_imu);
        }
      }

      // IMU YAW RATE
      if (key_value.first == "imu_yaw_rate")
      {
        use_outlier_rejection = std::get<0>(key_value.second);
        if (use_outlier_rejection)
        {
          outlier_threshold = std::get<1>(key_value.second);
          valid_imu_yaw_rate =
              imu_yaw_rate_buffer.interpolateData(1.0 / solver_->getSamplePeriod(), ref_ts, N_, subsampled_imu_yaw_rate,
                                                  mhe_common::outlier_check_fnc_imu_yaw, outlier_threshold, "bilinear");
        }
        else
        {
          valid_imu_yaw_rate = imu_yaw_rate_buffer.interpolateData(1.0 / solver_->getSamplePeriod(), ref_ts, N_,
                                                                   subsampled_imu_yaw_rate);
        }
      }

      // WHEEL ENCODERS
      if (key_value.first == "wheel_encoders")
      {
        use_outlier_rejection = std::get<0>(key_value.second);
        if (use_outlier_rejection)
        {
          outlier_threshold = std::get<1>(key_value.second);
          valid_wheel_encoders = wheel_encoder_buffer.interpolateData(
              1.0 / solver_->getSamplePeriod(), ref_ts, N_, subsampled_wheel_encoders, mhe_common::outlier_check_fnc_we,
              outlier_threshold, "bilinear");
        }
        else
        {
          valid_wheel_encoders = wheel_encoder_buffer.interpolateData(1.0 / solver_->getSamplePeriod(), ref_ts, N_,
                                                                      subsampled_wheel_encoders);
        }
      }

      // LIGHTHOUSE 1
      if (key_value.first == "lighthouse_" + base_station_id_ + "_1")
      {
        use_outlier_rejection = std::get<0>(key_value.second);
        if (use_outlier_rejection)
        {
          outlier_threshold = std::get<1>(key_value.second);
          valid_lighthouse_sweep_1 = lighthouse_sweep_1_buffer.interpolateData(
              1.0 / solver_->getSamplePeriod(), ref_ts, N_, subsampled_lighthouse_sweep_1,
              mhe_common::outlier_check_fnc_lh, outlier_threshold, "bilinear");
        }
        else
        {
          valid_lighthouse_sweep_1 = lighthouse_sweep_1_buffer.interpolateData(1.0 / solver_->getSamplePeriod(), ref_ts,
                                                                               N_, subsampled_lighthouse_sweep_1);
        }
      }

      // LIGHTHOUSE 2
      if (key_value.first == "lighthouse_" + base_station_id_ + "_2")
      {
        use_outlier_rejection = std::get<0>(key_value.second);
        if (use_outlier_rejection)
        {
          outlier_threshold = std::get<1>(key_value.second);
          valid_lighthouse_sweep_2 = lighthouse_sweep_2_buffer.interpolateData(
              1.0 / solver_->getSamplePeriod(), ref_ts, N_, subsampled_lighthouse_sweep_2,
              mhe_common::outlier_check_fnc_lh, outlier_threshold, "bilinear");
        }
        else
        {
          valid_lighthouse_sweep_2 = lighthouse_sweep_2_buffer.interpolateData(1.0 / solver_->getSamplePeriod(), ref_ts,
                                                                               N_, subsampled_lighthouse_sweep_2);
        }
      }
    }
  }

  input_buffer.interpolateDataNonuniform(horizon_shooting_ts_.data(), ref_ts, N_, subsampled_inputs);
  states_buffer.interpolateDataNonuniform(horizon_shooting_ts_.data(), ref_ts, N_, subsampled_states);

  Eigen::Matrix<double, pacejka_state::NX, pacejka_state::NX> P = config_.P;
  if (config_.use_internal_estimator)
  {
    // Try to cast internal estimator to ekf.
    // If this is possible we want to load the state estimate covariance matrix P from the ekf.
    // Otherwise we will load it from the config.
    auto ekf_ptr =
        std::dynamic_pointer_cast<crs_estimators::kalman::DiscreteEKF<pacejka_state, pacejka_input>>(estimator_);
    if (ekf_ptr)
    {
      P = ekf_ptr->getPosteriorCovariance();
    }
  }

  mhe_solvers::pacejka_solvers::cost_values mhe_costs = { P,
                                                          config_.Q,
                                                          config_.R_mocap,
                                                          config_.R_imu,
                                                          config_.R_imu_yaw_rate,
                                                          config_.R_wheel_encoders,
                                                          config_.R_lighthouse,
                                                          config_.eta };

  if (!is_initialized)
  {
    initializing = true;
    initialize(subsampled_states[0]);  // This is the initial state at this point (set to ekf estimate)
    initializing = false;
    is_initialized = true;
  }

  // Cast discrete model pointer to pacejka model pointer type
  auto discrete_model = std::static_pointer_cast<crs_models::pacejka_model::DiscretePacejkaModel>(MHE::discrete_model);

  // Cast solver to pacejka acados solver
  auto pacejka_solver = std::static_pointer_cast<mhe_solvers::pacejka_solvers::AcadosPacejkaMheSolver>(solver_);

  // Only for visualization / debugging
  for (size_t i = 0; i < subsampled_mocap.size(); i++)
  {
    if (!valid_mocap[i])
      subsampled_mocap[i] = 0 * subsampled_mocap[i];
  }

  // Setup parameter for initial solve
  for (int stage = 0; stage < N_; stage++)
  {
    last_solution.states_[stage * solver_->getStateDimension() + 3] =
        std::max(0.1, last_solution.states_[stage * solver_->getStateDimension() + 3]);
    // Set initial input (nothing to do, its zero)
  }
  // Run solver
  for (int current_stage = 0; current_stage < N_; current_stage++)
  {
    // next stage points to current_stage + 1. If current_stage is at end of horizon, next stage directy points to
    // current stage i.e. current_stage = 2 -> next_stage = 3, current_stage = 29 -> next_stage = 29, assuming horizon
    // of 30
    int next_stage = current_stage + (current_stage != N_ - 1);

    // Define reference at this stage
    mhe_solvers::pacejka_solvers::references references = {
      subsampled_states[current_stage],              // ekf / state buffer states
      subsampled_mocap[current_stage],               // mocap
      subsampled_imu[current_stage],                 // imu
      subsampled_imu_yaw_rate[current_stage],        // imu yaw rate
      subsampled_wheel_encoders[current_stage],      // wheel encoders
      subsampled_lighthouse_sweep_1[current_stage],  // lighthouse_sweep_1
      subsampled_lighthouse_sweep_2[current_stage],  // lighthouse_sweep_2
      Eigen::Vector2d(subsampled_inputs[current_stage].torque, subsampled_inputs[current_stage].steer),  // input
      valid_mocap[current_stage],                                                                        // valid_mocap
      valid_imu[current_stage],                                                                          // valid_imu
      valid_imu_yaw_rate[current_stage],        // valid_imu_yaw_rate
      valid_wheel_encoders[current_stage],      // valid_wheel_encoders
      valid_lighthouse_sweep_1[current_stage],  // valid_lighthouse_sweep_1
      valid_lighthouse_sweep_2[current_stage],  // valid_lighthouse_sweep_2
    };

    if (valid_lighthouse_sweep_1[current_stage] && valid_lighthouse_sweep_2[current_stage])
    {
      auto sensor_model_1 = key_to_sensor_model_.find("lighthouse_" + base_station_id_ + "_1");
      auto sensor_model_2 = key_to_sensor_model_.find("lighthouse_" + base_station_id_ + "_2");
      auto lighthouse_sensor_model_1 =
          std::dynamic_pointer_cast<crs_sensor_models::pacejka_sensor_models::LighthouseSensorModel>(
              sensor_model_1->second);
      auto lighthouse_sensor_model_2 =
          std::dynamic_pointer_cast<crs_sensor_models::pacejka_sensor_models::LighthouseSensorModel>(
              sensor_model_2->second);

      std::tuple<Eigen::Matrix<double, 2, 4>, Eigen::Vector3d, Eigen::Matrix3d, double> lighthouse_params_1 =
          lighthouse_sensor_model_1->getLighthouseParams();
      std::tuple<Eigen::Matrix<double, 2, 4>, Eigen::Vector3d, Eigen::Matrix3d, double> lighthouse_params_2 =
          lighthouse_sensor_model_2->getLighthouseParams();

      solver_->updateParams(current_stage,
                            discrete_model->getParams(),  // Model Dynamics
                            mhe_costs,                    // Costs
                            references,                   // Tracking point
                            lighthouse_params_1, lighthouse_params_2);
    }
    else
    {
      solver_->updateParams(current_stage,
                            discrete_model->getParams(),  // Model Dynamics
                            mhe_costs,                    // Costs
                            references,                   // Tracking point
                            {}, {});
    }

    solver_->setStateInitialGuess(next_stage, &last_solution.states_[current_stage * solver_->getStateDimension()]);
  }

  try
  {
    int exit_flag = solver_->solve(&last_solution.states_[0], &last_solution.inputs_[0]);
    total_num_solves += 1;

    if ((exit_flag > 0 && config_.solver_type == "ACADOS") || (exit_flag < 1 && config_.solver_type == "FORCES"))
    {
      num_solver_errors += 1;
      // ---------- ADD DIAGNOSTICS ----------
      if (MHE::log_diagnostic_data_)
      {
        // add number of solver errors and total number of solves
        std::vector<float> solver_data = { float(num_solver_errors), float(total_num_solves) };
        std::string solver_data_name = "MHE/solver";
        BaseEstimator<pacejka_state>::logDiagnosticData(solver_data_name, solver_data);
      }

      // If the solver fails, we can recover the internal estimate
      if (config_.use_internal_estimator && config_.recover_internal_estimate_if_solver_failure)
      {
        for (int stage = 0; stage < solver_->getHorizonLength(); stage++)
        {
          // Set initial guess to ekf estimation
          last_solution.states_[stage * solver_->getStateDimension() + 0] = subsampled_states[stage].pos_x;
          last_solution.states_[stage * solver_->getStateDimension() + 1] = subsampled_states[stage].pos_y;
          last_solution.states_[stage * solver_->getStateDimension() + 2] = subsampled_states[stage].yaw;
          last_solution.states_[stage * solver_->getStateDimension() + 3] =
              std::max(0.1, subsampled_states[stage].vel_x);
          last_solution.states_[stage * solver_->getStateDimension() + 4] = subsampled_states[stage].vel_y;
          last_solution.states_[stage * solver_->getStateDimension() + 5] = subsampled_states[stage].yaw_rate;

          // Set initial input
          // nothing to do, its zero
        }
      }
    }
  }
  catch (std::exception& e)
  {
    std::cerr << "Exception caught while solving MHE : " << e.what() << std::endl;
  }

  for (int current_stage = 0; current_stage < N_; current_stage++)
  {
    last_solution.trajectory_estimate_[current_stage] = Eigen::Vector3d(
        last_solution.states_[current_stage * solver_->getStateDimension() + mhe_solvers::pacejka_solvers::X],
        last_solution.states_[current_stage * solver_->getStateDimension() + mhe_solvers::pacejka_solvers::Y],
        last_solution.states_[current_stage * solver_->getStateDimension() + mhe_solvers::pacejka_solvers::YAW]);
  }

  if (!config_.use_internal_estimator)
  {
    // Add last state of the MHE to state buffer, used in next time step to compute difference in state for cost.
    states_buffer.clear();

    for (int i = 0; i < N_; i++)
    {
      states_buffer.addData(getStateFromSolution(i), timestamp - (N_ - i + 1) * solver_->getSamplePeriod());
    }
  }

  if (config_.print_solve_time)
  {
    auto execution_time =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - timestamp_now).count();
    std::cout << "MHE execution time: " << execution_time << " ms" << std::endl;
  }

  // Moving horizion. Last state is the current state estimate. Everything else is past trajectory.
  return getStateFromSolution(N_ - 1);
}  // namespace mhe

/**
 * @brief Returns the config
 *
 * @return mhe_config&
 */
pacejka_mhe_config& Pacejka_MHE::getConfig()
{
  return config_;
}

void Pacejka_MHE::setConfig(pacejka_mhe_config config)
{
  config_ = config;
}

}  // namespace mhe
}  // namespace crs_estimators
