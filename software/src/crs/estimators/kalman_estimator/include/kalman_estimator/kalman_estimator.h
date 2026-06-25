#ifndef SRC_CRS_ESTIMATORS_KALMAN_ESTIMATOR_INCLUDE_KALMAN_ESTIMATOR
#define SRC_CRS_ESTIMATORS_KALMAN_ESTIMATOR_INCLUDE_KALMAN_ESTIMATOR

#include <memory>

#include <dynamic_models/discrete_dynamic_model.h>
#include <dynamic_models/utils/data_conversion.h>
#include <sensor_models/sensor_measurement.h>
#include <sensor_models/sensor_model.h>
#include <mutex>

#include <Eigen/Dense>
#include <stdexcept>

#include <estimators/model_based_estimator.h>
#include "car_kalman_parameters.h"

namespace crs_estimators
{
namespace kalman
{
template <typename StateType, typename InputType>
class KalmanEstimator : public ModelBasedEstimator<StateType, InputType>
{
public:
  // Constructor with input
  KalmanEstimator(StateType initial_state, InputType initial_input,
                  const std::vector<std::pair<std::string, std::tuple<bool, std::string, double, int>>>
                      outlier_rejection_params = {},
                  bool log_diagnostic_data = false)
    : x_prior_(initial_state)
    , x_posterior_(initial_state)
    , previous_input_(initial_input)
    , best_state_(initial_state)
    , outlier_rejection_params_(outlier_rejection_params)
    , log_diagnostic_data_(log_diagnostic_data){};

  // Constructor
  KalmanEstimator(StateType initial_state,
                  const std::vector<std::pair<std::string, std::tuple<bool, std::string, double, int>>>
                      outlier_rejection_params = {},
                  bool log_diagnostic_data = false)
    : x_prior_(initial_state)
    , x_posterior_(initial_state)
    , best_state_(initial_state)
    , outlier_rejection_params_(outlier_rejection_params)
    , log_diagnostic_data_(log_diagnostic_data){};

  /**
   * @brief Function that gets called whenever a new input is applied.
   *        This internally calls the prediction step of the EKF with the older input and total duration it was applied
   * for.
   *
   * @param input e.g. control input
   * @param timestamp current time in s when this input was applied
   */
  void controlInputCallback(const InputType input, const double timestamp) override
  {
    // Make sure only one thread can access this function at a time
    std::lock_guard<std::mutex> lock(callback_mutex_);

    if (last_valid_ts_ != -1)  // There was an older input available which was received at time last_valid_ts_ / priors
                               // exist
    {
      double timestep = timestamp - last_valid_ts_;  // how long the previous input was applied (= how long we need to
                                                     // propogate the state)
      if (timestep < 0)
        return;
      predict(previous_input_, timestep);  // propogate the state through model / model prediction of state
    }

    previous_input_ = input;
    last_valid_ts_ = timestamp;
  }

  /**
   * @brief Function that gets called whenever a new measurement is received.
   *        This internally calls the measurement update step of the EKF.
   *        The predict step is called with the previous input,
   *        in order to make sure the prior state is propogated forward to the same timestamp as the received
   *        measurement.
   *
   * @param measurement
   */
  void measurementCallback(const crs_sensor_models::measurement measurement) override
  {
    // Update prior state to current time (time the measurement is available)
    controlInputCallback(previous_input_, measurement.timestamp);
    std::lock_guard<std::mutex> lock(callback_mutex_);
    // Apply posterior update
    measurementUpdate(measurement);
  }

  /**
   * @brief This is the prediction step of the Kalman Filter. The state is predicated based on the model.
   *        This function only updates the prior state of the Kalman Filter
   *
   * @param input e.g. control input
   * @param timestep how long to apply the model for (how long to integrate for)
   */
  virtual void predict(const InputType& input, const double timestep) = 0;

  /**
   * @brief This is the measurement update step of the Kalman Filter.
   *        The state prediction based on the model is updated using measurement inofrmation.
   *        This function updates the prior and posterior state of the Kalman Filter
   *
   * @param data measurement information, e.g. type of sensor, measurement data
   */
  virtual void measurementUpdate(const crs_sensor_models::measurement& data) = 0;

  StateType getStateEstimate() override
  {
    // return the current state estimate
    return best_state_;
  }

  /**
   * @brief Resets the state estimate to the given state
   *
   * @param state The state to which the state estimate should be reset
   */
  void resetStateEstimate(const StateType state) override
  {
    x_prior_ = state;
    x_posterior_ = state;
    best_state_ = state;
  }

  InputType getLastInput() const override
  {
    return previous_input_;
  }

  /**
   * @brief Adds a sensor model to the map (key and sensor_model)
   *
   * @param sensor_key
   * @param sensors_model
   */
  virtual void addSensorModel(std::string sensor_key,
                              std::shared_ptr<crs_sensor_models::SensorModel<StateType, InputType>> sensors_model) = 0;

  double getLastValidTs() const override
  {
    return last_valid_ts_;
  }

  /**
   * @brief Returns the posterior state estimate covariance matrix of the EKF
   *
   * @return P_Posterior
   */
  virtual Eigen::Matrix<double, StateType::NX, StateType::NX> getPosteriorCovariance() const = 0;

protected:
  StateType x_prior_;
  StateType x_posterior_;

  StateType best_state_;
  InputType previous_input_;

  // use_outlier_rejection, outlier_rejection_type, outlier_threshold, max_consecutive_outliers
  std::vector<std::pair<std::string, std::tuple<bool, std::string, double, int>>> outlier_rejection_params_;
  bool log_diagnostic_data_;

private:
  double last_valid_ts_ = -1;
  std::mutex callback_mutex_;
};
}  // namespace kalman
}  // namespace crs_estimators
#endif /* SRC_CRS_ESTIMATORS_KALMAN_ESTIMATOR_INCLUDE_KALMAN_ESTIMATOR */
