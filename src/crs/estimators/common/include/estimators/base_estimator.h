#ifndef ESTIMATORS_BASE_ESTIMATOR_H
#define ESTIMATORS_BASE_ESTIMATOR_H

#include <sensor_models/sensor_measurement.h>
#include <memory>
#include <mutex>

namespace crs_estimators
{
template <typename StateType>
class BaseEstimator
{
public:
  /**
   * @brief Construct a new Base Estimator object
   *
   */
  BaseEstimator(){};

  virtual StateType getStateEstimate(const double timestamp [[maybe_unused]])
  {
    // Default implementation, ignore timestamp
    return getStateEstimate();
  }

  /**
   * Returns the currently best state estimate
   */
  virtual StateType getStateEstimate() = 0;

  /**
   * @brief Get the last valid timestamp (i.e. timestamp for which the state estimate is valid)
   *
   * @return double
   */
  virtual double getLastValidTs() const = 0;

  /**
   * @brief Function that gets called when new measurements from sensor models arrive
   *
   */
  virtual void measurementCallback(const crs_sensor_models::measurement measurement) = 0;

  /* @brief Resets the state estimate to the given state
   *
   * @param state The state to which the state estimate should be reset
   */
  virtual void resetStateEstimate(const StateType state) = 0;

  /**
   * @brief Returns diagnostic data of the estimator. This is a
   * map of string to vector of floats. The string is the name of the
   * diagnostic data and the vector of floats is the data itself.
   *
   * @return std::map<std::string, std::vector<float>>
   */
  virtual std::vector<std::pair<std::string, std::vector<float>>> getDiagnosticData()
  {
    std::lock_guard<std::mutex> lock(BaseEstimator<StateType>::diagnostic_data_mutex_);
    auto data = diagnostic_data_;
    diagnostic_data_.clear();
    return data;
  }

protected:
  /**
   * @brief Log diagnostic data of the estimator.
   * @note Only use this function to log data, as it ensures thread safety.
   */
  inline void logDiagnosticData(const std::string& name, const std::vector<float>& data)
  {
    std::lock_guard<std::mutex> lock(BaseEstimator<StateType>::diagnostic_data_mutex_);
    diagnostic_data_.push_back({ name, data });
  }

private:
  /** Mutex to protect @see diagnostic_data_. Private by design so derived classes need accessors.  */
  std::mutex diagnostic_data_mutex_;
  std::vector<std::pair<std::string, std::vector<float>>> diagnostic_data_;
};

}  // namespace crs_estimators
#endif /* ESTIMATORS_BASE_ESTIMATOR_H */
