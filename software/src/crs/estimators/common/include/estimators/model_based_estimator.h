#ifndef ESTIMATORS_MODEL_BASED_ESTIMATOR_H
#define ESTIMATORS_MODEL_BASED_ESTIMATOR_H

#include <estimators/base_estimator.h>
#include <sensor_models/sensor_model.h>

namespace crs_estimators
{
template <typename StateType, typename InputType>
class ModelBasedEstimator : public BaseEstimator<StateType>
{
public:
  /**
   * @brief Construct a new Base Estimator object
   *
   */
  ModelBasedEstimator(){};

  /**
   * @brief Function that gets called when a input was applied
   */
  virtual void controlInputCallback(const InputType input, const double timestamp) = 0;

  /**
   * @brief Returns last valid input that was applied. Can be random if no input has been applied yet.
   */
  virtual InputType getLastInput() const = 0;

  /**
   * @brief Adds a sensor model to the map (key and sensor_model)
   */
  virtual void addSensorModel(std::string sensor_key,
                              std::shared_ptr<crs_sensor_models::SensorModel<StateType, InputType>> sensors_model) = 0;

  /**
   * Returns the currently best state estimate
   */
  virtual StateType getStateEstimate() = 0;
};
}  // namespace crs_estimators
#endif /* ESTIMATORS_MODEL_BASED_ESTIMATOR_H */
