#ifndef CUSTOM_SENSOR_MODE_TEMPLATE_IMU_SENSOR_MODEL_H
#define CUSTOM_SENSOR_MODE_TEMPLATE_IMU_SENSOR_MODEL_H

#include <sensor_models/sensor_model.h>

#include <custom_model_template/custom_input.h>
#include <custom_model_template/custom_state.h>
#include <custom_model_template/custom_model_continuous.h>

// ==================== TODO ====================
// - Change includes to the ones of your model
// - Change namespace from custom_sensor_model to something sensible
// - Change state and input types in this file
// - Also make sure you update the types in the comments

namespace crs_sensor_models
{
namespace custom_sensor_models
{

/**
 * @brief Creates a sensor model that is used to measure linear accelarations and angular velocities
 *
 */
class ImuSensorModel
  : public SensorModel<crs_models::custom_model::custom_state, crs_models::custom_model::custom_input>
{
public:
  // Option 1: Create an object with no process noise covariance matrix Q defined. Will use identity.
  /**
   * @brief Construct a new Imu Sensor Model.
   * @note Here, we show how to use the continuous dynamic model if the measurement is part of the state derivative.
   *
   * @param custom_cont the continuous model
   */
  ImuSensorModel(const std::shared_ptr<crs_models::custom_model::ContinuousCustomModel> custom_cont)
    : ImuSensorModel(custom_cont, Eigen::Matrix<double, crs_models::custom_model::custom_state::NX,
                                                crs_models::custom_model::custom_state::NX>::Identity())
  {
    std::cout << "[WARNING] No R Matrix specified for ImuSensorModel. Using identity Matrix! " << std::endl;
  }

  // Option 2: Create an object and specify the process noise covariance matrix Q yourself.
  /**
   * @brief Construct a new Imu Sensor Model. Note that the accelerations are not part of the pacejka state and
   * therefore the continuous model is needed
   *
   * @param pacejka_cont the continuous model
   * @param R measurement covariance Matrix
   */
  ImuSensorModel(const std::shared_ptr<crs_models::custom_model::ContinuousCustomModel> custom_cont,
                 const Eigen::Matrix<double, crs_models::custom_model::custom_state::NX,
                                     crs_models::custom_model::custom_state::NX>& R);

  static const std::string SENSOR_KEY;
};
}  // namespace custom_sensor_models
}  // namespace crs_sensor_models
#endif
