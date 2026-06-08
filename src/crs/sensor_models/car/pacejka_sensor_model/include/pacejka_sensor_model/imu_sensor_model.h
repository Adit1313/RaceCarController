#ifndef PACEJKA_SENSOR_MODEL_IMU_SENSOR_MODEL_H
#define PACEJKA_SENSOR_MODEL_IMU_SENSOR_MODEL_H

#include <sensor_models/sensor_model.h>

#include <pacejka_model/pacejka_car_input.h>
#include <pacejka_model/pacejka_car_state.h>
#include <pacejka_model/pacejka_continuous.h>

namespace crs_sensor_models
{
namespace pacejka_sensor_models
{
/**
 * @brief Creates a sensor model that is used to measure linear accelarations and angular velocities
 *
 */
class ImuSensorModel
  : public SensorModel<crs_models::pacejka_model::pacejka_car_state, crs_models::pacejka_model::pacejka_car_input>
{
public:
  // Option 1: Create an object with no process noise covariance matrix Q defined. Will use identity.
  /**
   * @brief Construct a new Imu Sensor Model. Note that the accelerations are not part of the pacejka state and
   * therefore the continuous model is needed
   *
   * @param pacejka_cont the continuous model
   */
  ImuSensorModel(const std::shared_ptr<crs_models::pacejka_model::ContinuousPacejkaModel> pacejka_cont)
    : ImuSensorModel(pacejka_cont, Eigen::Matrix3d::Identity())  // Measurement dimension is three
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
  ImuSensorModel(const std::shared_ptr<crs_models::pacejka_model::ContinuousPacejkaModel> pacejka_cont,
                 const Eigen::Matrix3d& R);  // Measurement dimension is three

  static const std::string SENSOR_KEY;
};
}  // namespace pacejka_sensor_models
}  // namespace crs_sensor_models
#endif
