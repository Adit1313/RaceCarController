#ifndef ROCKET_6_DOF_SENSOR_MODEL_IMU_SENSOR_MODEL_H
#define ROCKET_6_DOF_SENSOR_MODEL_IMU_SENSOR_MODEL_H

#include <sensor_models/sensor_model.h>

#include <rocket_6_dof_model/rocket_6_dof_input.h>
#include <rocket_6_dof_model/rocket_6_dof_state.h>
#include <rocket_6_dof_model/rocket_6_dof_continuous.h>

namespace crs_sensor_models
{
namespace rocket_6_dof_sensor_models
{
/**
 * @brief Creates a sensor model that is used to measure linear accelarations and angular velocities
 *
 */
class ImuSensorModel : public SensorModel<crs_models::rocket_6_dof_model::rocket_6_dof_state,
                                          crs_models::rocket_6_dof_model::rocket_6_dof_input>
{
public:
  // Option 1: Create an object with no process noise covariance matrix Q defined. Will use identity.
  /**
   * @brief Construct a new Imu Sensor Model. Note that the accelerations are not part of the rocket state and
   * therefore the continuous model is needed
   *
   * @param rocket_cont the continuous model
   */
  ImuSensorModel(const std::shared_ptr<crs_models::rocket_6_dof_model::ContinuousRocket6DofModel> rocket_cont)
    : ImuSensorModel(rocket_cont, Eigen::Matrix<double, 6, 6>::Identity())  // Measurement dimension is six
  {
    std::cout << "[WARNING] No R Matrix specified for ImuSensorModel. Using identity Matrix! " << std::endl;
  }

  // Option 2: Create an object and specify the process noise covariance matrix Q yourself.
  /**
   * @brief Construct a new Imu Sensor Model. Note that the accelerations are not part of the rocket state and
   * therefore the continuous model is needed
   *
   * @param rocket_cont the continuous model
   * @param R measurement covariance Matrix
   */
  ImuSensorModel(const std::shared_ptr<crs_models::rocket_6_dof_model::ContinuousRocket6DofModel> rocket_cont,
                 const Eigen::Matrix<double, 6, 6>& R);  // Measurement dimension is 6

  static const std::string SENSOR_KEY;
};
}  // namespace rocket_6_dof_sensor_models
}  // namespace crs_sensor_models
#endif
