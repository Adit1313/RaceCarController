#ifndef PACEJKA_SENSOR_MODEL_IMU_WITH_BIAS_SENSOR_MODEL_H
#define PACEJKA_SENSOR_MODEL_IMU_WITH_BIAS_SENSOR_MODEL_H

#include <sensor_models/sensor_model.h>

#include <pacejka_model/pacejka_continuous.h>

#include <imu_bias/imu_bias_continuous.h>
#include <stacked_model/pacejka_imu_bias_car_state.h>
#include <stacked_model/pacejka_imu_bias_car_input.h>
#include <stacked_model/stacked_two_models_continuous.h>

using StackedPacejkaImuModel = crs_models::stacked_model::ContinuousTwoStackedModels<
    crs_models::stacked_model::pacejka_imu_bias_car_state, crs_models::stacked_model::pacejka_imu_bias_car_input,
    crs_models::pacejka_model::ContinuousPacejkaModel, crs_models::imu_bias::ContinuousImuBias>;
namespace crs_sensor_models
{
namespace pacejka_sensor_models
{
/**
 * @brief Creates a sensor model that is used to measure linear accelarations and angular velocities
 *
 */
class ImuWithBiasSensorModel : public SensorModel<crs_models::stacked_model::pacejka_imu_bias_car_state,
                                                  crs_models::stacked_model::pacejka_imu_bias_car_input>
{
public:
  // Option 1: Create an object with no process noise covariance matrix Q defined. Will use identity.
  /**
   * @brief Construct a new Imu Sensor Model. Note that the accelerations are not part of the pacejka state and
   * therefore the continuous model is needed
   *
   * @param pacejka_cont the continuous model
   */
  ImuWithBiasSensorModel(const std::shared_ptr<StackedPacejkaImuModel> pacejka_cont)
    : ImuWithBiasSensorModel(pacejka_cont, Eigen::Matrix3d::Identity())  // Measurement dimension is three
  {
    std::cout << "[WARNING] No R Matrix specified for ImuWithBiasSensorModel. Using identity Matrix! " << std::endl;
  }

  // Option 2: Create an object and specify the process noise covariance matrix Q yourself.
  /**
   * @brief Construct a new Imu Sensor Model. Note that the accelerations are not part of the pacejka state and
   * therefore the continuous model is needed
   *
   * @param pacejka_cont the continuous model
   * @param R measurement covariance Matrix
   */
  ImuWithBiasSensorModel(const std::shared_ptr<StackedPacejkaImuModel> pacejka_cont,
                         const Eigen::Matrix3d& R);  // Measurement dimension is three

  static const std::string SENSOR_KEY;
};
}  // namespace pacejka_sensor_models
}  // namespace crs_sensor_models
#endif
