#ifndef PACEJKA_SENSOR_MODEL_MOCAP_WITH_IMU_BIAS_SENSOR_MODEL_H
#define PACEJKA_SENSOR_MODEL_MOCAP_WITH_IMU_BIAS_SENSOR_MODEL_H

// #include <pacejka_model/pacejka_car_input.h>
// #include <pacejka_model/pacejka_car_state.h>
#include <sensor_models/sensor_model.h>
#include <stacked_model/pacejka_imu_bias_car_state.h>
#include <stacked_model/pacejka_imu_bias_car_input.h>

namespace crs_sensor_models
{
namespace pacejka_sensor_models
{
class MocapWithImuBiasSensorModel : public SensorModel<crs_models::stacked_model::pacejka_imu_bias_car_state,
                                                       crs_models::stacked_model::pacejka_imu_bias_car_input>
{
public:
  // Option 1: Create an object with no process noise covariance matrix R defined. Will use identity.
  MocapWithImuBiasSensorModel()
    : MocapWithImuBiasSensorModel(Eigen::Matrix3d::Identity())  // Measurement dimension is three
  {
    std::cout << "[WARNING] No R Matrix specified for MocapWithImuBiasSensorModel. Using identity Matrix! "
              << std::endl;
  }

  // Option 2: Create an object and specify the process noise covariance matrix R yourself.
  MocapWithImuBiasSensorModel(const Eigen::Matrix3d& R);  // Measurement dimension is three

  static const std::string SENSOR_KEY;
};
}  // namespace pacejka_sensor_models
}  // namespace crs_sensor_models
#endif
