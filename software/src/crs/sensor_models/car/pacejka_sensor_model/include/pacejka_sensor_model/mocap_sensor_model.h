#ifndef PACEJKA_SENSOR_MODEL_MOCAP_SENSOR_MODEL_H
#define PACEJKA_SENSOR_MODEL_MOCAP_SENSOR_MODEL_H

#include <pacejka_model/pacejka_car_input.h>
#include <pacejka_model/pacejka_car_state.h>
#include <sensor_models/sensor_model.h>

namespace crs_sensor_models
{
namespace pacejka_sensor_models
{
class MocapSensorModel
  : public SensorModel<crs_models::pacejka_model::pacejka_car_state, crs_models::pacejka_model::pacejka_car_input>
{
public:
  // Option 1: Create an object with no process noise covariance matrix Q defined. Will use identity.
  MocapSensorModel() : MocapSensorModel(Eigen::Matrix3d::Identity())  // Measurement dimension is three
  {
    std::cout << "[WARNING] No R Matrix specified for MocapSensorModel. Using identity Matrix! " << std::endl;
  }

  // Option 2: Create an object and specify the measurement noise covariance matrix R yourself.
  MocapSensorModel(const Eigen::Matrix3d& R);  // Measurement dimension is three

  static const std::string SENSOR_KEY;
};
}  // namespace pacejka_sensor_models
}  // namespace crs_sensor_models
#endif
