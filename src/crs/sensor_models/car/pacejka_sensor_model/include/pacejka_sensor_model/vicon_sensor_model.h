#ifndef PACEJKA_SENSOR_MODEL_VICON_SENSOR_MODEL_H
#define PACEJKA_SENSOR_MODEL_VICON_SENSOR_MODEL_H

#include <pacejka_model/pacejka_car_input.h>
#include <pacejka_model/pacejka_car_state.h>
#include <sensor_models/sensor_model.h>

namespace crs_sensor_models
{
namespace pacejka_sensor_models
{
class ViconSensorModel
  : public SensorModel<crs_models::pacejka_model::pacejka_car_state, crs_models::pacejka_model::pacejka_car_input>
{
public:
  // Option 1: Create an object with no process noise covariance matrix Q defined. Will use identity.
  ViconSensorModel() : ViconSensorModel(Eigen::Matrix3d::Identity())  // Measurement dimension is three
  {
    std::cout << "[WARNING] No R Matrix specified for ViconSensorModel. Using identity Matrix! " << std::endl;
  }

  // Option 2: Create an object and specify the measurement noise covariance matrix R yourself.
  ViconSensorModel(const Eigen::Matrix3d& R);  // Measurement dimension is three

  static const std::string SENSOR_KEY;
};
}  // namespace pacejka_sensor_models
}  // namespace crs_sensor_models
#endif
