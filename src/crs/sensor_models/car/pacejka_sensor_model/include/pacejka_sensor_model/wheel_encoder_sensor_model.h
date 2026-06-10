#ifndef PACEJKA_SENSOR_MODEL_WHEEL_ENCODER_SENSOR_MODEL_H
#define PACEJKA_SENSOR_MODEL_WHEEL_ENCODER_SENSOR_MODEL_H

#include <pacejka_model/pacejka_car_input.h>
#include <pacejka_model/pacejka_car_state.h>
#include <sensor_models/sensor_model.h>

namespace crs_sensor_models
{
namespace pacejka_sensor_models
{
class WheelEncoderSensorModel
  : public SensorModel<crs_models::pacejka_model::pacejka_car_state, crs_models::pacejka_model::pacejka_car_input>
{
public:
  // Option 1: Create an object with no process noise covariance matrix Q defined. Will use identity.
  WheelEncoderSensorModel(double wheel_radius, double lf, double car_width)
    : WheelEncoderSensorModel(wheel_radius, lf, car_width,
                              Eigen::Matrix4d::Identity())  // Measurement dimension is three
  {
    std::cout << "[WARNING] No R Matrix specified for WheelEncoderSensorModel. Using identity Matrix! " << std::endl;
  }

  // Option 2: Create an object and specify the measurement noise covariance matrix R yourself.
  WheelEncoderSensorModel(double wheel_radius, double lf, double car_width,
                          const Eigen::Matrix4d& R);  // Measurement dimension is three

  static const std::string SENSOR_KEY;
};
}  // namespace pacejka_sensor_models
}  // namespace crs_sensor_models
#endif
