#ifndef KINEMATIC_SENSOR_MODEL_VICON_SENSOR_MODEL_H
#define KINEMATIC_SENSOR_MODEL_VICON_SENSOR_MODEL_H

#include <kinematic_model/kinematic_car_input.h>
#include <kinematic_model/kinematic_car_state.h>
#include <sensor_models/sensor_model.h>

namespace crs_sensor_models
{
namespace kinematic_sensor_models
{
class ViconSensorModel : public SensorModel<crs_models::kinematic_model::kinematic_car_state,
                                            crs_models::kinematic_model::kinematic_car_input>
{
public:
  // Option 1: Create an object with no process noise covariance matrix Q defined. Will use identity.
  ViconSensorModel() : ViconSensorModel(Eigen::Matrix3d::Identity())  // Measurement dimension is three
  {
    std::cout << "[WARNING] No R Matrix specified for ViconSensorModel. Using identity Matrix! " << std::endl;
  }

  // Option 2: Create an object and specify the process noise covariance matrix Q yourself.
  ViconSensorModel(const Eigen::Matrix3d& R);  // Measurement dimension is three

  static const std::string SENSOR_KEY;
};
}  // namespace kinematic_sensor_models
}  // namespace crs_sensor_models
#endif
