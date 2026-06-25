#ifndef CUSTOM_SENSOR_MODEL_TEMPLATE_MOCAP_SENSOR_MODEL_H
#define CUSTOM_SENSOR_MODEL_TEMPLATE_MOCAP_SENSOR_MODEL_H

#include <custom_model_template/custom_input.h>
#include <custom_model_template/custom_state.h>
#include <sensor_models/sensor_model.h>

// ==================== TODO ====================
// - Change includes to the ones of your model
// - Change namespace from custom_sensor_model to something sensible
// - Change state and input types in this file

namespace crs_sensor_models
{
namespace custom_sensor_models
{
class MocapSensorModel
  : public SensorModel<crs_models::custom_model::custom_state, crs_models::custom_model::custom_input>
{
public:
  // Option 1: Create an object with no process noise covariance matrix Q defined. Will use identity.
  MocapSensorModel()
    : MocapSensorModel(Eigen::Matrix<double, crs_models::custom_model::custom_state::NX,
                                     crs_models::custom_model::custom_state::NX>::Identity())
  {
    std::cout << "[WARNING] No R Matrix specified for MocapSensorModel. Using identity Matrix! " << std::endl;
  }

  // Option 2: Create an object and specify the process noise covariance matrix Q yourself.
  MocapSensorModel(const Eigen::Matrix<double, crs_models::custom_model::custom_state::NX,
                                       crs_models::custom_model::custom_state::NX>& R);

  static const std::string SENSOR_KEY;
};
}  // namespace custom_sensor_models
}  // namespace crs_sensor_models
#endif
