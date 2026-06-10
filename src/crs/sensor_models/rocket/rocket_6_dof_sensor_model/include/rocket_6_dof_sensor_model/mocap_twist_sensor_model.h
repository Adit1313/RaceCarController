#ifndef ROCKET_6_DOF_SENSOR_MODEL_MOCAP_TWIST_SENSOR_MODEL_H
#define ROCKET_6_DOF_SENSOR_MODEL_MOCAP_TWIST_SENSOR_MODEL_H

#include <rocket_6_dof_model/rocket_6_dof_input.h>
#include <rocket_6_dof_model/rocket_6_dof_state.h>
#include <sensor_models/sensor_model.h>

namespace crs_sensor_models
{
namespace rocket_6_dof_sensor_models
{
class MocapTwistSensorModel : public SensorModel<crs_models::rocket_6_dof_model::rocket_6_dof_state,
                                                 crs_models::rocket_6_dof_model::rocket_6_dof_input>
{
public:
  // Option 1: Create an object with no process noise covariance matrix Q defined. Will use identity.
  MocapTwistSensorModel()
    : MocapTwistSensorModel(Eigen::Matrix<double, 6, 6>::Identity())  // Measurement dimension is 6
  {
    std::cout << "[WARNING] No R Matrix specified for MocapTwistSensorModel. Using identity Matrix! " << std::endl;
  }

  // Option 2: Create an object and specify the process noise covariance matrix Q yourself.
  MocapTwistSensorModel(const Eigen::Matrix<double, 6, 6>& R);  // Measurement dimension is 6

  static const std::string SENSOR_KEY;
};
}  // namespace rocket_6_dof_sensor_models
}  // namespace crs_sensor_models
#endif
