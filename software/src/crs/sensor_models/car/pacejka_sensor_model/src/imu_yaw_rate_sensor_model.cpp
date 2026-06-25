
#include "pacejka_sensor_model/imu_yaw_rate_sensor_model.h"
#include <dynamic_models/utils/data_conversion.h>

namespace crs_sensor_models
{
namespace pacejka_sensor_models
{
// Option 2: Create an object and specify the process noise covariance matrix Q yourself.
/**
 * @brief Construct a new Imu Sensor Model. Note that the accelerations are not part of the pacejka state and therefore
 * the continuous model is needed
 *
 * @param pacejka_cont the continuous model
 * @param R measurement covariance Matrix
 */
ImuYawSensorModel::ImuYawSensorModel(const Eigen::Matrix<double, 1, 1>& R)
  : SensorModel(1, ImuYawSensorModel::SENSOR_KEY)  // Measurement dimension is three
{
  std::vector<casadi::MX> measured_states_mx = { state_mx[5] };  // yaw_rate
  measurement_function = casadi::Function("applyMeasurementModel", state_and_input_mx, measured_states_mx);

  setR(R);
}

const std::string ImuYawSensorModel::SENSOR_KEY = "imu_yaw_rate";

}  // namespace pacejka_sensor_models
}  // namespace crs_sensor_models
