
#include "pacejka_sensor_model/imu_sensor_model.h"
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
ImuSensorModel::ImuSensorModel(const std::shared_ptr<crs_models::pacejka_model::ContinuousPacejkaModel> pacejka_cont,
                               const Eigen::Matrix3d& R)
  : SensorModel(3, ImuSensorModel::SENSOR_KEY)  // Measurement dimension is three
{
  // returns x_dot, y_dot, yaw_dot, vx_dot, vy_dot, yaw_dot_dot
  auto cont_dynamics = pacejka_cont->getContinuousDynamics(state_mx, input_mx);

  //                                                 vx_dot,           vy_dot,        yaw_dot,
  std::vector<casadi::MX> measured_states_mx = { cont_dynamics[3], cont_dynamics[4], cont_dynamics[2] };
  measurement_function = casadi::Function("applyMeasurementModel", state_and_input_mx, measured_states_mx);

  setR(R);
}

const std::string ImuSensorModel::SENSOR_KEY = "imu";

}  // namespace pacejka_sensor_models
}  // namespace crs_sensor_models
