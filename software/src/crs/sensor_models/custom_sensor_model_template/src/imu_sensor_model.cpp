
#include "custom_sensor_model_template/imu_sensor_model.h"
#include <dynamic_models/utils/data_conversion.h>

// ==================== TODO ====================
// - Change namespace from custom_sensor_model to something sensible
// - Change state and input types in this file
// - Specify the Measurement Dimension by passing The correct number to the `SensorModel` Constructor (1)
// - Specify your state and input symbolic vairables (2), and construct the correct measured state (3)
// - Specify an appropriate sensor key (4)

namespace crs_sensor_models
{
namespace custom_sensor_models
{
// Option 2: Create an object and specify the process noise covariance matrix Q yourself.
/**
 * @brief Construct a new Imu Sensor Model. Note that the accelerations are not part of the pacejka state and therefore
 * the continuous model is needed
 *
 * @param custom_cont the continuous model
 * @param R measurement covariance Matrix
 */
// TODO (1)
ImuSensorModel::ImuSensorModel(const std::shared_ptr<crs_models::custom_model::ContinuousCustomModel> custom_cont,
                               const Eigen::Matrix<double, crs_models::custom_model::custom_state::NX,
                                                   crs_models::custom_model::custom_state::NX>& R)
  : SensorModel(3, ImuSensorModel::SENSOR_KEY)  // Measurement dimension is three
{
  // TODO (2)
  std::vector<casadi::MX> state_mx = { casadi::MX::sym("x"),   casadi::MX::sym("y"),   casadi::MX::sym("yaw"),
                                       casadi::MX::sym("v_x"), casadi::MX::sym("v_y"), casadi::MX::sym("yaw_rate") };
  std::vector<casadi::MX> input_mx = { casadi::MX::sym("steer"), casadi::MX::sym("torque") };

  // TODO (3)
  auto cont_dynamics = custom_cont->getContinuousDynamics(state_mx, input_mx);  // returns x_dot, y_dot, theta_dot,
                                                                                // vx_dot, vy_dot, yaw_dot_dot

  // TODO (3)
  //                                                 vx_dot,           vy_dot,        theta_dot,
  std::vector<casadi::MX> measured_states_mx = { cont_dynamics[3], cont_dynamics[4], cont_dynamics[2] };

  // `measurement_function` expects concatenated state and input as an argument
  state_mx.insert(state_mx.end(), input_mx.begin(), input_mx.end());
  measurement_function = casadi::Function("applyMeasurementModel", state_mx, measured_states_mx);

  setR(R);
}

// TODO (4)
const std::string ImuSensorModel::SENSOR_KEY = "imu";

}  // namespace custom_sensor_models
}  // namespace crs_sensor_models
