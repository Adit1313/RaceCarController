
#include "pacejka_with_imu_bias_sensor_model/imu_with_bias_sensor_model.h"
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
ImuWithBiasSensorModel::ImuWithBiasSensorModel(const std::shared_ptr<StackedPacejkaImuModel> pacejka_cont,
                                               const Eigen::Matrix3d& R)
  : SensorModel(3, ImuWithBiasSensorModel::SENSOR_KEY)  // Measurement dimension is three
{
  auto cont_dynamics = pacejka_cont->getContinuousDynamics(state_mx, input_mx);  // returns x_dot, y_dot, theta_dot,
                                                                                 // vx_dot, vy_dot, yaw_dot_dot

  //                                             vx_dot + bias_ax,          vy_dot + bias_ay,    theta_dot + bias_dyaw
  std::vector<casadi::MX> measured_states_mx = { cont_dynamics[3] + state_mx[6], cont_dynamics[4] + state_mx[7],
                                                 cont_dynamics[2] + state_mx[8] };
  measurement_function = casadi::Function("applyMeasurementModel", state_and_input_mx, measured_states_mx);

  setR(R);
}

const std::string ImuWithBiasSensorModel::SENSOR_KEY = "imu";

}  // namespace pacejka_sensor_models
}  // namespace crs_sensor_models
