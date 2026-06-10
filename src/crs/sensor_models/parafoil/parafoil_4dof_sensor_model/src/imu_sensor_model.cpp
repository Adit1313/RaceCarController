
#include "parafoil_4dof_sensor_model/imu_sensor_model.h"
#include <dynamic_models/utils/data_conversion.h>

namespace crs_sensor_models
{
namespace parafoil_4dof_sensor_models
{
/**
 * @brief Construct a new IMU Sensor Model. Note that the accelerations are not part of the 4 DoF parafoil state and
 * therefore the continuous model is needed
 *
 * @param parafoil_4dof_cont the continuous model
 * @param R measurement covariance Matrix
 */
ImuSensorModel::ImuSensorModel(
    const std::shared_ptr<crs_models::parafoil_4dof_model::ContinuousParafoil4dofModel> parafoil_4dof_cont,
    const Eigen::Matrix<double, 6, 6>& R)
  : SensorModel(6, ImuSensorModel::SENSOR_KEY)  // Measurement dimension is six
{
  std::vector<casadi::MX> state_mx = { casadi::MX::sym("x"),   casadi::MX::sym("y"),     casadi::MX::sym("z"),
                                       casadi::MX::sym("phi"), casadi::MX::sym("theta"), casadi::MX::sym("psi"),
                                       casadi::MX::sym("u"),   casadi::MX::sym("v"),     casadi::MX::sym("w") };
  std::vector<casadi::MX> input_mx = { casadi::MX::sym("delta_s"), casadi::MX::sym("delta_a") };

  auto cont_dynamics = parafoil_4dof_cont->getContinuousDynamics(state_mx, input_mx);
  //                                             p,                q,                                r,
  //                                             accel_1,          accel_2,                          accel_3,
  std::vector<casadi::MX> measured_states_mx = {
    cont_dynamics[3], sin(state_mx[3]) * cont_dynamics[5],        cos(state_mx[3]) * cont_dynamics[5],
    cont_dynamics[6], cont_dynamics[7] + 9.81 * sin(state_mx[3]), cont_dynamics[8] + 9.81 * cos(state_mx[3])
  };

  state_mx.insert(state_mx.end(), input_mx.begin(), input_mx.end());  // Append input at the end of state vector
  measurement_function = casadi::Function("applyMeasurementModel", state_mx, measured_states_mx);

  setR(R);
}

const std::string ImuSensorModel::SENSOR_KEY = "imu";

}  // namespace parafoil_4dof_sensor_models
}  // namespace crs_sensor_models
