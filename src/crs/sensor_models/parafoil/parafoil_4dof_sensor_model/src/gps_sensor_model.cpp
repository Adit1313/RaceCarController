
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "parafoil_4dof_sensor_model/gps_sensor_model.h"
#include <dynamic_models/utils/data_conversion.h>

namespace crs_sensor_models
{
namespace parafoil_4dof_sensor_models
{
/**
 * @brief Construct a new GPS Sensor Model.
 *
 * @param R measurement covariance Matrix
 */
GPSSensorModel::GPSSensorModel(const Eigen::Matrix<double, 6, 6>& R)
  : SensorModel(6, GPSSensorModel::SENSOR_KEY)  // Measurement dimension is six
{
  std::vector<casadi::MX> state_mx = { casadi::MX::sym("x"),   casadi::MX::sym("y"),     casadi::MX::sym("z"),
                                       casadi::MX::sym("phi"), casadi::MX::sym("theta"), casadi::MX::sym("psi"),
                                       casadi::MX::sym("u"),   casadi::MX::sym("v"),     casadi::MX::sym("w") };
  std::vector<casadi::MX> input_mx = { casadi::MX::sym("delta_s"), casadi::MX::sym("delta_a") };

  // NOTE(@norrisg, @naefjo): Convert yaw-pitch-roll (Tait-Bryan) angles to Rotation Matrix
  using Vectord3MX = Eigen::Matrix<casadi::MX, 3, 1>;
  Eigen::AngleAxis<casadi::MX> yaw_angle(state_mx[5], Vectord3MX::UnitZ());
  Eigen::AngleAxis<casadi::MX> pitch_angle(0, Vectord3MX::UnitY());
  Eigen::AngleAxis<casadi::MX> roll_angle(state_mx[3], Vectord3MX::UnitX());

  Eigen::Matrix<casadi::MX, 3, 3> R_IB = (yaw_angle * pitch_angle * roll_angle).toRotationMatrix();
  Vectord3MX I_velocities = R_IB * Vectord3MX(state_mx[6], state_mx[7], state_mx[8]);

  // x, y, z, v_x, v_y, v_z
  std::vector<casadi::MX> measured_states_mx = { state_mx[0],     state_mx[1],     state_mx[2],
                                                 I_velocities[0], I_velocities[1], I_velocities[2] };

  state_mx.insert(state_mx.end(), input_mx.begin(), input_mx.end());  // Append input at the end of state vector
  measurement_function = casadi::Function("applyMeasurementModel", state_mx, measured_states_mx);

  setR(R);
}

const std::string GPSSensorModel::SENSOR_KEY = "gps";

}  // namespace parafoil_4dof_sensor_models
}  // namespace crs_sensor_models
