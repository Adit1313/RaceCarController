
#include "rocket_6_dof_sensor_model/imu_sensor_model.h"
#include <dynamic_models/utils/data_conversion.h>

#include <Eigen/Geometry>

namespace crs_sensor_models
{
namespace rocket_6_dof_sensor_models
{
// Option 2: Create an object and specify the process noise covariance matrix Q yourself.
/**
 * @brief Construct a new Imu Sensor Model. Note that the accelerations are not part of the rocket state and therefore
 * the continuous model is needed
 *
 * @param rocket_cont the continuous model
 * @param R measurement covariance Matrix
 */
ImuSensorModel::ImuSensorModel(
    const std::shared_ptr<crs_models::rocket_6_dof_model::ContinuousRocket6DofModel> rocket_cont,
    const Eigen::Matrix<double, 6, 6>& R)
  : SensorModel(6, ImuSensorModel::SENSOR_KEY)  // Measurement dimension is 6
{
  std::vector<casadi::MX> state_mx = { casadi::MX::sym("x"),
                                       casadi::MX::sym("y"),
                                       casadi::MX::sym("z"),
                                       casadi::MX::sym("v_x"),
                                       casadi::MX::sym("v_y"),
                                       casadi::MX::sym("v_z"),
                                       casadi::MX::sym("quat_x"),
                                       casadi::MX::sym("quat_y"),
                                       casadi::MX::sym("quat_z"),
                                       casadi::MX::sym("quat_w"),
                                       casadi::MX::sym("angular_vx"),
                                       casadi::MX::sym("angular_vy"),
                                       casadi::MX::sym("angular_vz"),
                                       casadi::MX::sym("thrust_magnitude"),
                                       casadi::MX::sym("torque_x"),
                                       casadi::MX::sym("servo_angle_1"),
                                       casadi::MX::sym("servo_angle_2") };
  std::vector<casadi::MX> input_mx = { casadi::MX::sym("thrust_magnitude"), casadi::MX::sym("torque"),
                                       casadi::MX::sym("servo_angle_1"), casadi::MX::sym("servo_angle_2") };

  casadi::MX q_x = state_mx[6], q_y = state_mx[7], q_z = state_mx[8], q_w = state_mx[9];
  Eigen::Quaternion<casadi::MX> q_IB = Eigen::Quaternion<casadi::MX>(q_w, q_x, q_y, q_z);

  auto cont_dynamics = rocket_cont->getContinuousDynamics(state_mx, input_mx);
  Eigen::Matrix<casadi::MX, 3, 1> B_acceleration =
      q_IB.toRotationMatrix().transpose() *
      Eigen::Matrix<casadi::MX, 3, 1>(cont_dynamics[3], cont_dynamics[4], cont_dynamics[5]);

  std::vector<casadi::MX> measured_states_mx = { B_acceleration[0], B_acceleration[1], B_acceleration[2],
                                                 state_mx[10],      state_mx[11],      state_mx[12] };

  state_mx.insert(state_mx.end(), input_mx.begin(), input_mx.end());  // Append input at the end of state vector
  measurement_function = casadi::Function("applyMeasurementModel", state_mx, measured_states_mx);

  setR(R);
}

const std::string ImuSensorModel::SENSOR_KEY = "imu";

}  // namespace rocket_6_dof_sensor_models
}  // namespace crs_sensor_models
