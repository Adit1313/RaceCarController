
#include "rocket_6_dof_sensor_model/mocap_twist_sensor_model.h"
#include <dynamic_models/utils/data_conversion.h>

#include <Eigen/Geometry>

namespace crs_sensor_models
{
namespace rocket_6_dof_sensor_models
{
// Option 2: Create an object and specify the process noise covariance matrix Q yourself.
MocapTwistSensorModel::MocapTwistSensorModel(const Eigen::Matrix<double, 6, 6>& R)
  : SensorModel(6, MocapTwistSensorModel::SENSOR_KEY)  // Measurement dimension is 6
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

  Eigen::Matrix<casadi::MX, 3, 1> I_omega_IB =
      q_IB.toRotationMatrix() * Eigen::Matrix<casadi::MX, 3, 1>(state_mx[10], state_mx[11], state_mx[12]);

  std::vector<casadi::MX> measured_states_mx = { state_mx[3],   state_mx[4],   state_mx[5],
                                                 I_omega_IB[0], I_omega_IB[1], I_omega_IB[2] };

  state_mx.insert(state_mx.end(), input_mx.begin(), input_mx.end());  // Append input at the end of state vector
  measurement_function = casadi::Function("applyMeasurementModel", state_mx, measured_states_mx);

  setR(R);
}

const std::string MocapTwistSensorModel::SENSOR_KEY = "mocap_twist";

}  // namespace rocket_6_dof_sensor_models
}  // namespace crs_sensor_models
