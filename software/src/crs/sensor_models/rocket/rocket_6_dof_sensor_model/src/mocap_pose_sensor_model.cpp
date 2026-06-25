
#include "rocket_6_dof_sensor_model/mocap_pose_sensor_model.h"
#include <dynamic_models/utils/data_conversion.h>

namespace crs_sensor_models
{
namespace rocket_6_dof_sensor_models
{
// Option 2: Create an object and specify the process noise covariance matrix Q yourself.
MocapPoseSensorModel::MocapPoseSensorModel(const Eigen::Matrix<double, 7, 7>& R)
  : SensorModel(7, MocapPoseSensorModel::SENSOR_KEY)  // Measurement dimension is three
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

  std::vector<casadi::MX> measured_states_mx = { state_mx[0], state_mx[1], state_mx[2], state_mx[6],
                                                 state_mx[7], state_mx[8], state_mx[9] };

  state_mx.insert(state_mx.end(), input_mx.begin(), input_mx.end());  // Append input at the end of state vector
  measurement_function = casadi::Function("applyMeasurementModel", state_mx, measured_states_mx);

  setR(R);
}

const std::string MocapPoseSensorModel::SENSOR_KEY = "mocap_pose";

}  // namespace rocket_6_dof_sensor_models
}  // namespace crs_sensor_models
