#include "rocket_6_dof_model/rocket_6_dof_continuous.h"

#include <casadi/casadi.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace crs_models
{
namespace rocket_6_dof_model
{
ContinuousRocket6DofModel::ContinuousRocket6DofModel(rocket_6_dof_params params) : params(params)  // Constructor
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

  std::vector<casadi::MX> state_dot_mx = getContinuousDynamics(state_mx, input_mx);
  // append control inputs to state vector (casadi functions only take one input)
  for (const auto input : input_mx)
  {
    state_mx.push_back(input);
  }

  dynamics_f_ = casadi::Function("f_dot", state_mx, state_dot_mx);
}

/**
 * @brief evaluated state_dot (= f(x,u)) at current state and control input
 *
 * @param state
 * @param input
 * @return rocket_6_dof_state returns state_dot evaluated at current state and control input
 */
rocket_6_dof_state ContinuousRocket6DofModel::applyModel(const rocket_6_dof_state state, const rocket_6_dof_input input)
{
  rocket_6_dof_state state_dot_numerical;
  // Convert casadi symbols (mx) to casadi function to be evaluated at current state & control input
  dynamics_f_(commons::convertToConstVector(state, input), commons::convertToVector(state_dot_numerical));
  return state_dot_numerical;
}

/**
 * @brief Get the analytical state equations f(state, input) = state_dot
 *
 * @param state
 * @param control_input
 * @return std::vector<casadi::MX> returns the analytical state equations f(state, input) = state_dot
 */
std::vector<casadi::MX> ContinuousRocket6DofModel::getContinuousDynamics(const std::vector<casadi::MX> state,
                                                                         const std::vector<casadi::MX> control_input)
{
  // For the detailed equations, check out arXiv:2103.04709.
  using namespace casadi;
  assert(state.size() == 17);
  assert(control_input.size() == 4);

  // I_v
  auto velocity_x = state[3];
  auto velocity_y = state[4];
  auto velocity_z = state[5];

  // q_IB
  auto quaternion_x = state[6];
  auto quaternion_y = state[7];
  auto quaternion_z = state[8];
  auto quaternion_w = state[9];
  Eigen::Quaternion<MX> q_IB = Eigen::Quaternion<MX>(quaternion_w, quaternion_x, quaternion_y, quaternion_z);

  // B_omega_IB
  auto angular_velocity_x = state[10];
  auto angular_velocity_y = state[11];
  auto angular_velocity_z = state[12];
  Eigen::Matrix<MX, 3, 1> B_omega_IB =
      Eigen::Matrix<MX, 3, 1>(angular_velocity_x, angular_velocity_y, angular_velocity_z);

  auto thrust_magnitude = state[13];
  auto torque_x = state[14];
  auto servo_angle_1 = state[15];
  auto servo_angle_2 = state[16];

  auto thrust_magnitude_input = control_input[0];
  auto torque_x_input = control_input[1];
  auto servo_angle_1_input = control_input[2];
  auto servo_angle_2_input = control_input[3];

  MX position_x_dot = velocity_x;
  MX position_y_dot = velocity_y;
  MX position_z_dot = velocity_z;

  // Necessary because computeGimbalAngle expects the previous tilt angle of the plane
  // which is zero for the first gimbal angle.
  MX dummy_variable = 0.0;
  MX gimbal_angle_1 = computeGimbalAngle(servo_angle_1, dummy_variable);
  MX gimbal_angle_2 = computeGimbalAngle(servo_angle_2, gimbal_angle_1);

  Eigen::Matrix<MX, 3, 1> B_thrust_vector = Eigen::Matrix<MX, 3, 1>(
      thrust_magnitude * cos(gimbal_angle_1) * cos(gimbal_angle_2), thrust_magnitude * sin(gimbal_angle_2),
      -thrust_magnitude * sin(gimbal_angle_1) * cos(gimbal_angle_2));

  Eigen::Matrix<MX, 3, 1> I_acceleration = (1 / (double)(params.mass)) * q_IB.toRotationMatrix() * B_thrust_vector;
  I_acceleration.x() -= params.gravity_constant;

  // NOTE(@naefjo): Disable clang-format to make this initialization more legible
  // clang-format off
  Eigen::Matrix<MX, 4, 4> quaternion_propagation_matrix;
  quaternion_propagation_matrix << 0.0,                -angular_velocity_x,      -angular_velocity_y, -angular_velocity_z,
                                   angular_velocity_x,  0.0,                      angular_velocity_z, -angular_velocity_y,
                                   angular_velocity_y, -angular_velocity_z,       0.0,                 angular_velocity_x,
                                   angular_velocity_z,  angular_velocity_y,      -angular_velocity_x,  0.0;
  // clang-format on

  // NOTE(@naefjo): Indexing is [0:w, 1:x, 2:y, 3:z]
  Eigen::Matrix<MX, 4, 1> q_IB_dot = 0.5 * quaternion_propagation_matrix *
                                     Eigen::Matrix<MX, 4, 1>(quaternion_w, quaternion_x, quaternion_y, quaternion_z);

  Eigen::Matrix<MX, 3, 1> B_gimbal_torque =
      Eigen::Matrix<MX, 3, 1>(-params.thrust_cog_offset, 0.0, 0.0).cross(B_thrust_vector);
  Eigen::DiagonalMatrix<MX, 3> inertia_matrix(params.inertia_xx, params.inertia_yy, params.inertia_zz);
  Eigen::Matrix<MX, 3, 1> B_omega_IB_dot =
      inertia_matrix.inverse() * (B_gimbal_torque - B_omega_IB.cross(inertia_matrix * B_omega_IB));

  // Actuator dynamics are modeled as first order systems.
  MX thrust_magnitude_dot = (thrust_magnitude_input - thrust_magnitude) / params.thrust_magnitude_time_constant;
  // TODO(@naefjo): custom timescale
  MX torque_x_dot = (torque_x_input - torque_x) / params.thrust_magnitude_time_constant;
  MX servo_angle_1_dot = (servo_angle_1_input - servo_angle_1) / params.servo_angle_time_constant;
  MX servo_angle_2_dot = (servo_angle_2_input - servo_angle_2) / params.servo_angle_time_constant;

  std::vector<MX> state_dot = { position_x_dot,     position_y_dot,       position_z_dot,     I_acceleration.x(),
                                I_acceleration.y(), I_acceleration.z(),   q_IB_dot(1, 0),     q_IB_dot(2, 0),
                                q_IB_dot(3, 0),     q_IB_dot(0, 0),       B_omega_IB_dot.x(), B_omega_IB_dot.y(),
                                B_omega_IB_dot.z(), thrust_magnitude_dot, torque_x_dot,       servo_angle_1_dot,
                                servo_angle_2_dot };

  return state_dot;
}

casadi::MX ContinuousRocket6DofModel::computeGimbalAngle(casadi::MX& servo_angle, casadi::MX& tilt_axis_angle)
{
  using namespace casadi;
  casadi::MX intermediate_value_1 = params.gimbal_d + params.gimbal_a * cos(servo_angle);
  casadi::MX intermediate_value_2 = params.gimbal_e - params.gimbal_a * sin(servo_angle);
  casadi::MX u = params.gimbal_b * params.gimbal_b - params.gimbal_c * params.gimbal_c -
                 intermediate_value_1 * intermediate_value_1 - intermediate_value_2 * intermediate_value_2;
  casadi::MX v = 2 * params.gimbal_c * cos(tilt_axis_angle) * intermediate_value_2;
  casadi::MX w = -2 * params.gimbal_c * intermediate_value_1;

  casadi::MX intermediate_value_3 = w * w + v * v - u * u;
  casadi::MX gimbal_angle = 2 * atan((v - sqrt(intermediate_value_3)) / (u + w));
  return gimbal_angle;
}
}  // namespace rocket_6_dof_model
}  // namespace crs_models
