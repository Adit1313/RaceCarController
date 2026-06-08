#ifndef ROCKET_MODEL_6_DOF_TESTS_TEST_UTILS_H
#define ROCKET_MODEL_6_DOF_TESTS_TEST_UTILS_H
#include <random>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "rocket_6_dof_model/rocket_6_dof_input.h"
#include "rocket_6_dof_model/rocket_6_dof_state.h"
#include "rocket_6_dof_model/rocket_6_dof_params.h"
#include <experimental/filesystem>

namespace fs = std::experimental::filesystem;

// For random number generation
std::random_device rd;
std::default_random_engine eng(rd());  // NOLINT
std::uniform_real_distribution<float> distr(0.0, 1.0);

float getRandomFloat(float lower_limit, float upper_limit)
{
  return (distr(rd) * (upper_limit - lower_limit)) + lower_limit;
}

bool loadRandomRocketParams(crs_models::rocket_6_dof_model::rocket_6_dof_params& params, const std::string path)
{
  bool success = crs_models::rocket_6_dof_model::loadParamsFromFile(path, params);
  if (!success)
  {
    return false;
  }

  float rel_diff = 0.1;  // 10 percent difference
  params.mass *= 1 + getRandomFloat(-rel_diff, rel_diff);
  params.inertia_xx *= 1 + getRandomFloat(-rel_diff, rel_diff);
  params.inertia_yy *= 1 + getRandomFloat(-rel_diff, rel_diff);
  params.inertia_zz *= 1 + getRandomFloat(-rel_diff, rel_diff);
  params.thrust_cog_offset *= 1 + getRandomFloat(-rel_diff, rel_diff);
  params.thrust_magnitude_time_constant *= 1 + getRandomFloat(-rel_diff, rel_diff);
  params.servo_angle_time_constant *= 1 + getRandomFloat(-rel_diff, rel_diff);
  return true;
}

void loadRandomState(crs_models::rocket_6_dof_model::rocket_6_dof_state& state)
{
  state.position_x = getRandomFloat(5.0, 10.0);
  state.position_y = getRandomFloat(-10.0, 10.0);
  state.position_z = getRandomFloat(-10.0, 10.0);
  state.velocity_x = getRandomFloat(-1.0, 5.0);
  state.velocity_y = getRandomFloat(-1.0, 1.0);
  state.velocity_z = getRandomFloat(-1.0, 1.0);
  state.quaternion_w = 1.0;
  state.quaternion_x = 0.0;
  state.quaternion_y = 0.0;
  state.quaternion_z = 0.0;
  state.angular_velocity_x = getRandomFloat(-5.0, 5.0);
  state.angular_velocity_y = getRandomFloat(-5.0, 5.0);
  state.angular_velocity_z = getRandomFloat(-5.0, 5.0);
  state.thrust_magnitude = getRandomFloat(5.0, 10.0);
  state.torque_x = 0.0;
  state.servo_angle_1 = 0.0;
  state.servo_angle_2 = 0.0;
}

void loadRandomInput(crs_models::rocket_6_dof_model::rocket_6_dof_input& input)
{
  input.thrust_magnitude = getRandomFloat(0, 5.0);
  input.torque = getRandomFloat(-0.1, 0.1);
  input.servo_angle_1 = getRandomFloat(-1.0, 1.0);
  input.servo_angle_1 = getRandomFloat(-1.0, 1.0);
}

/**
 * @brief helper function to compute the kinetic energy of the rocket_6_dof for a given state
 *
 * @param state defines the rocket_6_dof state.
 * @param params rocket_6_dof parameters used in the model.
 * @return double value of the
 */
double computeKineticEnergy(const crs_models::rocket_6_dof_model::rocket_6_dof_state& state,
                            const crs_models::rocket_6_dof_model::rocket_6_dof_params& params)
{
  Eigen::Vector3d linear_velocity = Eigen::Vector3d(state.velocity_x, state.velocity_y, state.velocity_z);
  double translational_kinetic_energy = 0.5 * params.mass * linear_velocity.dot(linear_velocity);

  Eigen::Vector3d angular_velocity =
      Eigen::Vector3d(state.angular_velocity_x, state.angular_velocity_y, state.angular_velocity_z);
  Eigen::DiagonalMatrix<double, 3> inertia_tensor(params.inertia_xx, params.inertia_yy, params.inertia_zz);

  double rotational_kinetic_energy = 0.5 * angular_velocity.transpose() * inertia_tensor * angular_velocity;
  return translational_kinetic_energy + rotational_kinetic_energy;
}

/**
 * @brief Compute the work of the external forces done on the system.
 *
 * @param state current state of the rocket_6_dof
 * @param prev_state previous state of the rocket_6_dof
 * @param params rocket_6_dof params of the model
 * @param input input which was applied to the rocket_6_dof from prev_state to state
 * @return the work done by the external forces
 */
double computeWork(const crs_models::rocket_6_dof_model::rocket_6_dof_state& state,
                   const crs_models::rocket_6_dof_model::rocket_6_dof_state& prev_state,
                   const crs_models::rocket_6_dof_model::rocket_6_dof_params& params,
                   const crs_models::rocket_6_dof_model::rocket_6_dof_input& input)
{
  Eigen::Vector3d initial_com_position_W =
      Eigen::Vector3d(prev_state.position_x, prev_state.position_y, prev_state.position_z);
  Eigen::Vector3d final_com_position_W = Eigen::Vector3d(state.position_x, state.position_y, state.position_z);

  Eigen::Vector3d gimbal_offset_R = Eigen::Vector3d(-params.thrust_cog_offset, 0, 0);

  Eigen::Quaterniond initial_attitude_WR = Eigen::Quaterniond(prev_state.quaternion_w, prev_state.quaternion_x,
                                                              prev_state.quaternion_y, prev_state.quaternion_z);
  Eigen::Quaterniond final_attitude_WR =
      Eigen::Quaterniond(state.quaternion_w, state.quaternion_x, state.quaternion_y, state.quaternion_z);

  Eigen::Vector3d initial_gimbal_offset_W = initial_attitude_WR * gimbal_offset_R;
  Eigen::Vector3d final_gimbal_offset_W = final_attitude_WR * gimbal_offset_R;

  Eigen::Vector3d delta_position =
      (final_com_position_W + final_gimbal_offset_W) - (initial_com_position_W + initial_gimbal_offset_W);

  Eigen::Vector3d gravitational_force = Eigen::Vector3d(-params.mass * params.gravity_constant, 0.0, 0.0);
  Eigen::Vector3d input_force = final_attitude_WR * Eigen::Vector3d(input.thrust_magnitude, 0, 0);

  return delta_position.dot(gravitational_force + input_force);
}

#endif  // rocket_6_dof_MODEL_TESTS_TEST_UTILS_H
