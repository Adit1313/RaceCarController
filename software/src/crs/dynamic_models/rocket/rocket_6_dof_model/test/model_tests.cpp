#include <gtest/gtest.h>
#include <Eigen/Core>
#include <casadi/casadi.hpp>
#include <iostream>
#include <math.h>
#include <random>

#include "rocket_6_dof_model/rocket_6_dof_input.h"

#include "rocket_6_dof_model/rocket_6_dof_state.h"
#include "rocket_6_dof_model/rocket_6_dof_continuous.h"
#include "rocket_6_dof_model/rocket_6_dof_discrete.h"
#include "rocket_6_dof_model/rocket_6_dof_params.h"
#include "rocket_6_dof_model/tests/test_utils.h"
#include <dynamic_models/utils/data_conversion.h>
#include <experimental/filesystem>

#define N_ITER 100
#define ABS_ERR 5e-2

/**
 * @brief Checks if continuous model can be called
 */
TEST(rocket6DofTests, testContModel)
{
  for (int iteration_counter = 0; iteration_counter < N_ITER; iteration_counter++)
  {
    crs_models::rocket_6_dof_model::rocket_6_dof_params params;
    bool success = loadRandomRocketParams(params, "params/test_rocket_6_dof_params.yaml");
    ASSERT_TRUE(success);

    crs_models::rocket_6_dof_model::ContinuousRocket6DofModel model(params);
    // only has a position and forward velocity
    crs_models::rocket_6_dof_model::rocket_6_dof_state current_state;
    loadRandomState(current_state);

    crs_models::rocket_6_dof_model::rocket_6_dof_input current_input;
    loadRandomInput(current_input);

    auto state_dot = model.applyModel(current_state, current_input);
    EXPECT_EQ(state_dot.position_x, current_state.velocity_x);
    EXPECT_EQ(state_dot.position_y, current_state.velocity_y);
    EXPECT_EQ(state_dot.position_z, current_state.velocity_z);
    EXPECT_EQ(state_dot.thrust_magnitude, (current_input.thrust_magnitude - current_state.thrust_magnitude) /
                                              params.thrust_magnitude_time_constant);
    EXPECT_EQ(state_dot.torque_x,
              (current_input.torque - current_state.torque_x) / params.thrust_magnitude_time_constant);
    EXPECT_EQ(state_dot.servo_angle_1,
              (current_input.servo_angle_1 - current_state.servo_angle_1) / params.servo_angle_time_constant);
    EXPECT_EQ(state_dot.servo_angle_2,
              (current_input.servo_angle_2 - current_state.servo_angle_2) / params.servo_angle_time_constant);
    // Move current state
  }
}

/**
 * @brief Checks if the discrete system takes off if we apply a net positive thrust to the rocket_6_dof.
 *        Tests the casadi integrator by calling applyModel()
 */
TEST(rocket6DofTests, testDiscreteMoveForward)
{
  for (int iteration_counter = 0; iteration_counter < N_ITER; iteration_counter++)
  {
    crs_models::rocket_6_dof_model::rocket_6_dof_params params;
    bool success = loadRandomRocketParams(params, "params/test_rocket_6_dof_params.yaml");
    ASSERT_TRUE(success);
    crs_models::rocket_6_dof_model::DiscreteRocket6DofModel model(params);
    // only has a position and forward velocity
    crs_models::rocket_6_dof_model::rocket_6_dof_state current_state = {
      getRandomFloat(-5, 5),  // pos
      getRandomFloat(-5, 5),
      getRandomFloat(-5, 5),
      getRandomFloat(0, 5),  // vel
      0.0,
      0.0,
      1.0,  // quat
      0.0,
      0.0,
      0.0,
      0.0,  // rate
      0.0,
      0.0,
      params.mass * params.gravity_constant * getRandomFloat(1.2, 1.5),  // thrust
      0.0,
      0.0,
      0.0,
    };

    crs_models::rocket_6_dof_model::rocket_6_dof_input current_input = { params.mass * params.gravity_constant * 2, 0,
                                                                         0, 0 };

    for (int step = 0; step < 5; step++)
    {
      double ts = getRandomFloat(0.05, 0.1);
      auto next_state = model.applyModel(current_state, current_input, ts);

      EXPECT_LT(current_state.position_x, next_state.position_x);
      EXPECT_LT(current_state.velocity_x, next_state.velocity_x);
      // Move current state
      current_state = next_state;
    }
  }
}

/**
 * @brief Checks if the discrete systems quaternion stays normalized
 */
TEST(rocket6DofTests, testRotation)
{
  for (int iteration_counter = 0; iteration_counter < N_ITER; iteration_counter++)
  {
    crs_models::rocket_6_dof_model::rocket_6_dof_params params;
    bool success = loadRandomRocketParams(params, "params/test_rocket_6_dof_params.yaml");
    ASSERT_TRUE(success);
    crs_models::rocket_6_dof_model::DiscreteRocket6DofModel model(params);
    // only has a position and forward velocity
    crs_models::rocket_6_dof_model::rocket_6_dof_state current_state = {
      getRandomFloat(-5, 5),  // pos
      getRandomFloat(-5, 5),
      getRandomFloat(-5, 5),
      getRandomFloat(0, 5),  // vel
      0.0,
      0.0,
      1.0,  // quat
      0.0,
      0.0,
      0.0,
      0.1,  // rate
      1.0,
      0.4,
      params.mass * params.gravity_constant * getRandomFloat(1.2, 1.5),  // thrust
      0.0,
      0.0,
      0.0,
    };

    crs_models::rocket_6_dof_model::rocket_6_dof_input current_input = {
      getRandomFloat(1.2, 1.5), getRandomFloat(-0.5, 0.5), getRandomFloat(0, 0.5), getRandomFloat(0, 0.5)
    };

    for (int step = 0; step < 5; step++)
    {
      double ts = getRandomFloat(0.05, 0.1);
      auto next_state = model.applyModel(current_state, current_input, ts);
      EXPECT_NEAR(current_state.quaternion_w * current_state.quaternion_w +
                      current_state.quaternion_x * current_state.quaternion_x +
                      current_state.quaternion_y * current_state.quaternion_y +
                      current_state.quaternion_z * current_state.quaternion_z,
                  1.0, 1e-5);

      // Move current state
      current_state = next_state;
    }
  }
}

/**
 * @brief check whether the system is "physical". I.e. that energy is conserved.
 */
TEST(rocket6DofTests, testConservationOfEnergy)
{
  for (int iteration_counter = 0; iteration_counter < N_ITER; iteration_counter++)
  {
    crs_models::rocket_6_dof_model::rocket_6_dof_params params;
    // bool success = loadRandomRocketParams(params, "params/example_rocket_6_dof_params.yaml");
    bool success = crs_models::rocket_6_dof_model::loadParamsFromFile("params/test_rocket_6_dof_params.yaml", params);
    EXPECT_TRUE(success);

    crs_models::rocket_6_dof_model::DiscreteRocket6DofModel model(params);

    double input_thrust = params.mass * params.gravity_constant * getRandomFloat(1.2, 1.5);
    crs_models::rocket_6_dof_model::rocket_6_dof_state current_state = {
      getRandomFloat(-5, 5),  // pos
      getRandomFloat(-5, 5),
      getRandomFloat(-5, 5),
      getRandomFloat(0, 5),  // vel
      0.0,
      0.0,
      1.0,  // quat
      0.0,
      0.0,
      0.0,
      getRandomFloat(-0.5, 0.5),  // rate
      getRandomFloat(-0.5, 0.5),
      getRandomFloat(-0.5, 0.5),
      input_thrust,  // thrust
      0.0,
      0.0,
      0.0,
    };

    crs_models::rocket_6_dof_model::rocket_6_dof_input current_input = { input_thrust, 0.0, 0.0, 0.0 };

    for (int step = 0; step < 20; step++)
    {
      double ts = getRandomFloat(0.005, 0.01);
      auto next_state = model.applyModel(current_state, current_input, ts);

      double kinetic_energy_current_state = computeKineticEnergy(current_state, params);
      double kinetic_energy_next_state = computeKineticEnergy(next_state, params);
      double external_work = computeWork(next_state, current_state, params, current_input);
      EXPECT_NEAR(kinetic_energy_next_state - kinetic_energy_current_state, external_work, ABS_ERR);

      // Move current state
      current_state = next_state;
    }
  }
}

/**
 * @brief Checks if the discrete system takes off if we apply a net positive thrust to the rocket_6_dof.
 *        Tests the casadi integrator by calling applyModel()
 */
TEST(rocket6DofTests, testJacobianDimensions)
{
  crs_models::rocket_6_dof_model::rocket_6_dof_params params;
  bool success = loadRandomRocketParams(params, "params/test_rocket_6_dof_params.yaml");
  ASSERT_TRUE(success);
  crs_models::rocket_6_dof_model::DiscreteRocket6DofModel model(params);
  // only has a position and forward velocity
  crs_models::rocket_6_dof_model::rocket_6_dof_state current_state = {
    getRandomFloat(-5, 5),  // pos
    getRandomFloat(-5, 5),
    getRandomFloat(-5, 5),
    getRandomFloat(0, 5),  // vel
    0.0,
    0.0,
    1.0,  // quat
    0.0,
    0.0,
    0.0,
    0.0,  // rate
    0.0,
    0.0,
    params.mass * params.gravity_constant * getRandomFloat(1.2, 1.5),  // thrust
    0.0,
    0.0,
    0.0,
  };
  crs_models::rocket_6_dof_model::rocket_6_dof_input current_input = { 10.0, 0.0, 0.0, 0.0 };

  const double integration_time = 0.1;

  crs_models::rocket_6_dof_model::DiscreteRocket6DofModel::StateMatrix A =
      crs_models::rocket_6_dof_model::DiscreteRocket6DofModel::StateMatrix::Zero();
  crs_models::rocket_6_dof_model::DiscreteRocket6DofModel::InputMatrix B =
      crs_models::rocket_6_dof_model::DiscreteRocket6DofModel::InputMatrix::Zero();

  model.getJacobian(current_state, current_input, integration_time, A, B);

  std::cout << crs_models::rocket_6_dof_model::rocket_6_dof_state::NX << std::endl;
  std::cout << crs_models::rocket_6_dof_model::rocket_6_dof_input::NU << std::endl;

  EXPECT_EQ(A.rows(), current_state.NX);
  EXPECT_EQ(A.cols(), crs_models::rocket_6_dof_model::rocket_6_dof_state::NX);
  EXPECT_EQ(B.rows(), current_state.NX);
  EXPECT_EQ(B.cols(), current_input.NU);
}

int main(int ac, char* av[])
{
  testing::InitGoogleTest(&ac, av);
  return RUN_ALL_TESTS();
}
