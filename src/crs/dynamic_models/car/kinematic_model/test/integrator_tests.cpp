#include <gtest/gtest.h>
#include <Eigen/Core>
#include <casadi/casadi.hpp>
#include <iostream>
#include <math.h>
#include <random>

#include "kinematic_model/kinematic_car_input.h"
#include "kinematic_model/kinematic_car_state.h"
#include "kinematic_model/kinematic_continuous.h"
#include "kinematic_model/kinematic_discrete.h"
#include "kinematic_model/kinematic_params.h"
#include "kinematic_model/tests/test_utils.h"
#include <dynamic_models/utils/data_conversion.h>

#define N_ITER 50
#define EPSILON 1e-4
#define STATE_TOLERANCE 5e-2

/**
 * @brief Checks if the the RK4 integrator for getJacobians() works correctly. It does not test the Input Jacobian, as
 * it is never used (so far!)
 */
TEST(IntegratorTests, testRK4)
{
  Eigen::Matrix<double, crs_models::kinematic_model::kinematic_car_state::NX,
                crs_models::kinematic_model::kinematic_car_state::NX>
      Q;
  Q << 0.01, 0, 0, 0, 0, 0.01, 0, 0, 0, 0, 0.01, 0, 0, 0, 0, 0.0001;

  for (int iteration_counter = 0; iteration_counter < N_ITER; iteration_counter++)
  {
    crs_models::kinematic_model::kinematic_params params;
    bool success = loadRandomKinematicParams(params, "params/test_kinematic_params.yaml");
    ASSERT_TRUE(success);

    crs_models::kinematic_model::DiscreteKinematicModel model(params, Q);
    crs_models::kinematic_model::kinematic_car_state current_state = { getRandomFloat(-5, 5), getRandomFloat(-5, 5),
                                                                       getRandomFloat(-0.2, 0.2),
                                                                       getRandomFloat(0.4, 2) };
    crs_models::kinematic_model::kinematic_car_input current_input = { getRandomFloat(0.1, 0.4),
                                                                       getRandomFloat(-0.1, 0.1) };
    double ts = getRandomFloat(0.0025, 0.01);  // Valid from 400 - 100 Hz

    // Initialize matrices the proper way
    Eigen::Matrix<double, crs_models::kinematic_model::kinematic_car_state::NX,
                  crs_models::kinematic_model::kinematic_car_state::NX>
        A;
    A.setZero();

    Eigen::Matrix<double, crs_models::kinematic_model::kinematic_car_state::NX,
                  crs_models::kinematic_model::kinematic_car_input::NU>
        B;
    B.setZero();

    model.getJacobian(current_state, current_input, ts, A, B);

    // Define A_fd and B_fd as full matrices
    Eigen::Matrix<double, crs_models::kinematic_model::kinematic_car_state::NX,
                  crs_models::kinematic_model::kinematic_car_state::NX>
        A_fd;
    A_fd.setZero();

    Eigen::Matrix<double, crs_models::kinematic_model::kinematic_car_state::NX,
                  crs_models::kinematic_model::kinematic_car_input::NU>
        B_fd;
    B_fd.setZero();

    // Test state Jacobian
    for (int i = 0; i < crs_models::kinematic_model::kinematic_car_state::NX; i++)
    {
      crs_models::kinematic_model::kinematic_car_state state_plus = current_state;
      crs_models::kinematic_model::kinematic_car_state state_minus = current_state;
      Eigen::Matrix<double, crs_models::kinematic_model::kinematic_car_state::NX, 1> EPSILON_VEC;
      EPSILON_VEC.setZero();
      EPSILON_VEC(i) = EPSILON;
      state_plus += EPSILON_VEC;
      state_minus -= EPSILON_VEC;

      auto state_plus_next = model.applyModel(state_plus, current_input, ts);
      auto state_minus_next = model.applyModel(state_minus, current_input, ts);

      // Compute finite-difference derivative:
      Eigen::Matrix<double, crs_models::kinematic_model::kinematic_car_state::NX, 1> diff_vec =
          toEigen((state_plus_next - state_minus_next)) / (2 * EPSILON);

      for (int row = 0; row < crs_models::kinematic_model::kinematic_car_state::NX; row++)
      {
        A_fd(row, i) = diff_vec(row);
      }

      // Verify results
      for (int j = 0; j < crs_models::kinematic_model::kinematic_car_state::NX; j++)
      {
        ASSERT_NEAR(A_fd(j, i), A(j, i), STATE_TOLERANCE);
      }
    }
  }
}

int main(int ac, char* av[])
{
  testing::InitGoogleTest(&ac, av);
  return RUN_ALL_TESTS();
}
