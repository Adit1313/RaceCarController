#include <gtest/gtest.h>
#include <Eigen/Core>
#include <casadi/casadi.hpp>
#include <iostream>
#include <math.h>
#include <random>

#include "parafoil_4dof_model/parafoil_4dof_input.h"
#include "parafoil_4dof_model/parafoil_4dof_state.h"
#include "parafoil_4dof_model/parafoil_4dof_continuous.h"
#include "parafoil_4dof_model/parafoil_4dof_discrete.h"
#include "parafoil_4dof_model/parafoil_4dof_params.h"

#include "parafoil_4dof_model/tests/test_utils.h"
#include <dynamic_models/utils/data_conversion.h>
#include <experimental/filesystem>

#define N_ITER 100

/**
 * @brief Checks if the apply model functions throws errors.
 * Currently, this check if pretty useless. TODO(norrisg) Check with GT dynamics or sth.
 */
TEST(ModelTests, testApplyDynamics)
{
  crs_models::parafoil_4dof_model::parafoil_4dof_params params;
  loadRandomParafoil4dofParams(params, "params/test_parafoil_4dof_params.yaml");
  crs_models::parafoil_4dof_model::ContinuousParafoil4dofModel model(params);
  crs_models::parafoil_4dof_model::parafoil_4dof_state current_state;
  loadRandomState(current_state);
  crs_models::parafoil_4dof_model::parafoil_4dof_input current_input;
  loadRandomInput(current_input);
  model.applyModel(current_state, current_input);
}

/**
 * @brief Checks if the discrete system flies forward
 *        Tests the casadi integrator by calling applyModel()
 */
TEST(ModelTests, testDiscreteFliesForward)
{
  for (int iteration_counter = 0; iteration_counter < N_ITER; iteration_counter++)
  {
    crs_models::parafoil_4dof_model::parafoil_4dof_params params;
    loadRandomParafoil4dofParams(params, "params/test_parafoil_4dof_params.yaml");
    crs_models::parafoil_4dof_model::DiscreteParafoil4dofModel model(params);
    // only has a position and forward velocity
    crs_models::parafoil_4dof_model::parafoil_4dof_state current_state = {
      getRandomFloat(-5, 5), getRandomFloat(-5, 5), getRandomFloat(-5, 5), 0, 0, 0, 13.13, 0, 0
    };
    crs_models::parafoil_4dof_model::parafoil_4dof_input current_input = { 0, 0 };  // apply zero input

    for (int step = 0; step < 5; step++)
    {
      double ts = getRandomFloat(0.05, 0.1);
      auto next_state = model.applyModel(current_state, current_input, ts);
      EXPECT_LT(current_state.pos_x, next_state.pos_x);
      // Move current state
      current_state = next_state;
    }
  }
}

/**
 * @brief Caclucates the jacobian of the discrete system
 */
TEST(ModelTests, testDiscreteJacobian)
{
  crs_models::parafoil_4dof_model::parafoil_4dof_params params;
  loadRandomParafoil4dofParams(params, "params/test_parafoil_4dof_params.yaml");
  crs_models::parafoil_4dof_model::DiscreteParafoil4dofModel model(params);
  crs_models::parafoil_4dof_model::parafoil_4dof_state current_state = {
    getRandomFloat(-5, 5), getRandomFloat(-5, 5), getRandomFloat(-5, 5), 0, 0, 0, 13.13, 0, 0
  };
  crs_models::parafoil_4dof_model::parafoil_4dof_input current_input = { getRandomFloat(0.5, 0.5), 0 };

  Eigen::Matrix<double, 9, 9> F;
  Eigen::Matrix<double, 9, 2> D;
  model.getJacobian(current_state, current_input, 0.1, F, D);
}

// TODO(norrisg) write `testIntegrator` test similar to that of pacejka model

// TODO(norrisg) write `testContinuousJacobian` test similar to that of pacejka model

int main(int ac, char* av[])
{
  testing::InitGoogleTest(&ac, av);
  return RUN_ALL_TESTS();
}
