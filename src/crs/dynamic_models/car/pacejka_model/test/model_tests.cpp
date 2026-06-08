#include <gtest/gtest.h>
#include <Eigen/Core>
#include <casadi/casadi.hpp>
#include <iostream>
#include <math.h>
#include <random>

#include "pacejka_model/pacejka_car_input.h"
#include "pacejka_model/pacejka_car_state.h"
#include "pacejka_model/pacejka_continuous.h"
#include "pacejka_model/pacejka_discrete.h"
#include "pacejka_model/pacejka_params.h"

#include "pacejka_model/tests/test_utils.h"
#include <dynamic_models/utils/data_conversion.h>
#include <experimental/filesystem>

#define N_ITER 100
#define N_RUNS 10

/**
 * @brief Checks if the apply model functions throws errors.
 * Currently, this check if pretty useless. TODO(sabodmer) Check with GT dynamics or sth.
 */
TEST(PacejkaTests, testApplyDynamics)
{
  crs_models::pacejka_model::pacejka_params params;
  loadRandomPacejkaParams(params, "params/test_pacejka_params.yaml");
  crs_models::pacejka_model::ContinuousPacejkaModel model(params);
  crs_models::pacejka_model::pacejka_car_state current_state;
  loadRandomState(current_state);
  crs_models::pacejka_model::pacejka_car_input current_input;
  loadRandomInput(current_input);
  model.applyModel(current_state, current_input);
}

/**
 * @brief Checks if the discrete system drives forward
 *        Tests the casadi integrator by calling applyModel()
 */
TEST(PacejkaTests, testDiscreteDriveForward)
{
  for (int iteration_counter = 0; iteration_counter < N_ITER; iteration_counter++)
  {
    crs_models::pacejka_model::pacejka_params params;
    loadRandomPacejkaParams(params, "params/test_pacejka_params.yaml");
    Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Identity();
    crs_models::pacejka_model::DiscretePacejkaModel model(params, Q);
    // only has a position and forward velocity
    crs_models::pacejka_model::pacejka_car_state current_state = {
      getRandomFloat(-5, 5), getRandomFloat(-5, 5), 0, getRandomFloat(0.4, 2), 0, 0
    };
    crs_models::pacejka_model::pacejka_car_input current_input = { getRandomFloat(0.3, 1),
                                                                   0 };  // only apply torque input

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
TEST(PacejkaTests, testDiscreteJacobian)
{
  crs_models::pacejka_model::pacejka_params params;
  loadRandomPacejkaParams(params, "params/test_pacejka_params.yaml");
  Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Identity();
  crs_models::pacejka_model::DiscretePacejkaModel model(params, Q);
  crs_models::pacejka_model::pacejka_car_state current_state = {
    getRandomFloat(-5, 5), getRandomFloat(-5, 5), 0, getRandomFloat(0.4, 2), 0, 0
  };
  crs_models::pacejka_model::pacejka_car_input current_input = { getRandomFloat(0.3, 1), 0 };

  Eigen::Matrix<double, 6, 6> F;
  Eigen::Matrix<double, 6, 2> D;
  model.getJacobian(current_state, current_input, 0.1, F, D);
}

/**
 * @brief Caclucates the jacobian of the discrete system
 */
TEST(PacejkaTests, testIntegrator)
{
  auto start = std::chrono::high_resolution_clock::now();
  crs_models::pacejka_model::pacejka_params params;
  params.lr = 0;
  params.lf = 0;
  params.m = 1;
  params.I = 1;
  params.Df = 0;
  params.Cf = 0;
  params.Bf = 0;
  params.Dr = 0;
  params.Cr = 0;
  params.Br = 0;
  params.Cm1 = 0;
  params.Cm2 = 1;
  params.Cd0 = 0;
  params.Cd1 = 0;
  params.Cd2 = 0;
  /* With these parameters, dynamics are reduced to

     f(x) = [
      v_x*cos(yaw) - v_y*sin(yaw),
      v_x * sin(yaw) + v_y * cos(yaw),
      yaw_rate,
      - v_x * torque - 1 * v_y * yaw_rate
      - 1 * v_x * yaw_rate,
      0
  ] */

  Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Identity();
  crs_models::pacejka_model::DiscretePacejkaModel model(params, Q);
  for (int run = 0; run < N_ITER; run++)
  {
    crs_models::pacejka_model::pacejka_car_state current_state = { getRandomFloat(-5, 5),  0, 0,
                                                                   getRandomFloat(0.4, 2), 0, 0 };
    crs_models::pacejka_model::pacejka_car_input current_input = { getRandomFloat(0.3, 1), 0 };

    // Equation is now (Assuming yaw = 0 const)
    // x_dot_dot = -x_dot * torque
    // --> x_dot(t) = x_dot(0) * e^-(torque*t)
    double ts = getRandomFloat(0.001, 0.2);
    auto vx_gt_solution = current_state.vel_x * exp(-current_input.torque * ts);
    crs_models::pacejka_model::pacejka_car_state output_state = model.applyModel(current_state, current_input, ts);
    auto vx_integrator_solution = output_state.vel_x;
    double squared_error = (vx_gt_solution - vx_integrator_solution) * (vx_gt_solution - vx_integrator_solution);
    EXPECT_LT(squared_error, 0.0001);
  }
  auto time_after = std::chrono::high_resolution_clock::now();
  std::cout << "Took " << std::chrono::duration_cast<std::chrono::microseconds>(time_after - start).count() / N_ITER
            << "us/iter" << std::endl;
}

/**
 * Tests the jacobian computation of the continuous pacejka model
 */
TEST(PacejkaTests, testContinuousJacobian)
{
  crs_models::pacejka_model::pacejka_params params;
  crs_models::pacejka_model::loadParamsFromFile("params/test_pacejka_params.yaml", params);

  //   crs_sensor_models::pacejka_sensor_models::WheelEncoderSensorModel sensor_model(params.wheel_radius, params.lf,
  //                                                                                  params.car_width);

  crs_models::pacejka_model::ContinuousPacejkaModel pacejka_model(params);
  crs_models::ContinuousDynamicModel<crs_models::pacejka_model::pacejka_car_state,
                                     crs_models::pacejka_model::pacejka_car_input>& model = pacejka_model;

  for (int run = 0; run < N_RUNS; run++)
  {
    // Load State from file
    std::stringstream ss;
    ss << "data/state_" << run << ".csv";
    Eigen::Matrix<double, 1, 6> state = readCSV(ss.str(), 1, 6);
    ss.str("");
    ss.clear();

    // Load Input from file
    ss << "data/control_input_" << run << ".csv";
    Eigen::Matrix<double, 1, 2> input = readCSV(ss.str(), 1, 2);
    ss.str("");
    ss.clear();

    // Load pre-computed jacobian from file
    ss << "data/jacobian_A_" << run << ".csv";
    Eigen::Matrix<double, 6, 6> A_gt = readCSV(ss.str(), 6, 6);
    ss.clear();

    ss << "data/jacobian_B_" << run << ".csv";
    // Eigen::Matrix<double, 6, 2> B_gt = readCSV(ss.str(), 6, 2);

    // Check pacejka model jacobian is correct
    Eigen::Matrix<double, 6, 6> A;
    Eigen::Matrix<double, 6, 2> B;
    A.setZero(6, 6);
    B.setZero(6, 2);
    crs_models::pacejka_model::pacejka_car_state gt_dynamic_state;
    crs_models::pacejka_model::pacejka_car_input current_input;
    gt_dynamic_state.pos_x = state(0);
    gt_dynamic_state.pos_y = state(1);
    gt_dynamic_state.yaw = state(2);
    gt_dynamic_state.vel_x = state(3);
    gt_dynamic_state.vel_y = state(4);
    gt_dynamic_state.yaw_rate = state(5);
    current_input.torque = input(0);
    current_input.steer = input(1);

    model.getNumericalJacobian(gt_dynamic_state, current_input, A, B);
    EXPECT_TRUE(MatrixEquality(A_gt, A));
    // EXPECT_TRUE(MatrixEquality(B_gt, B));
  }
}

int main(int ac, char* av[])
{
  testing::InitGoogleTest(&ac, av);
  return RUN_ALL_TESTS();
}
