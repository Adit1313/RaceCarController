#include <gtest/gtest.h>
#include <Eigen/Core>
#include <casadi/casadi.hpp>
#include <iostream>
#include <random>

#include <imu_bias/imu_bias_continuous.h>
#include <stacked_model/pacejka_imu_bias_car_state.h>
#include <stacked_model/pacejka_imu_bias_car_input.h>
#include <stacked_model/stacked_two_models_continuous.h>

#include <pacejka_with_imu_bias_sensor_model/imu_with_bias_sensor_model.h>

#include <pacejka_model/pacejka_continuous.h>

#include <sensor_models/sensor_measurement.h>
#include <sensor_models/sensor_model.h>

#include <dynamic_models/utils/data_conversion.h>

#include <pacejka_model/tests/test_utils.h>

#include <fstream>
#include <Eigen/Dense>

#define N_RUNS 10

using StackedPacejkaImuModel = crs_models::stacked_model::ContinuousTwoStackedModels<
    crs_models::stacked_model::pacejka_imu_bias_car_state, crs_models::stacked_model::pacejka_imu_bias_car_input,
    crs_models::pacejka_model::ContinuousPacejkaModel, crs_models::imu_bias::ContinuousImuBias>;

/**
 * Tests the forward computation of the sensor model
 */
TEST(IMU_BIAS_TEST, testForwardModel)
{
  crs_models::pacejka_model::pacejka_params params;
  loadRandomPacejkaParams(params, "params/example_pacejka_params.yaml");

  std::shared_ptr<crs_models::pacejka_model::ContinuousPacejkaModel> pacejka_model =
      std::make_shared<crs_models::pacejka_model::ContinuousPacejkaModel>(params);

  std::shared_ptr<crs_models::imu_bias::ContinuousImuBias> imu_bias_model =
      std::make_shared<crs_models::imu_bias::ContinuousImuBias>();

  std::shared_ptr<StackedPacejkaImuModel> stacked_model =
      std::make_shared<StackedPacejkaImuModel>(pacejka_model, imu_bias_model);

  crs_sensor_models::pacejka_sensor_models::ImuWithBiasSensorModel sensor_model(stacked_model);

  for (int run = 0; run < N_RUNS; run++)
  {
    crs_models::stacked_model::pacejka_imu_bias_car_state gt_dynamic_state;
    loadRandomState(gt_dynamic_state);

    crs_models::stacked_model::pacejka_imu_bias_car_input current_input;
    loadRandomInput(current_input);

    auto f_dot = stacked_model->applyModel(gt_dynamic_state, current_input);
    Eigen::Vector3d measurement = sensor_model.applyModel(gt_dynamic_state, current_input);

    // Check output is correct. Note since we are using the continuous model,
    // vel_x now refers to the acceleration in x direction (same for vel_y, yaw)
    EXPECT_DOUBLE_EQ(measurement(0), f_dot.vel_x + f_dot.bias_ax);
    EXPECT_DOUBLE_EQ(measurement(1), f_dot.vel_y + f_dot.bias_ay);
    EXPECT_DOUBLE_EQ(measurement(2), f_dot.yaw + f_dot.bias_dyaw);
  }
}

/**
 * Tests the jacobian computation of the sensor model
 */
TEST(IMU_BIAS_TEST, testJacobian)
{
  crs_models::pacejka_model::pacejka_params params;
  crs_models::pacejka_model::loadParamsFromFile("params/example_pacejka_params.yaml", params);

  std::shared_ptr<crs_models::pacejka_model::ContinuousPacejkaModel> pacejka_model =
      std::make_shared<crs_models::pacejka_model::ContinuousPacejkaModel>(params);

  std::shared_ptr<crs_models::imu_bias::ContinuousImuBias> imu_bias_model =
      std::make_shared<crs_models::imu_bias::ContinuousImuBias>();

  std::shared_ptr<StackedPacejkaImuModel> stacked_model =
      std::make_shared<StackedPacejkaImuModel>(pacejka_model, imu_bias_model);

  crs_sensor_models::pacejka_sensor_models::ImuWithBiasSensorModel sensor_model(stacked_model);

  for (int run = 0; run < N_RUNS; run++)
  {
    // Load State from file
    std::stringstream ss;
    ss << "imu_bias/data/state_" << run << ".csv";
    Eigen::Matrix<double, 1, 9> state = readCSV(ss.str(), 1, 9);
    ss.str("");
    ss.clear();

    // Load Input from file
    ss << "imu_bias/data/control_input_" << run << ".csv";
    Eigen::Matrix<double, 1, 2> input = readCSV(ss.str(), 1, 2);
    ss.str("");
    ss.clear();

    // Load pre-computed jacobian from file
    ss << "imu_bias/data/jacobian_" << run << ".csv";
    Eigen::Matrix<double, 3, 9> H_gt = readCSV(ss.str(), 3, 9);

    // Check sensor model jacobian is correct
    Eigen::Matrix<double, -1, 9> H;
    H.setZero(3, 9);
    crs_models::stacked_model::pacejka_imu_bias_car_state gt_dynamic_state;
    crs_models::stacked_model::pacejka_imu_bias_car_input current_input;
    gt_dynamic_state.pos_x = state(0);
    gt_dynamic_state.pos_y = state(1);
    gt_dynamic_state.yaw = state(2);
    gt_dynamic_state.vel_x = state(3);
    gt_dynamic_state.vel_y = state(4);
    gt_dynamic_state.yaw_rate = state(5);
    gt_dynamic_state.bias_ax = state(6);
    gt_dynamic_state.bias_ay = state(7);
    gt_dynamic_state.bias_dyaw = state(8);
    current_input.torque = input(0);
    current_input.steer = input(1);

    sensor_model.getNumericalJacobian(gt_dynamic_state, current_input, H);
    EXPECT_TRUE(MatrixEquality(H_gt, H));
  }
}

int main(int ac, char* av[])
{
  testing::InitGoogleTest(&ac, av);
  return RUN_ALL_TESTS();
}
