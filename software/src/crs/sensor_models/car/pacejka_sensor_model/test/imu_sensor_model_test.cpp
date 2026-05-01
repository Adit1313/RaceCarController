#include <gtest/gtest.h>
#include <Eigen/Core>
#include <casadi/casadi.hpp>
#include <iostream>
#include <random>

#include <pacejka_model/pacejka_car_input.h>
#include <pacejka_model/pacejka_car_state.h>
#include <pacejka_model/pacejka_continuous.h>
#include <pacejka_model/pacejka_discrete.h>
#include <pacejka_model/tests/test_utils.h>

#include <pacejka_sensor_model/imu_sensor_model.h>
#include <sensor_models/sensor_measurement.h>

#include <dynamic_models/utils/data_conversion.h>

#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#define N_RUNS 10

/**
 * Tests the forward computation of the sensor model
 */
TEST(IMU_TEST, testForwardModel)
{
  crs_models::pacejka_model::pacejka_params params;
  loadRandomPacejkaParams(params, "params/example_pacejka_params.yaml");

  auto model = std::make_shared<crs_models::pacejka_model::ContinuousPacejkaModel>(params);
  crs_sensor_models::pacejka_sensor_models::ImuSensorModel sensor_model(model);

  for (int run = 0; run < N_RUNS; run++)
  {
    crs_models::pacejka_model::pacejka_car_state gt_dynamic_state;
    loadRandomState(gt_dynamic_state);

    crs_models::pacejka_model::pacejka_car_input current_input;
    loadRandomInput(current_input);

    auto f_dot = model->applyModel(gt_dynamic_state, current_input);
    Eigen::Vector3d measurement = sensor_model.applyModel(gt_dynamic_state, current_input);

    // Check output is correct. Note since we are using the continuous model,
    // vel_x now refers to the acceleration in x direction (same for vel_y, yaw)
    EXPECT_DOUBLE_EQ(measurement(0), f_dot.vel_x);
    EXPECT_DOUBLE_EQ(measurement(1), f_dot.vel_y);
    EXPECT_DOUBLE_EQ(measurement(2), f_dot.yaw);
  }
}

/**
 * Tests the jacobian computation of the sensor model
 */
TEST(IMU_TEST, testJacobian)
{
  crs_models::pacejka_model::pacejka_params params;
  crs_models::pacejka_model::loadParamsFromFile("params/example_pacejka_params.yaml", params);

  std::shared_ptr<crs_models::pacejka_model::ContinuousPacejkaModel> model =
      std::make_shared<crs_models::pacejka_model::ContinuousPacejkaModel>(params);
  crs_sensor_models::pacejka_sensor_models::ImuSensorModel sensor_model(model);

  for (int run = 0; run < N_RUNS; run++)
  {
    // Load State from file
    std::stringstream ss;
    ss << "imu/data/state_" << run << ".csv";
    Eigen::Matrix<double, 1, 6> state = readCSV(ss.str(), 1, 6);
    ss.str("");
    ss.clear();

    // Load Input from file
    ss << "imu/data/control_input_" << run << ".csv";
    Eigen::Matrix<double, 1, 2> input = readCSV(ss.str(), 1, 2);
    ss.str("");
    ss.clear();

    // Load pre-computed jacobian from file
    ss << "imu/data/jacobian_" << run << ".csv";
    Eigen::Matrix<double, 3, 6> H_gt = readCSV(ss.str(), 3, 6);

    // Check sensor model jacobian is correct
    Eigen::Matrix<double, -1, 6> H;
    H.setZero(3, 6);
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

    sensor_model.getNumericalJacobian(gt_dynamic_state, current_input, H);
    EXPECT_TRUE(MatrixEquality(H_gt, H));
  }
}
int main(int ac, char* av[])
{
  testing::InitGoogleTest(&ac, av);
  return RUN_ALL_TESTS();
}
