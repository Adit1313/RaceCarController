#include <gtest/gtest.h>
#include <Eigen/Core>
#include <casadi/casadi.hpp>
#include <iostream>
#include <random>

#include <imu_bias/imu_bias_continuous.h>
#include <stacked_model/pacejka_imu_bias_car_state.h>
#include <stacked_model/pacejka_imu_bias_car_input.h>
#include <stacked_model/stacked_two_models_continuous.h>

#include <pacejka_with_imu_bias_sensor_model/mocap_with_imu_bias_sensor_model.h>

#include <pacejka_model/pacejka_continuous.h>

#include <sensor_models/sensor_measurement.h>
#include <sensor_models/sensor_model.h>

#include <dynamic_models/utils/data_conversion.h>

#include <pacejka_model/tests/test_utils.h>

#include <fstream>
#include <Eigen/Dense>

#define N_RUNS 10

/**
 * Tests the forward computation of the sensor model
 */
TEST(MOCAP_WITH_IMU_BIAS_TEST, testForwardModel)
{
  crs_sensor_models::pacejka_sensor_models::MocapWithImuBiasSensorModel sensor_model;

  for (int run = 0; run < N_RUNS; run++)
  {
    crs_models::stacked_model::pacejka_imu_bias_car_state gt_dynamic_state;
    loadRandomState(gt_dynamic_state);

    crs_models::stacked_model::pacejka_imu_bias_car_input current_input;
    loadRandomInput(current_input);

    Eigen::Vector3d measurement = sensor_model.applyModel(gt_dynamic_state, current_input);

    // Check output is correct
    EXPECT_DOUBLE_EQ(measurement(0), gt_dynamic_state.pos_x);
    EXPECT_DOUBLE_EQ(measurement(1), gt_dynamic_state.pos_y);
    EXPECT_DOUBLE_EQ(measurement(2), gt_dynamic_state.yaw);
  }
}

/**
 * Tests the jacobian computation of the sensor model
 */
TEST(MOCAP_WITH_IMU_BIAS_TEST, testJacobian)
{
  crs_sensor_models::pacejka_sensor_models::MocapWithImuBiasSensorModel sensor_model;

  Eigen::Matrix<double, 3, 9> H_gt;
  H_gt << 1, 0, 0, 0, 0, 0, 0, 0, 0,  //
      0, 1, 0, 0, 0, 0, 0, 0, 0,      //
      0, 0, 1, 0, 0, 0, 0, 0, 0;

  for (int run = 0; run < N_RUNS; run++)
  {
    crs_models::stacked_model::pacejka_imu_bias_car_state gt_dynamic_state;
    loadRandomState(gt_dynamic_state);

    crs_models::stacked_model::pacejka_imu_bias_car_input current_input;
    loadRandomInput(current_input);

    Eigen::Matrix<double, -1, 9> H;
    H.setZero(3, 9);

    sensor_model.getNumericalJacobian(gt_dynamic_state, current_input, H);

    ASSERT_PRED2(MatrixEquality, H_gt, H);
  }
}

int main(int ac, char* av[])
{
  testing::InitGoogleTest(&ac, av);
  return RUN_ALL_TESTS();
}
