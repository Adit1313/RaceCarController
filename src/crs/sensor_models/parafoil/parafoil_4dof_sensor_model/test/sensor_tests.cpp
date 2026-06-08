#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <casadi/casadi.hpp>
#include <iostream>
#include <math.h>
#include <random>

#include <parafoil_4dof_model/parafoil_4dof_input.h>
#include <parafoil_4dof_model/parafoil_4dof_state.h>
#include <parafoil_4dof_model/parafoil_4dof_params.h>

#include "parafoil_4dof_sensor_model/gps_sensor_model.h"
#include "parafoil_4dof_sensor_model/imu_sensor_model.h"

#include "parafoil_4dof_sensor_model/tests/test_utils.h"
#include <dynamic_models/utils/data_conversion.h>
#include <experimental/filesystem>

#define N_ITER 100

/**
 * @brief Checks if the `applyModel` function of the GPS sensor throws errors.
 * Currently, this check if pretty useless besides making sure doesn't explode.
 * TODO(norrisg) Check with GT dynamics or sth.
 */
TEST(SensorTests, testGPSApplyModel)
{
  // instantiate necessary structs
  crs_models::parafoil_4dof_model::parafoil_4dof_state current_state;
  crs_models::parafoil_4dof_model::parafoil_4dof_input current_input;

  // load random data
  loadRandomState(current_state);
  loadRandomInput(current_input);

  // instantiate GPS sensor object
  crs_sensor_models::parafoil_4dof_sensor_models::GPSSensorModel sensor;

  // get measurement from sensor
  Eigen::Matrix<double, -1, 1> gps_measurement = sensor.applyModel(current_state, current_input);
}

/**
 * @brief Checks if the `applyModel` function of the IMU sensor throws errors.
 * Currently, this check if pretty useless besides making sure doesn't explode.
 * TODO(norrisg) Check with GT dynamics or sth.
 */
TEST(SensorTests, testIMUApplyModel)
{
  // instantiate necessary structs
  crs_models::parafoil_4dof_model::parafoil_4dof_params params;
  crs_models::parafoil_4dof_model::parafoil_4dof_state current_state;
  crs_models::parafoil_4dof_model::parafoil_4dof_input current_input;

  // load random data (from file)
  loadRandomParafoil4dofParams(params, "params/example_parafoil_4dof_params.yaml");
  loadRandomState(current_state);
  loadRandomInput(current_input);

  // create pointer to continuous 4 DoF parafoil model
  auto model = std::make_shared<crs_models::parafoil_4dof_model::ContinuousParafoil4dofModel>(params);

  // instantiate IMU sensor object
  crs_sensor_models::parafoil_4dof_sensor_models::ImuSensorModel sensor(model);

  // get measurement from sensor
  Eigen::Matrix<double, -1, 1> imu_measurement = sensor.applyModel(current_state, current_input);
}
TEST(SensorTests, testGPSSensorModel)
{
  // instantiate necessary structs
  crs_models::parafoil_4dof_model::parafoil_4dof_params params;
  crs_models::parafoil_4dof_model::parafoil_4dof_state current_state;
  crs_models::parafoil_4dof_model::parafoil_4dof_input current_input;

  // load random data (from file)
  loadRandomParafoil4dofParams(params, "params/example_parafoil_4dof_params.yaml");

  // Test the model for multiple inputs
  for (int trials = 0; trials < 10; ++trials)
  {
    loadRandomState(current_state);
    loadRandomInput(current_input);

    // instantiate IMU sensor object
    crs_sensor_models::parafoil_4dof_sensor_models::GPSSensorModel sensor;

    // get measurement from sensor
    Eigen::Matrix<double, -1, 1> sensor_measurement = sensor.applyModel(current_state, current_input);

    Eigen::Matrix3d R_IB;
    R_IB(0, 0) = cos(current_state.psi) * 1;
    R_IB(0, 1) =
        (cos(current_state.psi) * 0 * sin(current_state.phi) - sin(current_state.psi) * cos(current_state.phi));
    R_IB(0, 2) =
        (cos(current_state.psi) * 0 * cos(current_state.phi) + sin(current_state.psi) * sin(current_state.phi));
    R_IB(1, 0) = sin(current_state.psi) * 1;
    R_IB(1, 1) =
        (sin(current_state.psi) * 0 * sin(current_state.phi) + cos(current_state.psi) * cos(current_state.phi));
    R_IB(1, 2) =
        (sin(current_state.psi) * 0 * cos(current_state.phi) - cos(current_state.psi) * sin(current_state.phi));
    R_IB(2, 0) = -0;
    R_IB(2, 1) = 1 * sin(current_state.phi);
    R_IB(2, 2) = 1 * cos(current_state.phi);

    Eigen::Vector3d I_vel = R_IB * Eigen::Vector3d(current_state.vel_u, current_state.vel_v, current_state.vel_w);

    std::vector<double> expected_states = {
      current_state.pos_x, current_state.pos_y, current_state.pos_z, I_vel[0], I_vel[1], I_vel[2]
    };

    for (size_t i = 0; i < expected_states.size(); ++i)
    {
      EXPECT_NEAR(sensor_measurement[i], expected_states[i], 0.01);
    }
  }
}

int main(int ac, char* av[])
{
  testing::InitGoogleTest(&ac, av);
  return RUN_ALL_TESTS();
}
