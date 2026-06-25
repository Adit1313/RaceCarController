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

#include <pacejka_sensor_model/lighthouse_sensor_model.h>
#include <sensor_models/sensor_measurement.h>

#include <dynamic_models/utils/data_conversion.h>

#define N_RUNS 10

void loadRandomLighthouseParams(Eigen::Matrix<double, 2, 4>& sensor_positions, Eigen::Vector3d& bs_position,
                                Eigen::Matrix3d& bs_rotation, double& light_plane_tilt, double noise_level = 0.01)
{
  sensor_positions << 0.035, 0.02, 0.035, 0.02, 0.015, 0.015, -0.015, -0.015;
  bs_position << 0.423652, 0.449405, 1.99268;
  bs_rotation << -0.0939952, 0.993122, -0.0698045, -0.0609725, -0.0757257, -0.995263, -0.993704, -0.0892937, 0.067671;
  light_plane_tilt = -0.0508728;

  // Add random offset to lighthouse params
  sensor_positions += Eigen::Matrix<double, 2, 4>::Random() * noise_level;
  bs_position += Eigen::Vector3d::Random() * noise_level;
  bs_rotation += Eigen::Matrix3d::Random() * noise_level;
  light_plane_tilt += getRandomFloat(-noise_level, noise_level);
}

void computeAngles(const crs_models::pacejka_model::pacejka_car_state& state, Eigen::Vector3d position,
                   Eigen::Matrix3d rotation, double light_plane_tilt, Eigen::Matrix<double, 2, 4> sensor_positions,
                   Eigen::Vector4d& angles)
{
  double x = state.pos_x;
  double y = state.pos_y;
  double yaw = state.yaw;

  Eigen::Matrix<casadi::MX, 3, 2> rot;
  rot << cos(yaw), -sin(yaw), sin(yaw), cos(yaw), 0, 0;
  // Position of the car in world frame
  Eigen::Matrix<casadi::MX, 3, 1> pos_car{ x, y, 0 };
  // Positions of the sensors in world frame
  Eigen::Matrix<casadi::MX, 3, 4> pos_sensor = (rot * sensor_positions.cast<casadi::MX>()).colwise() + pos_car;
  // Base station rotation matrix and position vector
  Eigen::Matrix<casadi::MX, 3, 3> rot_bs = rotation.transpose().cast<casadi::MX>();
  Eigen::Matrix<casadi::MX, 3, 1> pos_bs = position.cast<casadi::MX>();
  // Positions in base satation reference frame
  Eigen::Matrix<casadi::MX, 3, 4> sbs = rot_bs * (pos_sensor.colwise() - pos_bs);
  // Calculate angles
  Eigen::Matrix<casadi::MX, 1, 4> alphas = (sbs.row(1).array() / sbs.row(0).array()).atan();
  Eigen::Matrix<casadi::MX, 1, 4> alpha =
      alphas.array() + ((sbs.row(2) * tan(light_plane_tilt)).array() /
                        (sbs.row(0).array().square() + sbs.row(1).array().square()).sqrt())
                           .asin();
  angles = alpha.cast<double>();
}

/**
 * Tests the forward computation of the sensor model
 */
TEST(LIGTHOUSE_TEST, testForwardModel)
{
  crs_models::pacejka_model::pacejka_params params;
  loadRandomPacejkaParams(params, "params/example_pacejka_params.yaml");

  Eigen::Matrix<double, 2, 4> sensor_positions;
  Eigen::Vector3d bs_position;
  Eigen::Matrix3d bs_rotation;
  double light_plane_tilt;

  for (int run = 0; run < N_RUNS; run++)
  {
    loadRandomLighthouseParams(sensor_positions, bs_position, bs_rotation, light_plane_tilt);

    crs_sensor_models::pacejka_sensor_models::LighthouseSensorModel sensor_model(
        Eigen::Matrix4d::Identity(), bs_position, bs_rotation, light_plane_tilt, sensor_positions);

    crs_models::pacejka_model::pacejka_car_state gt_dynamic_state;
    loadRandomState(gt_dynamic_state);

    crs_models::pacejka_model::pacejka_car_input current_input;
    loadRandomInput(current_input);

    // Compute lighthouse angle measurements for gt
    Eigen::Vector4d angles;
    computeAngles(gt_dynamic_state, bs_position, bs_rotation, light_plane_tilt, sensor_positions, angles);

    // Compute lighthouse angle measurements using sensor model
    Eigen::Matrix<double, 4, 1> measurement = sensor_model.applyModel(gt_dynamic_state, current_input);

    // Check output is correct
    EXPECT_DOUBLE_EQ(measurement(0), angles(0));
    EXPECT_DOUBLE_EQ(measurement(1), angles(1));
    EXPECT_DOUBLE_EQ(measurement(2), angles(2));
    EXPECT_DOUBLE_EQ(measurement(3), angles(3));
  }
}

/**
 * Tests the jacobian computation of the sensor model
 */
TEST(LIGTHOUSE_TEST, testJacobian)
{
  crs_models::pacejka_model::pacejka_params params;
  crs_models::pacejka_model::loadParamsFromFile("params/example_pacejka_params.yaml", params);

  // TODO: load these as parameters and add random variances in vlaues
  Eigen::Matrix<double, 2, 4> sensor_positions;
  Eigen::Vector3d bs_position;
  Eigen::Matrix3d bs_rotation;
  double light_plane_tilt;

  // No noise added to lighthouse params since python jacobians were computed without noise
  loadRandomLighthouseParams(sensor_positions, bs_position, bs_rotation, light_plane_tilt, 0.0);

  crs_sensor_models::pacejka_sensor_models::LighthouseSensorModel sensor_model(
      Eigen::Matrix4d::Identity(), bs_position, bs_rotation, light_plane_tilt, sensor_positions);

  for (int run = 0; run < N_RUNS; run++)
  {
    // Load State from file
    std::stringstream ss;
    ss << "lighthouse/data/state_" << run << ".csv";
    Eigen::Matrix<double, 1, 6> state = readCSV(ss.str(), 1, 6);
    ss.str("");
    ss.clear();

    // Load Input from file
    ss << "lighthouse/data/control_input_" << run << ".csv";
    Eigen::Matrix<double, 1, 2> input = readCSV(ss.str(), 1, 2);
    ss.str("");
    ss.clear();

    // Load pre-computed jacobian from file
    ss << "lighthouse/data/jacobian_" << run << ".csv";
    Eigen::Matrix<double, 4, 6> H_gt = readCSV(ss.str(), 4, 6);

    // Check sensor model jacobian is correct
    Eigen::Matrix<double, -1, 6> H;
    H.setZero(4, 6);
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
