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

#include <pacejka_sensor_model/wheel_encoder_sensor_model.h>
#include <sensor_models/sensor_measurement.h>

#include <dynamic_models/utils/data_conversion.h>

#include <iostream>
#include <fstream>
#include <Eigen/Dense>

#define N_RUNS 10

void computeWheelSpeeds(const crs_models::pacejka_model::pacejka_car_state& state,
                        const crs_models::pacejka_model::pacejka_car_input& input, double wheel_radius, double lf,
                        double car_width, Eigen::Vector4d& wheel_speeds)
{
  double vx = state.vel_x;
  double vy = state.vel_y;
  double dyaw = state.yaw_rate;
  double steer = input.steer;

  auto vx_fl = vx - 0.5 * car_width * dyaw;
  auto vy_fl = vy + lf * dyaw;
  auto vx_wheel_fl = vx_fl * cos(steer) + vy_fl * sin(steer);

  auto vx_fr = vx + 0.5 * car_width * dyaw;
  auto vy_fr = vy + lf * dyaw;
  auto vx_wheel_fr = vx_fr * cos(steer) + vy_fr * sin(steer);

  auto vx_wheel_rl = vx - 0.5 * car_width * dyaw;
  auto vx_wheel_rr = vx + 0.5 * car_width * dyaw;

  wheel_speeds << vx_wheel_fl / wheel_radius, vx_wheel_fr / wheel_radius, vx_wheel_rl / wheel_radius,
      vx_wheel_rr / wheel_radius;
}

/**
 * Tests the forward computation of the sensor model
 */
TEST(WHEEL_ENCODERS_TEST, testForwardModel)
{
  crs_models::pacejka_model::pacejka_params params;
  loadRandomPacejkaParams(params, "params/example_pacejka_params.yaml");

  crs_sensor_models::pacejka_sensor_models::WheelEncoderSensorModel sensor_model(params.wheel_radius, params.lf,
                                                                                 params.car_width);

  for (int run = 0; run < N_RUNS; run++)
  {
    crs_models::pacejka_model::pacejka_car_state gt_dynamic_state;
    loadRandomState(gt_dynamic_state);

    crs_models::pacejka_model::pacejka_car_input current_input;
    loadRandomInput(current_input);

    // Compute wheel encoder measurements for gt
    Eigen::Vector4d wheel_speeds;
    computeWheelSpeeds(gt_dynamic_state, current_input, params.wheel_radius, params.lf, params.car_width, wheel_speeds);

    // Compute wheel encoder measurements using sensor model
    Eigen::Vector4d measurement = sensor_model.applyModel(gt_dynamic_state, current_input);

    // Check output is correct
    EXPECT_DOUBLE_EQ(measurement(0), wheel_speeds(0));
    EXPECT_DOUBLE_EQ(measurement(1), wheel_speeds(1));
    EXPECT_DOUBLE_EQ(measurement(2), wheel_speeds(2));
    EXPECT_DOUBLE_EQ(measurement(3), wheel_speeds(3));
  }

  // Special case of yaw_rate = 0 and steer = 0
  for (int run = 0; run < N_RUNS; run++)
  {
    crs_models::pacejka_model::pacejka_car_state state;
    loadRandomState(state);
    state.yaw_rate = 0;
    crs_models::pacejka_model::pacejka_car_input input;
    loadRandomInput(input);
    input.steer = 0;
    Eigen::Vector4d wheel_speeds;
    wheel_speeds << state.vel_x / params.wheel_radius, state.vel_x / params.wheel_radius,
        state.vel_x / params.wheel_radius, state.vel_x / params.wheel_radius;

    // Compute wheel encoder measurements using sensor model
    Eigen::Vector4d measurement = sensor_model.applyModel(state, input);

    // Check output is correct
    EXPECT_DOUBLE_EQ(measurement(0), wheel_speeds(0));
    EXPECT_DOUBLE_EQ(measurement(1), wheel_speeds(1));
    EXPECT_DOUBLE_EQ(measurement(2), wheel_speeds(2));
    EXPECT_DOUBLE_EQ(measurement(3), wheel_speeds(3));
  }
}

/**
 * Tests the jacobian computation of the sensor model
 */
TEST(WHEEL_ENCODERS_TEST, testJacobian)
{
  crs_models::pacejka_model::pacejka_params params;
  crs_models::pacejka_model::loadParamsFromFile("params/example_pacejka_params.yaml", params);

  crs_sensor_models::pacejka_sensor_models::WheelEncoderSensorModel sensor_model(params.wheel_radius, params.lf,
                                                                                 params.car_width);

  for (int run = 0; run < N_RUNS; run++)
  {
    // Load State from file
    std::stringstream ss;
    ss << "wheel_encoders/data/state_" << run << ".csv";
    Eigen::Matrix<double, 1, 6> state = readCSV(ss.str(), 1, 6);
    ss.str("");
    ss.clear();

    // Load Input from file
    ss << "wheel_encoders/data/control_input_" << run << ".csv";
    Eigen::Matrix<double, 1, 2> input = readCSV(ss.str(), 1, 2);
    ss.str("");
    ss.clear();

    // Load pre-computed jacobian from file
    ss << "wheel_encoders/data/jacobian_" << run << ".csv";
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
