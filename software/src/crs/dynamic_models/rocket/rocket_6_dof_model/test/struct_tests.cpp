#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <casadi/casadi.hpp>
#include <iostream>
#include <random>

#include "rocket_6_dof_model/rocket_6_dof_input.h"
#include "rocket_6_dof_model/rocket_6_dof_state.h"
#include <dynamic_models/utils/data_conversion.h>

#define SUPRESS_OUTPUTS false

TEST(StateTest, struct_creation)
{
  /**
   * @brief Checks if the struct is created correctly (i.e. assigned the right values)
   */

  // Used to create random numbers
  std::random_device rd;
  std::default_random_engine eng(rd());  // NOLINT
  std::uniform_real_distribution<float> distr(0.0, 2.0);

  // Make 20 random  runs.
  for (int i = 0; i < 20; i++)
  {
    // Create random values for struct

    float posx = distr(rd);
    float posy = distr(rd);
    float posz = distr(rd);
    float vx = distr(rd);
    float vy = distr(rd);
    float vz = distr(rd);
    float quaternion_x = distr(rd);
    float quaternion_y = distr(rd);
    float quaternion_z = distr(rd);
    float quaternion_w = distr(rd);
    float angular_vx = distr(rd);
    float angular_vy = distr(rd);
    float angular_vz = distr(rd);
    float thrust_mag = distr(rd);
    float torque = distr(rd);
    float servo_1 = distr(rd);
    float servo_2 = distr(rd);

    crs_models::rocket_6_dof_model::rocket_6_dof_state my_state = {
      posx,         posy,       posz,       vx,         vy,         vz,     quaternion_x, quaternion_y, quaternion_z,
      quaternion_w, angular_vx, angular_vy, angular_vz, thrust_mag, torque, servo_1,      servo_2
    };
    // Make sure the random velocities of the struct match
    EXPECT_EQ(posx, my_state.position_x);
    EXPECT_EQ(posy, my_state.position_y);
    EXPECT_EQ(posz, my_state.position_z);
    EXPECT_EQ(vx, my_state.velocity_x);
    EXPECT_EQ(vy, my_state.velocity_y);
    EXPECT_EQ(vz, my_state.velocity_z);
    EXPECT_EQ(quaternion_x, my_state.quaternion_x);
    EXPECT_EQ(quaternion_y, my_state.quaternion_y);
    EXPECT_EQ(quaternion_z, my_state.quaternion_z);
    EXPECT_EQ(quaternion_w, my_state.quaternion_w);
    EXPECT_EQ(angular_vx, my_state.angular_velocity_x);
    EXPECT_EQ(angular_vy, my_state.angular_velocity_y);
    EXPECT_EQ(angular_vz, my_state.angular_velocity_z);
    EXPECT_EQ(thrust_mag, my_state.thrust_magnitude);
    EXPECT_EQ(torque, my_state.torque_x);
    EXPECT_EQ(servo_1, my_state.servo_angle_1);
    EXPECT_EQ(servo_2, my_state.servo_angle_2);
  }
}

TEST(StateTest, arithmetics)
{
  /**
   * @brief Checks if the arithmetic operators are correctly implemented (i.e.  outputs anything)
   */

  // Create new struct object
  crs_models::rocket_6_dof_model::rocket_6_dof_state my_state_1 = {
    1.0, 1.0, 0.0, 0.5, 0.5, 0.0, 3.0, 5.0, 6.0, 0.0, 0.0, 0.0, 1.0, 2.0, 0.0, 0.0, 4.0
  };  // define struct my_state
  crs_models::rocket_6_dof_model::rocket_6_dof_state my_state_2 = {
    1.0, 1.0, 0.0, 0.5, 0.5, 0.0, 3.0, 5.0, 6.0, 0.0, 0.0, 0.0, 1.0, 2.0, 0.0, 0.0, 4.0
  };  // define struct my_state

  crs_models::rocket_6_dof_model::rocket_6_dof_state result_sum = {
    2.0, 2.0, 0.0, 1.0, 1.0, 0.0, 6.0, 10.0, 12.0, 0.0, 0.0, 0.0, 2.0, 4.0, 0.0, 0.0, 8.0
  };  // correct result
  crs_models::rocket_6_dof_model::rocket_6_dof_state output_sum = my_state_1 + my_state_2;

  crs_models::rocket_6_dof_model::rocket_6_dof_state result_product = {
    2.0, 2.0, 0.0, 1.0, 1.0, 0.0, 6.0, 10.0, 12.0, 0.0, 0.0, 0.0, 2.0, 4.0, 0.0, 0.0, 8.0
  };  // correct result
  crs_models::rocket_6_dof_model::rocket_6_dof_state output_product = 2 * my_state_1;

  EXPECT_EQ(result_product, output_product);
  EXPECT_EQ(result_sum, output_sum);
  my_state_1 += my_state_2;
  EXPECT_EQ(my_state_1, output_sum);
}

//==============================================================================================================
TEST(InputTest, struct_creation)
{
  /**
   * @brief Checks if the struct is created correctly (i.e. assigned the right values)
   */

  // Used to create random numbers
  std::random_device rd;
  std::default_random_engine eng(rd());  // NOLINT
  std::uniform_real_distribution<float> distr(2.0, 0.0);

  // Make 20 random  runs.
  for (int i = 0; i < 20; i++)
  {
    // Create random values for struct
    float thrust_magnitude = distr(rd);
    float torque = distr(rd);
    float servo_angle_1 = distr(rd);
    float servo_angle_2 = distr(rd);

    crs_models::rocket_6_dof_model::rocket_6_dof_input my_input = { thrust_magnitude, torque, servo_angle_1,
                                                                    servo_angle_2 };
    // Make sure the random inputs of the struct match
    EXPECT_EQ(thrust_magnitude, my_input.thrust_magnitude);
    EXPECT_EQ(torque, my_input.torque);
    EXPECT_EQ(servo_angle_1, my_input.servo_angle_1);
    EXPECT_EQ(servo_angle_2, my_input.servo_angle_2);
  }
}

/**
 * @brief Checks that the conversion from state to vector works as expected
 */
TEST(DataConversion, convert_to_vectors)
{
  crs_models::rocket_6_dof_model::rocket_6_dof_state state = {
    1.0, 1.0, 0.0, 0.5, 0.5, 0.0, 3.0, 5.0, 6.0, 0.0, 0.0, 0.0, 1.0, 2.0, 0.0, 0.0, 4.0
  };  // define struct state

  crs_models::rocket_6_dof_model::rocket_6_dof_input input = { 1.0, 2.0, 3.0, 4.0 };  // define struct input

  // Check Read only
  auto state_const_vector = commons::convertToConstVector(state);
  EXPECT_EQ(*state_const_vector[0], state.position_x);
  EXPECT_EQ(*state_const_vector[1], state.position_y);
  EXPECT_EQ(*state_const_vector[2], state.position_z);
  EXPECT_EQ(*state_const_vector[3], state.velocity_x);
  EXPECT_EQ(*state_const_vector[4], state.velocity_y);
  EXPECT_EQ(*state_const_vector[5], state.velocity_z);
  EXPECT_EQ(*state_const_vector[6], state.quaternion_x);
  EXPECT_EQ(*state_const_vector[7], state.quaternion_y);
  EXPECT_EQ(*state_const_vector[8], state.quaternion_z);
  EXPECT_EQ(*state_const_vector[9], state.quaternion_w);
  EXPECT_EQ(*state_const_vector[10], state.angular_velocity_x);
  EXPECT_EQ(*state_const_vector[11], state.angular_velocity_y);
  EXPECT_EQ(*state_const_vector[12], state.angular_velocity_z);
  EXPECT_EQ(*state_const_vector[13], state.thrust_magnitude);
  EXPECT_EQ(*state_const_vector[14], state.torque_x);
  EXPECT_EQ(*state_const_vector[15], state.servo_angle_1);
  EXPECT_EQ(*state_const_vector[16], state.servo_angle_2);

  auto state_vector = commons::convertToVector(state);
  *state_vector[4] += 1.0;
  *state_vector[1] -= 13.0;
  *state_vector[9] *= 1.2;
  *state_vector[15] /= 0.9;
  EXPECT_EQ(*state_vector[0], state.position_x);
  EXPECT_EQ(*state_vector[1], state.position_y);
  EXPECT_EQ(*state_vector[2], state.position_z);
  EXPECT_EQ(*state_vector[3], state.velocity_x);
  EXPECT_EQ(*state_vector[4], state.velocity_y);
  EXPECT_EQ(*state_vector[5], state.velocity_z);
  EXPECT_EQ(*state_vector[6], state.quaternion_x);
  EXPECT_EQ(*state_vector[7], state.quaternion_y);
  EXPECT_EQ(*state_vector[8], state.quaternion_z);
  EXPECT_EQ(*state_vector[9], state.quaternion_w);
  EXPECT_EQ(*state_vector[10], state.angular_velocity_x);
  EXPECT_EQ(*state_vector[11], state.angular_velocity_y);
  EXPECT_EQ(*state_vector[12], state.angular_velocity_z);
  EXPECT_EQ(*state_vector[13], state.thrust_magnitude);
  EXPECT_EQ(*state_vector[14], state.torque_x);
  EXPECT_EQ(*state_vector[15], state.servo_angle_1);
  EXPECT_EQ(*state_vector[16], state.servo_angle_2);

  auto state_to_eigen = commons::convertToEigen<crs_models::rocket_6_dof_model::rocket_6_dof_state>(state);
  EXPECT_EQ(state_to_eigen[0], state.position_x);
  EXPECT_EQ(state_to_eigen[1], state.position_y);
  EXPECT_EQ(state_to_eigen[2], state.position_z);
  EXPECT_EQ(state_to_eigen[3], state.velocity_x);
  EXPECT_EQ(state_to_eigen[4], state.velocity_y);
  EXPECT_EQ(state_to_eigen[5], state.velocity_z);
  EXPECT_EQ(state_to_eigen[6], state.quaternion_x);
  EXPECT_EQ(state_to_eigen[7], state.quaternion_y);
  EXPECT_EQ(state_to_eigen[8], state.quaternion_z);
  EXPECT_EQ(state_to_eigen[9], state.quaternion_w);
  EXPECT_EQ(state_to_eigen[10], state.angular_velocity_x);
  EXPECT_EQ(state_to_eigen[11], state.angular_velocity_y);
  EXPECT_EQ(state_to_eigen[12], state.angular_velocity_z);
  EXPECT_EQ(state_to_eigen[13], state.thrust_magnitude);
  EXPECT_EQ(state_to_eigen[14], state.torque_x);
  EXPECT_EQ(state_to_eigen[15], state.servo_angle_1);
  EXPECT_EQ(state_to_eigen[16], state.servo_angle_2);

  auto eigen_to_state = commons::convertToState<crs_models::rocket_6_dof_model::rocket_6_dof_state>(state_to_eigen);
  EXPECT_EQ(eigen_to_state.position_x, state.position_x);
  EXPECT_EQ(eigen_to_state.position_y, state.position_y);
  EXPECT_EQ(eigen_to_state.position_z, state.position_z);
  EXPECT_EQ(eigen_to_state.velocity_x, state.velocity_x);
  EXPECT_EQ(eigen_to_state.velocity_y, state.velocity_y);
  EXPECT_EQ(eigen_to_state.velocity_z, state.velocity_z);
  EXPECT_EQ(eigen_to_state.quaternion_x, state.quaternion_x);
  EXPECT_EQ(eigen_to_state.quaternion_y, state.quaternion_y);
  EXPECT_EQ(eigen_to_state.quaternion_z, state.quaternion_z);
  EXPECT_EQ(eigen_to_state.quaternion_w, state.quaternion_w);
  EXPECT_EQ(eigen_to_state.angular_velocity_x, state.angular_velocity_x);
  EXPECT_EQ(eigen_to_state.angular_velocity_y, state.angular_velocity_y);
  EXPECT_EQ(eigen_to_state.angular_velocity_z, state.angular_velocity_z);
  EXPECT_EQ(eigen_to_state.thrust_magnitude, state.thrust_magnitude);
  EXPECT_EQ(eigen_to_state.torque_x, state.torque_x);
  EXPECT_EQ(eigen_to_state.servo_angle_1, state.servo_angle_1);
  EXPECT_EQ(eigen_to_state.servo_angle_2, state.servo_angle_2);

  auto input_const_vector = commons::convertToConstVector(input);
  EXPECT_EQ(*input_const_vector[0], input.thrust_magnitude);
  EXPECT_EQ(*input_const_vector[1], input.torque);
  EXPECT_EQ(*input_const_vector[2], input.servo_angle_1);
  EXPECT_EQ(*input_const_vector[3], input.servo_angle_2);
}

int main(int ac, char* av[])
{
  testing::InitGoogleTest(&ac, av);
  return RUN_ALL_TESTS();
}
