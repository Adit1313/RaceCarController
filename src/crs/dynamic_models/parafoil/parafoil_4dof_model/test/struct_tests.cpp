#include <gtest/gtest.h>
#include <casadi/casadi.hpp>
#include <iostream>
#include <random>

#include "parafoil_4dof_model/parafoil_4dof_input.h"
#include "parafoil_4dof_model/parafoil_4dof_state.h"
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
    float phiangle = distr(rd);
    float thetaangle = distr(rd);
    float psiangle = distr(rd);
    float vu = distr(rd);
    float vv = distr(rd);
    float vw = distr(rd);

    crs_models::parafoil_4dof_model::parafoil_4dof_state my_state = { posx,     posy, posz, phiangle, thetaangle,
                                                                      psiangle, vu,   vv,   vw };
    // Make sure the random velocities of the struct match
    EXPECT_EQ(posx, my_state.pos_x);
    EXPECT_EQ(posy, my_state.pos_y);
    EXPECT_EQ(posz, my_state.pos_z);
    EXPECT_EQ(phiangle, my_state.phi);
    EXPECT_EQ(thetaangle, my_state.theta);
    EXPECT_EQ(psiangle, my_state.psi);
    EXPECT_EQ(vu, my_state.vel_u);
    EXPECT_EQ(vv, my_state.vel_v);
    EXPECT_EQ(vw, my_state.vel_w);
  }
}

TEST(StateTest, arithmetics)
{
  /**
   * @brief Checks if the arithmetic operators are correctly implemented (i.e.  outputs anything)
   */

  // Create new struct object
  crs_models::parafoil_4dof_model::parafoil_4dof_state my_state_1 = {
    1.0, 1.0, 1.0, 0.0, 0.5, 0.0, 0.0, 0.5, 0.0
  };  // define struct my_state
  crs_models::parafoil_4dof_model::parafoil_4dof_state my_state_2 = {
    1.0, 1.0, 1.0, 0.0, 0.5, 0.0, 0.0, 0.5, 0.0
  };  // define struct my_state

  crs_models::parafoil_4dof_model::parafoil_4dof_state result_sum = {
    2.0, 2.0, 2.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0
  };  // correct result
  crs_models::parafoil_4dof_model::parafoil_4dof_state output_sum = my_state_1 + my_state_2;

  crs_models::parafoil_4dof_model::parafoil_4dof_state result_product = {
    2.0, 2.0, 2.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0
  };  // correct result
  crs_models::parafoil_4dof_model::parafoil_4dof_state output_product = 2 * my_state_1;

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
    float deflection_symmetric_input = distr(rd);
    float deflection_asymmetric_input = distr(rd);

    crs_models::parafoil_4dof_model::parafoil_4dof_input my_input = { deflection_symmetric_input,
                                                                      deflection_asymmetric_input };
    // Make sure the random inputs of the struct match
    EXPECT_EQ(deflection_symmetric_input, my_input.deflection_symmetric);
    EXPECT_EQ(deflection_asymmetric_input, my_input.deflection_asymmetric);
  }
}

TEST(InputTest, arithmetics)
{
  /**
   * @brief Checks if the arithmetic operators are correctly implemented (i.e.  outputs anything)
   */

  // Create new struct object
  crs_models::parafoil_4dof_model::parafoil_4dof_input my_input_1 = { 0.5, 0.5 };  // define struct my_input
  crs_models::parafoil_4dof_model::parafoil_4dof_input my_input_2 = { 0.5, 0.5 };  // define struct my_input
  crs_models::parafoil_4dof_model::parafoil_4dof_input result = { 1.0, 1.0 };      // correct result
  crs_models::parafoil_4dof_model::parafoil_4dof_input output = my_input_1 + my_input_2;
  EXPECT_EQ(result, output);
  my_input_1 += my_input_2;
  EXPECT_EQ(my_input_1, output);
}

/**
 * @brief Checks that the conversion from state to vector works as expected
 */
TEST(DataConversion, convert_to_vectors)
{
  crs_models::parafoil_4dof_model::parafoil_4dof_state state = {
    1.0, 1.0, 1.0, 0.0, 0.5, 0.0, 0.0, 0.5, 0.0
  };  // define struct state
  crs_models::parafoil_4dof_model::parafoil_4dof_state state_to_add = {
    1.0, 1.0, 1.0, 0.0, 0.5, 0.0, 0.0, 0.5, 0.0
  };  // define struct that will be added to state for tests

  crs_models::parafoil_4dof_model::parafoil_4dof_input input = { 0.5, 0.5 };         // define struct input
  crs_models::parafoil_4dof_model::parafoil_4dof_input input_to_add = { 0.5, 0.5 };  // define struct that will be added
                                                                                     // to input

  // Check Read only
  auto state_const_vector = commons::convertToConstVector(state);
  EXPECT_EQ(*state_const_vector[0], state.pos_x);
  EXPECT_EQ(*state_const_vector[1], state.pos_y);
  EXPECT_EQ(*state_const_vector[2], state.pos_y);
  EXPECT_EQ(*state_const_vector[3], state.phi);
  EXPECT_EQ(*state_const_vector[4], state.theta);
  EXPECT_EQ(*state_const_vector[5], state.psi);
  EXPECT_EQ(*state_const_vector[6], state.vel_u);
  EXPECT_EQ(*state_const_vector[7], state.vel_v);
  EXPECT_EQ(*state_const_vector[8], state.vel_w);

  auto input_const_vector = commons::convertToConstVector(input);
  EXPECT_EQ(*input_const_vector[0], input.deflection_symmetric);
  EXPECT_EQ(*input_const_vector[1], input.deflection_asymmetric);

  auto state_to_modify = state;
  // Check assigment
  auto state_writable_vector = commons::convertToVector(state_to_modify);
  (*state_writable_vector[0]) += state_to_add.pos_x;
  (*state_writable_vector[1]) += state_to_add.pos_y;
  (*state_writable_vector[2]) += state_to_add.pos_z;
  (*state_writable_vector[3]) += state_to_add.phi;
  (*state_writable_vector[4]) += state_to_add.theta;
  (*state_writable_vector[5]) += state_to_add.psi;
  (*state_writable_vector[6]) += state_to_add.vel_u;
  (*state_writable_vector[7]) += state_to_add.vel_v;
  (*state_writable_vector[8]) += state_to_add.vel_w;
  EXPECT_EQ(state_to_modify, state + state_to_add);

  auto input_to_modify = input;
  auto input_writable_vector = commons::convertToVector(input_to_modify);
  (*input_writable_vector[0]) += input_to_add.deflection_symmetric;
  (*input_writable_vector[1]) += input_to_add.deflection_asymmetric;
  EXPECT_EQ(input_to_modify, input + input_to_add);
}

int main(int ac, char* av[])
{
  testing::InitGoogleTest(&ac, av);
  return RUN_ALL_TESTS();
}
