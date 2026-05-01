#ifndef PARAFOIL_4DOF_MODEL_TESTS_TEST_UTILS_H
#define PARAFOIL_4DOF_MODEL_TESTS_TEST_UTILS_H
#include <random>

#include "parafoil_4dof_model/parafoil_4dof_input.h"
#include "parafoil_4dof_model/parafoil_4dof_state.h"
#include "parafoil_4dof_model/parafoil_4dof_params.h"
#include <dynamic_models/utils/data_conversion.h>
#include <experimental/filesystem>

namespace fs = std::experimental::filesystem;

// For random number generation
std::random_device rd;
std::default_random_engine eng(rd());  // NOLINT
std::uniform_real_distribution<float> distr(0.0, 1.0);

float getRandomFloat(float lower_limit, float upper_limit)
{
  return (distr(rd) * (upper_limit - lower_limit)) + lower_limit;
}

void loadRandomParafoil4dofParams(crs_models::parafoil_4dof_model::parafoil_4dof_params& params, const std::string path)
{
  crs_models::parafoil_4dof_model::loadParamsFromFile(path, params);

  float rel_diff = 0.1;  // 10 percent difference
  params.rho *= 1 + getRandomFloat(-rel_diff, rel_diff);
  params.m *= 1 + getRandomFloat(-rel_diff, rel_diff);
  params.S *= 1 + getRandomFloat(-rel_diff, rel_diff);
  params.C_L0 *= 1 + getRandomFloat(-rel_diff, rel_diff);
  params.C_Ldelta_s *= 1 + getRandomFloat(-rel_diff, rel_diff);
  params.C_D0 *= 1 + getRandomFloat(-rel_diff, rel_diff);
  params.C_Ddelta_s *= 1 + getRandomFloat(-rel_diff, rel_diff);
  params.T_phi *= 1 + getRandomFloat(-rel_diff, rel_diff);
  params.K_phi *= 1 + getRandomFloat(-rel_diff, rel_diff);
  params.deflection_symmetric_max *= 1 + getRandomFloat(-rel_diff, rel_diff);
  params.deflection_asymmetric_max *= 1 + getRandomFloat(-rel_diff, rel_diff);
}

void loadRandomState(crs_models::parafoil_4dof_model::parafoil_4dof_state& state)
{
  state.pos_x = getRandomFloat(-10, 10);
  state.pos_y = getRandomFloat(-10, 10);
  state.pos_z = getRandomFloat(-10, 10);
  state.phi = getRandomFloat(-3.1415, 3.1415);
  state.theta = getRandomFloat(-1.5707, 1.5707);
  state.psi = getRandomFloat(-3.1415, 3.1415);
  state.vel_u = getRandomFloat(0, 15);
  state.vel_v = getRandomFloat(-5, 5);
  state.vel_w = getRandomFloat(-5, 15);
}

void loadRandomInput(crs_models::parafoil_4dof_model::parafoil_4dof_input& input)
{
  input.deflection_symmetric = getRandomFloat(-0.5, 0.5);
  input.deflection_asymmetric = getRandomFloat(-0.5, 0.5);
}

#endif  // PARAFOIL_4DOF_MODEL_TESTS_TEST_UTILS_H
