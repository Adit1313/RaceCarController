#ifndef PACEJKA_MODEL_TESTS_TEST_UTILS_H
#define PACEJKA_MODEL_TESTS_TEST_UTILS_H
#include <random>
#include <Eigen/Core>
#include <iostream>
#include <fstream>

#include "pacejka_model/pacejka_car_input.h"
#include "pacejka_model/pacejka_car_state.h"
#include "pacejka_model/pacejka_params.h"
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

void loadRandomPacejkaParams(crs_models::pacejka_model::pacejka_params& params, const std::string path)
{
  crs_models::pacejka_model::loadParamsFromFile(path, params);

  float rel_diff = 0.1;  // 10 percent difference
  params.lr *= 1 + getRandomFloat(-rel_diff, rel_diff);
  params.lf *= 1 + getRandomFloat(-rel_diff, rel_diff);
  params.m *= 1 + getRandomFloat(-rel_diff, rel_diff);
  params.I *= 1 + getRandomFloat(-rel_diff, rel_diff);
  params.wheel_radius *= 1 + getRandomFloat(-rel_diff, rel_diff);
  params.car_width *= 1 + getRandomFloat(-rel_diff, rel_diff);

  params.Df *= 1 + getRandomFloat(-rel_diff, rel_diff);
  params.Cf *= 1 + getRandomFloat(-rel_diff, rel_diff);
  params.Bf *= 1 + getRandomFloat(-rel_diff, rel_diff);
  params.Dr *= 1 + getRandomFloat(-rel_diff, rel_diff);
  params.Cr *= 1 + getRandomFloat(-rel_diff, rel_diff);
  params.Br *= 1 + getRandomFloat(-rel_diff, rel_diff);
  params.Cm1 *= 1 + getRandomFloat(-rel_diff, rel_diff);
  params.Cm2 *= 1 + getRandomFloat(-rel_diff, rel_diff);
  params.Cd0 *= 1 + getRandomFloat(-rel_diff, rel_diff);
  params.Cd1 *= 1 + getRandomFloat(-rel_diff, rel_diff);
  params.Cd2 *= 1 + getRandomFloat(-rel_diff, rel_diff);
}

void loadRandomState(crs_models::pacejka_model::pacejka_car_state& state)
{
  state.pos_x = getRandomFloat(-10, 10);
  state.pos_y = getRandomFloat(-10, 10);
  state.yaw = getRandomFloat(-3.1415, 3.1415);
  state.vel_x = getRandomFloat(0.5, 1);
  state.vel_y = getRandomFloat(-0.5, 0.5);
  state.yaw_rate = getRandomFloat(-0.5, 0.5);
}

void loadRandomInput(crs_models::pacejka_model::pacejka_car_input& input)
{
  input.torque = getRandomFloat(0.1, 1);
  input.steer = getRandomFloat(-0.4, 0.4);
}

/**
 * @brief Checks if two matrices are equal. If not, prints the matrices and their difference
 */
bool MatrixEquality(const Eigen::MatrixXd& lhs, const Eigen::MatrixXd& rhs)
{
  if (lhs.isApprox(rhs, 1e-4))
  {
    return true;
  }
  std::cerr << "Matrix Equality Failed" << std::endl;
  std::cerr << "lhs: " << std::endl;
  std::cerr << lhs << std::endl;
  std::cerr << "rhs: " << std::endl;
  std::cerr << rhs << std::endl;
  std::cerr << "diff: " << std::endl;
  std::cerr << (lhs - rhs) << std::endl;
  return false;
}

/**
 * @brief Reads a csv file and returns a matrix
 */
Eigen::MatrixXd readCSV(std::string file, int rows, int cols)
{
  std::ifstream in(file);

  std::string line;

  int row = 0;
  int col = 0;

  Eigen::MatrixXd res = Eigen::MatrixXd(rows, cols);

  if (in.is_open())
  {
    while (std::getline(in, line))
    {
      char* ptr = (char*)line.c_str();
      int len = line.length();

      col = 0;

      char* start = ptr;
      for (int i = 0; i < len; i++)
      {
        if (ptr[i] == ',')
        {
          res(row, col++) = atof(start);
          start = ptr + i + 1;
        }
      }
      res(row, col) = atof(start);

      row++;
    }

    in.close();
  }
  return res;
}

#endif  // PACEJKA_MODEL_TESTS_TEST_UTILS_H
