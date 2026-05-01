#ifndef SENSOR_MODEL_TESTS_TEST_UTILS_H
#define SENSOR_MODEL_TESTS_TEST_UTILS_H
#include <random>
#include <Eigen/Core>
#include <iostream>
#include <fstream>

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

#endif  // SENSOR_MODEL_TESTS_TEST_UTILS_H
