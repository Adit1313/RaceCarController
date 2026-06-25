#pragma once

#include <string>

namespace crs_controls::pacejka_mpcc
{
struct mpcc_pacejka_config
{
  /**
   * @brief contouring cost
   *
   */
  double Q1;
  /**
   * @brief lag cost
   *
   */
  double Q2;
  /**
   * @brief dtorque cost
   *
   */
  double R1;
  /**
   * @brief dsteer cost
   *
   */
  double R2;
  /**
   * @brief  darclength cost
   *
   */
  double R3;
  /**
   * @brief arclength cost
   *
   */
  double q;
  /**
   * @brief State will be propagated by this amount before calculating control input
   *
   */
  double lag_compensation_time;
  /**
   * @brief Number of iterations to warmstart solver
   *
   */
  double warmstart_iterations = 1000;
  /**
   * @brief Type of solver that should be used. e.g. ACADOS
   *
   */
  std::string solver_type;

  /**
   * @brief How many iterations of SQP should be run
   */
  unsigned int max_sqp_iterations = 1;
};
}  // namespace crs_controls::pacejka_mpcc
