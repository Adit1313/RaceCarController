#ifndef MPC_CONTROLLER_KINEMATIC_CONTROLLER_TRACKING_MPC_KINEMATIC_CONFIG_H
#define MPC_CONTROLLER_KINEMATIC_CONTROLLER_TRACKING_MPC_KINEMATIC_CONFIG_H

#include <string>

namespace crs_controls
{
struct tracking_mpc_kinematic_config
{
  /**
   * @brief cost in x direction
   *
   */
  double Q1;
  /**
   * @brief cost in y direction
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
   * @brief State will be propagated by this amount before calculating control input
   *
   */
  double lag_compensation_time;
  /**
   * @brief Type of solver that should be used. e.g. ACADOS
   *
   */
  std::string solver_type;
};
}  // namespace crs_controls

#endif  // MPC_CONTROLLER_KINEMATIC_CONTROLLER_MPCC_KINEMATIC_CONFIG_H
