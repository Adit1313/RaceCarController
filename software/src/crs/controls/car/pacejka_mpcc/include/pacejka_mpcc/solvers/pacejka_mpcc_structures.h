#pragma once

#include <cstdint>

namespace crs_controls::pacejka_mpcc::solvers
{
/**
 * @brief Enum to easily reference a specific entry of the state array
 *
 */
enum class vars : std::uint8_t
{
  X,
  Y,
  YAW,
  VX,
  VY,
  DYAW,
  TORQUE,
  STEER,
  THETA,
};

enum class vars_curvilinear : std::uint8_t
{
  THETA,
  D,
  MU,
  VX,
  VY,
  DYAW,
  TORQUE,
  STEER,
};
/**
 * @brief Enum to easily reference a specific entry of the parameter array
 *
 */
enum class params : std::uint8_t
{
  X_LIN,
  Y_LIN,
  GRAD_X_LIN,
  GRAD_Y_LIN,
  THETA_PRED,
  PHI_LIN,
  Q1,
  Q2,
  R1,
  R2,
  R3,
  Q,
  L_REAR,
  L_FRONT,
  M,
  I,
  DF,
  CF,
  BF,
  DR,
  CR,
  BR,
  CM1,
  CM2,
  CD0,
  CD1,
  CD2,
  GAMMA,
  EPS,
  CAR_WIDTH,
  WHEEL_RADIUS
};

enum class params_curvilinear : std::uint8_t
{
  KAPPA,
  Q1,
  Q2,
  R1,
  R2,
  R3,
  Q,
  L_REAR,
  L_FRONT,
  M,
  I,
  DF,
  CF,
  BF,
  DR,
  CR,
  BR,
  CM1,
  CM2,
  CD0,
  CD1,
  CD2,
  GAMMA,
  EPS,
  CAR_WIDTH,
  WHEEL_RADIUS
};

/**
 * @brief Enum to easily reference a specific entry of the input array
 *
 */
enum inputs : std::uint8_t
{
  DTORQUE,
  DSTEER,
  DTHETA,
};

/**
 * @brief Struct containing the information for a reference point on the track
 *
 */
struct TrajectoryTrackPoint
{
  /**
   * @brief Reference point x coordinate
   *
   */
  double x;
  /**
   * @brief Reference point y coordinate
   *
   */
  double y;
  /**
   * @brief Gradient at reference point in x direction
   *
   */
  double grad_x;

  /**
   * @brief Gradient at reference point in y direction
   *
   */
  double grad_y;
  /**
   * @brief The current distance on the track at the given point
   *
   */
  double theta;
  /**
   * @brief The current angle of the track at the given point
   *
   */
  double phi;
};

/**
 * @brief Struct containing the information of a reference point on the track for the
 * curvilinear MPC controller
 */
struct TrajectoryReferenceCurvilinear
{
  /**
   * @brief Reference point curvature
   */
  double kappa;
};

/**
 * @brief Struct containing the costs of the mpc problem
 *
 */
struct TrackingCosts
{
  /**
   * @brief Contouring Cost
   *
   */
  double Q1;
  /**
   * @brief Lag cost
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
   * @brief darclength cost
   *
   */
  double R3;
  /**
   * @brief arclength cost
   *
   */
  double q;
};

}  // namespace crs_controls::pacejka_mpcc::solvers
