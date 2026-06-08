#pragma once

#include <cstdint>

namespace crs_controls::kinematic_tracking_mpc::solvers
{

/**
 * @brief Enum to easily reference a specific entry of the state array
 *
 */
enum vars : std::uint8_t
{
  X,
  Y,
  YAW,
  V,
  TORQUE,
  STEER
};

/**
 * @brief Enum to easily reference a specific entry of the parameter array
 *
 */
enum params : std::uint8_t
{
  X_LIN,
  Y_LIN,
  Q1,
  Q2,
  R1,
  R2,
  L_REAR,
  L_FRONT,
  A,
  B,
  TAU
};

/**
 * @brief Enum to easily reference a specific entry of the input array
 *
 */
enum inputs : std::uint8_t
{
  DTORQUE,
  DSTEER
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
};

}  // namespace crs_controls::kinematic_tracking_mpc::solvers
