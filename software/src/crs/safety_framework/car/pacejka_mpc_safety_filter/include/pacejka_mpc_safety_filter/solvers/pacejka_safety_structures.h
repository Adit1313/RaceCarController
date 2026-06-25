#pragma once

#include <cstdint>

namespace crs_safety::pacejka_mpc_safety_filter::solvers
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
  VX,
  VY,
  DYAW,
  TORQUE,
  STEER
};

/**
 * @brief Enum to easily reference a specific entry of the parameter array
 *
 */

enum parameters : std::uint8_t
{
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
  GAMMA,  // Split of the longitudinal force between front and rear wheels
  EPS,    // velocity at which the approximated polynomial is applied to the slip angle

  CAR_WIDTH,     // Width of the car
  CAR_OVERHANG,  // Distance between front wheel axis to front of the car

  XP_TRACK,   // Current reference point on track
  YP_TRACK,   // Current reference point on track
  YAW_TRACK,  // Current reference yaw on track

  XP_E,  // Terminal point
  YP_E,  // Terminal point
  YAW_E  // Terminal point
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

struct ReferenceInput
{
  double torque;
  double steer;

  double cost_torque;
  double cost_steer;

  double cost_delta_torque;
  double cost_delta_steer;
};

/**
 * @brief Struct containing the information for a reference point on the track
 *
 */
struct ReferenceOnTrack
{
  double xp_track;
  double yp_track;
  double yaw_track;
  double xp_e;
  double yp_e;
  double yaw_e;
};

}  // namespace crs_safety::pacejka_mpc_safety_filter::solvers
