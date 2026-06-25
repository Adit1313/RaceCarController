#ifndef ROCKET_6_DOF_MODEL_ROCKET_6_DOF_INPUT_H
#define ROCKET_6_DOF_MODEL_ROCKET_6_DOF_INPUT_H

#include <iostream>

namespace crs_models
{
namespace rocket_6_dof_model
{

struct rocket_6_dof_input
{
  double thrust_magnitude;
  double torque;
  double servo_angle_1;
  double servo_angle_2;

  static constexpr int NU = 4;

  rocket_6_dof_input() : thrust_magnitude(0), torque(0), servo_angle_1(0), servo_angle_2(0){};  // default constructor
  rocket_6_dof_input(double thrust_magnitude, double torque, double servo_angle_1, double servo_angle_2)
    : thrust_magnitude(thrust_magnitude)
    , torque(torque)
    , servo_angle_1(servo_angle_1)
    , servo_angle_2(servo_angle_2){};  // Constructor
};

inline rocket_6_dof_input operator+(const rocket_6_dof_input& a, const rocket_6_dof_input& b)
{
  rocket_6_dof_input added_struct;

  added_struct.thrust_magnitude = a.thrust_magnitude + b.thrust_magnitude;
  added_struct.torque = a.torque + b.torque;
  added_struct.servo_angle_1 = a.servo_angle_1 + b.servo_angle_1;
  added_struct.servo_angle_2 = a.servo_angle_2 + b.servo_angle_2;

  return added_struct;
}

inline void operator+=(rocket_6_dof_input& a, const rocket_6_dof_input& b)
{
  a.thrust_magnitude = a.thrust_magnitude + b.thrust_magnitude;
  a.torque = a.torque + b.torque;
  a.servo_angle_1 = a.servo_angle_1 + b.servo_angle_1;
  a.servo_angle_2 = a.servo_angle_2 + b.servo_angle_2;
}

inline bool operator==(const rocket_6_dof_input& a, const rocket_6_dof_input& b)
{
  return ((a.thrust_magnitude == b.thrust_magnitude) && (a.torque == b.torque) &&
          (a.servo_angle_1 == b.servo_angle_1) && (a.servo_angle_2 == b.servo_angle_2));
}

inline std::ostream& operator<<(std::ostream& os,
                                const rocket_6_dof_input& input)  // how to print struct (input) to output stream
                                                                  // (os)
{
  os << "Rocket Input:" << std::endl;
  os << " thrust_magnitude: " << std::to_string(input.thrust_magnitude) << std::endl;  // field of struct (input)
  os << " torque: " << std::to_string(input.torque) << std::endl;                      // field of struct (input)
  os << " servo_angle_1: " << std::to_string(input.servo_angle_1) << std::endl;        // field of struct (input)
  os << " servo_angle_2: " << std::to_string(input.servo_angle_2) << std::endl;        // field of struct (input)
  return os;
}

}  // namespace rocket_6_dof_model
}  // namespace crs_models
#endif
