#ifndef KINEMATIC_MODEL_KINEMATIC_CAR_INPUT_H
#define KINEMATIC_MODEL_KINEMATIC_CAR_INPUT_H

#include <iostream>
#include <Eigen/Dense>

namespace crs_models
{
namespace kinematic_model
{

struct kinematic_car_input
{
  double torque;
  double steer;

  static constexpr int NU = 2;

  kinematic_car_input() : torque(0), steer(0){};                                      // default constructor
  kinematic_car_input(double torque, double steer) : torque(torque), steer(steer){};  // Constructor
};

inline kinematic_car_input operator+(const kinematic_car_input& a, const kinematic_car_input& b)
{
  kinematic_car_input added_struct;

  added_struct.torque = a.torque + b.torque;
  added_struct.steer = a.steer + b.steer;

  return added_struct;
}

inline void operator-(const kinematic_car_input& a, const kinematic_car_input& b)
{
  kinematic_car_input subtracted_struct;

  subtracted_struct.torque = a.torque - b.torque;
  subtracted_struct.steer = a.steer - b.steer;
}

inline void operator+=(kinematic_car_input& a, const kinematic_car_input& b)
{
  a.torque = a.torque + b.torque;
  a.steer = a.steer + b.steer;
}

inline void operator+=(kinematic_car_input& a, const Eigen::Matrix<double, 2, 1>& b)
{
  a.torque = a.torque + b(0, 0);
  a.steer = a.steer + b(1, 0);
}

inline void operator-=(kinematic_car_input& a, const Eigen::Matrix<double, 2, 1>& b)
{
  a.torque = a.torque - b(0, 0);
  a.steer = a.steer - b(1, 0);
}

inline bool operator==(const kinematic_car_input& a, const kinematic_car_input& b)
{
  return (a.steer == b.steer) && (a.torque == b.torque);
}

inline std::ostream& operator<<(std::ostream& os,
                                const kinematic_car_input& input)  // how to print struct (input) to output stream (os)
{
  os << "model_input:" << std::endl;                               // Type of struct (input)
  os << " Torque: " << std::to_string(input.torque) << std::endl;  // field of struct (input)
  os << " Steer: " << std::to_string(input.steer) << std::endl;    // field of struct (input)
  return os;
}

}  // namespace kinematic_model
}  // namespace crs_models
#endif
