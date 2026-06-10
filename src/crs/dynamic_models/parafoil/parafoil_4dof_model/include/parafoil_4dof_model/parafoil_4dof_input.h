#ifndef PARAFOIL_4DOF_MODEL_PARAFOIL_4DOF_INPUT_H
#define PARAFOIL_4DOF_MODEL_PARAFOIL_4DOF_INPUT_H

#include <iostream>

namespace crs_models
{
namespace parafoil_4dof_model
{

struct parafoil_4dof_input
{
  double deflection_symmetric;
  double deflection_asymmetric;

  static constexpr int NU = 2;

  parafoil_4dof_input() : deflection_symmetric(0), deflection_asymmetric(0){};  // default constructor
  parafoil_4dof_input(double deflection_symmetric, double deflection_asymmetric)
    : deflection_symmetric(deflection_symmetric), deflection_asymmetric(deflection_asymmetric){};  // Constructor
};

inline parafoil_4dof_input operator+(const parafoil_4dof_input& a, const parafoil_4dof_input& b)
{
  parafoil_4dof_input added_struct;

  added_struct.deflection_asymmetric = a.deflection_asymmetric + b.deflection_asymmetric;
  added_struct.deflection_symmetric = a.deflection_symmetric + b.deflection_symmetric;

  return added_struct;
}

inline void operator+=(parafoil_4dof_input& a, const parafoil_4dof_input& b)
{
  a.deflection_asymmetric = a.deflection_asymmetric + b.deflection_asymmetric;
  a.deflection_symmetric = a.deflection_symmetric + b.deflection_symmetric;
}

inline bool operator==(const parafoil_4dof_input& a, const parafoil_4dof_input& b)
{
  return (a.deflection_asymmetric == b.deflection_asymmetric) && (a.deflection_symmetric == b.deflection_symmetric);
}

inline std::ostream& operator<<(std::ostream& os,
                                const parafoil_4dof_input& input)  // how to print struct (input) to output stream (os)
{
  os << "model_input:" << std::endl;  // Type of struct (input)
  os << " Deflection_symmetric: " << std::to_string(input.deflection_symmetric)
     << std::endl;  // field of struct (input)
  os << " Deflection_asymmetric: " << std::to_string(input.deflection_asymmetric)
     << std::endl;  // field of struct (input)
  return os;
}

}  // namespace parafoil_4dof_model
}  // namespace crs_models
#endif
