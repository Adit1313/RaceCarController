#ifndef PARAFOIL_4DOF_MODEL_PARAFOIL_4DOF_STATE_H
#define PARAFOIL_4DOF_MODEL_PARAFOIL_4DOF_STATE_H

#include <iostream>

namespace crs_models
{
namespace parafoil_4dof_model
{

// Generic state of parafoil system:
//    pos_{x,y,z}:    north, east, down position of parafoil system in inertial-frame
//    phi/theta/psi:  euler angles to determine orientation
//    vel_{u,v,w}:    forward, sideways, downward velocities in body-frame
struct parafoil_4dof_state
{
  double pos_x;
  double pos_y;
  double pos_z;
  double phi;
  double theta;
  double psi;
  double vel_u;
  double vel_v;
  double vel_w;

  static constexpr int NX = 9;

  parafoil_4dof_state()
    : pos_x(0), pos_y(0), pos_z(0), phi(0), theta(0), psi(0), vel_u(0), vel_v(0), vel_w(0){};  // default constructor
  parafoil_4dof_state(double pos_x, double pos_y, double pos_z, double phi, double theta, double psi, double vel_u,
                      double vel_v, double vel_w)
    : pos_x(pos_x)
    , pos_y(pos_y)
    , pos_z(pos_z)
    , phi(phi)
    , theta(theta)
    , psi(psi)
    , vel_u(vel_u)
    , vel_v(vel_v)
    , vel_w(vel_w){};  // Constructor
};

inline parafoil_4dof_state operator+(const parafoil_4dof_state& a, const parafoil_4dof_state& b)
{
  parafoil_4dof_state added_struct;

  added_struct.pos_x = a.pos_x + b.pos_x;
  added_struct.pos_y = a.pos_y + b.pos_y;
  added_struct.pos_z = a.pos_z + b.pos_z;
  added_struct.phi = a.phi + b.phi;
  added_struct.theta = a.theta + b.theta;
  added_struct.psi = a.psi + b.psi;
  added_struct.vel_u = a.vel_u + b.vel_u;
  added_struct.vel_v = a.vel_v + b.vel_v;
  added_struct.vel_w = a.vel_w + b.vel_w;

  return added_struct;
}

inline parafoil_4dof_state operator*(double b, const parafoil_4dof_state& a)
{
  parafoil_4dof_state product_struct;

  product_struct.pos_x = a.pos_x * b;
  product_struct.pos_y = a.pos_y * b;
  product_struct.pos_z = a.pos_z * b;
  product_struct.phi = a.phi * b;
  product_struct.theta = a.theta * b;
  product_struct.psi = a.psi * b;
  product_struct.vel_u = a.vel_u * b;
  product_struct.vel_v = a.vel_v * b;
  product_struct.vel_w = a.vel_w * b;

  return product_struct;
}

inline void operator+=(parafoil_4dof_state& a, const parafoil_4dof_state& b)
{
  a.pos_x = a.pos_x + b.pos_x;
  a.pos_y = a.pos_y + b.pos_y;
  a.pos_z = a.pos_z + b.pos_z;
  a.phi = a.phi + b.phi;
  a.theta = a.theta + b.theta;
  a.psi = a.psi + b.psi;
  a.vel_u = a.vel_u + b.vel_u;
  a.vel_v = a.vel_v + b.vel_v;
  a.vel_w = a.vel_w + b.vel_w;
}

inline bool operator==(const parafoil_4dof_state& a, const parafoil_4dof_state& b)
{
  return (a.pos_x == b.pos_x) && (a.pos_y == b.pos_y) && (a.pos_z == b.pos_z) && (a.phi == b.phi) &&
         (a.theta == b.theta) && (a.psi == b.psi) && (a.vel_u == b.vel_u) && (a.vel_v == b.vel_v) &&
         (a.vel_w == b.vel_w);
}

inline std::ostream& operator<<(std::ostream& os,
                                const parafoil_4dof_state& state)  // how to print struct (state) to output stream (os)
{
  os << "parafoil_4dof_state:" << std::endl;                     // Type of struct (state)
  os << " Pos_x: " << std::to_string(state.pos_x) << std::endl;  // field of struct (state)
  os << " Pos_y: " << std::to_string(state.pos_y) << std::endl;  // field of struct (state)
  os << " Pos_z: " << std::to_string(state.pos_z) << std::endl;  // field of struct (state)
  os << " Phi: " << std::to_string(state.phi) << std::endl;      // field of struct (state)
  os << " Theta: " << std::to_string(state.theta) << std::endl;  // field of struct (state)
  os << " Psi: " << std::to_string(state.psi) << std::endl;      // field of struct (state)
  os << " Vel_u: " << std::to_string(state.vel_u) << std::endl;  // field of struct (state)
  os << " Vel_v: " << std::to_string(state.vel_v) << std::endl;  // field of struct (state)
  os << " Vel_w: " << std::to_string(state.vel_w) << std::endl;  // field of struct (state)
  return os;
}

}  // namespace parafoil_4dof_model
}  // namespace crs_models
#endif
