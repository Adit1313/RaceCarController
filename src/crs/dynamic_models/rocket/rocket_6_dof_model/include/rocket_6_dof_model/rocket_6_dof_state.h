#ifndef ROCKET_6_DOF_MODEL_ROCKET_6_DOF_STATE_H
#define ROCKET_6_DOF_MODEL_ROCKET_6_DOF_STATE_H

#include <iostream>
#include <Eigen/Core>

namespace crs_models
{
namespace rocket_6_dof_model
{

struct rocket_6_dof_state
{
  /*
    Frame reference:
    - position is expressed in a global frame, where the x axis points upwards.
    - quaternion uses jpl notation internally (x, y, z, w) and expresses the rotation from rocket frame to world frame,
    i.e. the rotation matrix R_{WR}.
  */
  double position_x;
  double position_y;
  double position_z;
  double velocity_x;
  double velocity_y;
  double velocity_z;
  double quaternion_x;
  double quaternion_y;
  double quaternion_z;
  double quaternion_w;
  double angular_velocity_x;
  double angular_velocity_y;
  double angular_velocity_z;
  double thrust_magnitude;
  double torque_x;
  double servo_angle_1;
  double servo_angle_2;

  static constexpr int NX = 17;

  rocket_6_dof_state()
    : position_x(0)
    , position_y(0)
    , position_z(0)
    , velocity_x(0)
    , velocity_y(0)
    , velocity_z(0)
    , quaternion_x(0)
    , quaternion_y(0)
    , quaternion_z(0)
    , quaternion_w(1)
    , angular_velocity_x(0)
    , angular_velocity_y(0)
    , angular_velocity_z(0)
    , thrust_magnitude(0)
    , torque_x(0)
    , servo_angle_1(0)
    , servo_angle_2(0){};  // default constructor

  rocket_6_dof_state(double position_x, double position_y, double position_z, double velocity_x, double velocity_y,
                     double velocity_z, double quaternion_x, double quaternion_y, double quaternion_z,
                     double quaternion_w, double angular_velocity_x, double angular_velocity_y,
                     double angular_velocity_z, double thrust_magnitude, double torque_x, double servo_angle_1,
                     double servo_angle_2)
    : position_x(position_x)
    , position_y(position_y)
    , position_z(position_z)
    , velocity_x(velocity_x)
    , velocity_y(velocity_y)
    , velocity_z(velocity_z)
    , quaternion_x(quaternion_x)
    , quaternion_y(quaternion_y)
    , quaternion_z(quaternion_z)
    , quaternion_w(quaternion_w)
    , angular_velocity_x(angular_velocity_x)
    , angular_velocity_y(angular_velocity_y)
    , angular_velocity_z(angular_velocity_z)
    , thrust_magnitude(thrust_magnitude)
    , torque_x(torque_x)
    , servo_angle_1(servo_angle_1)
    , servo_angle_2(servo_angle_2){};  // Constructor
};

inline rocket_6_dof_state operator+(const rocket_6_dof_state& a, const rocket_6_dof_state& b)
{
  rocket_6_dof_state added_struct;

  added_struct.position_x = a.position_x + b.position_x;
  added_struct.position_y = a.position_y + b.position_y;
  added_struct.position_z = a.position_z + b.position_z;
  added_struct.velocity_x = a.velocity_x + b.velocity_x;
  added_struct.velocity_y = a.velocity_y + b.velocity_y;
  added_struct.velocity_z = a.velocity_z + b.velocity_z;
  added_struct.quaternion_x = a.quaternion_x + b.quaternion_x;
  added_struct.quaternion_y = a.quaternion_y + b.quaternion_y;
  added_struct.quaternion_z = a.quaternion_z + b.quaternion_z;
  added_struct.quaternion_w = a.quaternion_w + b.quaternion_w;
  added_struct.angular_velocity_x = a.angular_velocity_x + b.angular_velocity_x;
  added_struct.angular_velocity_y = a.angular_velocity_y + b.angular_velocity_y;
  added_struct.angular_velocity_z = a.angular_velocity_z + b.angular_velocity_z;
  added_struct.thrust_magnitude = a.thrust_magnitude + b.thrust_magnitude;
  added_struct.torque_x = a.torque_x + b.torque_x;
  added_struct.servo_angle_1 = a.servo_angle_1 + b.servo_angle_1;
  added_struct.servo_angle_2 = a.servo_angle_2 + b.servo_angle_2;

  return added_struct;
}

inline rocket_6_dof_state operator*(double b, const rocket_6_dof_state& a)
{
  rocket_6_dof_state product_struct;

  product_struct.position_x = a.position_x * b;
  product_struct.position_y = a.position_y * b;
  product_struct.position_z = a.position_z * b;
  product_struct.velocity_x = a.velocity_x * b;
  product_struct.velocity_y = a.velocity_y * b;
  product_struct.velocity_z = a.velocity_z * b;
  product_struct.quaternion_x = a.quaternion_x * b;
  product_struct.quaternion_y = a.quaternion_y * b;
  product_struct.quaternion_z = a.quaternion_z * b;
  product_struct.quaternion_w = a.quaternion_w * b;
  product_struct.angular_velocity_x = a.angular_velocity_x * b;
  product_struct.angular_velocity_y = a.angular_velocity_y * b;
  product_struct.angular_velocity_z = a.angular_velocity_z * b;
  product_struct.thrust_magnitude = a.thrust_magnitude * b;
  product_struct.torque_x = a.torque_x * b;
  product_struct.servo_angle_1 = a.servo_angle_1 * b;
  product_struct.servo_angle_2 = a.servo_angle_2 * b;

  return product_struct;
}

inline void operator+=(rocket_6_dof_state& a, const rocket_6_dof_state& b)
{
  a.position_x = a.position_x + b.position_x;
  a.position_y = a.position_y + b.position_y;
  a.position_z = a.position_z + b.position_z;
  a.velocity_x = a.velocity_x + b.velocity_x;
  a.velocity_y = a.velocity_y + b.velocity_y;
  a.velocity_z = a.velocity_z + b.velocity_z;
  a.quaternion_x = a.quaternion_x + b.quaternion_x;
  a.quaternion_y = a.quaternion_y + b.quaternion_y;
  a.quaternion_z = a.quaternion_z + b.quaternion_z;
  a.quaternion_w = a.quaternion_w + b.quaternion_w;
  a.angular_velocity_x = a.angular_velocity_x + b.angular_velocity_x;
  a.angular_velocity_y = a.angular_velocity_y + b.angular_velocity_y;
  a.angular_velocity_z = a.angular_velocity_z + b.angular_velocity_z;
  a.thrust_magnitude = a.thrust_magnitude + b.thrust_magnitude;
  a.torque_x = a.torque_x + b.torque_x;
  a.servo_angle_1 = a.servo_angle_1 + b.servo_angle_1;
  a.servo_angle_2 = a.servo_angle_2 + b.servo_angle_2;
}

inline void operator+=(rocket_6_dof_state& a, const Eigen::Matrix<double, 17, 1>& b)
{
  a.position_x = a.position_x + b(0, 0);
  a.position_y = a.position_y + b(1, 0);
  a.position_z = a.position_z + b(2, 0);
  a.velocity_x = a.velocity_x + b(3, 0);
  a.velocity_y = a.velocity_y + b(4, 0);
  a.velocity_z = a.velocity_z + b(5, 0);
  a.quaternion_x = a.quaternion_x + b(6, 0);
  a.quaternion_y = a.quaternion_y + b(7, 0);
  a.quaternion_z = a.quaternion_z + b(8, 0);
  a.quaternion_w = a.quaternion_w + b(9, 0);
  a.angular_velocity_x = a.angular_velocity_x + b(10, 0);
  a.angular_velocity_y = a.angular_velocity_y + b(11, 0);
  a.angular_velocity_z = a.angular_velocity_z + b(12, 0);
  a.thrust_magnitude = a.thrust_magnitude + b(13, 0);
  a.torque_x = a.torque_x + b(14, 0);
  a.servo_angle_1 = a.servo_angle_1 + b(15, 0);
  a.servo_angle_2 = a.servo_angle_2 + b(16, 0);
}

inline bool operator==(const rocket_6_dof_state& a, const rocket_6_dof_state& b)
{
  return ((a.position_x == b.position_x) && (a.position_y == b.position_y) && (a.position_z == b.position_z) &&
          (a.velocity_x == b.velocity_x) && (a.velocity_y == b.velocity_y) && (a.velocity_z == b.velocity_z) &&
          (a.quaternion_x == b.quaternion_x) && (a.quaternion_y == b.quaternion_y) &&
          (a.quaternion_z == b.quaternion_z) && (a.quaternion_w == b.quaternion_w) &&
          (a.angular_velocity_x == b.angular_velocity_x) && (a.angular_velocity_y == b.angular_velocity_y) &&
          (a.angular_velocity_z == b.angular_velocity_z) && (a.thrust_magnitude == b.thrust_magnitude) &&
          (a.torque_x == b.torque_x) && (a.servo_angle_1 == b.servo_angle_1) && (a.servo_angle_2 == b.servo_angle_2));
}

inline std::ostream& operator<<(std::ostream& os,
                                const rocket_6_dof_state& state)  // how to print struct (state) to output stream
                                                                  // (os)
{
  os << "Rocket 6 DoF State:" << std::endl;
  os << " position_x: " << std::to_string(state.position_x) << std::endl;                  // field of struct (state)
  os << " position_y: " << std::to_string(state.position_y) << std::endl;                  // field of struct (state)
  os << " position_z: " << std::to_string(state.position_z) << std::endl;                  // field of struct (state)
  os << " velocity_x: " << std::to_string(state.velocity_x) << std::endl;                  // field of struct (state)
  os << " velocity_y: " << std::to_string(state.velocity_y) << std::endl;                  // field of struct (state)
  os << " velocity_z: " << std::to_string(state.velocity_z) << std::endl;                  // field of struct (state)
  os << " quaternion_x: " << std::to_string(state.quaternion_x) << std::endl;              // field of struct (state)
  os << " quaternion_y: " << std::to_string(state.quaternion_y) << std::endl;              // field of struct (state)
  os << " quaternion_z: " << std::to_string(state.quaternion_z) << std::endl;              // field of struct (state)
  os << " quaternion_w: " << std::to_string(state.quaternion_w) << std::endl;              // field of struct (state)
  os << " angular_velocity_x: " << std::to_string(state.angular_velocity_x) << std::endl;  // field of struct (state)
  os << " angular_velocity_y: " << std::to_string(state.angular_velocity_y) << std::endl;  // field of struct (state)
  os << " angular_velocity_z: " << std::to_string(state.angular_velocity_z) << std::endl;  // field of struct (state)
  os << " thrust_magnitude: " << std::to_string(state.thrust_magnitude) << std::endl;      // field of struct (state)
  os << " torque_x: " << std::to_string(state.torque_x) << std::endl;                      // field of struct (state)
  os << " servo_angle_1: " << std::to_string(state.servo_angle_1) << std::endl;            // field of struct (state)
  os << " servo_angle_2: " << std::to_string(state.servo_angle_2) << std::endl;            // field of struct (state)
  return os;
}

}  // namespace rocket_6_dof_model
}  // namespace crs_models
#endif
