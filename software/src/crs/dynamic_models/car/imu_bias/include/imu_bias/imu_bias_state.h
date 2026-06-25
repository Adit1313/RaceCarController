#ifndef IMU_BIAS_STATE_H
#define IMU_BIAS_STATE_H

#include <iostream>

namespace crs_models
{
namespace imu_bias
{

struct imu_bias_state
{
  double bias_ax;
  double bias_ay;
  double bias_dyaw;

  static constexpr int NX = 3;

  imu_bias_state() : bias_ax(0), bias_ay(0), bias_dyaw(0){};  // default constructor
  imu_bias_state(double bias_ax, double bias_ay, double bias_dyaw)
    : bias_ax(bias_ax), bias_ay(bias_ay), bias_dyaw(bias_dyaw){};  // Constructor
};

inline imu_bias_state operator+(const imu_bias_state& a, const imu_bias_state& b)
{
  imu_bias_state added_struct;

  added_struct.bias_ax = a.bias_ax + b.bias_ax;
  added_struct.bias_ay = a.bias_ay + b.bias_ay;
  added_struct.bias_dyaw = a.bias_dyaw + b.bias_dyaw;

  return added_struct;
}

inline imu_bias_state operator*(double b, const imu_bias_state& a)
{
  imu_bias_state product_struct;

  product_struct.bias_ax = a.bias_ax * b;
  product_struct.bias_ay = a.bias_ay * b;
  product_struct.bias_dyaw = a.bias_dyaw * b;

  return product_struct;
}

inline void operator+=(imu_bias_state& a, const imu_bias_state& b)
{
  a.bias_ax = a.bias_ax + b.bias_ax;
  a.bias_ay = a.bias_ay + b.bias_ay;
  a.bias_dyaw = a.bias_dyaw + b.bias_dyaw;
}

inline bool operator==(const imu_bias_state& a, const imu_bias_state& b)
{
  return (a.bias_ax == b.bias_ax) && (a.bias_ay == b.bias_ay) && (a.bias_dyaw == b.bias_dyaw);
}

inline std::ostream& operator<<(std::ostream& os,
                                const imu_bias_state& state)  // how to print struct (state) to output stream (os)
{
  os << "imu_bias_state:" << std::endl;                                      // Type of struct (state)
  os << " v_x_dot bias: " << std::to_string(state.bias_ax) << std::endl;     // field of struct (state)
  os << " v_y_dot bias: " << std::to_string(state.bias_ay) << std::endl;     // field of struct (state)
  os << " yaw_rate bias: " << std::to_string(state.bias_dyaw) << std::endl;  // field of struct (state)
  return os;
}

}  // namespace imu_bias
}  // namespace crs_models
#endif
