#ifndef STACKE_MODEL_PACEJKA_IMU_BIAS_CAR_STATE_H
#define STACKE_MODEL_PACEJKA_IMU_BIAS_CAR_STATE_H

#include <iostream>
#include <pacejka_model/pacejka_car_state.h>
#include <imu_bias/imu_bias_state.h>

namespace crs_models
{
namespace stacked_model
{

struct pacejka_imu_bias_car_state : public crs_models::pacejka_model::pacejka_car_state,
                                    public crs_models::imu_bias::imu_bias_state
{
  static constexpr int NX = crs_models::pacejka_model::pacejka_car_state::NX + crs_models::imu_bias::imu_bias_state::NX;

  pacejka_imu_bias_car_state()
    : crs_models::pacejka_model::pacejka_car_state(), crs_models::imu_bias::imu_bias_state(){};  // default constructor
  pacejka_imu_bias_car_state(crs_models::pacejka_model::pacejka_car_state car_state,
                             crs_models::imu_bias::imu_bias_state bias)
    : crs_models::pacejka_model::pacejka_car_state(car_state)
    , crs_models::imu_bias::imu_bias_state(bias){};  // Constructor

  pacejka_imu_bias_car_state& operator=(const pacejka_model::pacejka_car_state& b)
  {
    pos_x = b.pos_x;
    pos_y = b.pos_y;
    yaw = b.yaw;
    vel_x = b.vel_x;
    vel_y = b.vel_y;
    yaw_rate = b.yaw_rate;
    return *this;
  }

  pacejka_imu_bias_car_state& operator=(const imu_bias::imu_bias_state& b)
  {
    bias_ax = b.bias_ax;
    bias_ay = b.bias_ay;
    bias_dyaw = b.bias_dyaw;
    return *this;
  }
};

inline pacejka_imu_bias_car_state operator+(const pacejka_imu_bias_car_state& a, const pacejka_imu_bias_car_state& b)
{
  pacejka_imu_bias_car_state added_struct;

  added_struct.pos_x = a.pos_x + b.pos_x;
  added_struct.pos_y = a.pos_y + b.pos_y;
  added_struct.yaw = a.yaw + b.yaw;
  added_struct.vel_x = a.vel_x + b.vel_x;
  added_struct.vel_y = a.vel_y + b.vel_y;
  added_struct.yaw_rate = a.yaw_rate + b.yaw_rate;
  added_struct.bias_ax = a.bias_ax + b.bias_ax;
  added_struct.bias_ay = a.bias_ay + b.bias_ay;
  added_struct.bias_dyaw = a.bias_dyaw + b.bias_dyaw;

  return added_struct;
}

inline pacejka_imu_bias_car_state operator*(double b, const pacejka_imu_bias_car_state& a)
{
  pacejka_imu_bias_car_state product_struct;

  product_struct.pos_x = a.pos_x * b;
  product_struct.pos_y = a.pos_y * b;
  product_struct.yaw = a.yaw * b;
  product_struct.vel_x = a.vel_x * b;
  product_struct.vel_y = a.vel_y * b;
  product_struct.yaw_rate = a.yaw_rate * b;
  product_struct.bias_ax = a.bias_ax * b;
  product_struct.bias_ay = a.bias_ay * b;
  product_struct.bias_dyaw = a.bias_dyaw * b;

  return product_struct;
}

inline void operator+=(pacejka_imu_bias_car_state& a, const pacejka_imu_bias_car_state& b)
{
  a.pos_x = a.pos_x + b.pos_x;
  a.pos_y = a.pos_y + b.pos_y;
  a.yaw = a.yaw + b.yaw;
  a.vel_x = a.vel_x + b.vel_x;
  a.vel_y = a.vel_y + b.vel_y;
  a.yaw_rate = a.yaw_rate + b.yaw_rate;
  a.bias_ax = a.bias_ax + b.bias_ax;
  a.bias_ay = a.bias_ay + b.bias_ay;
  a.bias_dyaw = a.bias_dyaw + b.bias_dyaw;
}

inline bool operator==(const pacejka_imu_bias_car_state& a, const pacejka_imu_bias_car_state& b)
{
  return (a.pos_x == b.pos_x) && (a.pos_y == b.pos_y) && (a.yaw == b.yaw) && (a.vel_x == b.vel_x) &&
         (a.vel_y == b.vel_y) && (a.yaw_rate == b.yaw_rate) && (a.bias_ax == b.bias_ax) && (a.bias_ay == b.bias_ay) &&
         (a.bias_dyaw == b.bias_dyaw);
}

inline std::ostream& operator<<(std::ostream& os,
                                const pacejka_imu_bias_car_state& state)  // how to print struct (state) to output
                                                                          // stream (os)
{
  os << "pacejka_imu_bias_car_state:" << std::endl;                              // Type of struct (state)
  os << " Pos_x: " << std::to_string(state.pos_x) << std::endl;                  // field of struct (state)
  os << " Pos_y: " << std::to_string(state.pos_y) << std::endl;                  // field of struct (state)
  os << " Yaw: " << std::to_string(state.yaw) << std::endl;                      // field of struct (state)
  os << " V_x: " << std::to_string(state.vel_x) << std::endl;                    // field of struct (state)
  os << " V_y: " << std::to_string(state.vel_y) << std::endl;                    // field of struct (state)
  os << " Yaw_rate: " << std::to_string(state.yaw_rate) << std::endl;            // field of struct (state)
  os << " IMU bias a_x: " << std::to_string(state.bias_ax) << std::endl;         // field of struct (state)
  os << " IMU bias a_y: " << std::to_string(state.bias_ay) << std::endl;         // field of struct (state)
  os << " IMU bias yaw_rate: " << std::to_string(state.bias_dyaw) << std::endl;  // field of struct (state)
  return os;
}

}  // namespace stacked_model
}  // namespace crs_models
#endif
