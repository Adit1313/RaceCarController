#ifndef STACKE_MODEL_PACEJKA_IMU_BIAS_CAR_INPUT_H
#define STACKE_MODEL_PACEJKA_IMU_BIAS_CAR_INPUT_H

#include <iostream>

#include <pacejka_model/pacejka_car_input.h>
#include <imu_bias/imu_bias_input.h>

namespace crs_models
{
namespace stacked_model
{

struct pacejka_imu_bias_car_input : public crs_models::pacejka_model::pacejka_car_input,
                                    public crs_models::imu_bias::imu_bias_input
{
  static constexpr int NU = crs_models::pacejka_model::pacejka_car_input::NU + crs_models::imu_bias::imu_bias_input::NU;

  pacejka_imu_bias_car_input()
    : crs_models::pacejka_model::pacejka_car_input(), crs_models::imu_bias::imu_bias_input(){};  // default constructor
  pacejka_imu_bias_car_input(crs_models::pacejka_model::pacejka_car_input input)
    : pacejka_car_input(input), crs_models::imu_bias::imu_bias_input(){};  // Constructor
};

inline pacejka_imu_bias_car_input operator+(const pacejka_imu_bias_car_input& a, const pacejka_imu_bias_car_input& b)
{
  pacejka_imu_bias_car_input added_struct;

  added_struct.steer = a.steer + b.steer;
  added_struct.torque = a.torque + b.torque;

  return added_struct;
}

inline void operator+=(pacejka_imu_bias_car_input& a, const pacejka_imu_bias_car_input& b)
{
  a.steer = a.steer + b.steer;
  a.torque = a.torque + b.torque;
}

inline bool operator==(const pacejka_imu_bias_car_input& a, const pacejka_imu_bias_car_input& b)
{
  return (a.steer == b.steer) && (a.torque == b.torque);
}

inline std::ostream& operator<<(std::ostream& os,
                                const pacejka_imu_bias_car_input& input)  // how to print struct (input) to output
                                                                          // stream (os)
{
  os << "pacejka_imu_bias_car_input:" << std::endl;                // Type of struct (input)
  os << " Torque: " << std::to_string(input.torque) << std::endl;  // field of struct (input)
  os << " Steer: " << std::to_string(input.steer) << std::endl;    // field of struct (input)
  return os;
}

}  // namespace stacked_model
}  // namespace crs_models
#endif /* STACKE_MODEL_PACEJKA_IMU_BIAS_CAR_INPUT_H */
