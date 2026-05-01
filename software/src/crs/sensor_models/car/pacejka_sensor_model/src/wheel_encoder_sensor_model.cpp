
#include "pacejka_sensor_model/wheel_encoder_sensor_model.h"
#include <dynamic_models/utils/data_conversion.h>

namespace crs_sensor_models
{
namespace pacejka_sensor_models
{
WheelEncoderSensorModel::WheelEncoderSensorModel(double wheel_radius, double lf, double car_width,
                                                 const Eigen::Matrix4d& R)
  : SensorModel(4, WheelEncoderSensorModel::SENSOR_KEY)  // Measurement dimension is three
{
  assert(wheel_radius > 0 &&
         "Invalid Wheel radius for WheelEncoderSensorModel. Make sure it is set in the model parameter config!");

  assert(car_width > 0 &&
         "Invalid car_width for WheelEncoderSensorModel. Make sure it is set in the model parameter config!");

  using namespace casadi;

  // The speed in each wheel frame is calculated as
  // v_wheel = v_car + omega x r_b_wheel #(r_b_wheel is the position of the wheel in the body frame)
  // The back wheels align with x axis of the body frame so the wheel speed in wheel turn direction
  // is the same as the car speed in x direction.
  // The front wheels are turned by steer angle so the wheel speed in wheel turn direction is
  // cos(steer) * v_x_wheel + sin(steer) * v_y_wheel

  auto vx = state_mx[3];
  auto vy = state_mx[4];
  auto dyaw = state_mx[5];
  auto steer = input_mx[1];

  auto vx_fr = vx + 0.5 * car_width * dyaw;
  auto vy_fr = vy + lf * dyaw;
  auto vx_wheel_fr = vx_fr * cos(steer) + vy_fr * sin(steer);

  auto vx_fl = vx - 0.5 * car_width * dyaw;
  auto vy_fl = vy + lf * dyaw;
  auto vx_wheel_fl = vx_fl * cos(steer) + vy_fl * sin(steer);

  auto vx_wheel_rl = vx - 0.5 * car_width * dyaw;
  auto vx_wheel_rr = vx + 0.5 * car_width * dyaw;

  // Unit of wheel speed is rad/s
  // front left, front right, rear left, rear right
  std::vector<casadi::MX> measured_states_mx = { vx_wheel_fl / wheel_radius, vx_wheel_fr / wheel_radius,
                                                 vx_wheel_rl / wheel_radius, vx_wheel_rr / wheel_radius };

  measurement_function = casadi::Function("applyMeasurementModel", state_and_input_mx, measured_states_mx);

  setR(R);
}

const std::string WheelEncoderSensorModel::SENSOR_KEY = "wheel_encoders";

}  // namespace pacejka_sensor_models
}  // namespace crs_sensor_models
