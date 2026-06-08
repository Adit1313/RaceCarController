
#include "pacejka_sensor_model/lighthouse_sensor_model.h"
#include <dynamic_models/utils/data_conversion.h>

namespace crs_sensor_models
{
namespace pacejka_sensor_models
{
// Option 2: Create an object and specify the process noise covariance matrix Q yourself.
LighthouseSensorModel::LighthouseSensorModel(const Eigen::Matrix4d& R, Eigen::Vector3d position,
                                             Eigen::Matrix3d rotation, double light_plane_tilt,
                                             Eigen::Matrix<double, 2, 4> sensor_positions)
  : SensorModel(4, LighthouseSensorModel::SENSOR_KEY)  // Measurement dimension is four
{
  Eigen::Matrix<casadi::MX, 3, 2> rot;
  rot << cos(state_mx[2]), -sin(state_mx[2]), sin(state_mx[2]), cos(state_mx[2]), 0, 0;
  // Position of the car in world frame
  Eigen::Matrix<casadi::MX, 3, 1> pos_car{ state_mx[0], state_mx[1], 0 };
  // Positions of the sensors in world frame
  // sensor_positions = 2D positions of the four lighthouse sensors relative to center of model (in lighthouse yaml)
  Eigen::Matrix<casadi::MX, 3, 4> pos_sensor = (rot * sensor_positions.cast<casadi::MX>()).colwise() + pos_car;
  // Base station rotation matrix and position vector
  Eigen::Matrix<casadi::MX, 3, 3> rot_bs = rotation.transpose().cast<casadi::MX>();
  Eigen::Matrix<casadi::MX, 3, 1> pos_bs = position.cast<casadi::MX>();
  // Positions in base satation reference frame
  Eigen::Matrix<casadi::MX, 3, 4> sbs = rot_bs * (pos_sensor.colwise() - pos_bs);
  // Calculate angles
  Eigen::Matrix<casadi::MX, 1, 4> alphas = (sbs.row(1).array() / sbs.row(0).array()).atan();
  Eigen::Matrix<casadi::MX, 1, 4> alpha =
      alphas.array() + ((sbs.row(2) * tan(light_plane_tilt)).array() /
                        (sbs.row(0).array().square() + sbs.row(1).array().square()).sqrt())
                           .asin();

  std::vector<casadi::MX> measured_states_mx = { alpha(0), alpha(1), alpha(2), alpha(3) };

  measurement_function = casadi::Function("applyMeasurementModel", state_and_input_mx, measured_states_mx);

  setR(R);
  setLighthouseParams(sensor_positions, position, rotation, light_plane_tilt);
}

const std::string LighthouseSensorModel::SENSOR_KEY = "lighthouse";

}  // namespace pacejka_sensor_models
}  // namespace crs_sensor_models
