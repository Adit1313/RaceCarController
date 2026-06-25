#ifndef PACEJKA_SENSOR_MODEL_LIGHTHOUSE_SENSOR_MODEL_H
#define PACEJKA_SENSOR_MODEL_LIGHTHOUSE_SENSOR_MODEL_H

#include <pacejka_model/pacejka_car_input.h>
#include <pacejka_model/pacejka_car_state.h>
#include <sensor_models/sensor_model.h>

namespace crs_sensor_models
{
namespace pacejka_sensor_models
{
class LighthouseSensorModel
  : public SensorModel<crs_models::pacejka_model::pacejka_car_state, crs_models::pacejka_model::pacejka_car_input>
{
public:
  // Option 1: Create an object with no process noise covariance matrix R defined. Will use identity.
  LighthouseSensorModel() : LighthouseSensorModel(Eigen::Matrix4d::Identity())  // Measurement dimension is four
  {
    std::cout << "[WARNING] No R Matrix specified for LighthouseSensorModel. Using identity Matrix! " << std::endl;
  }

  // Option 2: Create an object and specify only the process noise covariance matrix R.
  LighthouseSensorModel(const Eigen::Matrix4d& R)
    : LighthouseSensorModel(R, Eigen::Vector3d(0, 0, 2), (Eigen::Matrix3d() << 0, 1, 0, 0, 0, -1, -1, 0, 0).finished(),
                            0.5236, Eigen::Matrix<double, 2, 4>::Zero())  // Measurement dimension is four
  {
    std::cout << "[WARNING] No base station parameters specified for LighthouseSensorModel. Will not work properly! "
              << std::endl;
  }

  // Option 3: Create an object and specify the process noise covariance matrix R and the base station parameters.
  LighthouseSensorModel(const Eigen::Matrix4d& R, Eigen::Vector3d position, Eigen::Matrix3d rotation,
                        double light_plane_tilt,
                        Eigen::Matrix<double, 2, 4> sensor_positions_);  // Measurement dimension is four

  static const std::string SENSOR_KEY;

  /**
   * @brief Sets the lighthouse parameters
   *
   * @param sensor_positions 2D positions of the four lighthouse sensors relative to center of model
   * @param bs_position 3D position of the base station
   * @param bs_rotation 3D rotation matrix for the base station orientation
   * @param light_plane_tilt tilt of the lightplane emitted by the base station
   */
  void setLighthouseParams(Eigen::Matrix<double, 2, 4> sensor_positions, Eigen::Vector3d bs_position,
                           Eigen::Matrix3d bs_rotation, double light_plane_tilt)
  {
    sensor_positions_ = sensor_positions;
    base_station_position_ = bs_position;
    base_station_rotation_ = bs_rotation;
    light_plane_tilt_ = light_plane_tilt;
  }

  /**
   * @brief Returns the lighthouse parameters
   *
   * @return const std::tuple<Eigen::Matrix<double, 2, 4>, Eigen::Vector3d, Eigen::Matrix3d, double>
   */
  const std::tuple<Eigen::Matrix<double, 2, 4>, Eigen::Vector3d, Eigen::Matrix3d, double> getLighthouseParams()
  {
    std::tuple<Eigen::Matrix<double, 2, 4>, Eigen::Vector3d, Eigen::Matrix3d, double> lighthouse_params = {
      sensor_positions_, base_station_position_, base_station_rotation_, light_plane_tilt_
    };
    return lighthouse_params;
  }

protected:
  // The 3D position of the base station
  Eigen::Vector3d base_station_position_;

  // The 3D rotation matrix for the base station orientation
  Eigen::Matrix3d base_station_rotation_;

  // The tilt of the lightplane emitted by the base station
  double light_plane_tilt_;

  // The 2D positions of the four lighthouse sensors relative to center of model
  Eigen::Matrix<double, 2, 4> sensor_positions_;
};
}  // namespace pacejka_sensor_models
}  // namespace crs_sensor_models
#endif
