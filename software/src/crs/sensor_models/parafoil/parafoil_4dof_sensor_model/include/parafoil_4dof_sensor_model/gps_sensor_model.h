#ifndef PARAFOIL_4DOF_SENSOR_MODEL_GPS_SENSOR_MODEL_H
#define PARAFOIL_4DOF_SENSOR_MODEL_GPS_SENSOR_MODEL_H

#include <parafoil_4dof_model/parafoil_4dof_input.h>
#include <parafoil_4dof_model/parafoil_4dof_state.h>
#include <sensor_models/sensor_model.h>

namespace crs_sensor_models
{
namespace parafoil_4dof_sensor_models
{
class GPSSensorModel : public SensorModel<crs_models::parafoil_4dof_model::parafoil_4dof_state,
                                          crs_models::parafoil_4dof_model::parafoil_4dof_input>
{
public:
  /**
   * @brief Construct a new GPS Sensor Model. With no covariance matrix given the noise
   * is assumed to be the unit matrix.
   */
  GPSSensorModel() : GPSSensorModel(Eigen::Matrix<double, 6, 6>::Identity())  // Measurement dimension is six
  {
    std::cout << "[WARNING] No R Matrix specified for GPSSensorModel. Using identity Matrix! " << std::endl;
  }

  /**
   * @brief Construct a new GPS Sensor Model.
   *
   * @param R measurement covariance Matrix
   */
  GPSSensorModel(const Eigen::Matrix<double, 6, 6>& R);  // Measurement dimension is six

  static const std::string SENSOR_KEY;
};
}  // namespace parafoil_4dof_sensor_models
}  // namespace crs_sensor_models
#endif
