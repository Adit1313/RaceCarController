#ifndef PARAFOIL_4DOF_SENSOR_MODEL_IMU_SENSOR_MODEL_H
#define PARAFOIL_4DOF_SENSOR_MODEL_IMU_SENSOR_MODEL_H

#include <sensor_models/sensor_model.h>

#include <parafoil_4dof_model/parafoil_4dof_input.h>
#include <parafoil_4dof_model/parafoil_4dof_state.h>
#include <parafoil_4dof_model/parafoil_4dof_continuous.h>

namespace crs_sensor_models
{
namespace parafoil_4dof_sensor_models
{
/**
 * @brief Creates a sensor model that is used to measure linear accelarations and angular velocities
 *
 */
class ImuSensorModel : public SensorModel<crs_models::parafoil_4dof_model::parafoil_4dof_state,
                                          crs_models::parafoil_4dof_model::parafoil_4dof_input>
{
public:
  /**
   * @brief Construct a new IMU Sensor Model. Note that the accelerations are not part of the 4 DoF parafoil state and
   * therefore the continuous model is needed
   *
   * @param parafoil_4dof_cont the continuous model
   */
  ImuSensorModel(const std::shared_ptr<crs_models::parafoil_4dof_model::ContinuousParafoil4dofModel> parafoil_4dof_cont)
    : ImuSensorModel(parafoil_4dof_cont, Eigen::Matrix<double, 6, 6>::Identity())  // Measurement dimension is six
  {
    std::cout << "[WARNING] No R Matrix specified for ImuSensorModel. Using identity Matrix! " << std::endl;
  }

  /**
   * @brief Construct a new IMU Sensor Model. Note that the accelerations are not part of the 4 DoF parafoil state and
   * therefore the continuous model is needed
   *
   * @param parafoil_4dof_cont the continuous model
   * @param R measurement covariance Matrix
   */
  ImuSensorModel(const std::shared_ptr<crs_models::parafoil_4dof_model::ContinuousParafoil4dofModel> parafoil_4dof_cont,
                 const Eigen::Matrix<double, 6, 6>& R);  // Measurement dimension is six

  static const std::string SENSOR_KEY;
};
}  // namespace parafoil_4dof_sensor_models
}  // namespace crs_sensor_models
#endif
