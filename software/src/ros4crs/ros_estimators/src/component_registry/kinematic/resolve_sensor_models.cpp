#include "ros_estimators/component_registry/resolve_sensor_models.h"
#include <ros_crs_utils/parameter_io.h>

#include <kinematic_sensor_model/imu_sensor_model.h>
#include <kinematic_sensor_model/mocap_sensor_model.h>

#include <estimators/model_based_estimator.h>

#include <kinematic_model/kinematic_car_input.h>
#include <kinematic_model/kinematic_car_state.h>

typedef crs_models::kinematic_model::kinematic_car_state kinematic_car_state;
typedef crs_models::kinematic_model::kinematic_car_input kinematic_car_input;

typedef crs_sensor_models::kinematic_sensor_models::MocapSensorModel MocapSensorModelType;
typedef crs_sensor_models::kinematic_sensor_models::ImuSensorModel ImuSensorModelType;
typedef crs_estimators::ModelBasedEstimator<kinematic_car_state, kinematic_car_input> ModelBasedEstimatorType;

namespace registry
{
namespace estimators
{
// Provide implementation for MocapSensorModel
template <>
std::shared_ptr<MocapSensorModelType> loadSensorModel<MocapSensorModelType>(const ros::NodeHandle& nh_private,
                                                                            const std::string sensor_name,
                                                                            const parameter_io::empty_params parameters)
{
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  // Load R from params (pacejka_car_simulator.yaml)
  parameter_io::getMatrixFromParams<3, 3>(ros::NodeHandle(nh_private, "sensors/" + sensor_name + "/R"), R);
  return std::make_shared<MocapSensorModelType>(R);
};

// Provide implementation for ImuSensorModel
template <>
std::shared_ptr<ImuSensorModelType> loadSensorModel<ImuSensorModelType, crs_models::kinematic_model::kinematic_params>(
    const ros::NodeHandle& nh_private, const std::string sensor_name,
    const crs_models::kinematic_model::kinematic_params model_params)
{
  // IMU model needs continuous pacejka
  auto cont_model = std::make_shared<crs_models::kinematic_model::ContinuousKinematicModel>(model_params);
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  parameter_io::getMatrixFromParams<3, 3>(ros::NodeHandle(nh_private, "sensors/" + sensor_name + "/R"), R);
  return std::make_shared<ImuSensorModelType>(cont_model, R);
};

template <>
std::vector<std::string>
parseSensorModels<kinematic_car_state, kinematic_car_input>(ros::NodeHandle& nh, ros::NodeHandle& nh_private,
                                                            std::shared_ptr<ModelBasedEstimatorType> estimator)
{
  // LOAD SENSOR MODELS
  std::vector<std::string> sensors_to_load;
  nh_private.getParam("sensors/sensor_names", sensors_to_load);
  std::vector<std::string> loaded_sensors;
  // iterate over sensors_to_load
  for (const std::string sensor_name : sensors_to_load)
  {
    // Load sensor model & add to ekf
    if (sensor_name == "mocap")
    {
      std::shared_ptr<MocapSensorModelType> sensor_model =
          loadSensorModel<MocapSensorModelType>(nh_private, sensor_name, {});
      estimator->addSensorModel(sensor_name, sensor_model);
      // Add sensor model to loaded_sensors
      loaded_sensors.push_back(sensor_name);
    }
    else if (sensor_name == "imu")
    {
      crs_models::kinematic_model::kinematic_params params;
      // Load model parameters from global config (global nodehandle)
      parameter_io::getModelParams<crs_models::kinematic_model::kinematic_params>(
          ros::NodeHandle(nh, "model/model_params/"), params);
      // Then overwrite specific parameters from local config (private nodehandle)
      parameter_io::getModelParams<crs_models::kinematic_model::kinematic_params>(
          ros::NodeHandle(nh_private, "model/model_params/"), params, false);

      std::shared_ptr<ImuSensorModelType> sensor_model =
          loadSensorModel<ImuSensorModelType, crs_models::kinematic_model::kinematic_params>(nh_private, sensor_name,
                                                                                             params);
      estimator->addSensorModel(sensor_name, sensor_model);
      // Add sensor model to loaded_sensors
      loaded_sensors.push_back(sensor_name);
    }
    else
      ROS_ERROR_STREAM("Sensor " << sensor_name << " not supported");
  }
  return loaded_sensors;
};
}  // namespace estimators
}  // namespace registry
