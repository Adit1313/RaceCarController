#include "ros_estimators/component_registry/resolve_sensor_models.h"
#include <ros_crs_utils/parameter_io.h>

#include <math.h>
#include <pacejka_sensor_model/imu_sensor_model.h>
#include <pacejka_sensor_model/imu_yaw_rate_sensor_model.h>
#include <pacejka_sensor_model/mocap_sensor_model.h>
#include <pacejka_sensor_model/wheel_encoder_sensor_model.h>
#include <pacejka_sensor_model/lighthouse_sensor_model.h>

#include <estimators/model_based_estimator.h>

#include <pacejka_model/pacejka_car_input.h>
#include <pacejka_model/pacejka_car_state.h>

typedef crs_models::pacejka_model::pacejka_car_state pacejka_car_state;
typedef crs_models::pacejka_model::pacejka_car_input pacejka_car_input;

typedef crs_sensor_models::pacejka_sensor_models::MocapSensorModel MocapSensorModelType;
typedef crs_sensor_models::pacejka_sensor_models::ImuSensorModel ImuSensorModelType;
typedef crs_sensor_models::pacejka_sensor_models::ImuYawSensorModel ImuYawSensorModelType;
typedef crs_estimators::ModelBasedEstimator<pacejka_car_state, pacejka_car_input> ModelBasedEstimatorType;

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
std::shared_ptr<ImuSensorModelType> loadSensorModel<ImuSensorModelType, crs_models::pacejka_model::pacejka_params>(
    const ros::NodeHandle& nh_private, const std::string sensor_name,
    const crs_models::pacejka_model::pacejka_params model_params)
{
  // IMU model needs continuous pacejka
  auto cont_model = std::make_shared<crs_models::pacejka_model::ContinuousPacejkaModel>(model_params);
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  parameter_io::getMatrixFromParams<3, 3>(ros::NodeHandle(nh_private, "sensors/" + sensor_name + "/R"), R);
  return std::make_shared<ImuSensorModelType>(cont_model, R);
};

// Provide implementation for ImuYawSensorModel
template <>
std::shared_ptr<ImuYawSensorModelType>
loadSensorModel<ImuYawSensorModelType>(const ros::NodeHandle& nh_private, const std::string sensor_name,
                                       const parameter_io::empty_params parameters)
{
  Eigen::Matrix<double, 1, 1> R = Eigen::Matrix<double, 1, 1>::Identity();
  parameter_io::getMatrixFromParams<1, 1>(ros::NodeHandle(nh_private, "sensors/" + sensor_name + "/R"), R);
  return std::make_shared<ImuYawSensorModelType>(R);
};

template <>
std::vector<std::string>
parseSensorModels<pacejka_car_state, pacejka_car_input>(ros::NodeHandle& nh, ros::NodeHandle& nh_private,
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
      ROS_INFO_STREAM("Added the following sensor models to estimator: " << sensor_name);
      // Add sensor model to loaded_sensors
      loaded_sensors.push_back(sensor_name);
    }
    else if (sensor_name == crs_sensor_models::pacejka_sensor_models::WheelEncoderSensorModel::SENSOR_KEY)
    {
      Eigen::Matrix4d R = Eigen::Matrix4d::Identity();
      parameter_io::getMatrixFromParams<4, 4>(ros::NodeHandle(nh, sensor_name + "/R"), R);

      // We need the wheel radius for the wheel encoder sensor model
      crs_models::pacejka_model::pacejka_params params;
      // Load model parameters from global config (global nodehandle)
      parameter_io::getModelParams<crs_models::pacejka_model::pacejka_params>(
          ros::NodeHandle(nh, "model/model_params/"), params);
      // Then overwrite specific parameters from local config (private nodehandle)
      parameter_io::getModelParams<crs_models::pacejka_model::pacejka_params>(
          ros::NodeHandle(nh_private, "model/model_params/"), params, false);

      std::shared_ptr<crs_sensor_models::pacejka_sensor_models::WheelEncoderSensorModel> sensor_model =
          std::make_shared<crs_sensor_models::pacejka_sensor_models::WheelEncoderSensorModel>(
              params.wheel_radius, params.lf, params.car_width, R);

      estimator->addSensorModel(sensor_name, sensor_model);
      ROS_INFO_STREAM("Added the following sensor models to estimator: " << sensor_name);
      // Add sensor model to loaded_sensors
      loaded_sensors.push_back(sensor_name);
    }
    else if (sensor_name == "imu")
    {
      crs_models::pacejka_model::pacejka_params params;
      // Load model parameters from global config (global nodehandle)
      parameter_io::getModelParams<crs_models::pacejka_model::pacejka_params>(
          ros::NodeHandle(nh, "model/model_params/"), params);
      // Then overwrite specific parameters from local config (private nodehandle)
      parameter_io::getModelParams<crs_models::pacejka_model::pacejka_params>(
          ros::NodeHandle(nh_private, "model/model_params/"), params, false);

      std::shared_ptr<ImuSensorModelType> sensor_model =
          loadSensorModel<ImuSensorModelType, crs_models::pacejka_model::pacejka_params>(nh_private, sensor_name,
                                                                                         params);
      estimator->addSensorModel(sensor_name, sensor_model);
      ROS_INFO_STREAM("Added the following sensor models to estimator: " << sensor_name);
      // Add sensor model to loaded_sensors
      loaded_sensors.push_back(sensor_name);
    }
    else if (sensor_name == "imu_yaw_rate")
    {
      std::shared_ptr<ImuYawSensorModelType> sensor_model =
          loadSensorModel<ImuYawSensorModelType>(nh_private, sensor_name, {});
      estimator->addSensorModel(sensor_name, sensor_model);
      // Add sensor model to loaded_sensors
      loaded_sensors.push_back(sensor_name);
    }
    else if (sensor_name == "imu_yaw_rate")
    {
      std::shared_ptr<ImuYawSensorModelType> sensor_model =
          loadSensorModel<ImuYawSensorModelType>(nh_private, sensor_name, {});
      estimator->addSensorModel(sensor_name, sensor_model);
      // Add sensor model to loaded_sensors
      loaded_sensors.push_back(sensor_name);
    }
    else if (sensor_name == crs_sensor_models::pacejka_sensor_models::LighthouseSensorModel::SENSOR_KEY)
    {
      Eigen::Matrix<double, 2, 4> sensor_pos = Eigen::Matrix<double, 2, 4>::Zero();
      parameter_io::getMatrixFromParams<2, 4>(ros::NodeHandle(nh_private, "sensors/" + sensor_name + "/sensor_pos"),
                                              sensor_pos);

      std::vector<std::string> base_stations;
      nh_private.getParam("sensors/" + sensor_name + "/base_stations", base_stations);
      std::cout << base_stations << std::endl;
      for (auto base_station : base_stations)  // Parse sensor models
      {
        if (nh_private.hasParam("sensors/" + sensor_name + "/" + base_station))
        {
          std::string bs_node = "sensors/" + sensor_name + "/" + base_station;
          Eigen::Matrix4d R = Eigen::Matrix4d::Identity();
          int bs_ID = 0;
          Eigen::Vector3d bs_pos{ 0, 0, 2 };
          Eigen::Matrix3d bs_rot;
          bs_rot << 0, 1, 0, 0, 0, -1, -1, 0, 0;
          double dt1 = 0;
          double dt2 = 0;

          parameter_io::getMatrixFromParams<4, 4>(ros::NodeHandle(nh_private, bs_node + "/R"), R);
          nh_private.getParam(bs_node + "/bs_ID", bs_ID);
          parameter_io::getMatrixFromParams<3, 1>(ros::NodeHandle(nh_private, bs_node + "/P_bs"), bs_pos);
          parameter_io::getMatrixFromParams<3, 3>(ros::NodeHandle(nh_private, bs_node + "/R_bs"), bs_rot);

          nh_private.getParam(bs_node + "/dt1", dt1);
          nh_private.getParam(bs_node + "/dt2", dt2);

          double lp_tilt_1 = -M_PI / 6 - dt1;
          double lp_tilt_2 = M_PI / 6 - dt2;

          std::shared_ptr<crs_sensor_models::pacejka_sensor_models::LighthouseSensorModel> lighthouse_sensor_model_1 =
              std::make_shared<crs_sensor_models::pacejka_sensor_models::LighthouseSensorModel>(R, bs_pos, bs_rot,
                                                                                                lp_tilt_1, sensor_pos);
          std::shared_ptr<crs_sensor_models::pacejka_sensor_models::LighthouseSensorModel> lighthouse_sensor_model_2 =
              std::make_shared<crs_sensor_models::pacejka_sensor_models::LighthouseSensorModel>(R, bs_pos, bs_rot,
                                                                                                lp_tilt_2, sensor_pos);

          estimator->addSensorModel("lighthouse_" + std::to_string(bs_ID) + "_1", lighthouse_sensor_model_1);
          estimator->addSensorModel("lighthouse_" + std::to_string(bs_ID) + "_2", lighthouse_sensor_model_2);

          ROS_INFO_STREAM("Added the following sensor models to estimator: " << "lighthouse_" + std::to_string(bs_ID) +
                                                                                    "_1");
          ROS_INFO_STREAM("Added the following sensor models to estimator: " << "lighthouse_" + std::to_string(bs_ID) +
                                                                                    "_2");

          // Add sensor model to loaded_sensors
          loaded_sensors.push_back("lighthouse");
        }
        else
        {
          ROS_WARN_STREAM("Missing key for base_station: " << base_station
                                                           << ". Base station will not be used for estimator");
        }
      }
    }
    else
    {
      ROS_ERROR_STREAM("Unknown sensor model " << sensor_name << ". Sensor will not be used for ekf");
    }
  }

  return loaded_sensors;
};
}  // namespace estimators
}  // namespace registry
