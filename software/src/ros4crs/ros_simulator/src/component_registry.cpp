#include "ros_simulator/component_registry.h"
#include <ros_crs_utils/parameter_io.h>

#ifdef pacejka_model_FOUND
#include "ros_simulator/ros_pacejka_simulator.h"
#include <pacejka_sensor_model/imu_sensor_model.h>
#include <pacejka_sensor_model/imu_yaw_rate_sensor_model.h>
#include <pacejka_sensor_model/mocap_sensor_model.h>
#include <pacejka_sensor_model/wheel_encoder_sensor_model.h>
#include <pacejka_sensor_model/lighthouse_sensor_model.h>
#endif

#ifdef kinematic_model_FOUND
#include "ros_simulator/ros_kinematic_simulator.h"
#include <kinematic_sensor_model/imu_sensor_model.h>
#include <kinematic_sensor_model/mocap_sensor_model.h>
#endif

#ifdef rocket_6_dof_model_FOUND
#include "ros_simulator/ros_rocket_simulator.h"
#include <rocket_6_dof_sensor_model/full_state_sensor_model.h>
#include <rocket_6_dof_sensor_model/imu_sensor_model.h>
#include <rocket_6_dof_sensor_model/mocap_pose_sensor_model.h>
#include <rocket_6_dof_sensor_model/mocap_twist_sensor_model.h>
#endif

namespace ros_simulator
{
Simulator* resolveSimulator(ros::NodeHandle& nh, ros::NodeHandle& nh_private, const std::string& state_type,
                            const std::string& input_type, const std::vector<std::string>& sensors_to_load)
{
  //  ========================== PACEJKA MODEL  ==========================

#ifdef pacejka_model_FOUND
  if (state_type == "pacejka_car" && input_type == "pacejka_car")
  {
    // Create pacejka simulator
    auto* pacejka_simulator = new ros_simulator::PacejkaSimulator(nh, nh_private);

    for (auto sensor_name : sensors_to_load)  // Parse sensor models
    {
      std::string sensor_key;
      if (!nh_private.getParam("sensors/" + sensor_name + "/key", sensor_key))  // Load sensor key from params
                                                                                // (pacejka_car_simulator.yaml)
        sensor_key = sensor_name;

      // Measurement Delay
      double delay = 0.0;
      nh_private.getParam("sensors/" + sensor_name + "/delay", delay);

      if (sensor_key == crs_sensor_models::pacejka_sensor_models::MocapSensorModel::SENSOR_KEY)
      {
        // ===================== MOCAP MODEL ===============================================
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        parameter_io::getMatrixFromParams<3, 3>(ros::NodeHandle(nh_private, "sensors/" + sensor_name + "/R"),
                                                R);  // Load R from params (pacejka_car_simulator.yaml)

        // Create mocap sensor model using R
        std::shared_ptr<crs_sensor_models::pacejka_sensor_models::MocapSensorModel> mocap_sensor_model =
            std::make_shared<crs_sensor_models::pacejka_sensor_models::MocapSensorModel>(R);

        // Sensor used for simulation
        pacejka_simulator->registerSensorModel(mocap_sensor_model, delay);
      }
      else if (sensor_key == crs_sensor_models::pacejka_sensor_models::ImuSensorModel::SENSOR_KEY)
      {
        // ===================== IMU MODEL ===============================================
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        parameter_io::getMatrixFromParams<3, 3>(ros::NodeHandle(nh_private, "sensors/" + sensor_name + "/R"),
                                                R);  // Load R from params (pacejka_car_simulator.yaml)

        // Sensor used for simulation.
        // IMU sensor model requires model dynamics!
        crs_models::pacejka_model::pacejka_params params;
        parameter_io::getModelParams<crs_models::pacejka_model::pacejka_params>(
            ros::NodeHandle(nh, "model/model_params/"), params);

        auto cont_model = std::make_shared<crs_models::pacejka_model::ContinuousPacejkaModel>(params);
        pacejka_simulator->registerSensorModel(
            std::make_shared<crs_sensor_models::pacejka_sensor_models::ImuSensorModel>(cont_model, R), delay);
      }
      else if (sensor_key == crs_sensor_models::pacejka_sensor_models::ImuYawSensorModel::SENSOR_KEY)
      {
        // ===================== IMU YAW RATE MODEL ===============================================
        Eigen::Matrix<double, 1, 1> R = Eigen::Matrix<double, 1, 1>::Identity();
        parameter_io::getMatrixFromParams<1, 1>(ros::NodeHandle(nh_private, "sensors/" + sensor_name + "/R"),
                                                R);  // Load R from params (pacejka_car_simulator.yaml)
        // Create imu yaw rate sensor model using R
        std::shared_ptr<crs_sensor_models::pacejka_sensor_models::ImuYawSensorModel> imu_yaw_rate_sensor_model =
            std::make_shared<crs_sensor_models::pacejka_sensor_models::ImuYawSensorModel>(R);

        // Sensor used for simulation
        pacejka_simulator->registerSensorModel(imu_yaw_rate_sensor_model, delay);
      }
      else if (sensor_key == crs_sensor_models::pacejka_sensor_models::WheelEncoderSensorModel::SENSOR_KEY)
      {
        // ===================== Wheel encocer MODEL ===============================================
        Eigen::Matrix4d R = Eigen::Matrix4d::Identity();
        parameter_io::getMatrixFromParams<4, 4>(ros::NodeHandle(nh_private, "sensors/" + sensor_name + "/R"),
                                                R);  // Load R from params (pacejka_car_simulator.yaml)

        // Sensor used for simulation.
        crs_models::pacejka_model::pacejka_params params;
        parameter_io::getModelParams<crs_models::pacejka_model::pacejka_params>(
            ros::NodeHandle(nh, "model/model_params/"), params);

        // We need certain model params.
        // auto wheel_radius = params.wheel_radius;
        pacejka_simulator->registerSensorModel(
            std::make_shared<crs_sensor_models::pacejka_sensor_models::WheelEncoderSensorModel>(
                params.wheel_radius, params.lf, params.car_width, R),
            delay);
      }
      else if (sensor_key == crs_sensor_models::pacejka_sensor_models::LighthouseSensorModel::SENSOR_KEY)
      {
        // ===================== LIGHTHOUSE MODEL ===============================================
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
                std::make_shared<crs_sensor_models::pacejka_sensor_models::LighthouseSensorModel>(
                    R, bs_pos, bs_rot, lp_tilt_1, sensor_pos);
            std::shared_ptr<crs_sensor_models::pacejka_sensor_models::LighthouseSensorModel> lighthouse_sensor_model_2 =
                std::make_shared<crs_sensor_models::pacejka_sensor_models::LighthouseSensorModel>(
                    R, bs_pos, bs_rot, lp_tilt_2, sensor_pos);

            pacejka_simulator->registerSensorModel(lighthouse_sensor_model_1, delay,
                                                   "lighthouse_" + std::to_string(bs_ID) + "_1");
            pacejka_simulator->registerSensorModel(lighthouse_sensor_model_2, delay,
                                                   "lighthouse_" + std::to_string(bs_ID) + "_2");

            ROS_INFO_STREAM("Added the following sensor models to simulator: lighthouse_"
                            << std::to_string(bs_ID) + "_1");
            ROS_INFO_STREAM("Added the following sensor models to simulator: lighthouse_"
                            << std::to_string(bs_ID) + "_2");

            // Add sensor model to loaded_sensors
            // loaded_sensors.push_back("lighthouse");
          }
          else
          {
            ROS_WARN_STREAM("Missing key for base_station: " << base_station
                                                             << ". Base station will not be used for ekf");
          }
        }

        // Sensor used for simulation
      }

      // e.g. add imu model
      else
      {
        ROS_WARN_STREAM("Unknown sensor model " << sensor_name << ". Sensor model will not be loaded!");
      }
    }
    return pacejka_simulator;
  }
#endif

#ifdef kinematic_model_FOUND
  if (state_type == "kinematic_car" && input_type == "kinematic_car")
  {
    // Create kinematic_simulator simulator
    auto* kinematic_simulator = new ros_simulator::KinematicSimulator(nh, nh_private);
    for (auto sensor_name : sensors_to_load)  // Parse sensor models
    {
      std::string sensor_key;
      if (!nh_private.getParam("sensors/" + sensor_name + "/key", sensor_key))  // Load sensor key from params
                                                                                // (pacejka_car_simulator.yaml)
        sensor_key = sensor_name;
      // Measurement Delay
      double delay = 0.0;
      nh_private.getParam("sensors/" + sensor_name + "/delay", delay);

      if (sensor_key == crs_sensor_models::kinematic_sensor_models::MocapSensorModel::SENSOR_KEY)
      {
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        parameter_io::getMatrixFromParams<3, 3>(ros::NodeHandle(nh_private, "sensors/" + sensor_name + "/R"),
                                                R);  // Load R from params (pacejka_car_simulator.yaml)

        // Create mocap sensor model using R
        std::shared_ptr<crs_sensor_models::kinematic_sensor_models::MocapSensorModel> mocap_sensor_model =
            std::make_shared<crs_sensor_models::kinematic_sensor_models::MocapSensorModel>(R);

        // Sensor used for simulation
        kinematic_simulator->registerSensorModel(mocap_sensor_model, delay);
      }
      else if (sensor_key == crs_sensor_models::kinematic_sensor_models::ImuSensorModel::SENSOR_KEY)
      {
        // ===================== IMU MODEL ===============================================
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        parameter_io::getMatrixFromParams<3, 3>(ros::NodeHandle(nh_private, "sensors/" + sensor_name + "/R"),
                                                R);  // Load R from params (pacejka_car_simulator.yaml)

        // Sensor used for simulation.
        // IMU sensor model requires model dynamics!
        crs_models::kinematic_model::kinematic_params params;
        parameter_io::getModelParams<crs_models::kinematic_model::kinematic_params>(
            ros::NodeHandle(nh, "model/model_params/"), params);

        auto cont_model = std::make_shared<crs_models::kinematic_model::ContinuousKinematicModel>(params);
        kinematic_simulator->registerSensorModel(
            std::make_shared<crs_sensor_models::kinematic_sensor_models::ImuSensorModel>(cont_model, R), delay);
      }

      // e.g. add imu model
      else
      {
        ROS_WARN_STREAM("Unknown sensor model " << sensor_name << ". Sensor model will not be loaded!");
      }
    }
    return kinematic_simulator;
  }
#endif

  //  ========================== ROCKET MODEL  ==========================
#ifdef rocket_6_dof_model_FOUND
  if (state_type == "rocket" && input_type == "rocket")
  {
    // Create rocket simulator
    auto* rocket_simulator = new ros_simulator::RocketSimulator(nh, nh_private);

    for (auto sensor_name : sensors_to_load)  // Parse sensor models
    {
      std::string sensor_key;
      std::cout << "loading sensor " << sensor_name << std::endl;
      if (!nh_private.getParam("sensors/" + sensor_name + "/key", sensor_key))  // Load sensor key from params
                                                                                // (rocket_simulator.yaml)
        sensor_key = sensor_name;

      // Measurement Delay
      double delay = 0.0;
      nh_private.getParam("sensors/" + sensor_name + "/delay", delay);

      if (sensor_key == crs_sensor_models::rocket_6_dof_sensor_models::FullStateSensorModel::SENSOR_KEY)
      {
        Eigen::Matrix<double, crs_models::rocket_6_dof_model::rocket_6_dof_state::NX,
                      crs_models::rocket_6_dof_model::rocket_6_dof_state::NX>
            R = Eigen::Matrix<double, crs_models::rocket_6_dof_model::rocket_6_dof_state::NX,
                              crs_models::rocket_6_dof_model::rocket_6_dof_state::NX>::Identity();
        parameter_io::getMatrixFromParams<crs_models::rocket_6_dof_model::rocket_6_dof_state::NX,
                                          crs_models::rocket_6_dof_model::rocket_6_dof_state::NX>(
            ros::NodeHandle(nh_private, "sensors/" + sensor_name + "/R"),
            R);  // Load R from params (rocket_simulator.yaml)

        // Create full state sensor model using R
        std::shared_ptr<crs_sensor_models::rocket_6_dof_sensor_models::FullStateSensorModel> full_state_sensor_model =
            std::make_shared<crs_sensor_models::rocket_6_dof_sensor_models::FullStateSensorModel>(R);

        // Sensor used for simulation
        rocket_simulator->registerSensorModel(full_state_sensor_model, delay);
      }
      else if (sensor_key == crs_sensor_models::rocket_6_dof_sensor_models::ImuSensorModel::SENSOR_KEY)
      {
        Eigen::Matrix<double, 6, 6> R = Eigen::Matrix<double, 6, 6>::Identity();
        parameter_io::getMatrixFromParams<6, 6>(ros::NodeHandle(nh_private, "sensors/" + sensor_name + "/R"),
                                                R);  // Load R from params (rocket_simulator.yaml)

        // IMU sensor model requires model dynamics!
        crs_models::rocket_6_dof_model::rocket_6_dof_params params;
        parameter_io::getModelParams<crs_models::rocket_6_dof_model::rocket_6_dof_params>(
            ros::NodeHandle(nh, "model/model_params/"), params);

        auto cont_model = std::make_shared<crs_models::rocket_6_dof_model::ContinuousRocket6DofModel>(params);

        // Create full state sensor model using R
        std::shared_ptr<crs_sensor_models::rocket_6_dof_sensor_models::ImuSensorModel> imu_sensor_model =
            std::make_shared<crs_sensor_models::rocket_6_dof_sensor_models::ImuSensorModel>(cont_model, R);

        // Sensor used for simulation
        rocket_simulator->registerSensorModel(imu_sensor_model, delay);
      }
      else if (sensor_key == crs_sensor_models::rocket_6_dof_sensor_models::MocapPoseSensorModel::SENSOR_KEY)
      {
        Eigen::Matrix<double, 7, 7> R = Eigen::Matrix<double, 7, 7>::Identity();
        parameter_io::getMatrixFromParams<7, 7>(ros::NodeHandle(nh_private, "sensors/" + sensor_name + "/R"),
                                                R);  // Load R from params (rocket_simulator.yaml)

        // Create full state sensor model using R
        std::shared_ptr<crs_sensor_models::rocket_6_dof_sensor_models::MocapPoseSensorModel> mocap_pose_sensor_model =
            std::make_shared<crs_sensor_models::rocket_6_dof_sensor_models::MocapPoseSensorModel>(R);

        // Sensor used for simulation
        rocket_simulator->registerSensorModel(mocap_pose_sensor_model, delay);
      }
      else if (sensor_key == crs_sensor_models::rocket_6_dof_sensor_models::MocapTwistSensorModel::SENSOR_KEY)
      {
        Eigen::Matrix<double, 6, 6> R = Eigen::Matrix<double, 6, 6>::Identity();
        parameter_io::getMatrixFromParams<6, 6>(ros::NodeHandle(nh_private, "sensors/" + sensor_name + "/R"),
                                                R);  // Load R from params (rocket_simulator.yaml)

        // Create full state sensor model using R
        std::shared_ptr<crs_sensor_models::rocket_6_dof_sensor_models::MocapTwistSensorModel> mocap_twist_sensor_model =
            std::make_shared<crs_sensor_models::rocket_6_dof_sensor_models::MocapTwistSensorModel>(R);

        // Sensor used for simulation
        rocket_simulator->registerSensorModel(mocap_twist_sensor_model, delay);
      }
      else
      {
        ROS_WARN_STREAM("Unknown sensor model " << sensor_name << ". Sensor model will not be loaded!");
      }
    }
    return rocket_simulator;
  }
#endif

  return nullptr;
}
}  // namespace ros_simulator
