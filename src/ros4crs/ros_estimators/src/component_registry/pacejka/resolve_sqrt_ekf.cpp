#include <ros/ros.h>
#include "ros_estimators/state_estimator_ros.h"

#include <variant>

#include <ros_crs_utils/parameter_io.h>
#include "ros_estimators/component_registry/resolve_sensor_models.h"
#include "ros_estimators/component_registry/resolve_estimator.h"

#include <kalman_estimator/sqrt_ekf.h>
#include <kalman_estimator/car_kalman_parameters.h>

#include <pacejka_model/pacejka_car_input.h>
#include <pacejka_model/pacejka_car_state.h>
#include <pacejka_model/pacejka_discrete.h>

// Model
typedef crs_models::pacejka_model::pacejka_car_state pacejka_car_state;
typedef crs_models::pacejka_model::pacejka_car_input pacejka_car_input;
typedef crs_models::pacejka_model::DiscretePacejkaModel DiscretePacejkaModelType;

// Estimator
typedef crs_estimators::kalman::SqrtEKF<pacejka_car_state, pacejka_car_input> SqrtEkfType;

namespace registry
{
namespace estimators
{
template <>
std::shared_ptr<SqrtEkfType> loadCRSEstimator<SqrtEkfType>(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
{
  // LOAD EKF
  pacejka_car_state initial_state =
      parameter_io::getState<pacejka_car_state>(ros::NodeHandle(nh_private, "initial_state"));
  std::vector<std::string> sensors_to_load;
  nh_private.getParam("sensors/sensor_names", sensors_to_load);

  // Create shard pointer to (crs) sqrt ekf
  // This has two stages: First load generic model parameters which are shared with all components
  Eigen::Matrix<double, pacejka_car_state::NX, pacejka_car_state::NX> Q;
  parameter_io::getMatrixFromParams<pacejka_car_state::NX, pacejka_car_state::NX>(ros::NodeHandle(nh, "model/Q"), Q);
  crs_models::pacejka_model::pacejka_params params;
  parameter_io::getModelParams<crs_models::pacejka_model::pacejka_params>(ros::NodeHandle(nh, "model/model_params/"),
                                                                          params);
  // Then overwrite specific parameters from local config (private nodehandle)
  parameter_io::getModelParams<crs_models::pacejka_model::pacejka_params>(
      ros::NodeHandle(nh_private, "model/model_params/"), params, false);
  parameter_io::getMatrixFromParams<pacejka_car_state::NX, pacejka_car_state::NX>(
      ros::NodeHandle(nh_private, "model/Q"), Q);

  // Create Pacejka model
  std::shared_ptr<DiscretePacejkaModelType> model = std::make_shared<DiscretePacejkaModelType>(params, Q);

  pacejka_car_input initial_input =
      parameter_io::getInput<pacejka_car_input>(ros::NodeHandle(nh_private, "initial_input"));

  Eigen::Matrix<double, pacejka_car_state::NX, pacejka_car_state::NX> P_init;
  if (!parameter_io::getMatrixFromParams<pacejka_car_state::NX, pacejka_car_state::NX>(
          ros::NodeHandle(nh_private, "P_init"), P_init))
  {
    ROS_WARN_STREAM("[Pacejka SQRT EKF Component Registry] No initial P set. Using identity.");
    P_init = Eigen::Matrix<double, pacejka_car_state::NX, pacejka_car_state::NX>::Identity();
  }

  // Create EKF
  std::vector<std::pair<std::string, std::tuple<bool, std::string, double, int>>> outlier_rejection_params;
  bool log_diagnostic_data = false;

  for (const std::string sensor_name : sensors_to_load)
  {
    crs_estimators::kalman::car_kalman_parameters kalman_params =
        parameter_io::getConfig<crs_estimators::kalman::car_kalman_parameters>(
            ros::NodeHandle(nh_private, "sensors/" + sensor_name + "/outlier_rejection"));

    std::tuple<bool, std::string, double, int> outlier_settings = { kalman_params.use_outlier_rejection,
                                                                    kalman_params.outlier_rejection_type,
                                                                    kalman_params.outlier_threshold,
                                                                    kalman_params.max_consecutive_outliers };

    if (!nh_private.getParam("log_diagnostic_data", log_diagnostic_data))
    {
      ROS_WARN_STREAM("[Pacejka EKF Component Registry] No log_diagnostic_data set. Using false.");
    }

    if (sensor_name == "lighthouse")  // TODO, ugly, unify with key, name refactoring of sensor models.
    {
      std::string base_station_id = "100";
      int bs_ID = 100;
      std::vector<std::string> base_stations;
      nh_private.getParam("sensors/" + sensor_name + "/base_stations", base_stations);

      for (auto base_station : base_stations)  // Parse sensor models
      {
        std::string bs_node = "sensors/" + sensor_name + "/" + base_station;
        nh_private.getParam(bs_node + "/bs_ID", bs_ID);

        if (bs_ID == 0)
        {
          base_station_id = "0";
          outlier_rejection_params.push_back(
              std::make_pair<>(sensor_name + "_" + base_station_id + "_1", outlier_settings));
          outlier_rejection_params.push_back(
              std::make_pair<>(sensor_name + "_" + base_station_id + "_2", outlier_settings));
        }
        else if (bs_ID == 5)
        {
          base_station_id = "5";
          outlier_rejection_params.push_back(
              std::make_pair<>(sensor_name + "_" + base_station_id + "_1", outlier_settings));
          outlier_rejection_params.push_back(
              std::make_pair<>(sensor_name + "_" + base_station_id + "_2", outlier_settings));
        }
        else
        {
          ROS_ERROR_STREAM("Base station ID not recognized. Must be 0 or 5.");
          return nullptr;
        }
      }
    }
    else
    {
      outlier_rejection_params.push_back(std::make_pair<>(sensor_name, outlier_settings));
    }
  }
  auto ekf = std::make_shared<SqrtEkfType>(model, initial_state, initial_input, P_init, outlier_rejection_params,
                                           log_diagnostic_data);
  return ekf;
}

}  // namespace estimators
}  // namespace registry
