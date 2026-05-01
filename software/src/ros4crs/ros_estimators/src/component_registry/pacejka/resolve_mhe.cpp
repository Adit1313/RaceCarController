#include <ros/ros.h>
#include "ros_estimators/state_estimator_ros.h"

#include <ros_crs_utils/parameter_io.h>
#include "ros_estimators/component_registry/resolve_sensor_models.h"
#include "ros_estimators/component_registry/resolve_estimator.h"

#include <mh_estimator/pacejka_mhe.h>

#include <pacejka_model/pacejka_car_input.h>
#include <pacejka_model/pacejka_car_state.h>
#include <pacejka_model/pacejka_discrete.h>

// Model
typedef crs_models::pacejka_model::pacejka_car_state pacejka_car_state;
typedef crs_models::pacejka_model::pacejka_car_input pacejka_car_input;
typedef crs_models::pacejka_model::DiscretePacejkaModel DiscretePacejkaModelType;

// Estimator
namespace registry
{
namespace estimators
{
template <>
std::shared_ptr<crs_estimators::mhe::Pacejka_MHE>
loadCRSEstimator<crs_estimators::mhe::Pacejka_MHE>(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
{
  std::shared_ptr<crs_estimators::kalman::DiscreteEKF<pacejka_car_state, pacejka_car_input>> internal_ekf =
      registry::estimators::loadCRSEstimator<crs_estimators::kalman::DiscreteEKF<pacejka_car_state, pacejka_car_input>>(
          nh, nh_private);
  // Register sensor models for internal
  parseSensorModels<pacejka_car_state, pacejka_car_input>(nh, nh_private, internal_ekf);

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

  crs_estimators::pacejka_mhe_config config = parameter_io::getConfig<crs_estimators::pacejka_mhe_config>(nh_private);

  // Outlier rejection parameters
  std::vector<std::pair<std::string, std::tuple<bool, double>>> outlier_rejection_params;
  bool log_diagnostic_data = false;

  // negative bs id does not exist. Initialize to -1, overwrite if lighthouse is used.
  std::string base_station_id = "-1";

  for (const std::string sensor_name : sensors_to_load)
  {
    crs_estimators::pacejka_mhe_config mhe_params_outlier =
        parameter_io::getOutlierParams<crs_estimators::pacejka_mhe_config>(
            ros::NodeHandle(nh_private, "sensors/" + sensor_name + "/MHE_outlier_rejection"));

    std::tuple<bool, double> outlier_settings = { mhe_params_outlier.use_outlier_rejection,
                                                  mhe_params_outlier.outlier_threshold };

    if (!nh_private.getParam("log_diagnostic_data", log_diagnostic_data))
    {
      ROS_WARN_STREAM("[Pacejka EKF Component Registry] No log_diagnostic_data set. Using false.");
    }

    if (sensor_name == "lighthouse")  // TODO, ugly, unify with key, name refactoring of sensor models.
    {
      base_station_id = "100";
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

  return std::make_shared<crs_estimators::mhe::Pacejka_MHE>(config, model, initial_state, initial_input, internal_ekf,
                                                            base_station_id, outlier_rejection_params,
                                                            log_diagnostic_data);
}

}  // namespace estimators
}  // namespace registry
