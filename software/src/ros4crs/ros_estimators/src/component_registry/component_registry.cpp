
#include "ros_estimators/component_registry/component_registry.h"
#include "ros_estimators/data_converter.h"

#include <estimators/base_estimator.h>
#include <estimators/model_based_estimator.h>

#include <ros_crs_utils/parameter_io.h>
#include <ros_crs_utils/state_message_conversion.h>
#include <ros_estimators/car_estimator/car_estimator.h>

#include <ros_estimators/component_registry/resolve_estimator.h>

// ========= MODELS =========
// Pacejka
#ifdef pacejka_model_FOUND
#include <pacejka_model/pacejka_car_input.h>
#include <pacejka_model/pacejka_car_state.h>
#include <pacejka_model/pacejka_discrete.h>
#include <pacejka_sensor_model/imu_sensor_model.h>
#include <pacejka_sensor_model/imu_yaw_rate_sensor_model.h>
#include <pacejka_sensor_model/mocap_sensor_model.h>

typedef crs_models::pacejka_model::pacejka_car_state pacejka_car_state;
typedef crs_models::pacejka_model::pacejka_car_input pacejka_car_input;
#endif

// Kinematic

#ifdef kinematic_model_FOUND
#include <kinematic_model/kinematic_discrete.h>
#include <kinematic_sensor_model/imu_sensor_model.h>
#include <kinematic_sensor_model/mocap_sensor_model.h>

typedef crs_models::kinematic_model::kinematic_car_input kinematic_car_input;
typedef crs_models::kinematic_model::kinematic_car_state kinematic_car_state;
typedef crs_models::kinematic_model::kinematic_params KinematicParamsType;
#endif

// Stacked
#if defined(imu_bias_FOUND) && defined(stacked_model_FOUND)
#include <stacked_model/pacejka_imu_bias_car_state.h>
#include <imu_bias/imu_bias_discrete.h>
#include <stacked_model/stacked_two_models_discrete.h>
#include <pacejka_with_imu_bias_sensor_model/imu_with_bias_sensor_model.h>
#include <pacejka_with_imu_bias_sensor_model/mocap_with_imu_bias_sensor_model.h>

typedef crs_models::stacked_model::pacejka_imu_bias_car_state pacejka_imu_bias_car_state;
#endif

// ========= ESTIMATORS =========
// Kalman (Ekf, SqrtEkf)
#ifdef kalman_estimator_FOUND

// Pacejka
#ifdef pacejka_model_FOUND
#include <kalman_estimator/discrete_ekf.h>
#include <kalman_estimator/sqrt_ekf.h>

typedef crs_estimators::kalman::DiscreteEKF<pacejka_car_state, pacejka_car_input> PacjekaEkfType;
typedef crs_estimators::kalman::SqrtEKF<pacejka_car_state, pacejka_car_input> PacejkaSqrtEkfType;
#endif

// Kinematic
#ifdef kinematic_model_FOUND
typedef crs_estimators::kalman::DiscreteEKF<kinematic_car_state, kinematic_car_input> KinematicEkfType;
typedef crs_estimators::kalman::SqrtEKF<kinematic_car_state, kinematic_car_input> KinematicSqrtEkfType;
#endif
#endif

// Lowpass
#ifdef lowpass_estimator_FOUND
#include <lowpass_estimator/pacejka_lowpass_estimator.h>

typedef crs_estimators::lowpass_estimator::PacejkaLowpassEstimator PacejkaLowpassEstimatorType;
#endif

// MHE
#ifdef mh_estimator_FOUND
#include <mh_estimator/pacejka_mhe.h>
#endif

namespace registry
{
namespace estimators
{
ros_estimators::RosStateEstimator* parseEstimator(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
{
  std::string estimator_type;
  std::string state_type;

  // Load estimator type
  if (!ros::NodeHandle(nh_private).getParam("type", estimator_type))
  {
    ROS_ERROR("No Estimator Type specified! Available type options are: "
              "discrete_ekf, sqrt_ekf, lowpass, mhe");
    return nullptr;
  }
  // Load state type
  if (!ros::NodeHandle(nh_private).getParam("initial_state/type", state_type))
  {
    ROS_ERROR("No State Type specified! Available type options are: "
              "pacejka_car, kinematic_car, pacejka_imu_bias_car");
    return nullptr;
  }

  // ======== PACEJKA MODEL ========
  if (state_type == "pacejka_car")
  {
#ifdef pacejka_model_FOUND
    //--------------------------------------------------------------------------------
    if (estimator_type == "discrete_ekf")
    {
#ifdef kalman_estimator_FOUND
      return loadCarEstimator<pacejka_car_state, pacejka_car_input, PacjekaEkfType>(nh, nh_private);
#endif
      ROS_ERROR("[Ros Estimators Component Registry] Package kalman_estimator not found!");
    }
    //--------------------------------------------------------------------------------
    if (estimator_type == "sqrt_ekf")
    {
#ifdef kalman_estimator_FOUND
      return loadCarEstimator<pacejka_car_state, pacejka_car_input, PacejkaSqrtEkfType>(nh, nh_private);
#endif
      ROS_ERROR("[Ros Estimators Component Registry] Package kalman_estimator not found!");
    }
    //--------------------------------------------------------------------------------
    if (estimator_type == "lowpass")
    {
#ifdef lowpass_estimator_FOUND
      return loadCarEstimator<pacejka_car_state, pacejka_car_input, PacejkaLowpassEstimatorType>(nh, nh_private);
#endif
      ROS_ERROR("[Ros Estimators Component Registry] Package lowpass_estimator not found!");
    }
    if (estimator_type == "mhe")
    {
#ifdef mh_estimator_FOUND
      return loadCarEstimator<pacejka_car_state, pacejka_car_input, crs_estimators::mhe::Pacejka_MHE>(nh, nh_private);
#endif
      ROS_ERROR("[Ros Estimators Component Registry] Package lowpass_estimator not found!");
    }

#endif
    ROS_ERROR("[Ros Estimators Component Registry] Package pacejka_model not found!");
  }

  // ======== KINEMATIC MODEL ========
  if (state_type == "kinematic_car")
  {
#ifdef kinematic_model_FOUND
    //--------------------------------------------------------------------------------
    if (estimator_type == "discrete_ekf")
    {
#ifdef kalman_estimator_FOUND
      return loadCarEstimator<kinematic_car_state, kinematic_car_input, KinematicEkfType, KinematicParamsType>(
          nh, nh_private);
#endif
      ROS_ERROR("[Ros Estimators Component Registry] Package kalman_estimator not found!");
    }
    //--------------------------------------------------------------------------------
    if (estimator_type == "sqrt_ekf")
    {
#ifdef kalman_estimator_FOUND
      return loadCarEstimator<kinematic_car_state, kinematic_car_input, KinematicSqrtEkfType, KinematicParamsType>(
          nh, nh_private);
#endif
      ROS_ERROR("[Ros Estimators Component Registry] Package kalman_estimator not found!");
    }

#endif
    ROS_ERROR("[Ros Estimators Component Registry] Package kinematic_model not found!");
  }

  return nullptr;
}

}  // namespace estimators
}  // namespace registry
