#include <ros/ros.h>
#include "ros_estimators/state_estimator_ros.h"

#include <ros_crs_utils/parameter_io.h>
#include "ros_estimators/component_registry/resolve_estimator.h"

#include <lowpass_estimator/lowpass_estimator.h>
#include <lowpass_estimator/pacejka_lowpass_estimator.h>

#include <pacejka_model/pacejka_car_state.h>

// Model
typedef crs_models::pacejka_model::pacejka_car_state pacejka_car_state;

// Estimator
typedef crs_estimators::lowpass_estimator::PacejkaLowpassEstimator PacejkaLowpassEstimatorType;

namespace registry
{
namespace estimators
{

template <>
std::shared_ptr<PacejkaLowpassEstimatorType> loadCRSEstimator<PacejkaLowpassEstimatorType>(ros::NodeHandle& nh,
                                                                                           ros::NodeHandle& nh_private)
{
  pacejka_car_state initial_state =
      parameter_io::getState<pacejka_car_state>(ros::NodeHandle(nh_private, "initial_state"));

  crs_estimators::lowpass_estimator::car_lowpass_parameters parameters =
      parameter_io::getConfig<crs_estimators::lowpass_estimator::car_lowpass_parameters>(nh_private);
  auto lowpass_estimator = std::make_shared<PacejkaLowpassEstimatorType>(parameters, initial_state);

  return lowpass_estimator;
}

}  // namespace estimators
}  // namespace registry
