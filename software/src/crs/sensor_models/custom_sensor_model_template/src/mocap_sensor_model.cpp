
#include "custom_sensor_model_template/mocap_sensor_model.h"
#include <dynamic_models/utils/data_conversion.h>

// ==================== TODO ====================
// - Change namespace from custom_sensor_model to something sensible
// - Change state and input types in this file
// - Specify the Measurement Dimension by passing The correct number to the `SensorModel` Constructor (1)
// - Specify your state and input symbolic vairables (2), and construct the correct measured state (3)
// - Specify an appropriate sensor key (4)

namespace crs_sensor_models
{
namespace custom_sensor_models
{
// Option 2: Create an object and specify the process noise covariance matrix Q yourself.
// TODO (1)
MocapSensorModel::MocapSensorModel(const Eigen::Matrix<double, crs_models::custom_model::custom_state::NX,
                                                       crs_models::custom_model::custom_state::NX>& R)
  : SensorModel(3, MocapSensorModel::SENSOR_KEY)
{
  // TODO (2)
  std::vector<casadi::MX> state_mx = { casadi::MX::sym("x"),   casadi::MX::sym("y"),   casadi::MX::sym("yaw"),
                                       casadi::MX::sym("v_x"), casadi::MX::sym("v_y"), casadi::MX::sym("yaw_rate") };
  std::vector<casadi::MX> input_mx = { casadi::MX::sym("steer"), casadi::MX::sym("torque") };

  // TODO (3)
  std::vector<casadi::MX> measured_states_mx = { state_mx[0], state_mx[1], state_mx[2] };

  // `measurement_function` expects concatenated state and input as an argument
  state_mx.insert(state_mx.end(), input_mx.begin(), input_mx.end());
  measurement_function = casadi::Function("applyMeasurementModel", state_mx, measured_states_mx);

  setR(R);
}

// TODO (4)
const std::string MocapSensorModel::SENSOR_KEY = "mocap";

}  // namespace custom_sensor_models
}  // namespace crs_sensor_models
