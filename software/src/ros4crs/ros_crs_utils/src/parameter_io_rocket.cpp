#include <ros_crs_utils/parameter_io.h>

#include <rocket_6_dof_model/rocket_6_dof_params.h>
#include <rocket_6_dof_model/rocket_6_dof_state.h>
#include <rocket_6_dof_model/rocket_6_dof_input.h>

namespace parameter_io
{

template <>
crs_models::rocket_6_dof_model::rocket_6_dof_state getState(const ros::NodeHandle& nh)
{
  crs_models::rocket_6_dof_model::rocket_6_dof_state state;
  std::vector<double> state_as_vec;
  if (!nh.getParam("value", state_as_vec))
  {
    ROS_WARN_STREAM("Could not load initial state from parameters.");
  }
  else
  {
    state.position_x = state_as_vec[0];
    state.position_y = state_as_vec[1];
    state.position_z = state_as_vec[2];
    state.velocity_x = state_as_vec[3];
    state.velocity_y = state_as_vec[4];
    state.velocity_z = state_as_vec[5];
    state.quaternion_x = state_as_vec[6];
    state.quaternion_y = state_as_vec[7];
    state.quaternion_z = state_as_vec[8];
    state.quaternion_w = state_as_vec[9];
    state.angular_velocity_x = state_as_vec[10];
    state.angular_velocity_y = state_as_vec[11];
    state.angular_velocity_z = state_as_vec[12];
    state.thrust_magnitude = state_as_vec[13];
    state.torque_x = state_as_vec[14];
    state.servo_angle_1 = state_as_vec[15];
    state.servo_angle_2 = state_as_vec[16];
  }
  return state;
}

template <>
crs_models::rocket_6_dof_model::rocket_6_dof_input getInput(const ros::NodeHandle& nh)
{
  crs_models::rocket_6_dof_model::rocket_6_dof_input input;
  std::vector<double> input_as_vec;
  if (!nh.getParam("value", input_as_vec))  // Load initial input from params
                                            // (rocket_ekf.yaml)
  {
    ROS_WARN_STREAM("Could not load initial input!");
  }
  else
  {
    input.thrust_magnitude = input_as_vec[0];
    input.torque = input_as_vec[1];
    input.servo_angle_1 = input_as_vec[2];
    input.servo_angle_2 = input_as_vec[3];
  }
  return input;
}

template <>
void getModelParams<>(const ros::NodeHandle& nh, crs_models::rocket_6_dof_model::rocket_6_dof_params& params,
                      bool verbose /* = true*/)
{
  if (!nh.getParam("mass", params.mass) && verbose)
    ROS_WARN_STREAM(" RocketParams: did not load mass. Namespace " << nh.getNamespace());
  if (!nh.getParam("gravity_constant", params.gravity_constant) && verbose)
    ROS_WARN_STREAM(" RocketParams: did not load gravity_constant. Namespace " << nh.getNamespace());
  if (!nh.getParam("inertia_xx", params.inertia_xx) && verbose)
    ROS_WARN_STREAM(" RocketParams: did not load inertia_xx. Namespace " << nh.getNamespace());
  if (!nh.getParam("inertia_yy", params.inertia_yy) && verbose)
    ROS_WARN_STREAM(" RocketParams: did not load inertia_yy. Namespace " << nh.getNamespace());
  if (!nh.getParam("inertia_zz", params.inertia_zz) && verbose)
    ROS_WARN_STREAM(" RocketParams: did not load inertia_zz. Namespace " << nh.getNamespace());
  if (!nh.getParam("thrust_cog_offset", params.thrust_cog_offset) && verbose)
    ROS_WARN_STREAM(" RocketParams: did not load thrust_cog_offset. Namespace " << nh.getNamespace());
  if (!nh.getParam("thrust_magnitude_time_constant", params.thrust_magnitude_time_constant) && verbose)
    ROS_WARN_STREAM(" RocketParams: did not load thrust_magnitude_time_constant. Namespace " << nh.getNamespace());
  if (!nh.getParam("servo_angle_time_constant", params.servo_angle_time_constant) && verbose)
    ROS_WARN_STREAM(" RocketParams: did not load servo_angle_time_constant. Namespace " << nh.getNamespace());
  if (!nh.getParam("gimbal_a", params.gimbal_a) && verbose)
    ROS_WARN_STREAM(" RocketParams: did not load gimbal_a. Namespace " << nh.getNamespace());
  if (!nh.getParam("gimbal_b", params.gimbal_b) && verbose)
    ROS_WARN_STREAM(" RocketParams: did not load gimbal_b. Namespace " << nh.getNamespace());
  if (!nh.getParam("gimbal_c", params.gimbal_c) && verbose)
    ROS_WARN_STREAM(" RocketParams: did not load gimbal_c. Namespace " << nh.getNamespace());
  if (!nh.getParam("gimbal_d", params.gimbal_d) && verbose)
    ROS_WARN_STREAM(" RocketParams: did not load gimbal_d. Namespace " << nh.getNamespace());
  if (!nh.getParam("gimbal_e", params.gimbal_e) && verbose)
    ROS_WARN_STREAM(" RocketParams: did not load gimbal_e. Namespace " << nh.getNamespace());
}
}  // namespace parameter_io
