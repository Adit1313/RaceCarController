#ifndef ROCKET_POSITION_PID_ROCKET_HIGH_LEVEL_PID_CONTROLLER_CONFIG
#define ROCKET_POSITION_PID_ROCKET_HIGH_LEVEL_PID_CONTROLLER_CONFIG

#include <vector>
#include <Eigen/Core>

#include "rocket_control_commons/rocket_high_level_controller_base_config.h"

namespace crs_controls
{
struct rocket_high_level_pid_controller_config : public rocket_high_level_controller_base_config
{
  /**
   * @brief Position Control P gain
   */
  Eigen::Vector3d position_p_gain;

  /**
   * @brief Velocity Control P gain
   */
  Eigen::Vector3d velocity_p_gain;

  /**
   * @brief Velocity Control I gain
   */
  Eigen::Vector3d velocity_i_gain;

  /**
   * @brief Velocity Control D gain
   */
  Eigen::Vector3d velocity_d_gain;

  /**
   * @brief Velocity integral limits
   */
  Eigen::Vector3d velocity_i_limits;

  /**
   * @brief max allowed tilt angle of the rocket to
   * track the position setpoint
   */
  double max_angle;
};

inline std::ostream& operator<<(std::ostream& os, const rocket_high_level_pid_controller_config& controller_config)
{
  os << "rocket_altitude_pid_controller_config:" << std::endl;
  os << " position_p_gain: " << std::to_string(controller_config.position_p_gain.x()) << " "
     << std::to_string(controller_config.position_p_gain.y()) << " "
     << std::to_string(controller_config.position_p_gain.z()) << std::endl;
  os << " velocity_p_gain: " << std::to_string(controller_config.velocity_p_gain.x()) << " "
     << std::to_string(controller_config.velocity_p_gain.y()) << " "
     << std::to_string(controller_config.velocity_p_gain.z()) << std::endl;
  os << " velocity_i_gain: " << std::to_string(controller_config.velocity_i_gain.x()) << " "
     << std::to_string(controller_config.velocity_i_gain.y()) << " "
     << std::to_string(controller_config.velocity_i_gain.z()) << std::endl;
  os << " velocity_d_gain: " << std::to_string(controller_config.velocity_d_gain.x()) << " "
     << std::to_string(controller_config.velocity_d_gain.y()) << " "
     << std::to_string(controller_config.velocity_d_gain.z()) << std::endl;
  os << " velocity_i_limits: " << std::to_string(controller_config.velocity_i_limits.x()) << " "
     << std::to_string(controller_config.velocity_i_limits.y()) << " "
     << std::to_string(controller_config.velocity_i_limits.z()) << std::endl;
  os << " max_angle: " << std::to_string(controller_config.max_angle) << std::endl;
  os << " loop_rate: " << std::to_string(controller_config.loop_rate) << std::endl;

  return os;
}

}  // namespace crs_controls
#endif /* ROCKET_PID_ROCKET_PID_CONTROLLER_CONFIG */
