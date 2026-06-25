#ifndef ROCKET_CONTROL_COMMONS_ROCKET_ATTITUDE_CONTROLLER_CONFIG
#define ROCKET_CONTROL_COMMONS_ROCKET_ATTITUDE_CONTROLLER_CONFIG

#include <vector>
#include <Eigen/Core>

namespace crs_controls
{
struct rocket_attitude_controller_config
{
  /**
   * @brief Attitude Control P gain
   */
  Eigen::Vector3d attitude_p_gain;

  /**
   * @brief Attitude Rate Control P gain
   */
  Eigen::Vector3d rate_p_gain;

  /**
   * @brief Attitude Rate Control I gain
   */
  Eigen::Vector3d rate_i_gain;

  /**
   * @brief Attitude Rate Control D gain
   */
  Eigen::Vector3d rate_d_gain;

  /**
   * @brief Rate integrator limits
   */
  Eigen::Vector3d rate_i_limits;

  /**
   * @brief Loop rate of the controller
   */
  double loop_rate;

  /**
   * @brief numerator coefficients of the derivative filter
   */
  std::vector<double> derivative_filter_b;

  /**
   * @brief denominator coefficients of the derivative filter
   */
  std::vector<double> derivative_filter_a;
};

inline std::ostream& operator<<(std::ostream& os, const rocket_attitude_controller_config& controller_config)
{
  os << "rocket_attitude_controller_config:" << std::endl;
  os << " attitude_p_gain: " << std::to_string(controller_config.attitude_p_gain.x()) << " "
     << std::to_string(controller_config.attitude_p_gain.y()) << " "
     << std::to_string(controller_config.attitude_p_gain.z()) << std::endl;
  os << " rate_p_gain: " << std::to_string(controller_config.rate_p_gain.x()) << " "
     << std::to_string(controller_config.rate_p_gain.y()) << " " << std::to_string(controller_config.rate_p_gain.z())
     << std::endl;
  os << " rate_i_gain: " << std::to_string(controller_config.rate_i_gain.x()) << " "
     << std::to_string(controller_config.rate_i_gain.y()) << " " << std::to_string(controller_config.rate_i_gain.z())
     << std::endl;
  os << " rate_d_gain: " << std::to_string(controller_config.rate_d_gain.x()) << " "
     << std::to_string(controller_config.rate_d_gain.y()) << " " << std::to_string(controller_config.rate_d_gain.z())
     << std::endl;
  os << " rate_i_limits: " << std::to_string(controller_config.rate_i_limits.x()) << " "
     << std::to_string(controller_config.rate_i_limits.y()) << " "
     << std::to_string(controller_config.rate_i_limits.z()) << std::endl;
  os << " loop rate: " << std::to_string(controller_config.loop_rate) << std::endl;

  os << " derivative filter numerator:\n  ";
  for (const double elem : controller_config.derivative_filter_b)
  {
    os << elem << " ";
  }
  os << std::endl;

  os << " derivative filter denominator:\n  ";
  for (const double elem : controller_config.derivative_filter_a)
  {
    os << elem << " ";
  }
  os << std::endl;
  return os;
}

}  // namespace crs_controls

#endif /* ROCKET_CONTROL_COMMONS_ROCKET_ATTITUDE_CONTROLLER_CONFIG */
