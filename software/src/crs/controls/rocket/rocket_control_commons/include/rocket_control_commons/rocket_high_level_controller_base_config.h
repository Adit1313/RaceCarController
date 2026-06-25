#ifndef ROCKET_CONTROL_COMMONS_ROCKET_HIGH_LEVEL_CONTROLLER_BASE_CONFIG
#define ROCKET_CONTROL_COMMONS_ROCKET_HIGH_LEVEL_CONTROLLER_BASE_CONFIG

#include <vector>
#include <Eigen/Core>

namespace crs_controls
{
struct rocket_high_level_controller_base_config
{
  /**
   * @brief Loop rate of the controller
   */
  float loop_rate;
};

inline std::ostream& operator<<(std::ostream& os, const rocket_high_level_controller_base_config& controller_config)
{
  os << "rocket_high_level_controller_base_config:" << std::endl;
  os << " loop rate: " << std::to_string(controller_config.loop_rate) << std::endl;

  return os;
}

}  // namespace crs_controls
#endif /* ROCKET_CONTROL_COMMONS_ROCKET_HIGH_LEVEL_CONTROLLER_BASE_CONFIG */
