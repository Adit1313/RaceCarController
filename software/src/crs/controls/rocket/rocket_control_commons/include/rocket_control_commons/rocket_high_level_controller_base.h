#ifndef ROCKET_CONTROL_COMMONS_ROCKET_HIGH_LEVEL_CONTROLLER_BASE
#define ROCKET_CONTROL_COMMONS_ROCKET_HIGH_LEVEL_CONTROLLER_BASE

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>

#include "rocket_6_dof_model/rocket_6_dof_state.h"
#include "rocket_6_dof_model/rocket_6_dof_params.h"

#include "rocket_control_commons/rocket_high_level_controller_base_config.h"

namespace crs_controls
{

struct rocket_high_level_controller_control_input
{
  /**
   * @brief The desired rocket state which is produced by the high level controller.
   */
  Eigen::Quaterniond attitude_setpoint;

  /**
   * @brief The desired rocket input which is produced by the high level controller.
   */
  double thrust_setpoint_x;
};

template <typename configType = rocket_high_level_controller_base_config>
class RocketHighLevelControllerBase
{
protected:
  configType config_;
  std::shared_ptr<crs_models::rocket_6_dof_model::rocket_6_dof_params> model_;

  std::vector<double> debug_states_;

  // For safety reasons, the rocket needs to receive an arm signal before the actuators spin up.
  // This flag can be used to set default setpoints in `getControlInput` while the rocket is not armed.
  bool is_armed_ = false;
  // This flag can be used to prevent integrator windup, etc. while the rocket is not in fully autonomous mode.
  bool is_in_autonomous_mode_ = false;

public:
  /**
   * @brief Construct a new Base Allocator object.
   */
  RocketHighLevelControllerBase(configType config,
                                std::shared_ptr<crs_models::rocket_6_dof_model::rocket_6_dof_params> model)
    : config_(config), model_(model), debug_states_(0)
  {
    std::cout << "High level controller config:\n" << config_ << std::endl;
  };

  /**
   * @brief Compute the high level control commands to control the rocket.
   *
   * @param current_state is the current system state of the rocket.
   * @param desired_state is the desired system state of the rocket.
   * @return rocket_high_level_controller_control_input consisting of desired attitude and thrust
   * NOTE(@naefjo): Currently, we only accept a single desired state and not a trajectory of states.
   * This interface is subject to change once the trajectory classes have been implemented.
   */
  virtual rocket_high_level_controller_control_input
  getControlInput(const crs_models::rocket_6_dof_model::rocket_6_dof_state& current_state,
                  const crs_models::rocket_6_dof_model::rocket_6_dof_state& desired_state) = 0;

  /**
   * @brief Returns wether the controller is initialized
   *
   * @return true if controller is already initialized
   * @return false if controller is not initialized
   */
  virtual bool isInitializing() = 0;

  /**
   * @brief Returns the loop rate of the controller.
   *
   *  @return float which specifies the loop rate of the controller in Hz
   */
  float getLoopRate()
  {
    return config_.loop_rate;
  }

  configType& getConfig()
  {
    return config_;
  }

  void setConfig(configType config)
  {
    config_ = config;
  }

  /**
   * @brief Debug function which can be used to return some useful debug info from the controller
   *
   * @return const std::vector<double> containing relevant debug info about the controller
   */
  const std::vector<double> getDebugControllerState() const
  {
    return debug_states_;
  }

  /**
   * @brief Set internal state variables of the controller
   *
   * @param internal_state internal state parameters to be set in the controller.
   * takes the format [is_armed, is_in_autonomous_mode]
   */
  void setInternalControllerState(const std::vector<bool>& internal_state)
  {
    is_armed_ = internal_state[0];
    is_in_autonomous_mode_ = internal_state[1];
  }
};

}  // namespace crs_controls

#endif /* ROCKET_CONTROL_COMMONS_ROCKET_HIGH_LEVEL_CONTROLLER_BASE */
