#ifndef ROCKET_CONTROL_COMMONS_ROCKET_CONTROLLER
#define ROCKET_CONTROL_COMMONS_ROCKET_CONTROLLER

#include <boost/atomic.hpp>
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <boost/thread.hpp>

#include "commons/base_trajectory.h"
#include "control_commons/model_based_controller.h"
#include "rocket_6_dof_model/rocket_6_dof_state.h"
#include "rocket_6_dof_model/rocket_6_dof_input.h"
#include "rocket_6_dof_model/rocket_6_dof_params.h"

#include "rocket_control_commons/rocket_6_dof_allocation.h"
#include "rocket_control_commons/rocket_attitude_controller.h"
#include "rocket_control_commons/rocket_high_level_controller_base.h"

namespace crs_controls
{

template <typename highLevelControllerType, typename lowLevelControllerType, typename allocatorType>
struct rocket_controller_config;

template <typename highLevelControllerType, typename lowLevelControllerType, typename allocatorType>
class RocketController
  : public ModelBasedController<crs_models::rocket_6_dof_model::rocket_6_dof_params,
                                crs_models::rocket_6_dof_model::rocket_6_dof_state,
                                crs_models::rocket_6_dof_model::rocket_6_dof_input, ThreeDofPositionTrajectory>
{
private:
  std::shared_ptr<highLevelControllerType> high_level_controller_;
  std::shared_ptr<lowLevelControllerType> low_level_controller_;
  std::shared_ptr<allocatorType> control_allocation_;

  boost::atomic<crs_models::rocket_6_dof_model::rocket_6_dof_state> current_rocket_state_;
  boost::atomic<rocket_high_level_controller_control_input> high_level_controller_setpoint_;
  boost::asio::io_context io_;
  boost::asio::steady_timer t_;
  bool is_initial_run_;

public:
  /**
   * @brief Constructor for the RocketController class.
   *
   * @param config the config struct for the high and low level controller as well as the allocation
   * @param model shared pointer to the rocket model
   * @param trajectory shared pointer to the rocket trajectory
   */
  RocketController(rocket_controller_config<highLevelControllerType, lowLevelControllerType, allocatorType> config,
                   std::shared_ptr<crs_models::rocket_6_dof_model::rocket_6_dof_params> model,
                   std::shared_ptr<ThreeDofPositionTrajectory> trajectory)
    : ModelBasedController<crs_models::rocket_6_dof_model::rocket_6_dof_params,
                           crs_models::rocket_6_dof_model::rocket_6_dof_state,
                           crs_models::rocket_6_dof_model::rocket_6_dof_input, ThreeDofPositionTrajectory>(model,
                                                                                                           trajectory)
    , t_(io_)
    , is_initial_run_(true)
  {
    // Initialize controllers
    setControllers(config);

    high_level_controller_setpoint_.store(
        { Eigen::Quaterniond::Identity(), this->model_->mass * this->model_->gravity_constant });

    std::cout << "[RocketController] Successfully instantiated rocket controller" << std::endl;
  }

  /**
   * @brief Instantiates the rocket controller and allocation pointers based on the provided config.
   *
   * @param config containting the relevant controller and allocation configurations
   */
  void setControllers(rocket_controller_config<highLevelControllerType, lowLevelControllerType, allocatorType> config)
  {
    high_level_controller_ =
        std::make_shared<highLevelControllerType>(config.high_level_controller_config, this->model_);
    low_level_controller_ = std::make_shared<lowLevelControllerType>(config.low_level_controller_config);
    control_allocation_ = std::make_shared<allocatorType>(this->model_, config.allocation_config);
  }

  /**
   * @brief Initialize the high level controller callback
   * NOTE(@naefjo): jump start the timer by the callback manually.
   * io_context::run is a blocking function which starts the event processing loop responsible for execution the
   * timer callbacks, thus we start it in a different thread.
   */
  void initializeHighLevelController()
  {
    t_.expires_after(boost::asio::chrono::milliseconds(static_cast<long int>(0)));
    t_.wait();
    highLevelControllerCallback();
    boost::thread(boost::bind(&boost::asio::io_context::run, &io_));
  }

  /**
   * @brief executes the low level control loop and allocation of the rocket
   *
   * @param state current measured state of the system
   * @param timestamp current timestamp, will be ignored
   * @return crs_models::rocket_6_dof_model::rocket_6_dof_input
   */
  crs_models::rocket_6_dof_model::rocket_6_dof_input getControlInput(
      crs_models::rocket_6_dof_model::rocket_6_dof_state state, double timestamp [[maybe_unused]] = 0) override
  {
    current_rocket_state_.store(state);

    if (is_initial_run_)
    {
      is_initial_run_ = false;
      initializeHighLevelController();
    }

    rocket_high_level_controller_control_input desired_rocket_state_local = high_level_controller_setpoint_.load();

    Eigen::Vector3d desired_torque =
        low_level_controller_->getControlInput(state, desired_rocket_state_local.attitude_setpoint);

    Eigen::Vector3d desired_force(desired_rocket_state_local.thrust_setpoint_x, 0.0, 0.0);

    return control_allocation_->getControlInput(desired_force, desired_torque);
  }

  /**
   * @brief executes the high level controller
   *
   * NOTE(@naefjo): based on this tutorial.
   * https://www.boost.org/doc/libs/1_81_0/doc/html/boost_asio/tutorial/tuttimer4/src.html
   */
  void highLevelControllerCallback()
  {
    crs_models::rocket_6_dof_model::rocket_6_dof_state current_rocket_state = current_rocket_state_.load();
    Eigen::Vector3d trajectory_setpoint = this->trajectory_->getClosestTrajectoryPoint(Eigen::Vector3d(
        current_rocket_state.position_x, current_rocket_state.position_y, current_rocket_state.position_z));

    crs_models::rocket_6_dof_model::rocket_6_dof_state desired_rocket_state;
    desired_rocket_state.position_x = trajectory_setpoint.x();
    desired_rocket_state.position_y = trajectory_setpoint.y();
    desired_rocket_state.position_z = trajectory_setpoint.z();

    rocket_high_level_controller_control_input hl_desired_inputs =
        high_level_controller_->getControlInput(current_rocket_state, desired_rocket_state);

    high_level_controller_setpoint_.store(hl_desired_inputs);

    float loop_rate = high_level_controller_->getLoopRate();

    // NOTE(@naefjo): register the callback with the timer to again expire in 1/looprate seconds.
    t_.expires_at(t_.expiry() + boost::asio::chrono::milliseconds(static_cast<long int>(1000.0 / loop_rate)));
    t_.async_wait(boost::bind(&RocketController::highLevelControllerCallback, this));
  }

  /**
   *  Sets the internal config
   *  Allows e.g. to update PID gains during runtime, or change target velocity.
   *
   * @param config is the config struct containing the configs for the high-level and low-level controller and
   * the allocation respectively.
   */
  void setConfig(rocket_controller_config<highLevelControllerType, lowLevelControllerType, allocatorType> config)
  {
    high_level_controller_->setConfig(config.high_level_controller_config);
    low_level_controller_->setConfig(config.low_level_controller_config);
  }

  rocket_controller_config<highLevelControllerType, lowLevelControllerType, allocatorType> getConfig()
  {
    return { high_level_controller_->getConfig(), low_level_controller_->getConfig(),
             control_allocation_->getConfig() };
  }

  bool isInitializing()
  {
    return false;
  }

  /**
   * @brief Debug function which can be used to return some useful debug info from the controller
   *
   * @return const std::vector<double> containing relevant debug info about the controller
   */
  const std::vector<double> getDebugControllerState() const override
  {
    std::vector<double> debug_states_ll = low_level_controller_->getDebugControllerState();
    std::vector<double> debug_states_hl = high_level_controller_->getDebugControllerState();
    std::vector<double> debug_states_alloc = control_allocation_->getDebugControllerState();
    debug_states_hl.insert(debug_states_hl.end(), debug_states_ll.begin(), debug_states_ll.end());
    debug_states_hl.insert(debug_states_hl.end(), debug_states_alloc.begin(), debug_states_alloc.end());
    return debug_states_hl;
  }

  /**
   * @brief Set internal state variables of the controller
   *
   * @param internal_state internal state parameters to be set in the controller.
   * takes the format [is_armed, is_in_autonomous_mode]
   */
  void setInternalControllerState(const std::vector<bool>& internal_state) override
  {
    high_level_controller_->setInternalControllerState(internal_state);
    low_level_controller_->setInternalControllerState(internal_state);
  }
};

}  // namespace crs_controls
#endif /* ROCKET_CONTROL_COMMONS_ROCKET_CONTROLLER */
