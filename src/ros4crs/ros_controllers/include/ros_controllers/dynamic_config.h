#ifndef SRC_ROS_ROS_CONTROLLERS_INCLUDE_ROS_CONTROLLERS_DYNAMIC_CONFIG
#define SRC_ROS_ROS_CONTROLLERS_INCLUDE_ROS_CONTROLLERS_DYNAMIC_CONFIG

#ifdef pid_controller_FOUND
#include "ros_controllers/PIDConfig.h"
#include <pid_controller/pacejka_pid_controller.h>
#endif

#ifdef ff_fb_controller_FOUND
#include "ros_controllers/ff_fbConfig.h"
#include <ff_fb_controller/ff_fb_controller.h>
#endif

#ifdef PACEJKA_MPCC_FOUND
#include "ros_controllers/pacejka_mpccConfig.h"
#include <pacejka_mpcc/mpcc_pacejka_controller.h>
#include <pacejka_mpcc/mpcc_pacejka_config.h>
#endif

#ifdef rocket_position_pid_FOUND
#include "ros_controllers/rocket_high_level_pidConfig.h"
#include "ros_controllers/rocket_low_level_pidConfig.h"
#include <rocket_position_pid/rocket_controller_specializations.h>
#endif

#include <dynamic_reconfigure/server.h>

namespace ros_controllers
{
#ifdef pid_controller_FOUND
/**
 * @brief Class that creates a dynamic_callback parameter server and connects it with the underlying controller
 * Allows to tune a controller using a GUI interface
 *
 */
class DynamicPIDConfigServer
{
protected:
  std::shared_ptr<crs_controls::PacejkaPIDController> controller_;
  dynamic_reconfigure::Server<PIDConfig> server;
  dynamic_reconfigure::Server<PIDConfig>::CallbackType f;

public:
  DynamicPIDConfigServer(const ros::NodeHandle& nh, std::shared_ptr<crs_controls::PacejkaPIDController> controller);
  /**
   * @brief Callback that gets called from the dynamic reconfigure service
   *
   * @param config
   * @param level
   */
  void callback(PIDConfig& config, uint32_t level);
};
#endif

#ifdef ff_fb_controller_FOUND
/**
 * @brief Class that creates a dynamic_callback parameter server and connects it with the underlying controller
 * Allows to tune a controller using a GUI interface
 *
 */
class DynamicFfFbConfigServer
{
protected:
  std::shared_ptr<crs_controls::FfFbController> controller_;
  dynamic_reconfigure::Server<ff_fbConfig> server;
  dynamic_reconfigure::Server<ff_fbConfig>::CallbackType f;

public:
  DynamicFfFbConfigServer(const ros::NodeHandle& nh, std::shared_ptr<crs_controls::FfFbController> controller);
  /**
   * @brief Callback that gets called from the dynamic reconfigure service
   *
   * @param config
   * @param level
   */
  void callback(ff_fbConfig& config, uint32_t level);
};
#endif

#ifdef PACEJKA_MPCC_FOUND
/**
 * @brief Class that creates a dynamic_callback parameter server and connects it with the underlying controller
 * Allows to tune a controller using a GUI interface
 *
 */
template <typename SolverType>
class DynamicPacejkaMPCCConfigServer
{
protected:
  std::shared_ptr<crs_controls::pacejka_mpcc::PacejkaMpccController<SolverType>> controller_;
  dynamic_reconfigure::Server<pacejka_mpccConfig> server;
  dynamic_reconfigure::Server<pacejka_mpccConfig>::CallbackType f;

public:
  DynamicPacejkaMPCCConfigServer(
      const ros::NodeHandle& nh,
      std::shared_ptr<crs_controls::pacejka_mpcc::PacejkaMpccController<SolverType>> controller)
    : server(nh), controller_(controller)
  {
    f = boost::bind(&DynamicPacejkaMPCCConfigServer::callback, this, _1, _2);
    server.setCallback(f);
  };

  /**
   * @brief Callback that gets called from the dynamic reconfigure service
   *
   * @param config
   * @param level
   */
  void callback(ros_controllers::pacejka_mpccConfig& config, uint32_t level)
  {
    if (!controller_)
      return;

    crs_controls::pacejka_mpcc::mpcc_pacejka_config pid_cfg = controller_->getConfig();
    pid_cfg.Q1 = config.Q1;
    pid_cfg.Q2 = config.Q2;
    pid_cfg.R1 = config.R1;
    pid_cfg.R2 = config.R2;
    pid_cfg.R3 = config.R3;
    pid_cfg.q = config.q;
    pid_cfg.lag_compensation_time = config.lag_compensation_time;
    controller_->setConfig(pid_cfg);
  };
};
#endif

#ifdef rocket_position_pid_FOUND
/**
 * @brief Class that creates a dynamic_callback parameter server and connects it with the underlying controller
 * Allows to tune a controller using a GUI interface
 *
 */
class DynamicRocketPidConfigServer
{
protected:
  std::shared_ptr<crs_controls::Rocket6DofPidController> controller_;
  dynamic_reconfigure::Server<rocket_high_level_pidConfig> hl_server;
  dynamic_reconfigure::Server<rocket_low_level_pidConfig> ll_server;
  dynamic_reconfigure::Server<rocket_high_level_pidConfig>::CallbackType hl_f;
  dynamic_reconfigure::Server<rocket_low_level_pidConfig>::CallbackType ll_f;

public:
  DynamicRocketPidConfigServer(const ros::NodeHandle& nh,
                               std::shared_ptr<crs_controls::Rocket6DofPidController> controller);
  /**
   * @brief Callback that gets called from the dynamic reconfigure service
   *
   * @param config
   * @param level
   */
  void hl_callback(rocket_high_level_pidConfig& config, uint32_t level);

  /**
   * @brief Callback that gets called from the dynamic reconfigure service
   *
   * @param config
   * @param level
   */
  void ll_callback(rocket_low_level_pidConfig& config, uint32_t level);
};

#endif

}  // namespace ros_controllers
#endif /* SRC_ROS_ROS_CONTROLLERS_INCLUDE_ROS_CONTROLLERS_DYNAMIC_CONFIG */
