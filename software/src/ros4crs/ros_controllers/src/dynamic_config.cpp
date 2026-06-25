#include "ros_controllers/dynamic_config.h"

namespace ros_controllers
{
#ifdef pid_controller_FOUND
DynamicPIDConfigServer::DynamicPIDConfigServer(const ros::NodeHandle& nh,
                                               std::shared_ptr<crs_controls::PacejkaPIDController> controller)
  : server(nh), controller_(controller)
{
  f = boost::bind(&DynamicPIDConfigServer::callback, this, _1, _2);
  server.setCallback(f);
};

void DynamicPIDConfigServer::callback(ros_controllers::PIDConfig& config, uint32_t level)
{
  if (!controller_)
    return;

  crs_controls::pid_config pid_cfg = controller_->getConfig();
  pid_cfg.Kd = config.Kd;
  pid_cfg.Kp = config.Kp;
  pid_cfg.Ki = config.Ki;
  pid_cfg.use_filter = config.use_filter;
  pid_cfg.target_velocity = config.target_velocity;
  pid_cfg.lag_compensation_time = config.lag_compensation_time;
  controller_->setConfig(pid_cfg);
};
#endif

#ifdef ff_fb_controller_FOUND
DynamicFfFbConfigServer::DynamicFfFbConfigServer(const ros::NodeHandle& nh,
                                                 std::shared_ptr<crs_controls::FfFbController> controller)
  : server(nh), controller_(controller)
{
  f = boost::bind(&DynamicFfFbConfigServer::callback, this, _1, _2);
  server.setCallback(f);
};

void DynamicFfFbConfigServer::callback(ros_controllers::ff_fbConfig& config, uint32_t level)
{
  if (!controller_)
    return;

  crs_controls::FfFbConfig pid_cfg = controller_->getConfig();
  pid_cfg.Kd = config.Kd;
  pid_cfg.Kp = config.Kp;
  pid_cfg.Ki = config.Ki;
  pid_cfg.target_velocity = config.target_velocity;
  pid_cfg.K_torque_curv = config.K_torque_curv;
  pid_cfg.lag_compensation_time = config.lag_compensation_time;
  pid_cfg.mean_curv_dist = config.mean_curv_dist;
  pid_cfg.use_filter = config.use_filter;
  controller_->setConfig(pid_cfg);
};
#endif

#ifdef rocket_position_pid_FOUND
DynamicRocketPidConfigServer::DynamicRocketPidConfigServer(
    const ros::NodeHandle& nh, std::shared_ptr<crs_controls::Rocket6DofPidController> controller)
  : hl_server(ros::NodeHandle(nh, "high_level_controller"))
  , ll_server(ros::NodeHandle(nh, "low_level_controller"))
  , controller_(controller)
{
  hl_f = boost::bind(&DynamicRocketPidConfigServer::hl_callback, this, _1, _2);
  ll_f = boost::bind(&DynamicRocketPidConfigServer::ll_callback, this, _1, _2);
  hl_server.setCallback(hl_f);
  ll_server.setCallback(ll_f);
};

void DynamicRocketPidConfigServer::hl_callback(ros_controllers::rocket_high_level_pidConfig& config, uint32_t level)
{
  if (!controller_)
    return;

  crs_controls::rocket_controller_config<crs_controls::RocketHighLevelPidController,
                                         crs_controls::RocketAttitudeController, crs_controls::Rocket6DofAllocation>
      rocket_pid_cfg = controller_->getConfig();

  rocket_pid_cfg.high_level_controller_config.position_p_gain.x() = config.position_p_gain_x;
  rocket_pid_cfg.high_level_controller_config.position_p_gain.y() = config.position_p_gain_y;
  rocket_pid_cfg.high_level_controller_config.position_p_gain.z() = config.position_p_gain_z;

  rocket_pid_cfg.high_level_controller_config.velocity_p_gain.x() = config.velocity_p_gain_x;
  rocket_pid_cfg.high_level_controller_config.velocity_p_gain.y() = config.velocity_p_gain_y;
  rocket_pid_cfg.high_level_controller_config.velocity_p_gain.z() = config.velocity_p_gain_z;

  rocket_pid_cfg.high_level_controller_config.velocity_i_gain.x() = config.velocity_i_gain_x;
  rocket_pid_cfg.high_level_controller_config.velocity_i_gain.y() = config.velocity_i_gain_y;
  rocket_pid_cfg.high_level_controller_config.velocity_i_gain.z() = config.velocity_i_gain_z;

  rocket_pid_cfg.high_level_controller_config.velocity_d_gain.x() = config.velocity_d_gain_x;
  rocket_pid_cfg.high_level_controller_config.velocity_d_gain.y() = config.velocity_d_gain_y;
  rocket_pid_cfg.high_level_controller_config.velocity_d_gain.z() = config.velocity_d_gain_z;

  rocket_pid_cfg.high_level_controller_config.velocity_i_limits.x() = config.velocity_i_limits_x;
  rocket_pid_cfg.high_level_controller_config.velocity_i_limits.y() = config.velocity_i_limits_y;
  rocket_pid_cfg.high_level_controller_config.velocity_i_limits.z() = config.velocity_i_limits_z;

  rocket_pid_cfg.high_level_controller_config.max_angle = config.max_angle;

  controller_->setConfig(rocket_pid_cfg);
};

void DynamicRocketPidConfigServer::ll_callback(ros_controllers::rocket_low_level_pidConfig& config, uint32_t level)
{
  if (!controller_)
    return;

  crs_controls::rocket_controller_config<crs_controls::RocketHighLevelPidController,
                                         crs_controls::RocketAttitudeController, crs_controls::Rocket6DofAllocation>
      rocket_pid_cfg = controller_->getConfig();

  rocket_pid_cfg.low_level_controller_config.attitude_p_gain.x() = config.attitude_p_gain_x;
  rocket_pid_cfg.low_level_controller_config.attitude_p_gain.y() = config.attitude_p_gain_y;
  rocket_pid_cfg.low_level_controller_config.attitude_p_gain.z() = config.attitude_p_gain_z;

  rocket_pid_cfg.low_level_controller_config.rate_p_gain.x() = config.rate_p_gain_x;
  rocket_pid_cfg.low_level_controller_config.rate_p_gain.y() = config.rate_p_gain_y;
  rocket_pid_cfg.low_level_controller_config.rate_p_gain.z() = config.rate_p_gain_z;

  rocket_pid_cfg.low_level_controller_config.rate_i_gain.x() = config.rate_i_gain_x;
  rocket_pid_cfg.low_level_controller_config.rate_i_gain.y() = config.rate_i_gain_y;
  rocket_pid_cfg.low_level_controller_config.rate_i_gain.z() = config.rate_i_gain_z;

  rocket_pid_cfg.low_level_controller_config.rate_d_gain.x() = config.rate_d_gain_x;
  rocket_pid_cfg.low_level_controller_config.rate_d_gain.y() = config.rate_d_gain_y;
  rocket_pid_cfg.low_level_controller_config.rate_d_gain.z() = config.rate_d_gain_z;

  rocket_pid_cfg.low_level_controller_config.rate_i_limits.x() = config.rate_i_limits_x;
  rocket_pid_cfg.low_level_controller_config.rate_i_limits.y() = config.rate_i_limits_y;
  rocket_pid_cfg.low_level_controller_config.rate_i_limits.z() = config.rate_i_limits_z;

  controller_->setConfig(rocket_pid_cfg);
};
#endif
}  // namespace ros_controllers
