#pragma once

#include <string>

namespace crs_safety::pacejka_mpc_safety_filter
{
struct pacejka_mpc_safety_config
{
  double dist_targ_multiplier;
  double min_terminal_dist;
  double max_terminal_dist;

  bool use_torque_filter;
  double dist_decrement_max;

  std::string solver_type;

  double loookahead_time;

  std::string reference_method;

  double threshold;

  double cost_steer;
  double cost_torque;
  double cost_delta_torque;
  double cost_delta_steer;
};
}  // namespace crs_safety::pacejka_mpc_safety_filter
