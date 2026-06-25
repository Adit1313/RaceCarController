#pragma once

#include <vector>

#include "commons/trajectory.h"
#include "commons/static_track_trajectory.h"
#include "pacejka_model/pacejka_discrete.h"
#include "pacejka_model/pacejka_car_input.h"
#include "pacejka_model/pacejka_car_state.h"
#include "pacejka_model/pacejka_params.h"
#include "safety_framework/mpc_based_safety_filter.h"

#include "pacejka_mpc_safety_filter/config.h"
#include "pacejka_mpc_safety_filter/solvers/acados_pacejka_safety_model_solver.h"

namespace crs_safety::pacejka_mpc_safety_filter
{

class PacejkaMpcSafetyFilter
  : public MpcBasedSafetyFilter<crs_models::pacejka_model::pacejka_car_state,
                                crs_models::pacejka_model::pacejka_car_input,
                                crs_models::pacejka_model::DiscretePacejkaModel, solvers::AcadosPacejkaSafetySolver>
{
public:
  PacejkaMpcSafetyFilter(pacejka_mpc_safety_config config,
                         std::shared_ptr<crs_controls::StaticTrackTrajectory> trajectory,
                         std::shared_ptr<crs_models::pacejka_model::DiscretePacejkaModel> model);

  /**
   * @brief Get the Safe Control Input
   *
   * @param state
   * @param control_input
   * @return crs_models::pacejka_model::pacejka_car_input
   */
  crs_models::pacejka_model::pacejka_car_input
  getSafeControlInput(const crs_models::pacejka_model::pacejka_car_state state,
                      const crs_models::pacejka_model::pacejka_car_input control_input) override;

  /**
   * @brief reference trajectory and planned mpc trajectory for visualization
   *
   */
  std::vector<std::vector<double>> planned_trajectory;  // Contains reference trajectory

  /**
   * @brief If true, the last input was overridden by the safety filter
   *
   */
  bool input_overridden = false;

private:
  /**
   * @brief Centerline of track
   *
   */
  std::shared_ptr<crs_controls::StaticTrackTrajectory> trajectory_;
  /**
   * @brief Configuration parameters
   *
   */
  pacejka_mpc_safety_config config_;
  /**
   * @brief Last distance that was expected to be driven using the current input torque and mpc horizion
   *
   */
  double prev_dist_target_ = 0;

  /**
   * @brief Returns reference on the centerline for the current state and input of the car
   *
   * @param state
   * @param input
   * @return std::vector<std::pair<int, crs_models::pacejka_model::pacejka_car_state>>
   */
  std::vector<std::pair<int, crs_models::pacejka_model::pacejka_car_state>>
  calculateReferenceTrajectory(const crs_models::pacejka_model::pacejka_car_state state,
                               const crs_models::pacejka_model::pacejka_car_input input);
};

}  // namespace crs_safety::pacejka_mpc_safety_filter
