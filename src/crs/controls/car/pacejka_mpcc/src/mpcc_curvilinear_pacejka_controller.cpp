/**
 * Definitions of the curvilinear specific template specializations.
 *
 */

#include "pacejka_mpcc/impl/mpcc_curvilinear_pacejka_controller.h"

#include "commons/curvilinear_track_trajectory.h"

namespace crs_controls::pacejka_mpcc
{

template <>
typename PacejkaCurvilinearMpccController::MpcParameters
PacejkaCurvilinearMpccController::generateCurrentSolverParameters()
{
  std::shared_ptr<crs_controls::CurvilinearTrackTrajectory> track =
      getTrajectory<crs_controls::CurvilinearTrackTrajectory>();

  MpcParameters parameters;

  for (int stage = 0; stage < solver_.N; stage++)
  {
    double distance_on_track =
        std::fmod(last_solution_.x[stage][static_cast<int>(pacejka_vars::THETA)], track->getMaxArcLength());
    int reference_track_index = distance_on_track * track->getDensity();

    last_reference_on_track_[stage] = {
      track->operator[](reference_track_index).x(),
      track->operator[](reference_track_index).y(),
      track->getTrackAngle(reference_track_index),
    };

    crs_controls::pacejka_mpcc::solvers::TrajectoryReferenceCurvilinear reference;
    reference.kappa = track->getCurvature(reference_track_index);

    parameters[stage] = { model_->getParams(),
                          { config_.Q1, config_.Q2, config_.R1, config_.R2, config_.R3, config_.q },
                          reference };
  }
  return parameters;
}

template <>
typename PacejkaCurvilinearMpccController::StateArray PacejkaCurvilinearMpccController::convertToLocalCoordinates(
    const crs_models::pacejka_model::pacejka_car_state& state,
    const crs_models::pacejka_model::pacejka_car_input& input [[maybe_unused]], double theta [[maybe_unused]])
{
  std::shared_ptr<crs_controls::CurvilinearTrackTrajectory> track =
      getTrajectory<crs_controls::CurvilinearTrackTrajectory>();
  Eigen::Vector2d query_point(state.pos_x, state.pos_y);
  curvilinear_coord coord = track->getClosestCurvilinearCoordinate(query_point, state.yaw);

  return { laps_ * track->getMaxArcLength() + coord.s,
           coord.d,
           coord.mu,
           state.vel_x,
           state.vel_y,
           state.yaw_rate,
           last_input_.torque,
           last_input_.steer };
}

template <>
crs_models::pacejka_model::pacejka_car_state PacejkaCurvilinearMpccController::convertToGlobalCoordinates(
    const PacejkaCurvilinearMpccController::StateArray& local_state)
{
  curvilinear_coord curv_coord = {
    local_state[static_cast<int>(pacejka_vars::THETA)],
    local_state[static_cast<int>(pacejka_vars::D)],
    local_state[static_cast<int>(pacejka_vars::MU)],
  };

  double x_vel = local_state[static_cast<int>(pacejka_vars::VX)];
  double y_vel = local_state[static_cast<int>(pacejka_vars::VY)];
  double dyaw = local_state[static_cast<int>(pacejka_vars::DYAW)];

  // Convert curvilinear coordinates to cartesian
  cartesian_coord cart_coord =
      getTrajectory<crs_controls::CurvilinearTrackTrajectory>()->getClosestCartCoordinate(curv_coord);

  // NOTE(@naefjo): standard MPCC is: x, y, yaw, vx, vy, dyaw.
  return { cart_coord.x, cart_coord.y, cart_coord.yaw, x_vel, y_vel, dyaw };
}

template <>
PacejkaCurvilinearMpccController::StateArray
PacejkaCurvilinearMpccController::generateInitialization(const int reference_track_index, const double current_velocity)
{
  auto track = getStaticTrack();
  return {
    track->getArcLength(reference_track_index),
    0.0,
    0.0,
    current_velocity,                                               // v_x
    0.0,                                                            // v_y
    track->getCurvature(reference_track_index) * current_velocity,  // dyaw
  };
}

}  // namespace crs_controls::pacejka_mpcc
