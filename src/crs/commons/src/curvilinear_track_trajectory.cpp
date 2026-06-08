
#include "commons/curvilinear_track_trajectory.h"

namespace crs_controls
{
curvilinear_coord CurvilinearTrackTrajectory::getClosestCurvilinearCoordinate(const Eigen::Vector2d& query_point,
                                                                              const double& yaw_angle)
{
  // index of the closest point on the track
  const track_error error = getTrackError(query_point);

  // Convert error to curvilinear coordinates
  curvilinear_coord coords;
  coords.d = -error.side * error.lateral_error;

  // Converts track_idx to distance along the track
  coords.s = getArcLength(error.index);

  // Track yaw angle at the closest point, mapped to [-pi, pi]
  coords.mu = std::fmod(yaw_angle - getTrackAngle(error.index) + M_PI, 2 * M_PI) - M_PI;

  return coords;
}

cartesian_coord CurvilinearTrackTrajectory::getClosestCartCoordinate(const curvilinear_coord& query_point)
{
  // index of the closest point on the track
  const int reference_track_index = query_point.s * getDensity();

  // Get the position and yaw angle of the point on the track
  const Eigen::Vector2d pos_track = operator[](reference_track_index);
  double yaw_track = getTrackAngle(reference_track_index);

  // Calculate the position of the car
  cartesian_coord coords;
  coords.x = pos_track.x() - query_point.d * std::sin(yaw_track);
  coords.y = pos_track.y() + query_point.d * std::cos(yaw_track);
  coords.yaw = yaw_track + query_point.mu;

  return coords;
}
}  // namespace crs_controls
