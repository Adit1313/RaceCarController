#ifndef SRC_CRS_COMMONS_INCLUDE_CURVLINEAR_STATIC_TRACK_TRAJECTORY
#define SRC_CRS_COMMONS_INCLUDE_CURVLINEAR_STATIC_TRACK_TRAJECTORY

#include <Eigen/Core>
#include "commons/static_track_trajectory.h"

namespace crs_controls
{
/**
 * @brief Struct with the curvilinear coordinates of the car
 *
 */
struct curvilinear_coord
{
  /**
   * @brief Curvilinear coordinate s. Unwrapped distance along the track centerline.
   *
   */
  double s;

  /**
   * @brief Curvilinear coordinate d. Lateral distance from the track centerline. Positive if the car is on the left
   * side of the track.
   *
   */
  double d;

  /**
   * @brief Curvilinear coordinate mu. Difference between the yaw angle of the track and the yaw angle of the vehicle.
   * Positive if yaw > track_yaw.
   *
   */
  double mu;
};

/**
 * @brief Struct with the cartesian coordinates of the car
 *
 */
struct cartesian_coord
{
  /**
   * @brief Cartesian coordinate x.
   *
   */
  double x;

  /**
   * @brief Cartesian coordinate y.
   *
   */
  double y;

  /**
   * @brief Cartesian coordinate yaw.
   *
   */
  double yaw;
};

class CurvilinearTrackTrajectory : public StaticTrackTrajectory
{
public:
  /**
   * @brief Construct a new Track ManagerTrajectory
   *
   * @param x_coord x_coordinates of the track
   * @param y_coord y_coordinates of the track
   * @param x_rate x_rate of the track
   * @param y_rate y_rate of the track
   * @param width width of the track
   * @param curvature curvature of the track
   * @param tangent_angle tangent angle of the track
   * @param arc_length arc langth of the track
   * @param density density of the track centerline points
   */
  CurvilinearTrackTrajectory(std::vector<double> x_coord, std::vector<double> y_coord, std::vector<double> x_rate,
                             std::vector<double> y_rate, double width, std::vector<double> curvature,
                             std::vector<double> tangent_angle, std::vector<double> arc_length, double density)
    : StaticTrackTrajectory(x_coord, y_coord, x_rate, y_rate, width, curvature, tangent_angle, arc_length, density){};

  /**
   * @brief Construct a new Track Manager
   *
   * @param x_coord x_coordinates of the track
   * @param y_coord y_coordinates of the track
   * @param x_rate x_rate of the track
   * @param y_rate y_rate of the track
   * @param width width of the track
   */
  CurvilinearTrackTrajectory(std::vector<double> x_coord, std::vector<double> y_coord, std::vector<double> x_rate,
                             std::vector<double> y_rate, double width)
    : StaticTrackTrajectory(x_coord, y_coord, x_rate, y_rate, width){};

  /**
   * @brief Construct a new Track Manager
   *
   * @param x_coord x_coordinates of the track
   * @param y_coord y_coordinates of the track
   */
  CurvilinearTrackTrajectory(std::vector<double> x_coord, std::vector<double> y_coord)
    : StaticTrackTrajectory(x_coord, y_coord){};

  /**
   * @brief Construct a new Track Manager object
   *
   * @param points
   */
  CurvilinearTrackTrajectory(const std::vector<Eigen::Vector2d>& points) : StaticTrackTrajectory(points){};

  /**
   * Returns the curvilinear coordinates of a query point with respect to the track.
   *
   * @param query_point The query point consisting of x and y coordinates.
   * @param yaw_angle The yaw angle of the vehicle.
   * @param s_complete The length of the completed laps so far.
   * @param precision The precision of the binary search.
   *
   * @return curvilinear_coord
   *
   */
  curvilinear_coord getClosestCurvilinearCoordinate(const Eigen::Vector2d& query_point, const double& yaw_angle);

  /**
   * Returns the cartesian coordinates of a curvilinear query point.
   *
   * @param query_point The query point consisting of curvilinear coordinates.
   *
   * @return cartesian_coord
   *
   */
  cartesian_coord getClosestCartCoordinate(const curvilinear_coord& query_point);
};

}  // namespace crs_controls

#endif /* SRC_CRS_COMMONS_INCLUDE_CURVLINEAR_STATIC_TRACK_TRAJECTORY */
