#ifndef TRAJECTORY
#define TRAJECTORY

#include <Eigen/Core>
#include <vector>

#include "commons/base_trajectory.h"

namespace crs_controls
{

class Trajectory : public BaseTrajectory<Eigen::Vector2d>
{
protected:
  // edges of Voronoi Distribution
  std::vector<double> voronoi_edges_x_;
  std::vector<double> voronoi_edges_y_;

public:
  Trajectory() : BaseTrajectory(){};

  /**
   * @brief Construct a new Trajectory
   *
   * @param trajectory_coordinates is a std::vector of Eigen::Vector2d containing the x and
   * y coordinates of the trajectory
   */
  Trajectory(const std::vector<Eigen::Vector2d>& trajectory_coordinates)
    : BaseTrajectory<Eigen::Vector2d>(trajectory_coordinates){};

  /**
   * @brief Construct a new Trajectory
   *
   * @param x_coord x_coordinates of the trajectory
   * @param y_coord y_coordinates of the trajectory
   */
  Trajectory(std::vector<double> x_coord, std::vector<double> y_coord)
  {
    for (size_t i = 0; i < x_coord.size(); ++i)
    {
      trajectory_coordinates_.push_back(Eigen::Vector2d(x_coord[i], y_coord[i]));
    }
  };

  /**
   * @brief Get the Closest Track Point Idx object
   *
   * @param query_point
   * @param precision
   * @return int
   */
  int getClosestTrackPointIdx(const Eigen::Vector2d& query_point, int precision = 16);

  /**
   * @brief Get the trajectory point that is closest to the given query point.
   * @note This function uses a binary search internally (first probing for the smallest distance using a step size of
   * "precision").
   *
   * @param query_point position to query closest trajectory point
   * @param precision probing step size for binary search
   * @return Eigen::Vector2d
   */
  Eigen::Vector2d getClosestTrackPoint(const Eigen::Vector2d& query_point, int precision = 16);

  /**
   * @brief Returnes the last closest point of the trajectory that was previously requested.
   *
   * @return const Eigen::Vector2d
   */
  const Eigen::Vector2d getLastRequestedTrackPoint() const;

  /**
   * @brief Get the Last Requested Track Angle (only for visualization)
   *
   * @return const double
   */
  virtual double getLastRequestedTrackAngle() const = 0;

  /**
   * @brief Get the Voronoi Edges (only for visualization). Leave empty if unused.
   *
   * @return std::vetcor<double>
   */
  std::vector<double> getVorEdgesX();
  std::vector<double> getVorEdgesY();
};

}  // namespace crs_controls

#endif /* TRAJECTORY */
