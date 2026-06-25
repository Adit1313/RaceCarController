#include "commons/dynamic_point_trajectory.h"
#include <iostream>
#include <numeric>

namespace crs_controls
{
void DynamicPointTrajectory::resetTrajectory(std::vector<double> x_coord, std::vector<double> y_coord)
{
  assert((x_coord.size() == y_coord.size()) && "x_coord and y_coord differ in size");
  trajectory_coordinates_.clear();
  trajectory_coordinates_.reserve(x_coord.size());

  for (unsigned int i = 0; i < x_coord.size(); i++)
  {
    trajectory_coordinates_.push_back(Eigen::Vector2d(x_coord[i], y_coord[i]));
  }

  last_query_index_ = -1;
}

void DynamicPointTrajectory::resetTrajectory(std::vector<Eigen::Vector2d> pts)
{
  BaseTrajectory::resetTrajectory(pts);
}

void DynamicPointTrajectory::resetVorEdges(std::vector<double> x_edge, std::vector<double> y_edge)
{
  voronoi_edges_x_ = x_edge;
  voronoi_edges_y_ = y_edge;
}

double DynamicPointTrajectory::getLastRequestedTrackAngle() const
{
  return 0;
}
}  // namespace crs_controls
