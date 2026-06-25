#ifndef BASE_TRAJECTORY
#define BASE_TRAJECTORY
#include <cmath>
#include <type_traits>
#include <tuple>
#include <vector>
#include <iostream>

#include <Eigen/Core>

namespace crs_controls
{

template <typename TrajectoryStateType>
class BaseTrajectory
{
public:
  // This makes the underlying StateType of the Trajectory accessible to outside code
  using StateType = TrajectoryStateType;

  BaseTrajectory(){};

  /**
   * @brief Construct a new BaseTrajectory
   *
   * @param trajectory_coordinates is a std::vector of TrajectoryStateType containing the
   * coordinates of the trajectory
   */
  BaseTrajectory(const std::vector<TrajectoryStateType>& trajectory_coordinates)
    : trajectory_coordinates_(trajectory_coordinates){};

  /**
   * @brief Computes the norm for two points on the trajectory for the given vectorspace.
   * @note Some Caveats:
   *      - Might make sense to create distinct classes for the TrajectoryStateType which implement this
   *        as a static funciton as it is not inherently related to the trajectory itself.
   *      - For any non-Eigen types, this function needs to be explicitly specialized.
   *
   * @param state_1 a state in the TrajectoryStateType vectorspace
   * @param state_2 another state in the TrajectoryStateType vectorspace
   * @return float distance between the two states.
   */
  float trajectoryNorm(const TrajectoryStateType& state_1, const TrajectoryStateType& state_2)
  {
    return (state_1 - state_2).norm();
  }

  /**
   * @brief Returns the trajectory point at the given index.
   * @note this function performs index wrapping, allowing to pass indices that are negative or bigger than the
   * trajectory length
   *
   * @param i index to request
   * @return TrajectoryStateType the trajectory point at position i
   */
  TrajectoryStateType operator[](int i) const
  {
    // Handle roll over (-1 -> x_coords.size() - 1)
    while (i < 0)
      i += trajectory_coordinates_.size();

    // Handle roll over (x_coords.size() + 1 -> 1)
    while (i >= static_cast<int>(trajectory_coordinates_.size()))
      i -= trajectory_coordinates_.size();

    return trajectory_coordinates_[i];
  }

  /**
   * @brief Get the Closest Trajectory Point Idx object
   *
   * @param query_point
   * @param precision
   * @return int
   */
  int getClosestTrajectoryPointIdx(const TrajectoryStateType& query_point, int precision = 16)
  {
    int num_points = trajectory_coordinates_.size();
    if (num_points <= 1)
    {
      last_query_index_ = 0;
      return 0;
    }

    int best_length = 0;
    double best_distance = INFINITY;

    // rough search
    for (int scan_length = 0; scan_length < num_points; scan_length += precision)
    {
      double scan_distance = trajectoryNorm(query_point, trajectory_coordinates_[scan_length]);
      if (scan_distance < best_distance)
      {
        best_length = scan_length;
        best_distance = scan_distance;
      }
    }

    //  binary search
    int prev_length = 0;
    double prev_distance = INFINITY;
    int next_length = 0;
    double next_distance = INFINITY;

    for (precision /= 2; precision > 0; precision /= 2)
    {
      prev_length = best_length - precision;
      next_length = best_length + precision;
      if (prev_length >= 0)
      {
        prev_distance = trajectoryNorm(query_point, trajectory_coordinates_[prev_length]);
        if (prev_distance < best_distance)
        {
          best_length = prev_length;
          best_distance = prev_distance;
        }
      }
      if (next_length <= num_points)
      {
        next_distance = trajectoryNorm(query_point, trajectory_coordinates_[next_length]);
        if (next_distance < best_distance)
        {
          best_length = next_length;
          best_distance = next_distance;
        }
      }
    }

    last_query_index_ = best_length;
    return best_length;
  }

  /**
   * @brief Get the trajectory point that is closest to the given query point.
   * @note This function uses a binary search internally (first probing for the smallest distance using a step size of
   * "precision").
   *
   * @param query_point position to query closest trajectory point
   * @param precision probing step size for binary search
   * @return TrajectoryStateType corresponding to closest point on the trajectory
   */
  TrajectoryStateType getClosestTrajectoryPoint(const TrajectoryStateType& query_point, int precision = 16)
  {
    int point_idx = getClosestTrajectoryPointIdx(query_point, precision);

    return trajectory_coordinates_[point_idx];
  }

  /**
   * @brief Returnes the last closest point of the trajectory that was previously requested.
   *
   * @return const TrajectoryStateType
   */
  const TrajectoryStateType getLastRequestedTrajectoryPoint() const
  {
    return trajectory_coordinates_[last_query_index_];
  }

  void resetTrajectory(std::vector<TrajectoryStateType> new_trajectory)
  {
    trajectory_coordinates_ = new_trajectory;
    last_query_index_ = -1;
  }

protected:
  /**
   * @brief Coordinate vector of the trajectory
   */
  std::vector<TrajectoryStateType> trajectory_coordinates_;

  /**
   * @brief Last queried trajectory point
   */
  int last_query_index_ = 0;
};

typedef BaseTrajectory<Eigen::Vector3d> ThreeDofPositionTrajectory;

}  // namespace crs_controls

#endif /* BASE_TRAJECTORY*/
