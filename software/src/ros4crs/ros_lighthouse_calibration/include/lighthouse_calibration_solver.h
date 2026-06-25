#pragma once

#include <stdexcept>
#include <optional>
#include <vector>
#include <map>

#include <Eigen/Geometry>

#include "lighthouse_calibration.h"

/**
 * @file    lighthouse_calibration_solver.h
 * @brief   An interface to the solver for finding the Lighthouse calibration.
 */

namespace ros_lighthouse
{

class LighthouseCalibrationSolver
{
public:
  /**
   * @brief Constructor.
   * @param print_debug_info  Whether to print debug information of the solver.
   */
  LighthouseCalibrationSolver(bool print_debug_info = false) : print_debug_information_(print_debug_info){};

  /**
   * @brief   Solve for the Lighthouse calibration.
   * @return  True if the calibration was found, false otherwise.
   * @throws  std::argument_error If the number of points in angles and
   *          positions don't match.
   */
  bool solve(const BaseStation& base_station, const CalibrationDataset& calibration_data,
             const LighthousePose& initial_condition);

  /**
   * @brief Get the base station's rotation.
   *
   * @return std::optional<Eigen::Matrix3d> The base station's rotation
   *         if the calibration was found.
   */
  std::optional<Eigen::Quaterniond> getBaseStationOrientation() const
  {
    return orientation_;
  };

  /**
   * @brief Get the base station's position.
   *
   * @return std::optional<Eigen::Vector3d> The base station's position
   *         if the calibration was found.
   */
  std::optional<Eigen::Vector3d> getBasestationPosition() const
  {
    return position_;
  }

private:
  bool print_debug_information_;
  std::optional<Eigen::Quaterniond> orientation_ = {};
  std::optional<Eigen::Vector3d> position_ = {};
};

}  // namespace ros_lighthouse
