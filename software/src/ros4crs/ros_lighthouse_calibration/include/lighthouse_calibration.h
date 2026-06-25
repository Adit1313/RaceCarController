/**
 * @file    lighthouse_calibration.h
 * @author  Lukas Vogel (vogellu@ethz.ch)
 * @brief   Data structures used for the Lighthouse calibration.
 */

#pragma once

#include <optional>
#include <vector>

namespace ros_lighthouse
{

/**
 * @brief Primary data structure to calibrate a Lighthouse base station.
 *
 * A base station is identified by its ID. It may have other associated
 * data that may or may not be known, including its tilt offsets and
 * a dataset of calibration measurements taken for this base station.
 *
 * Any fields that are not currently known are set to std::nullopt.
 */
struct BaseStation
{
  using base_station_id = int;

  std::optional<base_station_id> id;  ///< ID of the base station

  std::optional<double> dt1;  ///< Tilt offset for light plane #1 (rad)
  std::optional<double> dt2;  ///< Tilt offset for light plane #2 (rad)

  // Implement < operator
  bool operator<(const BaseStation& other) const
  {
    return id < other.id;
  }
};

/** A vector of track points in [m] in the frame of the track. */
using TrackpointSet = std::vector<std::pair<double, double>>;
/** A vector of angle measurements. */
using MeasuredAngleSet = std::vector<std::pair<double, double>>;
/** A set of track points and corresponding angles that can serve as a basis for calibrating a base station. */
using CalibrationDataset = std::pair<TrackpointSet, MeasuredAngleSet>;

/**
 * @brief A struct to hold the calibration (= position and rotation) for a Lighthouse station.
 */
struct LighthousePose
{
  /** Position of the Lighthouse station in the track frame (m). */
  std::array<double, 3> position;

  /** Rotation angles of the Lighthouse station w.r.t. the track frame (rad). */
  std::array<double, 3> angles;
};

}  // namespace ros_lighthouse
