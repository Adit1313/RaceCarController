/**
 * @file    lighthouse_calibration_collector.h
 * @author  Tobias Bodewig, Lukas Vogel (vogellu@ethz.ch)
 * @brief   A state machine tool to interactively collect Lighthouse calibration data.
 */

#pragma once

#include <crs_msgs/lighthouse_sweep.h>
#include "lighthouse_calibration.h"
#include "ootx_decoder.h"

namespace ros_lighthouse
{
enum CalibrationState
{
  INIT,
  READ_CALIB_STREAM,
  WAIT_FOR_POSITION,
  COLLECT_DATA,
  FINISHED
};

// Used to keep track of average during data collection phase
struct AngleAverage
{
  double angle_0_sum_;
  double angle_1_sum_;
  uint angle_0_count_;
  uint angle_1_count_;
};

/**
 * @brief A class to collect Lighthouse calibration data interactively on the console.
 *
 * To use this object to collect the calibration data, instantiate it using a set of x-y coordinates
 * where measurements should be taken. Then, subscribe to the lighthouse_sweep topic
 * and make sure that @see lighthouseCallback is called for each new sweep. Call @see spinOnce in a loop.
 * Once @see isFinished returns true, the data collection is complete and the data can be retrieved using
 * @see getPointCoordinates and @see getAverageAngles.
 */
class LighthouseCalibrationCollector
{
public:
  /**
   * @brief Construct the collector and initialize the known x-y coordinates known where only the angles
   * should be collected.
   *
   * @param sweeps_per_point How many sweeps to collect per point.
   * @param points A vector of x-y coordinates of the points to collect data for. Will skip user input for number of
   * coordinates as well as the coordinates themselves.
   */
  LighthouseCalibrationCollector(unsigned int sweeps_per_point, TrackpointSet points,
                                 std::vector<BaseStation> base_stations)
    : sweeps_per_point_(sweeps_per_point), point_coordinates_(points)
  {
    if (points.size() < 6)
    {
      // Complain about too few points
      throw std::invalid_argument("At least 6 points are required for calibration.");
    }

    point_count_ = points.size();

    if (!base_stations.empty())
    {
      for (auto& base_station : base_stations)
      {
        collected_data_[base_station.id.value()] = {
          {
              .point_coordinates = points,
              .average_angles = {},
          },      // calibration dataset
          INIT,   // calibration state
          { 0 },  // ootx decoder state,
          { 0 },  // angle average
          0,      // current point
          base_station,
        };
      }
    }
    else
    {
      auto_detect_base_stations_ = true;
    }
  };

  /** Process a new Lighthouse Sweep coming in. */
  void lighthouseCallback(const crs_msgs::lighthouse_sweep& msg);

  bool isFinished() const
  {
    if (collected_data_.empty())
    {
      return false;
    }
    for (auto& [_, data] : collected_data_)
    {
      if (std::get<1>(data) != FINISHED)
      {
        return false;
      }
    }

    return true;
  }

  unsigned int getActivePointIndex()
  {
    return current_point_post_ + 1;
  }

  bool hasNewData()
  {
    if (new_data_)
    {
      new_data_ = false;
      return true;
    }
    return false;
  }

  /** Update the state machine. */
  void spinOnce();

  /**
   * @brief Get the collected measurements.
   * @note This function should only be called after @see isFinished returns true.
   */
  std::map<BaseStation, CalibrationDataset> getCalibrationMeasurements() const
  {
    if (!isFinished())
    {
      throw std::runtime_error("Cannot get calibration measurements before data collection is finished.");
    }

    std::map<BaseStation, CalibrationDataset> result;
    for (auto& [base_station_id, data] : collected_data_)
    {
      BaseStation base_station = std::get<5>(data);
      result[base_station] = std::get<0>(data);
      result[base_station].first = point_coordinates_;
    }

    return result;
  };

private:
  /**
   * @brief Retrieves a base station from an ID. If create is true, inserts it if not found.
   * @returns A reference to the base station, or an empty optional if not found.
   */
  std::optional<BaseStation::base_station_id> findBasestationById(BaseStation::base_station_id id, bool create = false);

  void setAllStates(CalibrationState state)
  {
    for (auto& [_, data] : collected_data_)
    {
      std::get<1>(data) = state;
    }
  }

  /** How many sweeps to collect per point to average the angles. */
  unsigned int sweeps_per_point_;
  int sweeps_to_wait_auto_detect_ = 2000;  // ~20s for 1 base station, ~5s for 4 base stations

  /* Collected data -------------------------------------------------------- */

  unsigned int current_point_post_ = 0;  ///< Point index that all base stations are past.
  bool new_data_ = false;

  /**
   * @brief A map of all the collected data.
   *
   * The map may be empty if base stations have yet to be detected.
   */
  std::map<BaseStation::base_station_id, std::tuple<CalibrationDataset, CalibrationState, ootxDecoderState_t,
                                                    AngleAverage, unsigned int, BaseStation>>
      collected_data_;
  bool auto_detect_base_stations_ = false;

  unsigned int point_count_;

  const TrackpointSet point_coordinates_;
};

}  // namespace ros_lighthouse
