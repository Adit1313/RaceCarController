/**
 * @file    lighthouse_calibration_collector.cpp
 * @author  Tobias Bodewig, Lukas Vogel (vogellu@ethzch)
 * @brief   A state_ machine tool to interactively collect Lighthouse calibration data.
 */

#include "lighthouse_calibration_collector.h"
#include "lighthouse_calibration.h"

namespace ros_lighthouse
{
void LighthouseCalibrationCollector::lighthouseCallback(const crs_msgs::lighthouse_sweep& msg)
{
  BaseStation::base_station_id id = msg.polynomial >> 1;
  if (id > 10)
    return;
  if (auto_detect_base_stations_)
  {
    sweeps_to_wait_auto_detect_--;
    if (sweeps_to_wait_auto_detect_ <= 0)
    {
      std::cout << "Stopped auto-detecting. Any base stations encountered after this will not be calibrated."
                << std::endl;
      setAllStates(READ_CALIB_STREAM);
      auto_detect_base_stations_ = false;
    }
  }

  if (auto base_station_id = findBasestationById(id, auto_detect_base_stations_))
  {
    // Structured binding for quick access in the following.
    // calibration_data, state and decoder_state are now for the base station that sent the sweep.
    auto& [calibration_data, state, decoder_state, angle_average, current_point, base_station] =
        collected_data_[*base_station_id];

    switch (state)
    {
      case INIT: {
        base_station.id = msg.polynomial >> 1;
        state = READ_CALIB_STREAM;
        std::cout << "Receiving calibration bit stream from base station. This can take up to a minute." << std::endl;
        break;
      }

      case READ_CALIB_STREAM: {
        if (msg.first_sweep)
        {
          bool is_complete = ootxDecoderProcessBit(&decoder_state, msg.polynomial);
          if (is_complete)
          {
            std::cout << "Received full calibration bit stream from base station " << base_station.id.value()
                      << std::endl;
            std::cout << "tilt0: " << fp16_to_float(decoder_state.frame.tilt0) << std::endl;
            std::cout << "tilt1: " << fp16_to_float(decoder_state.frame.tilt1) << std::endl;
            base_station.dt1 = fp16_to_float(decoder_state.frame.tilt0);
            base_station.dt2 = fp16_to_float(decoder_state.frame.tilt1);
            state = WAIT_FOR_POSITION;
          }
        }
      }
      break;
      case COLLECT_DATA: {
        double average_angle = (msg.angle_0 + msg.angle_1 + msg.angle_2 + msg.angle_3) / 4;
        if (msg.first_sweep)
        {
          angle_average.angle_0_sum_ += average_angle;
          angle_average.angle_0_count_++;
        }
        else
        {
          angle_average.angle_1_sum_ += average_angle;
          angle_average.angle_1_count_++;
        }
        if (angle_average.angle_0_count_ >= sweeps_per_point_ && angle_average.angle_1_count_ >= sweeps_per_point_)
        {
          double average_angle_0 = angle_average.angle_0_sum_ / angle_average.angle_0_count_;
          double average_angle_1 = angle_average.angle_1_sum_ / angle_average.angle_1_count_;
          calibration_data.second.push_back(std::pair<double, double>(average_angle_0, average_angle_1));
          std::cout << "Angles received (bs " << base_station.id.value() << "): " << average_angle_0 << ", "
                    << average_angle_1 << std::endl;
          current_point++;
          if (current_point >= point_count_)
          {
            state = FINISHED;
          }
          else
          {
            state = WAIT_FOR_POSITION;
          }
        }
      }
      break;
      default:
        break;
    }
  }
  else
  {
    // We don't know this base station and we are not in auto-detect mode
    std::cerr << "Received sweep from unknown base station " << id << std::endl;
  }
}  // namespace ros_lighthouse

void LighthouseCalibrationCollector::spinOnce()
{
  // Check if all states of the base stations are equal
  bool all_equal = true;
  if (collected_data_.empty() && auto_detect_base_stations_)
  {
    return;  // fine, wait for more base station
  }
  else
  {
    //    throw std::runtime_error("No base stations found and should not auto-detect. This should never happen.");
  }

  CalibrationState first_state = std::get<1>(collected_data_.begin()->second);
  for (auto [bs, data] : collected_data_)
  {
    if (std::get<1>(data) != first_state)
    {
      return;  // not all states are equal, wait some more
    }
  }

  auto current_point = std::get<4>(collected_data_.begin()->second);
  current_point_post_ = current_point;
  new_data_ = true;  /// A new point is processed, data for the old is available

  switch (first_state)
  {
    case WAIT_FOR_POSITION: {
      std::cout << "Position car on point number " << current_point + 1 << " of " << point_count_ << " on track."
                << std::endl;

      double x = point_coordinates_[current_point].first;
      double y = point_coordinates_[current_point].second;
      std::cout << "Coordinates of point " << current_point + 1 << ": x = " << x << ", y = " << y << std::endl;
      std::cout << "Confirm start of measurement by pressing ENTER." << std::endl;
      char c = std::cin.get();
      std::cin.clear();
      fflush(stdin);

      for (auto& [bs, data] : collected_data_)
      {
        std::get<3>(data) = { 0, 0, 0, 0 };
      }

      setAllStates(COLLECT_DATA);
    }
    break;
    default:
      break;
  }
}

std::optional<BaseStation::base_station_id>
LighthouseCalibrationCollector::findBasestationById(BaseStation::base_station_id id, bool create)
{
  auto it = collected_data_.begin();

  for (; it != collected_data_.end(); it++)
  {
    if (it->first == id)
    {
      return id;
    }
  }

  // Not found create if requested
  if (!create)
  {
    return std::nullopt;
  }

  BaseStation base_station{ .id = id };
  auto inserted =
      collected_data_.insert({ id,
                               {
                                   {
                                       .point_coordinates = {},  // todo: actually copy points from another base station
                                       .average_angles = {},
                                   },      // calibration dataset
                                   INIT,   // calibration state
                                   { 0 },  // ootx decoder state,
                                   { 0 },  // angle average
                                   0,      // current point,
                                   base_station,
                               } });
  std::cout << "Detected base station with id " << id << "." << std::endl;
  return id;
}

}  // namespace ros_lighthouse
