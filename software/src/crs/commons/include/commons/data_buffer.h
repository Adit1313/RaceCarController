#ifndef COMMONS_DATA_BUFFER_H
#define COMMONS_DATA_BUFFER_H

#include <Eigen/Core>
#include <vector>
#include <boost/circular_buffer.hpp>
#include <pacejka_model/pacejka_car_state.h>
#include <mutex>

typedef crs_models::pacejka_model::pacejka_car_state pacejka_state;

template <typename DataType>
class DataBuffer
{
  typedef bool (*outlier_check_fn)(DataType, DataType, double);  // type for conciseness

public:
  std::mutex access_mutex;
  // typedef outlier_check_fnc_ptr;
  /**
   * @brief Construct a new Data Buffer object
   *
   * @param size size of buffer
   */
  DataBuffer(int size)
  {
    data_ = boost::circular_buffer<DataType>(size);
    ts_ = boost::circular_buffer<double>(size);
  };

  /**
   * Dummy default outlier check function which always returns false (i.e. no outliers)
   */
  static bool outlierCheck(const DataType a [[maybe_unused]], const DataType b [[maybe_unused]],
                           double th [[maybe_unused]])
  {
    return false;
  }

private:
  /**
   * @brief
   *
   * @param data data we are adding to buffer
   * @param ts timestamp of data
   */
  boost::circular_buffer<DataType> data_;
  boost::circular_buffer<double> ts_;

public:
  /**
   * @brief Add data to buffer
   *
   * @param data to add
   * @param ts timestamp of data
   */
  void addData(DataType data, double ts)
  {
    std::lock_guard<std::mutex> lock(access_mutex);
    data_.push_back(data);
    ts_.push_back(ts);
  }

  int size() const
  {
    return data_.size();
  }

  DataType getData(int idx) const
  {
    return data_[idx];
  }

  double getTimestamp(int idx) const
  {
    return ts_[idx];
  }

  void clear()
  {
    data_.clear();
    ts_.clear();
  }

  /**
   * @brief Get the Data object at the front of the buffer
   */
  DataType front()
  {
    return data_.front();
  }

  /**
   * @brief Returns the total timespan of the data in the buffer
   */
  double getTimespan()
  {
    return ts_.back() - ts_.front();
  }

  /**
   * @brief Interpolate data at given timesteps
   *
   * @param sample_steps delta time steps to interpolate to
   * @param last_ts timestamp of last data point
   * @param length length of data to interpolate (in number of samples)
   * @param slack slack in interpolation (in seconds).
   */
  std::vector<bool> interpolateDataNonuniform(double sampling_steps[], double last_ts, int length,
                                              std::vector<DataType>& subsampled_data, double slack = 0.002)
  {
    std::lock_guard<std::mutex> lock(access_mutex);

    double timespan = 0;
    for (int step = 0; step < length; step++)
    {
      timespan += sampling_steps[step];
    }

    double start = last_ts - timespan;
    double ref_ts = start;
    std::vector<bool> valid;

    int timestep_selection_idx = 0;

    int output_idx = 0;

    size_t t_idx = 1;  // Points to the next timestamp to be used for interpolation.
    while (t_idx < ts_.size() && output_idx < length)
    {
      auto previous_ts = ts_[t_idx - 1];
      auto current_ts = ts_[t_idx];
      double interpolation_ts = sampling_steps[timestep_selection_idx];

      if (current_ts > ref_ts)  // this means that we have one timestamp in the future and one in the past of the
                                // reference timestamp.
      {
        // interpolate
        double a = ref_ts - previous_ts;
        double b = current_ts - ref_ts;
        DataType previous_data = data_[t_idx - 1];
        DataType current_data = data_[t_idx];

        if (a > interpolation_ts && b > interpolation_ts)
        {
          valid.push_back(false);  // measurements are too far apart
        }
        else
        {
          valid.push_back(true);
        }

        subsampled_data[output_idx] = (1.0 / (1 / a + 1 / b)) * (1 / a * previous_data + 1 / b * current_data);
        output_idx += 1;

        ref_ts += interpolation_ts;
        timestep_selection_idx++;

        if (output_idx == length)  // Already have all elements we need. Can stop now.
        {
          break;
        }
      }
      else
      {  // Current timestamp is in the past (current_ts < ref). Move to next timestamp.
        t_idx++;
      }
    }

    if (output_idx < length)  // Interpolation requried a future timestamp.
    {
      if (last_ts - ts_.back() < slack && output_idx == length - 1)
      {
        subsampled_data[output_idx] = data_.back();  // Add last datapoint as ZOH.
        valid.push_back(true);
      }
      else
      {
        while (output_idx < length)
        {
          subsampled_data[output_idx] = data_.back();
          valid.push_back(false);
          output_idx++;
        }
        // subsampled_data[output_idx] = data_.back()
        // std::cout << "Could not interpolate data to desired sample frequency. Timestamps are too far "
        //                          "apart. Timestamp missmatch: " +
        //                          std::to_string(last_ts - ts_.back()) + " seconds." << std::endl;
      }
    }

    return valid;
  };

  /**
   * @brief Interpolate data to a given sample frequency
   *
   * @param sample_frq sample frequency to interpolate to
   * @param last_ts timestamp of last data point
   * @param length length of data to interpolate (in number of samples)
   * @param slack slack in interpolation (in seconds).
   *      This is used to allow for interpolation of the last data point.
   *      If the last data point is within the slack, it will be added as a zero order hold.
   *
   */
  std::vector<bool> interpolateData(double sample_frq, double last_ts, int length,
                                    std::vector<DataType>& subsampled_data,
                                    std::string interpolation_method = "bilinear", double slack = 0.002)

  {
    return interpolateData(sample_frq, last_ts, length, subsampled_data, DataBuffer::outlierCheck, 1,
                           interpolation_method, slack);
  };

  std::vector<bool> interpolateData(double sample_frq, double last_ts, int length,
                                    std::vector<DataType>& subsampled_data, outlier_check_fn outlier_check,
                                    double outlier_th = 1, std::string interpolation_method = "bilinear",
                                    double slack = 0.002)
  {
    std::lock_guard<std::mutex> lock(access_mutex);

    double start = last_ts - length * 1 / sample_frq;
    double ref_ts = start;
    std::vector<bool> valid;
    bool last_valid = true;

    int output_idx = 0;

    size_t t_idx = 1;  // Points to the next timestamp to be used for interpolation.
    while (t_idx < ts_.size() && output_idx < length)
    {
      auto previous_ts = ts_[t_idx - 1];
      auto current_ts = ts_[t_idx];

      if (current_ts > ref_ts)  // this means that we have one timestamp in the future and one in the past of the
                                // reference timestamp.
      {
        // interpolate
        double a = ref_ts - previous_ts;
        double b = current_ts - ref_ts;
        DataType previous_data = data_[t_idx - 1];
        DataType current_data = data_[t_idx];

        bool is_outlier = outlier_check(previous_data, current_data, outlier_th);

        if ((a > 1 / sample_frq && b > 1 / sample_frq) || is_outlier)
        {
          valid.push_back(false);  // measurements are too far apart
          last_valid = false;
        }
        else
        {
          valid.push_back(true);
          last_valid = true;
        }

        if (interpolation_method == "bilinear")
        {
          subsampled_data[output_idx] = (1.0 / (1 / a + 1 / b)) * (1 / a * previous_data + 1 / b * current_data);
        }
        else if (interpolation_method == "nearest")
        {
          subsampled_data[output_idx] = a < b ? previous_data : current_data;
        }
        else
        {
          throw std::invalid_argument("Interpolation method not supported");
        }
        output_idx += 1;

        ref_ts += 1 / sample_frq;

        if (output_idx == length)  // Already have all elements we need. Can stop now.
        {
          break;
        }
      }
      else
      {  // Current timestamp is in the past (current_ts < ref). Move to next timestamp.
        t_idx++;
      }
    }

    if (output_idx < length)  // Interpolation requried a future timestamp.
    {
      if (last_ts - ts_.back() < slack && output_idx == length - 1 && last_valid)
      {
        subsampled_data[output_idx] = data_.back();  // Add last datapoint as ZOH.
        valid.push_back(true);
      }
      else
      {
        while (output_idx < length)
        {
          subsampled_data[output_idx] = data_.back();
          valid.push_back(false);
          output_idx++;
        }
        // subsampled_data[output_idx] = data_.back()
        // std::cout << "Could not interpolate data to desired sample frequency. Timestamps are too far "
        //                          "apart. Timestamp missmatch: " +
        //                          std::to_string(last_ts - ts_.back()) + " seconds." << std::endl;
      }
    }

    return valid;
  };
};

#endif /* COMMONS_DATA_BUFFER_H */
