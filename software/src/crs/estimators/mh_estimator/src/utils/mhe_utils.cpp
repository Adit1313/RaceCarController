
#include <cmath>
#include <mh_estimator/utils/mhe_utils.h>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include <Eigen/Core>
#include <vector>

namespace mhe_common
{
// --------------------------- Outlier Check Functions ---------------------------
bool outlier_check_fnc_mocap(const Eigen::Vector3d a, const Eigen::Vector3d b, double threshold)
{
  bool outlier = (a - b).norm() > threshold;
  if (outlier)
    std::cout << "Mocap outlier detected: " << (a - b).norm() << std::endl;
  return outlier;
}

bool outlier_check_fnc_lh(const Eigen::Vector4d a, const Eigen::Vector4d b, double threshold)
{
  bool outlier = (a - b).norm() > threshold;
  if (outlier)
    std::cout << "Lighthouse outlier detected: " << (a - b).norm() << std::endl;
  return outlier;
}

bool outlier_check_fnc_we(const Eigen::Vector4d a, const Eigen::Vector4d b, double threshold)
{
  bool outlier = (a - b).norm() > threshold;
  if (outlier)
    std::cout << "Wheel encoder outlier detected: " << (a - b).norm() << std::endl;
  return outlier;
}

bool outlier_check_fnc_imu(const Eigen::Vector3d a, const Eigen::Vector3d b, double threshold)
{
  bool outlier = (a - b).norm() > threshold;
  if (outlier)
  {
    std::cout << "Imu outlier detected! " << " Norm: " << (a - b).norm() << " ,Threshold: " << threshold << std::endl;
  }
  return outlier;
}

bool outlier_check_fnc_imu_yaw(const Eigen::Matrix<double, 1, 1> a, const Eigen::Matrix<double, 1, 1> b,
                               double threshold)
{
  //   bool outlier = (a - b).norm() > threshold;
  bool outlier = abs(a[0] - b[0]) > threshold;
  if (outlier)
    std::cout << "Imu yaw rate outlier detected: " << abs(a[0] - b[0]) << std::endl;

  return outlier;
}

// --------------------------- Internal Filter Functions ---------------------------
void filter_imu_yaw_rate(const DataBuffer<Eigen::Matrix<double, 1, 1>>& buffer,
                         DataBuffer<Eigen::Matrix<double, 1, 1>>& filtered_buffer, std::string internal_filter_type)
{
  // Filter median
  boost::circular_buffer<Eigen::Matrix<double, 1, 1>> window;
  window.set_capacity(5);

  for (int i = 0; i < buffer.size(); i++)
  {
    if (internal_filter_type == "exponential")
    {
      // Exponential filter case
      double alpha = 0.8;
      if (i == 0)
      {
        filtered_buffer.addData(buffer.getData(i), buffer.getTimestamp(i));
        continue;
      }
      auto prev_data = filtered_buffer.getData(i - 1);
      filtered_buffer.addData((1 - alpha) * prev_data + alpha * buffer.getData(i), buffer.getTimestamp(i));
    }
    else if (internal_filter_type == "median")
    {
      // Median filter case
      int window_size = 15;
      window.push_back(buffer.getData(i));
      if (i < window_size)
      {
        filtered_buffer.addData(buffer.getData(i), buffer.getTimestamp(i));

        continue;
      }
      std::vector<Eigen::Matrix<double, 1, 1>> sorted_data;
      std::partial_sort_copy(window.begin(), window.end(), sorted_data.begin(), sorted_data.end(),
                             [](const Eigen::Matrix<double, 1, 1>& a, const Eigen::Matrix<double, 1, 1>& b) {
                               return a.norm() < b.norm();
                             });
      filtered_buffer.addData(sorted_data[int(window_size / 2)], buffer.getTimestamp(i));
    }
  }
}

void filter_imu(const DataBuffer<Eigen::Vector3d>& buffer, DataBuffer<Eigen::Vector3d>& filtered_buffer,
                std::string internal_filter_type)
{
  // Filter median
  boost::circular_buffer<Eigen::Vector3d> window;
  int window_size = 20;

  window.set_capacity(window_size);

  for (int i = 0; i < buffer.size(); i++)
  {
    if (internal_filter_type == "exponential")
    {
      // Exponential filter case
      double alpha = 0.8;
      if (i == 0)
      {
        filtered_buffer.addData(buffer.getData(i), buffer.getTimestamp(i));
        continue;
      }
      auto prev_data = filtered_buffer.getData(i - 1);
      filtered_buffer.addData((1 - alpha) * prev_data + alpha * buffer.getData(i), buffer.getTimestamp(i));
    }
    else if (internal_filter_type == "median")
    {
      std::vector<double> sorted_acc_x;
      std::vector<double> sorted_acc_y;
      std::vector<double> sorted_yaw_rate;

      int steps = 0;
      for (int window_idx = 0; window_idx < window_size; window_idx++)
      {
        int lookup_idx = i + window_idx - int(window_size / 2);
        if (lookup_idx < 0 && lookup_idx >= buffer.size())
        {
          continue;
        }
        auto data = buffer.getData(lookup_idx);
        sorted_acc_x.push_back(data(0));
        sorted_acc_y.push_back(data(1));
        sorted_yaw_rate.push_back(data(2));
        steps++;
      }
      Eigen::Vector3d median = Eigen::Vector3d::Zero();
      std::nth_element(sorted_acc_x.begin(), sorted_acc_x.begin() + sorted_acc_x.size() / 2, sorted_acc_x.end());
      std::nth_element(sorted_acc_y.begin(), sorted_acc_y.begin() + sorted_acc_y.size() / 2, sorted_acc_y.end());
      std::nth_element(sorted_yaw_rate.begin(), sorted_yaw_rate.begin() + sorted_yaw_rate.size() / 2,
                       sorted_yaw_rate.end());

      median(0) = sorted_acc_x[sorted_acc_x.size() / 2];
      median(1) = sorted_acc_y[sorted_acc_y.size() / 2];
      median(2) = sorted_yaw_rate[sorted_yaw_rate.size() / 2];

      filtered_buffer.addData(median, buffer.getTimestamp(i));
    }
    else if (internal_filter_type == "mean")
    {
      // mean filter case
      Eigen::Vector3d mean = Eigen::Vector3d::Zero();
      int steps = 0;
      for (int window_idx = 0; window_idx < window_size; window_idx++)
      {
        int lookup_idx = i + window_idx - int(window_size / 2);
        if (lookup_idx < 0 && lookup_idx >= buffer.size())
        {
          continue;
        }
        mean += buffer.getData(lookup_idx);
        steps++;
      }
      mean /= steps;
      filtered_buffer.addData(mean, buffer.getTimestamp(i));
    }
  }
}

void filter_wheel_encoders(const DataBuffer<Eigen::Vector4d>& buffer, DataBuffer<Eigen::Vector4d>& filtered_buffer,
                           std::string internal_filter_type)
{
  // Filter median
  boost::circular_buffer<Eigen::Vector4d> window;
  int window_size = 20;

  window.set_capacity(window_size);

  // std::string filter_type = "median";

  for (int i = 0; i < buffer.size(); i++)
  {
    if (internal_filter_type == "exponential")
    {
      // Exponential filter case
      double alpha = 0.8;
      if (i == 0)
      {
        filtered_buffer.addData(buffer.getData(i), buffer.getTimestamp(i));
        continue;
      }
      auto prev_data = filtered_buffer.getData(i - 1);
      filtered_buffer.addData((1 - alpha) * prev_data + alpha * buffer.getData(i), buffer.getTimestamp(i));
    }
    else if (internal_filter_type == "median")
    {
      std::vector<double> sorted_wheel_1;
      std::vector<double> sorted_wheel_2;
      std::vector<double> sorted_wheel_3;
      std::vector<double> sorted_wheel_4;

      int steps = 0;
      for (int window_idx = 0; window_idx < window_size; window_idx++)
      {
        int lookup_idx = i + window_idx - int(window_size / 2);
        if (lookup_idx < 0 && lookup_idx >= buffer.size())
        {
          continue;
        }
        auto data = buffer.getData(lookup_idx);
        sorted_wheel_1.push_back(data(0));
        sorted_wheel_2.push_back(data(1));
        sorted_wheel_3.push_back(data(2));
        sorted_wheel_4.push_back(data(3));
        steps++;
      }
      Eigen::Vector4d median = Eigen::Vector4d::Zero();
      std::nth_element(sorted_wheel_1.begin(), sorted_wheel_1.begin() + sorted_wheel_1.size() / 2,
                       sorted_wheel_1.end());
      std::nth_element(sorted_wheel_2.begin(), sorted_wheel_2.begin() + sorted_wheel_2.size() / 2,
                       sorted_wheel_2.end());
      std::nth_element(sorted_wheel_3.begin(), sorted_wheel_3.begin() + sorted_wheel_3.size() / 2,
                       sorted_wheel_3.end());
      std::nth_element(sorted_wheel_4.begin(), sorted_wheel_4.begin() + sorted_wheel_4.size() / 2,
                       sorted_wheel_4.end());

      median(0) = sorted_wheel_1[sorted_wheel_1.size() / 2];
      median(1) = sorted_wheel_2[sorted_wheel_2.size() / 2];
      median(2) = sorted_wheel_3[sorted_wheel_3.size() / 2];
      median(3) = sorted_wheel_4[sorted_wheel_4.size() / 2];

      filtered_buffer.addData(median, buffer.getTimestamp(i));
    }
    else if (internal_filter_type == "mean")
    {
      // mean filter case
      Eigen::Vector4d mean = Eigen::Vector4d::Zero();
      int steps = 0;
      for (int window_idx = 0; window_idx < window_size; window_idx++)
      {
        int lookup_idx = i + window_idx - int(window_size / 2);
        if (lookup_idx < 0 && lookup_idx >= buffer.size())
        {
          continue;
        }
        mean += buffer.getData(lookup_idx);
        steps++;
      }
      mean /= steps;
      filtered_buffer.addData(mean, buffer.getTimestamp(i));
    }
  }
}

void filter_lighthouse(const DataBuffer<Eigen::Vector4d>& buffer, DataBuffer<Eigen::Vector4d>& filtered_buffer,
                       std::string internal_filter_type)
{
  // Filter median
  boost::circular_buffer<Eigen::Vector4d> window;
  int window_size = 20;

  window.set_capacity(window_size);

  // std::string filter_type = "median";

  for (int i = 0; i < buffer.size(); i++)
  {
    if (internal_filter_type == "exponential")
    {
      // Exponential filter case
      double alpha = 0.8;
      if (i == 0)
      {
        filtered_buffer.addData(buffer.getData(i), buffer.getTimestamp(i));
        continue;
      }
      auto prev_data = filtered_buffer.getData(i - 1);
      filtered_buffer.addData((1 - alpha) * prev_data + alpha * buffer.getData(i), buffer.getTimestamp(i));
    }
    else if (internal_filter_type == "median")
    {
      std::vector<double> sorted_angle_1;
      std::vector<double> sorted_angle_2;
      std::vector<double> sorted_angle_3;
      std::vector<double> sorted_angle_4;

      int steps = 0;
      for (int window_idx = 0; window_idx < window_size; window_idx++)
      {
        int lookup_idx = i + window_idx - int(window_size / 2);
        if (lookup_idx < 0 && lookup_idx >= buffer.size())
        {
          continue;
        }
        auto data = buffer.getData(lookup_idx);
        sorted_angle_1.push_back(data(0));
        sorted_angle_2.push_back(data(1));
        sorted_angle_3.push_back(data(2));
        sorted_angle_4.push_back(data(3));
        steps++;
      }
      Eigen::Vector4d median = Eigen::Vector4d::Zero();
      std::nth_element(sorted_angle_1.begin(), sorted_angle_1.begin() + sorted_angle_1.size() / 2,
                       sorted_angle_1.end());
      std::nth_element(sorted_angle_2.begin(), sorted_angle_2.begin() + sorted_angle_2.size() / 2,
                       sorted_angle_2.end());
      std::nth_element(sorted_angle_3.begin(), sorted_angle_3.begin() + sorted_angle_3.size() / 2,
                       sorted_angle_3.end());
      std::nth_element(sorted_angle_4.begin(), sorted_angle_4.begin() + sorted_angle_4.size() / 2,
                       sorted_angle_4.end());

      median(0) = sorted_angle_1[sorted_angle_1.size() / 2];
      median(1) = sorted_angle_2[sorted_angle_2.size() / 2];
      median(2) = sorted_angle_3[sorted_angle_3.size() / 2];
      median(3) = sorted_angle_4[sorted_angle_4.size() / 2];

      filtered_buffer.addData(median, buffer.getTimestamp(i));
    }
    else if (internal_filter_type == "mean")
    {
      // mean filter case
      Eigen::Vector4d mean = Eigen::Vector4d::Zero();
      int steps = 0;
      for (int window_idx = 0; window_idx < window_size; window_idx++)
      {
        int lookup_idx = i + window_idx - int(window_size / 2);
        if (lookup_idx < 0 && lookup_idx >= buffer.size())
        {
          continue;
        }
        mean += buffer.getData(lookup_idx);
        steps++;
      }
      mean /= steps;
      filtered_buffer.addData(mean, buffer.getTimestamp(i));
    }
  }
}

void filter_mocap(const DataBuffer<Eigen::Vector3d>& buffer, DataBuffer<Eigen::Vector3d>& filtered_buffer,
                  std::string internal_filter_type)
{
  // Filter median
  boost::circular_buffer<Eigen::Vector3d> window;
  int window_size = 20;

  window.set_capacity(window_size);

  // std::string filter_type = "median";

  for (int i = 0; i < buffer.size(); i++)
  {
    if (internal_filter_type == "exponential")
    {
      // Exponential filter case
      double alpha = 0.8;
      if (i == 0)
      {
        filtered_buffer.addData(buffer.getData(i), buffer.getTimestamp(i));
        continue;
      }
      auto prev_data = filtered_buffer.getData(i - 1);
      filtered_buffer.addData((1 - alpha) * prev_data + alpha * buffer.getData(i), buffer.getTimestamp(i));
    }
    else if (internal_filter_type == "median")
    {
      std::vector<double> sorted_pos_x;
      std::vector<double> sorted_pos_y;
      std::vector<double> sorted_yaw;

      int steps = 0;
      for (int window_idx = 0; window_idx < window_size; window_idx++)
      {
        int lookup_idx = i + window_idx - int(window_size / 2);
        if (lookup_idx < 0 && lookup_idx >= buffer.size())
        {
          continue;
        }
        auto data = buffer.getData(lookup_idx);
        sorted_pos_x.push_back(data(0));
        sorted_pos_y.push_back(data(1));
        sorted_yaw.push_back(data(2));
        steps++;
      }
      Eigen::Vector3d median = Eigen::Vector3d::Zero();
      std::nth_element(sorted_pos_x.begin(), sorted_pos_x.begin() + sorted_pos_x.size() / 2, sorted_pos_x.end());
      std::nth_element(sorted_pos_y.begin(), sorted_pos_y.begin() + sorted_pos_y.size() / 2, sorted_pos_y.end());
      std::nth_element(sorted_yaw.begin(), sorted_yaw.begin() + sorted_yaw.size() / 2, sorted_yaw.end());

      median(0) = sorted_pos_x[sorted_pos_x.size() / 2];
      median(1) = sorted_pos_y[sorted_pos_y.size() / 2];
      median(2) = sorted_yaw[sorted_yaw.size() / 2];

      filtered_buffer.addData(median, buffer.getTimestamp(i));
    }
    else if (internal_filter_type == "mean")
    {
      // mean filter case
      Eigen::Vector3d mean = Eigen::Vector3d::Zero();
      int steps = 0;
      for (int window_idx = 0; window_idx < window_size; window_idx++)
      {
        int lookup_idx = i + window_idx - int(window_size / 2);
        if (lookup_idx < 0 && lookup_idx >= buffer.size())
        {
          continue;
        }
        mean += buffer.getData(lookup_idx);
        steps++;
      }
      mean /= steps;
      filtered_buffer.addData(mean, buffer.getTimestamp(i));
    }
  }
}

}  // namespace mhe_common
