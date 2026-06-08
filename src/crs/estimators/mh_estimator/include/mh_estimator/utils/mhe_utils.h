#ifndef SRC_CRS_ESTIMATORS_MH_ESTIMATOR_INCLUDE_MH_ESTIMATOR_UTILS
#define SRC_CRS_ESTIMATORS_MH_ESTIMATOR_INCLUDE_MH_ESTIMATOR_UTILS

#include <cmath>
#include <commons/data_buffer.h>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include <Eigen/Core>
#include <vector>

namespace mhe_common
{
// --------------------------- Outlier Check Functions ---------------------------
bool outlier_check_fnc_mocap(const Eigen::Vector3d a, const Eigen::Vector3d b, double threshold);

bool outlier_check_fnc_lh(const Eigen::Vector4d a, const Eigen::Vector4d b, double threshold);

bool outlier_check_fnc_we(const Eigen::Vector4d a, const Eigen::Vector4d b, double threshold);

bool outlier_check_fnc_imu(const Eigen::Vector3d a, const Eigen::Vector3d b, double threshold);

bool outlier_check_fnc_imu_yaw(const Eigen::Matrix<double, 1, 1> a, const Eigen::Matrix<double, 1, 1> b,
                               double threshold);

// --------------------------- Internal Filter Functions ---------------------------
void filter_imu_yaw_rate(const DataBuffer<Eigen::Matrix<double, 1, 1>>& buffer,
                         DataBuffer<Eigen::Matrix<double, 1, 1>>& filtered_buffer, std::string internal_filter_type);

void filter_imu(const DataBuffer<Eigen::Vector3d>& buffer, DataBuffer<Eigen::Vector3d>& filtered_buffer,
                std::string internal_filter_type);

void filter_wheel_encoders(const DataBuffer<Eigen::Vector4d>& buffer, DataBuffer<Eigen::Vector4d>& filtered_buffer,
                           std::string internal_filter_type);

void filter_lighthouse(const DataBuffer<Eigen::Vector4d>& buffer, DataBuffer<Eigen::Vector4d>& filtered_buffer,
                       std::string internal_filter_type);

void filter_mocap(const DataBuffer<Eigen::Vector3d>& buffer, DataBuffer<Eigen::Vector3d>& filtered_buffer,
                  std::string internal_filter_type);

}  // namespace mhe_common
#endif /* SRC_CRS_ESTIMATORS_MH_ESTIMATOR_INCLUDE_MH_ESTIMATOR_UTILS */
