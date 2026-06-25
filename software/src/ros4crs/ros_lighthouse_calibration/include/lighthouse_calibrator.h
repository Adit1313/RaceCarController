/**
 * @file    lighthouse_calibator.h
 * @brief   Calibrate the Lighthouse tracking system and determine the parameters for the Lighthouse sensor model.
 */

#pragma once

#include <Eigen/Geometry>

#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "lighthouse_calibration.h"

namespace ros_lighthouse
{
class LighthouseCalibrator
{
public:
  LighthouseCalibrator(ros::NodeHandle& nh, ros::NodeHandle& nh_private) : nh_(nh), nh_private_(nh_private)
  {
    loadParameters();
    setupROS();
  }

  /**
   * @brief Load the existing calibration data. For all remaining data, collect interactively.
   *
   * For a base station entry in the parameter server: if there are already angle measurements,
   * the base station is skipped in the data collection.
   *
   * After the method returns, call solveCalibrationProblem() to solve the calibration problem.
   */
  void collectCalibrationData();

  void solveCalibrationProblem();

  /** Print the results and the resulting sensor models. */
  void publishCalibrationResults(std::ostream& out);

private:
  /** Set up ROS subscribers/publishers/services. */
  void setupROS();

  /** Load all required parameters from the ROS parameter server. */
  void loadParameters();

  /** Solve the calibration problem with the available data for a single station. */
  std::optional<Eigen::Affine3d> solveSingleCalibrationProblem(const BaseStation& base_station,
                                                               const CalibrationDataset& data);

  /* Calibration data ------------------------------------------------------- */

  TrackpointSet calibration_points_;                              ///< The set of calibration track points.
  std::map<BaseStation, CalibrationDataset> calibration_data_{};  ///< Calibration data collected (so far).
  std::map<BaseStation, Eigen::Affine3d> calibration_results_{};  ///< Calibration results (so far).

  /* Parameters loaded from the ROS parameter server. ----------------------- */

  int sweeps_per_point_;                    ///< Number of sweeps to collect per point.
  Eigen::Matrix<double, 2, 4> sensor_pos_;  ///< Position of sensors in the car frame.
  Eigen::Matrix<double, 4, 4> R_;           ///< Noise covariance on the sensor measurements.

  Eigen::Affine3d initial_condition_;  ///< Initial guess for the solver.

  /* ROS interaction -------------------------------------------------------- */

  ros::NodeHandle nh_;          ///< Node handle
  ros::NodeHandle nh_private_;  ///< Node handle for the private namespace

  ros::Subscriber lighthouse_sub_;                ///< Subscribes to /lighthouse to receive sweeps
  ros::ServiceClient solver_client_;              ///< Calls the solver service
  ros::Publisher calibration_markers_pub_;        ///< Publishes the current marker array
  tf2_ros::StaticTransformBroadcaster tf_pub_{};  ///< Publishes the poses of identified base stations
};
}  // namespace ros_lighthouse
