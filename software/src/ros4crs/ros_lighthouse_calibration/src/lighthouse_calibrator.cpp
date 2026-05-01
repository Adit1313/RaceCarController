/**
 * @file    lighthouse_calibator.cpp
 * @brief   Calibrate the Lighthouse tracking system and determine the parameters for the Lighthouse sensor model.
 */

#include "lighthouse_calibrator.h"

#include <visualization_msgs/MarkerArray.h>
#include <ros_crs_utils/parameter_io.h>
#include <ros_lighthouse_calibration/Calibrate.h>

#include "calibration_visualization.h"
#include "lighthouse_calibration_collector.h"
#include "lighthouse_calibration_io.h"

namespace ros_lighthouse
{
void LighthouseCalibrator::setupROS()
{
  calibration_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("calibration_points", 1, true);
  solver_client_ =
      nh_.serviceClient<ros_lighthouse_calibration::Calibrate>("lighthouse_calibration_solver_node/calibrate");
  solver_client_.waitForExistence();  // block until the service is available
  // The static broadcaster can be used as default constructed
  // The subscriber to /lighthouse is only set up when collecting data.
}

void LighthouseCalibrator::loadParameters()
{
  // Easy I/O is done here
  if (!nh_private_.getParam("sweeps_per_point", sweeps_per_point_))
  {
    sweeps_per_point_ = 100;
    ROS_WARN_STREAM("No parameter sweeps_per_point found. Using default value of " << sweeps_per_point_);
  }

  parameter_io::getMatrixFromParams<2, 4>(ros::NodeHandle(nh_private_, "sensor_pos"), sensor_pos_);
  parameter_io::getMatrixFromParams<4, 4>(ros::NodeHandle(nh_private_, "R"), R_);

  Eigen::Matrix<double, -1, 2> points_matrix;
  parameter_io::getMatrixFromParams<-1, 2>(ros::NodeHandle(nh_, "lighthouse_calibration/points"), points_matrix);

  calibration_points_.resize(points_matrix.rows());
  for (size_t i = 0; i < points_matrix.rows(); i++)
  {
    calibration_points_[i] = { points_matrix(i, 0), points_matrix(i, 1) };
  }

  // Messy I/O is done in the separate _io.* files
  initial_condition_ = loadInitialCondition(nh_private_);
}

void LighthouseCalibrator::collectCalibrationData()
{
  // Load the existing calibration data from the parameter server
  auto existing_data = loadAvailableBaseStations(nh_);

  // Plot the calibration points.
  std::vector<Eigen::Vector3d> eigen_points(calibration_points_.size());
  std::transform(calibration_points_.begin(), calibration_points_.end(), eigen_points.begin(),
                 [](const auto& point) { return Eigen::Vector3d(point.first, point.second, 0); });
  ros_lighthouse::publishCalibrationPoints(calibration_markers_pub_, eigen_points, 0);

  // Find all base stations with incomplete data. These will be forwarded to the collector.
  std::vector<ros_lighthouse::BaseStation> remaining_stations;
  std::map<ros_lighthouse::BaseStation, ros_lighthouse::CalibrationDataset> completed_calibration_data;

  if (existing_data.empty())
  {
    // No data available whatsoever. Auto-discover the stations on the fly and let the collector handle everything.
    ROS_INFO_STREAM("No base station IDs found. Automatically determining base stations.");
    remaining_stations = {};
  }
  else
  {
    // We have some data. Let's see what we have and what we need to collect.
    for (auto [base_station, measured_angles] : existing_data)
    {
      // Check if we have angles that are associated with this base station
      if (measured_angles.has_value())
      {
        // This base station is skipping the data collection.
        completed_calibration_data.insert({
            base_station,
            {
                .first = calibration_points_,      // points
                .second = measured_angles.value()  // angles
            },
        });
      }
      else
      {
        // No angles found. We need to collect data for this base station.
        remaining_stations.push_back(base_station);
      }
    }

    // Already add the data that we have to the calibration data
    calibration_data_ = completed_calibration_data;

    if (remaining_stations.empty())
    {
      // We can exit early: all stations had data associated with them.
      return;
    }
  }

  // Now, collect all remaining data
  ros_lighthouse::LighthouseCalibrationCollector collector(sweeps_per_point_, calibration_points_, remaining_stations);
  ros::Subscriber sub =
      nh_.subscribe("lighthouse", 10, &ros_lighthouse::LighthouseCalibrationCollector::lighthouseCallback, &collector);

  ROS_INFO_STREAM("Waiting for first Lighthouse msg.");

  // This will block the thread for a while.
  while (ros::ok() && !collector.isFinished())
  {
    ros::spinOnce();
    collector.spinOnce();

    // Check if the collector has updated data. If yes, plot the current calibration position and available poses.
    if (collector.hasNewData())
    {
      // Plot the calibration points.
      std::vector<Eigen::Vector3d> eigen_points(calibration_points_.size());
      std::transform(calibration_points_.begin(), calibration_points_.end(), eigen_points.begin(),
                     [](const auto& point) { return Eigen::Vector3d(point.first, point.second, 0); });
      ros_lighthouse::publishCalibrationPoints(calibration_markers_pub_, eigen_points, collector.getActivePointIndex());
    }
  }

  auto interactive_calibration_data = collector.getCalibrationMeasurements();
  calibration_data_.insert(interactive_calibration_data.begin(), interactive_calibration_data.end());
}

void LighthouseCalibrator::solveCalibrationProblem()
{
  for (auto& [base_station, data] : calibration_data_)
  {
    std::optional result = solveSingleCalibrationProblem(base_station, data);
    if (result.has_value())
    {
      ROS_INFO_STREAM("Calibration successful for base station " << base_station.id.value() << ".");
      calibration_results_.insert(std::pair<BaseStation, Eigen::Affine3d>{ base_station, result.value() });
    }
    else
    {
      ROS_ERROR_STREAM("Calibration failed for base station " << base_station.id.value() << "!");
    }
  }
}

std::optional<Eigen::Affine3d> LighthouseCalibrator::solveSingleCalibrationProblem(const BaseStation& base_station,
                                                                                   const CalibrationDataset& data)
{
  // Formulate the service request to the solver node
  ros_lighthouse_calibration::Calibrate srv;
  ros_lighthouse_calibration::Calibrate::Request& req = srv.request;

  req.base_station.id = base_station.id.value();
  req.base_station.dt1 = base_station.dt1.value();
  req.base_station.dt2 = base_station.dt2.value();

  for (auto& [x, y] : data.first)
  {
    req.points.push_back(x);
    req.points.push_back(y);
  }
  for (auto& [angle1, angle2] : data.second)
  {
    req.angles.push_back(angle1);
    req.angles.push_back(angle2);
  }

  const auto translation = initial_condition_.translation();
  const auto orientation = Eigen::Quaterniond(initial_condition_.linear());

  req.initial_guess.position.x = translation.x();
  req.initial_guess.position.y = translation.y();
  req.initial_guess.position.z = translation.z();
  req.initial_guess.orientation.x = orientation.x();
  req.initial_guess.orientation.y = orientation.y();
  req.initial_guess.orientation.z = orientation.z();
  req.initial_guess.orientation.w = orientation.w();

  if (solver_client_.call(srv))
  {
    ROS_INFO_STREAM("Calibration successful for base station " << base_station.id.value() << ".");
    Eigen::Affine3d out;
    auto& pose = srv.response.pose;
    out.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    out.linear() = Eigen::Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)
                       .toRotationMatrix();
    return out;
  }
  else
  {
    ROS_ERROR_STREAM("Calibration failed for base station " << base_station.id.value() << "!");
    return {};
  }
}

void LighthouseCalibrator::publishCalibrationResults(std::ostream& out)
{
  // Output the measurements that were taken for each base station
  std::array position = { initial_condition_.translation().x(), initial_condition_.translation().y(),
                          initial_condition_.translation().z() };
  std::array angles = { initial_condition_.rotation().eulerAngles(0, 1, 2)[0],
                        initial_condition_.rotation().eulerAngles(0, 1, 2)[1],
                        initial_condition_.rotation().eulerAngles(0, 1, 2)[2] };
  writeCalibrationMeasurements(out, calibration_data_,
                               {
                                   .position = position,
                                   .angles = angles,
                               });

  // Write the header of the sensor model configuration
  std::vector<ros_lighthouse::BaseStation> base_stations;
  for (auto& [base_station, _] : calibration_data_)
  {
    base_stations.push_back(base_station);
  }
  writeSensorModelHeader(out, sensor_pos_, base_stations);

  // Publish the calibration results as a TF tree
  for (auto& [base_station, result] : calibration_results_)
  {
    const auto translation = result.translation();
    const auto orientation = Eigen::Quaterniond(result.linear());

    geometry_msgs::TransformStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    msg.child_frame_id = "bs" + std::to_string(base_station.id.value());

    msg.transform.translation.x = translation.x();
    msg.transform.translation.y = translation.y();
    msg.transform.translation.z = translation.z();
    msg.transform.rotation.x = orientation.x();
    msg.transform.rotation.y = orientation.y();
    msg.transform.rotation.z = orientation.z();
    msg.transform.rotation.w = orientation.w();

    tf_pub_.sendTransform(msg);

    writeBaseStationSensorModel(out, base_station, R_, result.translation(), result.rotation());
  }

  // Print the ending banner.
  ros_lighthouse::writeEndBanner(out);
}

}  // namespace ros_lighthouse
