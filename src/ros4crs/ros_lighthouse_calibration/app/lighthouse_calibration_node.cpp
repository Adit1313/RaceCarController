/**
 * @file    lighthouse_calibration_node.cpp
 * @author  Tobias Bodewig
 * @brief   Node to calibrate the Lighthouse tracking system and determine the parameters for the
 *          Lighthouse sensor model.
 */

#include <ros/ros.h>
#include <Eigen/Core>
#include <ros_crs_utils/parameter_io.h>

#include "lighthouse_calibration.h"
#include "lighthouse_calibration_solver.h"
#include "lighthouse_calibration_collector.h"
#include "lighthouse_calibration_io.h"

/* Calibration data loading helpers ----------------------------------------- */

/**
 * @brief Collect calibration data for a set of base stations if the track points are known.
 *
 * @param nh            Node handle
 * @param nh_private    Private node handle
 * @param points        Set of track points to use for calibration
 * @param base_stations List of base stations to collect data for. If empty, auto-detect.
 * @returns A map from base station to calibration data.
 */
std::map<ros_lighthouse::BaseStation, ros_lighthouse::CalibrationDataset>
collectDataforBasestations(ros::NodeHandle& nh, ros::NodeHandle& nh_private, ros_lighthouse::TrackpointSet points,
                           std::vector<ros_lighthouse::BaseStation> base_stations)
{
  int sweeps_per_point;
  nh_private.getParam("sweeps_per_point", sweeps_per_point);

  // Instantiate the collector with the points we already know
  ros_lighthouse::LighthouseCalibrationCollector collector(sweeps_per_point, points, base_stations);

  // Set up subscriber to /<NAMESPACE>/lighthouse. Automatically unsubscribes as soon as this method exits.
  ros::Subscriber sub =
      nh.subscribe("lighthouse", 10, &ros_lighthouse::LighthouseCalibrationCollector::lighthouseCallback, &collector);

  ROS_INFO_STREAM("Waiting for first Lighthouse msg.");

  while (ros::ok() && !collector.isFinished())
  {
    ros::spinOnce();
    collector.spinOnce();
  }

  return collector.getCalibrationMeasurements();
}

/**
 * @brief Loads calibration data from the parameter server and collects the remaining data interactively.
 *
 * @param nh         Node handle
 * @param nh_private Private node handle
 * @return A map from base station to calibration data.
 */
std::map<ros_lighthouse::BaseStation, ros_lighthouse::CalibrationDataset>
loadCalibrationFromFile(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
{
  // Instead of collecting the data interactively, load a configuration from the parameter server
  ROS_INFO_STREAM("Using calibration configuration from parameter server.");

  auto base_stations = ros_lighthouse::loadAvailableBaseStations(nh);
  ros_lighthouse::TrackpointSet points;

  // Load the calibration points from the config
  Eigen::Matrix<double, -1, 2> points_matrix;
  parameter_io::getMatrixFromParams<-1, 2>(ros::NodeHandle(nh, "lighthouse_calibration/points"), points_matrix);
  for (size_t i = 0; i < points_matrix.rows(); i++)
  {
    points.push_back(std::pair<double, double>(points_matrix(i, 0), points_matrix(i, 1)));
  }

  // Find all base stations that have calibration data associated with them. These will be forwarded to the collector.
  // The others will have a set associated with the existing points and angles.
  std::vector<ros_lighthouse::BaseStation> base_stations_to_calibrate;
  std::map<ros_lighthouse::BaseStation, ros_lighthouse::CalibrationDataset> calibration_data;

  if (!base_stations.empty())
  {
    for (auto it : base_stations)
    {
      // Check if we have angles that are associated with this base station
      if (it.second.has_value())
      {
        calibration_data.insert({
            it.first,  // base station
            {
                .first = points,             // points
                .second = it.second.value()  // angles
            },
        });
      }
      else
      {
        // No angles found. We need to collect data for this base station.
        base_stations_to_calibrate.push_back(it.first);
      }
    }

    if (!base_stations_to_calibrate.empty())
    {
      auto calibration_data_to_add = collectDataforBasestations(nh, nh_private, points, base_stations_to_calibrate);
      calibration_data.insert(calibration_data_to_add.begin(), calibration_data_to_add.end());
    }
  }
  else
  {
    // No base stations passed at all -> find available stations automatically
    ROS_INFO_STREAM("No base station IDs found. Automatically determining base stations.");
    auto calibration_data_to_add = collectDataforBasestations(nh, nh_private, points, {});
    calibration_data.insert(calibration_data_to_add.begin(), calibration_data_to_add.end());
  }

  return calibration_data;
}

/**
 * @brief Collect calibration data fully interactively.
 *
 * This method will auto-detect all base stations and also ask the user for the calibration points.
 * Only the number of sweeps is loaded from the parameter server.
 *
 * @param nh          Node handle
 * @param nh_private  Private node handle
 * @return A map from discovered base station(s) to calibration data.
 */
std::map<ros_lighthouse::BaseStation, ros_lighthouse::CalibrationDataset>
makeCalibrationInteractively(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
{
  // Collect the calibration data interactively
  ROS_INFO("Starting interactive calibration.");

  int sweeps_per_point;
  nh_private.getParam("sweeps_per_point", sweeps_per_point);

  ros_lighthouse::LighthouseCalibrationCollector collector(sweeps_per_point);

  // Set up subscriber to /<NAMESPACE>/lighthouse. Automatically unsubscribes as soon as this method exits.
  ros::Subscriber sub =
      nh.subscribe("lighthouse", 10, &ros_lighthouse::LighthouseCalibrationCollector::lighthouseCallback, &collector);

  ROS_INFO("Waiting for first Lighthouse msg.");

  while (ros::ok() && !collector.isFinished())
  {
    ros::spinOnce();
    collector.spinOnce();
  }

  return collector.getCalibrationMeasurements();
}

/* Entry point -------------------------------------------------------------- */

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ros_lighthouse_calibration");
  ros::NodeHandle nh;                                 // /<NAMESPACE>/*
  ros::NodeHandle nh_private = ros::NodeHandle("~");  // /<NAMESPACE>/lighthouse_calibration_node/*

  // Load the position of the sensor on the car and the measurement covariance
  Eigen::Matrix<double, 2, 4> sensor_pos_ = Eigen::Matrix<double, 2, 4>::Zero();
  Eigen::Matrix<double, 4, 4> R_ = Eigen::Matrix<double, 4, 4>::Identity();
  parameter_io::getMatrixFromParams<2, 4>(ros::NodeHandle(nh_private, "sensor_pos"), sensor_pos_);
  parameter_io::getMatrixFromParams<4, 4>(ros::NodeHandle(nh_private, "R"), R_);

  // Load the initial condition to use for solving w.r.t. the Lighthouse base station poses
  ros_lighthouse::LighthousePose initial_condition = ros_lighthouse::loadInitialCondition(nh_private);

  // Load the data to use for calibration. Either the parameter server contains a point set and
  // potentially a set of angles, or all the data is collected interactively.
  std::map<ros_lighthouse::BaseStation, ros_lighthouse::CalibrationDataset> calibration_basis;

  bool use_config = false;
  if (nh_private.getParam("use_config", use_config) && use_config)
  {
    // Load calibration data & initial condition from file
    calibration_basis = loadCalibrationFromFile(nh, nh_private);
  }
  else
  {
    // Make entire calibration interactively
    calibration_basis = makeCalibrationInteractively(nh, nh_private);
  }

  // Output the measurements that were taken for each base station
  writeCalibrationMeasurements(std::cout, calibration_basis, initial_condition);

  // Write the header of the sensor model configuration
  std::vector<ros_lighthouse::BaseStation> base_stations;
  for (auto& [base_station, _] : calibration_basis)
  {
    base_stations.push_back(base_station);
  }
  writeSensorModelHeader(std::cout, sensor_pos_, base_stations);

  // Solve for the sensor model parameters for each base station and output them
  for (auto& [base_station, calibration_data] : calibration_basis)
  {
    ros_lighthouse::LighthouseCalibrationSolver solver;

    bool success = false;
    try
    {
      success = solver.solve(base_station, calibration_data, initial_condition);
    }
    catch (std::exception& e)
    {
      ROS_ERROR_STREAM("Solver failed! Reason: " << e.what());
    }

    if (success)
    {
      writeBaseStationSensorModel(std::cout, base_station, R_, solver.getBasestationPosition().value(),
                                  solver.getBasestationRotation().value());
    }
    else
    {
      ROS_ERROR_STREAM("Calibration failed for base station " << base_station.id.value()
                                                              << "! See calibration data above for more information.");
    }
  }

  // Print the ending banner.
  ros_lighthouse::writeEndBanner(std::cout);
  return 0;
}
