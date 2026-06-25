/**
 * @file    lighthouse_calibration_io.h
 * @author  Lukas Vogel (vogellu@ethz.ch)
 * @brief   Contains helper functions for reading and writing calibration data
 */

#pragma once

#include <iostream>
#include <map>
#include <ros/ros.h>
#include <Eigen/Geometry>

#include "lighthouse_calibration.h"

namespace ros_lighthouse
{
/* Data loading functions -------------------------------------------------- */

/**
 * @brief Load the initial condition from the ROS parameter server.
 *
 * @param nh Node handle where /initial_condition is loaded from.
 * @return ros_lighthouse::LighthousePose
 */
Eigen::Affine3d loadInitialCondition(const ros::NodeHandle& nh);

/**
 * @brief Finds the available base stations on the parameter server and loads their calibration data.
 *
 * A base station may have associated with it a set of angles that were measured during a previous calibration run.
 * If such a set is available, it will be loaded.
 *
 * The return value of this function is a map from base station to calibration data. If a base station has no
 * calibration data associated with it, the map will not contain an entry for that base station.
 * @param nh_private
 * @return std::map<ros_lighthouse::BaseStation, ros_lighthouse::MeasuredAngleSet>
 */
std::map<BaseStation, std::optional<MeasuredAngleSet>> loadAvailableBaseStations(const ros::NodeHandle& nh);

/* Data writing functions -------------------------------------------------- */

void writeCalibrationMeasurements(std::ostream& out, const std::map<BaseStation, CalibrationDataset>& calibration_basis,
                                  const LighthousePose& initial_condition);
void writeSensorModelHeader(std::ostream& out, Eigen::Matrix<double, 2, 4> sensor_pos,
                            std::vector<BaseStation> base_stations);
void writeBaseStationSensorModel(std::ostream& out, BaseStation base_station, Eigen::Matrix<double, 4, 4> R,
                                 Eigen::Vector3d pos, Eigen::Matrix3d rot);
void writeEndBanner(std::ostream& out);

}  // namespace ros_lighthouse
