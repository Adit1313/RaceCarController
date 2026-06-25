/**
 * @file    lighthouse_calibration_io.cpp
 * @author  Lukas Vogel (vogellu@ethz.ch)
 * @brief   Contains helper functions for reading and writing calibration data
 */

#include "lighthouse_calibration_io.h"

#include <ros/ros.h>
#include <casadi/casadi.hpp>
#include <Eigen/Geometry>

#include <ros_crs_utils/parameter_io.h>

#include "lighthouse_calibration.h"
#include "lighthouse_calibration_solver.h"
#include "lighthouse_calibration_collector.h"

namespace ros_lighthouse
{
/* Data loading functions -------------------------------------------------- */

Eigen::Affine3d loadInitialCondition(const ros::NodeHandle& nh)
{
  Eigen::Matrix<double, 1, 3> initial_position, initial_angles;

  parameter_io::getMatrixFromParams<1, 3>(ros::NodeHandle(nh, "initial_condition/position"), initial_position);
  parameter_io::getMatrixFromParams<1, 3>(ros::NodeHandle(nh, "initial_condition/angles"), initial_angles);

  Eigen::Matrix3d rot = (Eigen::AngleAxisd(initial_angles(0, 2), Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(initial_angles(0, 1), Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(initial_angles(0, 0), Eigen::Vector3d::UnitX()))
                            .toRotationMatrix();
  auto transform = Eigen::Affine3d(rot);
  transform.translation() = initial_position.transpose();

  return transform;
}

std::map<BaseStation, std::optional<MeasuredAngleSet>> loadAvailableBaseStations(const ros::NodeHandle& nh)
{
  std::vector<std::string> bs_to_load;
  ros::NodeHandle nh_scoped(nh, "lighthouse_calibration");

  if (!nh_scoped.getParam("base_stations", bs_to_load))
  {
    ROS_WARN("Key lighthouse_calibration/base_stations not found in parameter server. "
             "Trying to identify base stations automatically.");
    return {};
  }

  // Note that if the array is empty, this will return an empty vector. This is by design! If the array is empty,
  // we try to identify the base stations automatically.
  std::map<ros_lighthouse::BaseStation, std::optional<ros_lighthouse::MeasuredAngleSet>> base_station_data;

  for (auto bs : bs_to_load)
  {
    ROS_INFO_STREAM("Loading base station " << bs);
    ros_lighthouse::BaseStation::base_station_id id;
    double dt1, dt2;

    ros::NodeHandle bs_nh(nh_scoped, bs);
    bs_nh.getParam("id", id);
    bs_nh.getParam("dt1", dt1);
    bs_nh.getParam("dt2", dt2);

    // Check if there is a set of angles associated with this base station
    if (bs_nh.hasParam("angles"))
    {
      // Load the angles
      Eigen::Matrix<double, -1, 2> angles_matrix;
      parameter_io::getMatrixFromParams<-1, 2>(ros::NodeHandle(bs_nh, "angles"), angles_matrix);

      // Convert the calibration data to the format used by the solver
      ros_lighthouse::MeasuredAngleSet angles;
      for (size_t i = 0; i < angles_matrix.rows(); i++)
      {
        angles.push_back(std::pair<double, double>(angles_matrix(i, 0), angles_matrix(i, 1)));
      }

      // Insert the base station with the associated angles
      base_station_data.insert({ { id, dt1, dt2 }, angles });
    }
    else
    {
      // Insert a base station with no associated measured angles
      base_station_data.insert({ { id, dt1, dt2 }, std::nullopt });
    }
  }

  return base_station_data;
}

/* Data writing functions -------------------------------------------------- */

void outputMatrix(const Eigen::Matrix<double, -1, -1>& m, std::ostream& out)
{
  out << "[";
  for (size_t row = 0; row < m.rows(); row++)
  {
    out << "[";
    for (size_t col = 0; col < m.cols(); col++)
    {
      out << m(row, col);
      if (col < m.cols() - 1)
      {
        out << ", ";
      }
    }
    out << "]";
    if (row < m.rows() - 1)
    {
      out << ", ";
    }
  }
  out << "]";
}

void writeCalibrationMeasurements(std::ostream& out, const std::map<BaseStation, CalibrationDataset>& calibration_basis,
                                  const LighthousePose& initial_condition)
{
  out << "###################################################" << std::endl;
  out << "########### Lighthouse Calibration Data ###########" << std::endl;
  out << "###################################################" << std::endl;
  out << std::endl;
  out << "lighthouse_calibration:" << std::endl;
  out << "  base_stations: [";

  for (auto& [base_station, calibration_data] : calibration_basis)
  {
    out << "\"bs" << base_station.id.value() << "\"";
    if (&base_station != &calibration_basis.rbegin()->first)
    {
      out << ", ";
    }
  }

  out << "]" << std::endl;
  out << "  initial_condition:" << std::endl;
  out << "    position:" << std::endl;
  out << "      value: [[" << initial_condition.position[0] << ", " << initial_condition.position[1] << ", "
      << initial_condition.position[2] << "]]" << std::endl;
  out << "    angles:" << std::endl;
  out << "      value: [[" << initial_condition.angles[0] << ", " << initial_condition.angles[1] << ", "
      << initial_condition.angles[2] << "]]" << std::endl;
  out << "  points:" << std::endl;
  out << "    value: " << calibration_basis.begin()->second.first << std::endl;
  out << std::endl;

  for (auto& [base_station, calibration_data] : calibration_basis)
  {
    out << "  bs" << base_station.id.value() << ":" << std::endl;
    out << "    id: " << base_station.id.value() << std::endl;
    out << "    dt1: " << base_station.dt1.value() << std::endl;
    out << "    dt2: " << base_station.dt2.value() << std::endl;
    out << "    angles:" << std::endl;
    out << "      value: ";
    out << calibration_data.second << std::endl;
    out << std::endl;
  }
}

void writeSensorModelHeader(std::ostream& out, Eigen::Matrix<double, 2, 4> sensor_pos,
                            std::vector<BaseStation> base_stations)
{
  out << std::endl;
  out << "###################################################" << std::endl;
  out << "################ Sensor model data ################" << std::endl;
  out << "###################################################" << std::endl;

  out << std::endl;
  out << "  lighthouse:" << std::endl;
  out << "    sensor_pos:" << std::endl;
  out << "      value: ";
  outputMatrix(sensor_pos, out);
  out << std::endl;
  out << "    key: lighthouse" << std::endl;
  out << "    base_stations: [";

  for (size_t i = 0; i < base_stations.size(); i++)
  {
    out << "\"bs" << base_stations[i].id.value() << "\"";
    if (i < base_stations.size() - 1)
    {
      out << ", ";
    }
  }

  out << "]" << std::endl;
}

void writeBaseStationSensorModel(std::ostream& out, BaseStation base_station, Eigen::Matrix<double, 4, 4> R,
                                 Eigen::Vector3d pos, Eigen::Matrix3d rot)
{
  out << std::endl;
  out << "    bs" << base_station.id.value() << ":" << std::endl;
  out << "      bs_ID: " << base_station.id.value() << std::endl;
  out << "      R:" << std::endl;
  out << "        value: ";
  outputMatrix(R, out);
  out << std::endl;
  out << "      P_bs:" << std::endl;
  out << "        value: [[" << pos(0) << "], [" << pos(1) << "], [" << pos(2) << "]]" << std::endl;
  out << "      R_bs:" << std::endl;
  out << "        value: ";
  outputMatrix(rot, out);
  out << std::endl;
  out << "      dt1: " << base_station.dt1.value() << std::endl;
  out << "      dt2: " << base_station.dt2.value() << std::endl;
}

void writeEndBanner(std::ostream& out)
{
  out << std::endl;
  out << "###################################################" << std::endl;
  out << "################# End of config ###################" << std::endl;
  out << "###################################################" << std::endl;

  out << "Calibration finished. Copy the sensor model data to estimator.yaml!" << std::endl;
}

}  // namespace ros_lighthouse
