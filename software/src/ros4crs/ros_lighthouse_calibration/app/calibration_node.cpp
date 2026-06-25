/**
 * @file    calibration_node.cpp
 * @brief   Node to calibrate the Lighthouse tracking system and determine the parameters for the
 *          Lighthouse sensor model.
 */

#include <fstream>
#include <ros/ros.h>

#include "lighthouse_calibrator.h"

/* Entry point -------------------------------------------------------------- */

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ros_lighthouse_calibration");
  ros::NodeHandle nh;                                 // /<NAMESPACE>/*
  ros::NodeHandle nh_private = ros::NodeHandle("~");  // /<NAMESPACE>/lighthouse_calibration_node/*

  ros_lighthouse::LighthouseCalibrator calibrator(nh, nh_private);
  calibrator.collectCalibrationData();
  calibrator.solveCalibrationProblem();

  // Open a file in the current log directory to save the calibration results.
  std::string path = "/code/lighthouse_calibration_results_";
  path += std::to_string(ros::WallTime::now().sec) + ".txt";
  std::ofstream log_file(path);

  calibrator.publishCalibrationResults(std::cout);
  calibrator.publishCalibrationResults(log_file);

  ROS_INFO("Calibration results saved to %s", path.c_str());

  ros::spin();  // all done here
  return 0;
}
