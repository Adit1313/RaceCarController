/**
 * @file    solver_node.cpp
 * @brief   ROS node offering a service to solve the Lighthouse calibration problem.
 */

#include <ros/ros.h>
#include <Eigen/Geometry>

#include "ros_lighthouse_calibration/Calibrate.h"
#include "lighthouse_calibration.h"
#include "lighthouse_calibration_solver.h"

/**
 * @brief Service callback to solve the calibration problem.
 *
 * The service internally calls the calibration solver using CasADi to
 * solve the optimization problem. On success, the base station pose
 * is returned in the response.
 *
 * @param req The request containing the calibration data.
 * @param res The response containing the calibration results.
 * @return True if the calibration was successful, false otherwise.
 */
bool calibrate(ros_lighthouse_calibration::Calibrate::Request& req,
               ros_lighthouse_calibration::Calibrate::Response& res);

/* Entry point -------------------------------------------------------------- */

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ros_lighthouse_calibration_solver");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Advertise the calibration service
  ros::ServiceServer service = nh_private.advertiseService("calibrate", calibrate);

  ros::spin();
  return 0;
}

/* Service callback implementation ------------------------------------------ */

bool calibrate(ros_lighthouse_calibration::Calibrate::Request& req,
               ros_lighthouse_calibration::Calibrate::Response& res)
{
  using namespace ros_lighthouse;

  // Check if the request is valid: all pairs are complete and the id is valid.
  if (req.base_station.id > 15)
  {
    ROS_ERROR("Invalid base station ID: %d", req.base_station.id);
    return false;
  }
  if (req.angles.size() % 2 != 0)
  {
    ROS_ERROR("Invalid number of angles: %lu", req.angles.size());
    return false;
  }
  if (req.points.size() % 2 != 0)
  {
    ROS_ERROR("Invalid number of points: %lu", req.points.size());
    return false;
  }
  if (req.points.size() / 2 != req.angles.size() / 2)
  {
    ROS_ERROR("Number of points and angles do not match: %lu vs %lu", req.points.size(), req.angles.size());
    return false;
  }

  // Parse into the data structures used for calibration
  BaseStation base_station{ .id = req.base_station.id, .dt1 = req.base_station.dt1, .dt2 = req.base_station.dt2 };
  TrackpointSet trackpoints{};
  MeasuredAngleSet measured_angles{};

  for (size_t i = 0; i < req.points.size(); i += 2)
  {
    trackpoints.push_back({ req.points[i], req.points[i + 1] });
    measured_angles.push_back({ req.angles[i], req.angles[i + 1] });
  }

  const CalibrationDataset dataset{ trackpoints, measured_angles };
  LighthousePose initial_guess{};
  initial_guess.position = { req.initial_guess.position.x, req.initial_guess.position.y, req.initial_guess.position.z };
  auto rot_angles = Eigen::Quaterniond(req.initial_guess.orientation.w, req.initial_guess.orientation.x,
                                       req.initial_guess.orientation.y, req.initial_guess.orientation.z)
                        .toRotationMatrix()
                        .eulerAngles(0, 1, 2);
  initial_guess.angles = { rot_angles[0], rot_angles[1], rot_angles[2] };

  // Attempt to solve for the calibration.
  LighthouseCalibrationSolver solver(false);
  try
  {
    if (!solver.solve(base_station, dataset, initial_guess))
    {
      ROS_ERROR("Failed to solve calibration.");
      return false;
    }
  }
  catch (const std::invalid_argument& e)
  {
    ROS_ERROR("Invalid calibration data: %s", e.what());
    return false;
  }

  // Get the results and fill the response
  auto orientation = solver.getBaseStationOrientation().value();
  auto position = solver.getBasestationPosition().value();

  res.pose.position.x = position[0];
  res.pose.position.y = position[1];
  res.pose.position.z = position[2];

  res.pose.orientation.x = orientation.x();
  res.pose.orientation.y = orientation.y();
  res.pose.orientation.z = orientation.z();
  res.pose.orientation.w = orientation.w();

  return true;
}
