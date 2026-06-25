/**
 * @file    calibration_visualization.h
 * @brief   Visualization / plotting helpers for the calibration process
 */

#pragma once

#include <ros/ros.h>
#include <Eigen/Core>

namespace ros_lighthouse
{
/**
 * @brief Publishes the set of calibration points to a marker array.
 *
 * All markers ahead of the active point are displayed in green, while the
 * others are red. The active point is displayed in blue.
 *
 * @param pub The publisher to publish the markers to.
 * @param points The set of calibration points.
 * @param active_point Index pointing to the currently active point or points.end() if all points have been processed.
 */
void publishCalibrationPoints(ros::Publisher& pub, const std::vector<Eigen::Vector3d>& points, size_t active_point);

}  // namespace ros_lighthouse
