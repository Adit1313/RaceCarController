/**
 * @file    calibration_visualization.cpp
 * @brief   Visualization / plotting helpers for the calibration process
 */

#include "calibration_visualization.h"

#include <visualization_msgs/MarkerArray.h>

namespace ros_lighthouse
{
void publishCalibrationPoints(ros::Publisher& pub, const std::vector<Eigen::Vector3d>& points, size_t active_point)
{
  visualization_msgs::MarkerArray markers_msg;
  auto& markers = markers_msg.markers;

  visualization_msgs::Marker marker_template{};
  marker_template.header.frame_id = "world";
  marker_template.header.stamp = ros::Time();

  marker_template.ns = "lighthouse_calibration";
  marker_template.id = 0;
  marker_template.type = visualization_msgs::Marker::SPHERE;
  marker_template.action = visualization_msgs::Marker::ADD;
  marker_template.pose.orientation.w = 1.0;
  marker_template.scale.x = 0.1;
  marker_template.scale.y = 0.1;
  marker_template.scale.z = 0.1;
  marker_template.color.a = 1.0;

  // Allocate space for all markers plus corresponding text labels
  markers.resize(2 * points.size(), marker_template);  // note that the resize() does not invalidate our points iterator

  for (size_t i = 0; i < markers.size(); ++i)
  {
    auto point_idx = i / 2;  // integer divison points to the correct point for the label as well

    auto& marker = markers[i];
    marker.id = i;
    marker.pose.position.x = points[point_idx].x();
    marker.pose.position.y = points[point_idx].y();
    marker.pose.position.z = points[point_idx].z();

    // Green if processed, red if not yet, blue if currently selected
    marker.color.g = point_idx < active_point ? 1.0 : 0.0;
    marker.color.r = point_idx > active_point ? 1.0 : 0.0;
    marker.color.b = point_idx == active_point ? 1.0 : 0.0;

    // If this is a text marker, set the text
    if (i % 2 == 1)
    {
      marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker.pose.position.z += 0.2;
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;
      std::ostringstream ss;
      ss << point_idx + 1 << " (x=" << points[point_idx].x() << ", y=" << points[point_idx].y() << ")" << std::endl;
      marker.text = ss.str();
    }
  }

  pub.publish(markers_msg);
}

}  // namespace ros_lighthouse
