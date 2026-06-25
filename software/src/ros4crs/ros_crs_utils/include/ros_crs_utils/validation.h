#ifndef SRC_ROS_ROS_CRS_UTILS_INCLUDE_ROS_CRS_UTILS_VALIDATION
#define SRC_ROS_ROS_CRS_UTILS_INCLUDE_ROS_CRS_UTILS_VALIDATION

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

inline bool is_valid_marker(const visualization_msgs::Marker& marker)
{
  if (marker.points.size() > 0)
  {
    for (const auto& pt : marker.points)
    {
      if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z) || std::isinf(pt.x) || std::isinf(pt.y) ||
          std::isinf(pt.z))
      {
        ROS_WARN_STREAM_THROTTLE(1, "Invalid point in marker msg. Point was: " << pt);
        return false;
      }
    }
  }
  else
  {
    if (std::isnan(marker.pose.position.x) || std::isnan(marker.pose.position.y) ||
        std::isnan(marker.pose.position.z) || std::isinf(marker.pose.position.x) ||
        std::isinf(marker.pose.position.y) || std::isinf(marker.pose.position.z))
    {
      ROS_WARN_STREAM_THROTTLE(1, "Invalid position in marker msg. Position was: " << marker.pose.position);
      return false;
    }

    // Check quaternion
    auto x = marker.pose.orientation.x;
    auto y = marker.pose.orientation.y;
    auto z = marker.pose.orientation.z;
    auto w = marker.pose.orientation.w;

    auto yaw = atan2(2.0 * (y * z + w * x), w * w - x * x - y * y + z * z);
    auto pitch = asin(-2.0 * (x * z - w * y));
    auto roll = atan2(2.0 * (x * y + w * z), w * w + x * x - y * y - z * z);

    if (std::isnan(yaw) || std::isnan(pitch) || std::isnan(roll) || std::isinf(yaw) || std::isinf(pitch) ||
        std::isinf(roll))
    {
      ROS_WARN_STREAM_THROTTLE(1, "Invalid orientation in marker msg. Roll was: " << roll << ", Pitch was: " << pitch
                                                                                  << ", Yaw was: " << yaw);
      return false;
    }
  }

  return true;
}

#endif /* SRC_ROS_ROS_CRS_UTILS_INCLUDE_ROS_CRS_UTILS_VALIDATION */
