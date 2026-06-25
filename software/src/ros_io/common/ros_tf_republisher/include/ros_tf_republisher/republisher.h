/**
 * @file  republisher.h
 * @brief A ROS node splitting messages from /tf into separate topics by child frame id.
 */

#pragma once

#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>

namespace ros_io
{
class Republisher
{
public:
  Republisher(ros::NodeHandle& nh, unsigned int queue_size) : nh_(nh)
  {
    sub_ = nh_.subscribe("/tf", queue_size, &Republisher::processTFMessage, this);
  };

  /**
   * @brief Process a message from the /tf topic.
   *
   * For each transform in the message, publish it to the corresponding topic, creating the topic if necessary.
   */
  void processTFMessage(const tf2_msgs::TFMessage& msg)
  {
    for (const auto& transform : msg.transforms)
    {
      auto& publisher = lookupPublisher(transform.child_frame_id);
      publisher.publish(transform);
    }
  };

private:
  /**
   * @brief Look up the publisher pertaining to a particular frame.
   *
   * If the publisher for frame does not exist yet, it will be created.
   */
  inline ros::Publisher& lookupPublisher(const std::string frame)
  {
    const auto frame_name = getFrameName(frame);

    decltype(publishers_)::iterator it;  // will point to the publisher for the frame

    // Attempt to find the publisher corresponding to the frame. Lock to prevent simultaneous lookup and insertion.
    {
      std::lock_guard<std::mutex> lock(publishers_mutex_);

      it = publishers_.find(frame_name);
      if (it != publishers_.end())
        return it->second;

      // If the publisher does not exist, create it.
      auto publisher = nh_.advertise<geometry_msgs::TransformStamped>(frame_name, 5);
      auto [new_it, _] = publishers_.emplace(frame_name, publisher);
      it = new_it;
    }
    ROS_INFO("Created republisher for frame %s", frame_name.c_str());
    return it->second;
  }

  /** Frame name without any leading slashes. */
  inline std::string getFrameName(const std::string& frame)
  {
    return frame.starts_with("/") ? frame.substr(1) : frame;
  }

  ros::NodeHandle nh_;
  ros::Subscriber sub_;

  /** Publishers for each separate child frame. */
  std::unordered_map<std::string, ros::Publisher> publishers_;
  std::mutex publishers_mutex_;
};

};  // namespace ros_io
