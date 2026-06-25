#!/usr/bin/env python3

import rospy
import time

from geometry_msgs.msg import PoseStamped


class PosePublisher:
    def __init__(self):
        self._sub = rospy.Subscriber(
            "/qualisys/PAPER/pose", PoseStamped, self.pose_callback
        )
        self._pub = rospy.Publisher(
            "/qualisys/PAPER/pose_corr", PoseStamped, queue_size=10
        )

    def pose_callback(self, msg):
        msg.header.frame_id = "world"
        self._pub.publish(msg)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("pose_publisher")
    clock_publisher = PosePublisher()
    clock_publisher.run()
