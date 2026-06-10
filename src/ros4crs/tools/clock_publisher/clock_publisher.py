#!/usr/bin/env python3

import rospy
import time

from rosgraph_msgs.msg import Clock


class ClockPublisher:
    def __init__(self):
        self.clock_rate = rospy.get_param("~clock_rate", 1.0)
        self.pub = rospy.Publisher("/clock", Clock, queue_size=10)

        # Set start time to current system time of this pc
        system_time = time.time()
        system_time_s = int(system_time)
        system_time_ns = int((system_time - system_time_s) * 1e9)
        self.curr_time = rospy.Time(system_time_s, system_time_ns)

    def run(self):
        while not rospy.is_shutdown():
            # Publishes clock all 1 ms
            self.curr_time = self.curr_time + rospy.Duration(0.001)  # 1ms
            self.pub.publish(self.curr_time)
            time.sleep(0.001 * 1 / self.clock_rate)


if __name__ == "__main__":
    rospy.init_node("clock_publisher")
    clock_publisher = ClockPublisher()
    rospy.loginfo(
        f"Clock publisher started. Running at {clock_publisher.clock_rate:.2f} Hz"
    )
    clock_publisher.run()
