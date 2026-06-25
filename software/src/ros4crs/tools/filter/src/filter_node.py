#!/usr/bin/python3.8

import os
import sys

# sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import rospy
import time

from sensor_msgs.msg import Imu
from crs_msgs.msg import car_wheel_speed
from filter.cfg import LowPassFilterConfig, ButterworthFilterConfig, MedianFilterConfig
import numpy as np
from scipy import signal

from dynamic_reconfigure.server import Server


class MedianFilter:
    def __init__(self):
        self.state = None
        self.initialized = False
        self.window_size = rospy.get_param("~window_size", 3)
        print("Created MedianFilter with window size: ", self.window_size)

    def set_params(self, window_size, groups):
        if not self.initialized:
            self.initialized = True
            return
        self.window_size = window_size

    def update_config(self, config):
        config["window_size"] = self.window_size
        print("Updated window size to: ", self.window_size)
        return config

    def process(self, data):
        if self.state is None:
            self.state = np.array([data])

        self.state = np.append(self.state, [data], axis=0)

        if len(self.state) < self.window_size:
            return data

        self.state = self.state[-self.window_size :]
        state = np.median(self.state, axis=0)

        return state


class ButterworthFilter:
    """Simple n-th order Butterworth filter"""

    def __init__(self):
        self.state = None
        self.initialized = False
        self.order = rospy.get_param("~order", 1)
        self.cutoff = rospy.get_param("~cutoff", 0.1)
        self.sample_rate = rospy.get_param("~sample_rate", 1)
        print(
            "Created ButterworthFilter with order: ",
            self.order,
            " cutoff: ",
            self.cutoff,
            " sample_rate: ",
            self.sample_rate,
        )
        self.b, self.a = self.butter_lowpass()

    def butter_lowpass(self):
        # nyquist = 0.5 * self.sample_rate
        # normal_cutoff = self.cutoff / nyquist
        # b, a = signal.butter(self.order, normal_cutoff, btype="low", analog=False)
        b, a = signal.butter(
            self.order, self.cutoff, btype="low", analog=False, fs=self.sample_rate
        )
        self.zi = signal.lfilter_zi(b, a)

        return b, a

    def set_params(self, order, cutoff, sample_rate, groups):
        if not self.initialized:
            self.initialized = True
            return
        self.order = order
        self.cutoff = cutoff
        self.sample_rate = sample_rate
        self.b, self.a = self.butter_lowpass()

    def update_config(self, config):
        config["order"] = self.order
        config["cutoff"] = self.cutoff
        config["sample_rate"] = self.sample_rate
        print(
            "Updated order to: ",
            self.order,
            " cutoff to: ",
            self.cutoff,
            " sample_rate to: ",
            self.sample_rate,
        )
        self.b, self.a = self.butter_lowpass()
        return config

    def process(self, data):
        if self.state is None:
            self.state = np.array([data])

        self.state = np.append(self.state, [data], axis=0)

        if len(self.state) < self.order * 2:
            return data

        self.state = self.state[-500:]
        state = signal.lfilter(self.b, self.a, self.state, axis=0)

        return state[-1]


class ExponentialFilter:
    def __init__(self):
        self.state = None
        self.alpha = rospy.get_param("~alpha", 1)
        self.initialized = False
        print("Created ExponentialFilter with alpha: ", self.alpha)

    def set_params(self, alpha, groups):

        if not self.initialized:
            self.initialized = True
            return
        self.alpha = alpha

    def update_config(self, config):
        config["alpha"] = self.alpha
        print("Updated alpha to: ", self.alpha)
        return config

    def process(self, data):
        if self.state is None:
            self.state = data
        else:
            data = self.alpha * data + (1 - self.alpha) * self.state
            self.state = data
        return data


class FilteringNode:
    def __init__(self):
        msg_type = rospy.get_param("~msg_type")
        if msg_type == "sensor_msgs/Imu":
            msg_type = Imu
        elif msg_type == "crs_msgs/car_wheel_speed":
            msg_type = car_wheel_speed
        filter_type = rospy.get_param("~filter_type")
        topic = rospy.get_param("~topic")

        self._sub = rospy.Subscriber(topic, msg_type, self.callback)
        self._pub = rospy.Publisher(topic + "_filt", msg_type, queue_size=10)
        self.msg_type = msg_type
        self.topic = topic
        self.filter_type = filter_type

        self.filter = None
        if filter_type == "low_pass":
            self.filter = ExponentialFilter()
            self.srv = Server(LowPassFilterConfig, self.config_cb)
        elif filter_type == "butterworth":
            self.filter = ButterworthFilter()
            self.srv = Server(ButterworthFilterConfig, self.config_cb)
        elif filter_type == "median":
            self.filter = MedianFilter()
            self.srv = Server(MedianFilterConfig, self.config_cb)
        else:
            raise ValueError("Invalid filter type")

    def config_cb(self, config, level):
        self.filter.set_params(**config)
        self.filter.update_config(config)
        return config

    def callback(self, msg):
        if self.msg_type == Imu:
            msg = self.filter_imu(msg)

        elif self.msg_type == car_wheel_speed:
            msg = self.filter_wheel_speed(msg)
        else:
            raise ValueError("Invalid message type")

        self._pub.publish(msg)

    def filter_imu(self, msg):
        data = np.array(
            [
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.angular_velocity.z,
            ]
        )
        filtered_data = self.filter.process(data)
        msg.linear_acceleration.x = filtered_data[0]
        msg.linear_acceleration.y = filtered_data[1]
        msg.angular_velocity.z = filtered_data[2]
        return msg

    def filter_wheel_speed(self, msg):
        data = np.array(
            [msg.front_left, msg.front_right, msg.rear_left, msg.rear_right]
        )
        filtered_data = self.filter.process(data)
        msg.front_left = filtered_data[0]
        msg.front_right = filtered_data[1]
        msg.rear_left = filtered_data[2]
        msg.rear_right = filtered_data[3]
        return msg

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("FilteringNode", anonymous=True)
    clock_publisher = FilteringNode()
    clock_publisher.run()
