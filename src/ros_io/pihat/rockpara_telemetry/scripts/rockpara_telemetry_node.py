#!/usr/bin/env python3

import rospy
import serial
import struct
import time

from std_msgs.msg import Bool, Float32MultiArray
from crs_msgs.msg import bool_array_stamped
from rockpara_telemetry import Command


CONST_I_127 = (1 << 7) - 1
CONST_I_255 = (1 << 8) - 1


class RockparaTelemetryNode(object):
    """Node handling receiving telemetry msgs and publishing in ROS"""

    def __init__(self, node_name: str = "rockpara_telemetry_node"):
        rospy.init_node(node_name, anonymous=True)
        # --- parameter(s)
        frequency: float = rospy.get_param("~frequency")
        self._serial_addr: str = rospy.get_param("~serial_address")
        self._serial_baudrate: int = rospy.get_param("~serial_baudrate")
        self._serial_timeout: float = rospy.get_param("~serial_timeout")

        # --- publisher(s)
        self._pub_arm = rospy.Publisher(
            "~arm",
            Bool,
            queue_size=10,
        )
        self._pub_disarm = rospy.Publisher(
            "~disarm",
            Bool,
            queue_size=10,
        )
        self._pub_manual_commands = rospy.Publisher(
            "~manual_commands",
            Float32MultiArray,
            queue_size=10,
        )
        self._pub_manual_override = rospy.Publisher(
            "~manual_override",
            Bool,
            queue_size=10,
        )

        self._pub_controller_settings = rospy.Publisher(
            "~controller_settings", bool_array_stamped, queue_size=10
        )

        self.last_stamp_joy = time.time()
        # initialize device
        self._dev = self._init_device()

        rospy.loginfo("Entering main loop...")
        self._timer = rospy.Timer(rospy.Duration(1.0 / frequency), self._timer_callback)

        self._is_armed: bool = False
        """Boolean value indicating whether the rocket is currently in an armed state or not."""

    def _init_device(self):
        # telemetry device
        return serial.Serial(
            self._serial_addr,
            baudrate=self._serial_baudrate,
            timeout=self._serial_timeout,
        )

    def _timer_callback(self, _):
        # try read all pending commands from buffer
        while self._dev.in_waiting > 0:
            try:
                # read number of bytes
                try_size = self._dev.read(4)
                if len(try_size) != 4:
                    return
                size = struct.unpack("!I", try_size)[0]
                b_msg = self._dev.read(size)
                try:
                    if len(b_msg) > 0:
                        self.last_stamp_joy = time.time()
                        cmd = Command()
                        cmd.ParseFromString(b_msg)
                        # rospy.loginfo(f"Values received: {cmd}")
                        up_down = cmd.ud / float(CONST_I_127)
                        left_right = cmd.lr / float(CONST_I_127)
                        throttle = cmd.throttle / float(CONST_I_255)
                        throttle_diff = cmd.throttle_diff / float(CONST_I_127)
                        override_engage = cmd.override
                        arm = cmd.arm
                        # if arm:
                        #     rospy.loginfo(f"Arm received")
                        disarm = cmd.disarm
                        self.react_joystick(
                            override_engage,
                            throttle,
                            throttle_diff,
                            left_right,
                            up_down,
                            arm,
                            disarm,
                        )

                except Exception as e:
                    print(time.time())
                    print(e, ", skip")

                # todo: send back heartbeat ACK
                t_joy_since_last = time.time() - self.last_stamp_joy
                if t_joy_since_last < 0.5 and len(b_msg) > 0:
                    # send_heartbeat()
                    pass
            except Exception as e:
                print(e)
                self._dev = self._init_device()

    def react_joystick(
        self,
        override_engage: bool,
        throttle: float,
        throttle_diff: float,
        side_servo: float,
        front_servo: float,
        arm: bool,
        disarm: bool,
    ):
        # override
        msg = Bool()
        msg.data = override_engage
        self._pub_manual_override.publish(msg)
        # arm
        msg.data = arm
        self._pub_arm.publish(msg)
        # if arm:
        #     rospy.loginfo("---- Published arm")
        # disarm
        msg.data = disarm
        self._pub_disarm.publish(msg)

        controller_settings_msg = bool_array_stamped()
        controller_settings_msg.header.stamp = rospy.Time.now()
        # arm and disarm are event style values so we need register changes to these parameters in self._is_armed
        # and leave it unchanged otherwise
        self._is_armed = True if arm else False if disarm else self._is_armed
        is_in_autonomous_mode = not override_engage and self._is_armed
        controller_settings_msg.data = [self._is_armed, is_in_autonomous_mode]
        self._pub_controller_settings.publish(controller_settings_msg)

        # sanity
        assert 0.0 <= throttle <= 1.0
        assert -1.0 <= front_servo <= 1.0
        assert -1.0 <= side_servo <= 1.0

        # transfer servo values
        # msg_front = self._transfer_servo_range(front_servo)
        # msg_side = self._transfer_servo_range(side_servo)
        msg_front = front_servo
        msg_side = side_servo

        # make sure total thrust stays the same
        max_thr_diff_val = min(1.0 - throttle, throttle)
        # throttle_diff is a proportion [0.0, 1.0]
        throttle_diff_value = throttle_diff * max_thr_diff_val
        # rospy.loginfo(f"Diff: {throttle_diff}, {max_thr_diff_val}, {throttle_diff_value}")

        # form msgs
        msg = Float32MultiArray()
        msg.data.append(msg_front)
        msg.data.append(msg_side)
        msg.data.append(throttle)
        msg.data.append(throttle + throttle_diff_value)
        msg.data.append(throttle - throttle_diff_value)
        self._pub_manual_commands.publish(msg)

    def _transfer_servo_range(
        self,
        value: float,
        in_min: float = -1.0,
        in_max: float = 1.0,
        out_min: float = 0.0,
        out_max: float = 1.0,
    ) -> float:
        assert in_max >= in_min and out_max >= out_min
        assert in_min <= value <= in_max
        # todo: add possibility to invert

        in_range = in_max - in_min
        proportion = (value - in_min) / in_range
        out_range = out_max - out_min

        return out_min + out_range * proportion


if __name__ == "__main__":
    node = RockparaTelemetryNode()
    try:
        # Keep it spinning to keep the node alive
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
