#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
import sys, os
import re
import numpy as np

from crs_msgs.msg import car_input, car_state_cart

from rospy_controllers import resolve_controller
from rospy_controllers.utils import (
    DataStreamer,
    DataStreamerThread,
    SolveTimeAverager,
    get_log_path,
)


class ControllerNode:
    def __init__(self):
        # Get the ~private namespace parameters from command line or launch file.
        controller_type = rospy.get_param("~controller_type")
        state_type = rospy.get_param("~state_type")
        input_type = rospy.get_param("~input_type")
        max_rate = rospy.get_param("~max_rate")

        try:
            model_params = {"model_params": rospy.get_param("~model_params")}
            rospy.loginfo("Loading model parameters from controller namespace")
        except:
            model_params = rospy.get_param("model")
            rospy.loginfo("Loading model parameters from model namespace")

        print(f"model params: {model_params}")

        self.controller = resolve_controller(controller_type, state_type, input_type)(
            rospy.get_param("track"),
            model_params,
            rospy.get_param("~controller_params"),
        )

        self.input_pub = rospy.Publisher("~control_input", car_input, queue_size=1)
        self.state_sub = rospy.Subscriber(
            "~state", car_state_cart, self.state_cb, queue_size=1
        )

        self.controller_execution_timer = rospy.Timer(
            period=rospy.Duration(1 / float(max_rate)), callback=self.controller_cb
        )

        self.solve_time_averager = SolveTimeAverager(
            ["total", "preparation", "feedback"], unit="ms"
        )
        self.first_iteration = True
        self.current_state = None

        self.terminate_after_n_solves = rospy.get_param("~terminate_after_n_solves")
        self.num_solves = 0

        # Set up logging threads
        buffer_size = 1000
        if self.terminate_after_n_solves is not None:
            buffer_size = self.terminate_after_n_solves

        self.data_streamers = {}
        self.log_properties = rospy.get_param("~log_controller_node_properties")
        if self.log_properties is not None:
            log_path = get_log_path()
            os.makedirs(log_path, exist_ok=True)
            datetime_str_day = re.search(r"experiment_(([0-9]+_?)+)", log_path).group(1)
            datetime_str_sec = os.path.basename(log_path)
            experiment_name = rospy.get_param("~experiment_name")

            for node_prop_name, node_prop_dict in self.log_properties.items():
                for log_name, properties in node_prop_dict.items():
                    log_file = os.path.join(
                        os.path.dirname(log_path),
                        f"{datetime_str_sec}_{datetime_str_day.replace('_', '-')}-{datetime_str_sec.replace('_', '-')}_{experiment_name}_{log_name}.csv",
                    )
                    self.data_streamers[log_name] = DataStreamer(
                        log_file, buffer_size=buffer_size
                    )

    def state_cb(self, msg):
        self.current_state = msg

    def controller_cb(self, timer_event):
        if self.current_state is None or (
            self.terminate_after_n_solves is not None
            and self.num_solves >= self.terminate_after_n_solves
        ):
            return

        # Feedback phase
        self.solve_time_averager.start_timer("total")
        self.solve_time_averager.start_timer("feedback")

        self.input_pub.publish(self.controller.get_input(self.current_state))
        _, avg_feedback_time = self.solve_time_averager.read_timer(
            "feedback", update_average=not self.first_iteration
        )

        # Preparation phase
        self.solve_time_averager.start_timer("preparation")
        if hasattr(self.controller, "rti_preparation_phase"):
            self.controller.rti_preparation_phase()

        _, avg_preparation_time = self.solve_time_averager.read_timer(
            "preparation", update_average=not self.first_iteration
        )
        _, avg_total_time = self.solve_time_averager.read_timer(
            "total", update_average=not self.first_iteration
        )

        # Logging
        if not self.first_iteration:
            rospy.loginfo_throttle(
                1,
                f"Total solve time: {avg_total_time:.3f} ms\tfeeback: {avg_feedback_time:.3f} ms\tprep: {avg_preparation_time:.3f}",
            )

        for node_prop_name, node_prop_dict in self.log_properties.items():
            node_prop = getattr(self, node_prop_name)
            for log_name, node_prop_props in node_prop_dict.items():
                # for log_name, data_streamer in self.data_streamers.items():
                num_list = [
                    str(n)
                    for node_prop_prop_name in node_prop_props
                    for n in np.atleast_1d(
                        getattr(
                            node_prop,
                            node_prop_prop_name,
                        )
                    )
                ]
                self.data_streamers[log_name].write(", ".join(num_list) + ",\n")

        self.first_iteration = False
        self.num_solves += 1

        # visualizer
        self.controller.publish_visualization()

        print(f"Number of solves: {self.num_solves}")
        if (
            self.terminate_after_n_solves is not None
            and self.num_solves >= self.terminate_after_n_solves
        ):
            print("Waiting for queue to empty")
            for data_streamer in self.data_streamers.values():
                data_streamer.terminate()
            print("Kill me", flush=True)


# Main function.
if __name__ == "__main__":
    # Initialize the node and name it.

    rospy.init_node("controller_node")

    try:
        controller_node = ControllerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
