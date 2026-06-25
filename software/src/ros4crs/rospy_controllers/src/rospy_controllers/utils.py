# ------------------------------ hash file utils ------------------------------
import hashlib


def check_file_changes(
    files: list, hashfile_name: str, rewrite_hash=False, rewrite_hash_if_changed=True
):
    curr_hash = get_build_file_hashes(files)
    try:
        with open(hashfile_name, "r", encoding="utf-8") as hashfile:
            last_hash = hashfile.read()
            hash_changed = last_hash != curr_hash
    except FileNotFoundError:
        hash_changed = True

    if rewrite_hash or (hash_changed and rewrite_hash_if_changed):
        with open(hashfile_name, "w", encoding="utf-8") as hashfile:
            hashfile.write(curr_hash)

    return hash_changed


def get_build_file_hashes(files):
    solver_code = b""
    for file in files:
        with open(file, "rb") as gen_file:
            solver_code += gen_file.read()

    return hashlib.md5(solver_code).hexdigest()


# ------------------------------ logging utils ------------------------------

from collections import deque
from time import sleep, time, perf_counter
import threading
import queue
import os
from datetime import datetime


def get_log_path():
    this_file_dir = os.path.dirname(os.path.abspath(__file__))
    current_experiment_path_file = os.path.join(
        this_file_dir, "../../../crs_launch/script/current_experiment_path.txt"
    )

    try:
        with open(current_experiment_path_file, "r") as f:
            log_path_for_copy_configs = f.read().strip("\n")
    except FileNotFoundError:
        print(f"File {current_experiment_path_file} not found. Creating new log path.")
        datetime_now = datetime.now()
        # now=experiment_$(date +"%Y_%m_%d")
        now = datetime_now.strftime("experiment_%Y_%m_%d")
        # $(date +"%H_%M_%S")
        time = datetime_now.strftime("%H_%M_%S")
        # TODO: get arg_type passed to crs.launch (write to other file?)
        arg_type = "sim"

        log_path_for_copy_configs = os.path.join(
            this_file_dir, "../../../../../experiments", now, arg_type, time
        )

    return log_path_for_copy_configs


class RecursiveAverager:
    """Computes a recursive average without needing to keep track of past data."""

    def __init__(self):
        self._n = 0
        self._avg = 0

    def update(self, x):
        """Update recursive average with a new value"""
        self._n += 1
        self._avg = ((self._n - 1) * self._avg + x) / self._n
        return self._avg


class MovingAverager:
    """Computes a moving average of a quantity."""

    def __init__(self, window=100):
        self._queue = deque(maxlen=window)
        self._prev_ma = None

    def update(self, value):
        """Update moving average with a new value"""
        self._queue.append(value)
        ma = sum(self._queue) / len(self._queue)
        self._prev_ma = ma
        return ma

    @property
    def average(self) -> float:
        """Current moving average"""
        return self._prev_ma


class DictRecursiveAverager:
    def __init__(self):
        self._n = 0
        self._avg = {}

    def update(self, x):
        self._n += 1
        for key, val in x.items():
            self._avg[key] = ((self._n - 1) * self._avg.get(key, 0) + val) / self._n

        return self._avg


class SolveTimeAverager:
    def __init__(self, timer_names, unit="s"):
        self.unit_conversion = {
            "s": 1e0,
            "ms": 1e3,
            "us": 1e6,
            "ns": 1e9,
        }
        self.unit = unit
        self.start_time_dict = {}
        self.averager_dict = {}
        for timer_name in timer_names:
            self.start_time_dict[timer_name] = 0.0
            self.averager_dict[timer_name] = RecursiveAverager()
            setattr(self, timer_name, 0.0)

    def start_timer(self, timer_name):
        self.start_time_dict[timer_name] = perf_counter()

    def read_timer(self, timer_name, update_average=True):
        now = perf_counter()
        new_datum = self.unit_conversion[self.unit] * (
            now - self.start_time_dict[timer_name]
        )
        new_average = None
        if update_average:
            new_average = self.averager_dict[timer_name].update(new_datum)
        setattr(self, timer_name, new_datum)

        return new_datum, new_average


class ThrottledPrinter:
    def __init__(self, throttle_time=5, should_print=True):
        self._throttle_time = throttle_time
        self._prev_print_time = 0
        self._should_print = should_print

    def print(self, thing_to_print):
        curr_time = time()
        if (
            self._should_print
            and curr_time - self._prev_print_time > self._throttle_time
        ):
            self._prev_print_time = curr_time
            print(thing_to_print)


class DataStreamer:
    def __init__(self, file_path, buffer_size=0):
        self.queue = queue.Queue()
        self.file_path = file_path
        self.is_finished = False
        self.lines_in_buffer = 0
        self.buffer_size = buffer_size
        self.thread = threading.Thread(target=self.write_to_file, daemon=True)
        self.thread.start()
        print(f"Thread started for {file_path}")

    def write_to_file(self):
        with open(self.file_path, "a") as f:
            while True:
                while self.lines_in_buffer >= self.buffer_size:
                    line = self.queue.get()
                    if line is None:
                        self.is_finished = True
                        break
                    print(f"Printing line {line}")
                    f.write(line)
                    self.queue.task_done()

                if self.is_finished:
                    break

                print("Sleeping 1sec")
                sleep(1)

    def write(self, line):
        self.queue.put(line)
        self.lines_in_buffer += 1
        # if self.lines_in_buffer >= self.buffer_size:
        #     self.write_buffer_to_file()

    def terminate(self):
        self.queue.put(None)
        self.lines_in_buffer += 1
        self.thread.join()


class DataStreamerThread(threading.Thread):
    def __init__(self, file_path):
        super().__init__()
        self.file_path = file_path
        self.queue = queue.Queue()
        self.is_finished = False

    def run(self):
        with open(self.file_path, "a") as f:
            while True:
                line = self.queue.get_nowait()
                if line is None and not self.is_finished:
                    sleep(0.01)
                    print("Waiting")
                    continue
                elif line is None and self.is_finished:
                    print("Finished writing to file")
                    break
                print(f"Printing line {line}")
                f.write(line)
                self.queue.task_done()

    def write(self, line):
        print("Writing to queue")
        self.queue.put(line)


# ------------------------------ track utils ------------------------------
import numpy as np
import math


def unwrap_yaw_angle(tangent_angle):
    tangent_angle_unwrapped = [0] * 2 * len(tangent_angle)
    for i in range(len(tangent_angle)):
        tangent_angle_unwrapped[i] = tangent_angle[i]
        tan_ang = tangent_angle[i]
        if i == 0:
            tangent_angle_unwrapped[i] = tan_ang
        else:
            diff = tan_ang - tangent_angle_unwrapped[i - 1]
            if abs(diff) < 1.0:
                tangent_angle_unwrapped[i] = tan_ang
            else:
                tangent_angle_unwrapped[i] = math.pi + (math.pi + tan_ang)

    for i in range(len(tangent_angle), len(tangent_angle_unwrapped)):
        tangent_angle_unwrapped[i] = (
            tangent_angle_unwrapped[i - len(tangent_angle)] + 2.0 * math.pi
        )

    return tangent_angle_unwrapped


def get_closest_track_point_idx(
    pos_x: float, pos_y: float, track_x: np.array, track_y: np.array
):
    # euclidean distance for all points
    dist = (pos_x - track_x) ** 2 + (pos_y - track_y) ** 2
    dist_min = np.argmin(dist)

    return dist_min
