import copy

import numpy as np
import casadi as ca

from .sensor_model import SensorModel

from typing import List


class MotionCaptureSensorModel(SensorModel):
    """Sensor model for a motion capture system that measures the full state vector."""

    def __init__(self, measured_states: List):
        """
        Initialize the motion capture sensor model.

        Args:
            measured_states: List of states to be measured.
        """
        self.measured_states = measured_states
        self.num_measurements = len(measured_states)

    def calc_y(
        self,
        x: np.array,
        u: np.array,
        numerical: bool = False,
        params: np.ndarray = None,
    ):
        if numerical:
            return np.array([x[state] for state in self.measured_states])
        else:
            return ca.vertcat(*[x[state] for state in self.measured_states])
