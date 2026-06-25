import copy

import numpy as np
import casadi as ca

from .sensor_model import SensorModel


class WheelEncoderSensorModel(SensorModel):
    """Sensor model for wheel encoders."""

    num_measurements = 4

    def __init__(self, wheel_radius_param, lr_param, lf_param, width_param):
        """
        Initialize the wheel encoder sensor model.

        Args:
            wheel_radius_param: Key of the parameter for the wheel radius
            lr_param: Key of the parameter for lr
            lf_param: Key of the parameter for lf
            width_param: Key of the parameter for the width
        """
        self.wheel_radius_param = wheel_radius_param
        self.lr_param = lr_param
        self.lf_param = lf_param
        self.width_param = width_param

    def calc_y(
        self,
        x: np.array,
        u: np.array,
        numerical: bool = False,
        params: np.ndarray = None,
    ):
        vx = x[3]
        vy = x[4]
        omega = x[5]

        delta = u[1]
        p_width = params[self.width_param]
        p_lr = params[self.lr_param]
        p_lf = params[self.lf_param]
        p_wr = params[self.wheel_radius_param]

        # speed of each wheel for wheel encoder model
        vx_rr = vx + 0.5 * p_width * omega
        vx_rl = vx - 0.5 * p_width * omega

        vx_fr = vx + 0.5 * p_width * omega
        vy_fr = vy + p_lf * omega

        vx_fl = vx - 0.5 * p_width * omega
        vy_fl = vy + p_lf * omega

        vx_wheel_fr = vx_fr * ca.cos(delta) + vy_fr * ca.sin(delta)
        vx_wheel_fl = vx_fl * ca.cos(delta) + vy_fl * ca.sin(delta)
        vx_wheel_rl = vx_rl
        vx_wheel_rr = vx_rr

        if numerical:
            return np.array([vx_wheel_fl, vx_wheel_fr, vx_wheel_rl, vx_wheel_rr]) / p_wr
        else:
            return ca.vertcat(vx_wheel_fl, vx_wheel_fr, vx_wheel_rl, vx_wheel_rr) / p_wr
