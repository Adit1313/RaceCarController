# Calculates the gt jacobians using simpy.
import sympy as sp
import yaml
import numpy as np
import os
from typing import List


def get_measurement_vector(
    state: List[sp.Symbol], input: List[sp.Symbol], params: dict
) -> sp.Matrix:
    v_x = state[3]
    v_y = state[4]
    yaw_rate = state[5]

    steer = input[1]

    vx_fr = v_x + 0.5 * params["car_width"] * yaw_rate
    vy_fr = v_y + params["lf"] * yaw_rate
    vx_wheel_fr = vx_fr * sp.cos(steer) + vy_fr * sp.sin(steer)

    vx_fl = v_x - 0.5 * params["car_width"] * yaw_rate
    vy_fl = v_y + params["lf"] * yaw_rate
    vx_wheel_fl = vx_fl * sp.cos(steer) + vy_fl * sp.sin(steer)

    vx_wheel_rl = v_x - 0.5 * params["car_width"] * yaw_rate
    vx_wheel_rr = v_x + 0.5 * params["car_width"] * yaw_rate

    # Unit of wheel speed is rad/s
    # define f
    f = sp.Matrix(
        [
            vx_wheel_fl / params["wheel_radius"],
            vx_wheel_fr / params["wheel_radius"],
            vx_wheel_rl / params["wheel_radius"],
            vx_wheel_rr / params["wheel_radius"],
        ]
    )
    return f
