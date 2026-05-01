# Calculates the gt jacobians using simpy.
import sympy as sp
import yaml
import numpy as np
import os
from typing import List


def get_measurement_vector(
    state: List[sp.Symbol], input: List[sp.Symbol], params: dict
) -> sp.Matrix:
    x = state[0]
    y = state[1]
    yaw = state[2]

    # Note: row x col
    rot = sp.Matrix([[sp.cos(yaw), -sp.sin(yaw)], [sp.sin(yaw), sp.cos(yaw)], [0, 0]])
    # Position of the car in world frame. pos_car is a 3x1 matrix
    pos_car = sp.Matrix([x, y, 0])
    # Positions of the sensors in world frame. pos_sensors is a 3x4 matrix
    pos_sensor = (rot * params["sensor_positions"]) + sp.Matrix.hstack(
        *[pos_car for _ in range(4)]
    )
    # Base station rotation matrix and position vector. rot_bs is a 3x3 matrix
    rot_bs = params["bs_rotation"].T
    # M is a 3x4 matrix that has the bs_position as each column
    M = sp.Matrix(
        [
            [
                params["bs_position"],
                params["bs_position"],
                params["bs_position"],
                params["bs_position"],
            ]
        ]
    )
    # Positions in base satation reference frame
    sbs = rot_bs * (pos_sensor - M)
    # Calculate angles. alpha is a 4x1 matrix
    alphas = sp.zeros(1, 4)
    for i in range(4):
        alphas[i] = sp.atan(sbs[1, i] / sbs[0, i])
    alpha = sp.zeros(1, 4)
    for i in range(4):
        alpha[i] = alphas[i] + sp.asin(
            (sbs[2, i] * sp.tan(params["light_plane_tilt"]))
            / ((sbs[0, i] ** 2 + sbs[1, i] ** 2) ** 0.5)
        )

    # Unit of wheel speed is rad/s
    # define f
    f = sp.Matrix([alpha[0], alpha[1], alpha[2], alpha[3]])
    return f
