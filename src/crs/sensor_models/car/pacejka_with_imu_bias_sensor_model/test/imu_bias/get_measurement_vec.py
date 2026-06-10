# Calculates the gt jacobians using simpy.
import sympy as sp
import yaml
import numpy as np
import os
from typing import List


def get_measurement_vector(
    state: List[sp.Symbol], input: List[sp.Symbol], params: dict
) -> sp.Matrix:
    vx = state[3]
    vy = state[4]
    yaw_rate = state[5]
    bias_ax = state[6]
    bias_ay = state[7]
    bias_yaw_rate = state[8]

    torque = input[0]
    steer = input[1]

    # Intermediate function values
    w_r = -vy + params["lr"] * yaw_rate
    b_r = 0.5 * w_r / (params["eps"] * params["eps"] + w_r * w_r) + 3 / (
        2 * params["eps"]
    ) * sp.atan(w_r / params["eps"])
    c_r = -1 / (2 * params["eps"] * params["eps"] * params["eps"]) * sp.atan(
        w_r / params["eps"]
    ) - w_r / (2 * params["eps"] * params["eps"]) * 1 / (
        w_r * w_r + params["eps"] * params["eps"]
    )
    g_r = b_r * vx + c_r * vx * vx * vx

    w_f = -vy - params["lf"] * yaw_rate
    b_f = 0.5 * w_f / (params["eps"] * params["eps"] + w_f * w_f) + 3 / (
        2 * params["eps"]
    ) * sp.atan(w_f / params["eps"])
    c_f = -1 / (2 * params["eps"] * params["eps"] * params["eps"]) * sp.atan(
        w_f / params["eps"]
    ) - w_f / (2 * params["eps"] * params["eps"]) * 1 / (
        w_f * w_f + params["eps"] * params["eps"]
    )
    g_f = b_f * vx + c_f * vx * vx * vx

    # Slip Angles
    ar = sp.Piecewise(
        (g_r, vx < params["eps"]), (sp.atan2(-vy + params["lr"] * yaw_rate, vx), True)
    )
    af = sp.Piecewise(
        (g_f, vx < params["eps"]),
        (steer + sp.atan2(-vy - params["lf"] * yaw_rate, vx), True),
    )

    # Forces on the wheels
    Fm = (params["Cm1"] - params["Cm2"] * vx) * torque  # motor force
    Ffriction = (
        -params["Cd0"] - params["Cd1"] * vx - params["Cd2"] * vx * vx
    )  # friction force

    Fx_f = Fm * (1 - params["gamma"])  # front wheel force, x component
    Fx_r = Fm * params["gamma"]  # rear wheel force, x component

    Fy_f = params["Df"] * sp.sin(
        params["Cf"] * sp.atan(params["Bf"] * af)
    )  # front wheel force, y component
    Fy_r = params["Dr"] * sp.sin(
        params["Cr"] * sp.atan(params["Br"] * ar)
    )  # rear wheel force, y component

    # dynamics including apparent forces
    Fx = (
        Fx_r
        + Fx_f * sp.cos(steer)
        - Fy_f * sp.sin(steer)
        + params["m"] * vy * yaw_rate
        + Ffriction
    )
    Fy = (
        Fy_r + Fx_f * sp.sin(steer) + Fy_f * sp.cos(steer) - params["m"] * vx * yaw_rate
    )

    # Output of f(state, control)
    v_x_dot = Fx / params["m"]
    v_y_dot = Fy / params["m"]

    # define f
    f = sp.Matrix([v_x_dot + bias_ax, v_y_dot + bias_ay, yaw_rate + bias_yaw_rate])
    return f
