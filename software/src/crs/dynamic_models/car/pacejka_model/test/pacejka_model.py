import sympy as sp
import yaml
import numpy as np
import os
from typing import List

from casadi import if_else, atan2


# def pacejka_model(constraints: Dict[str, float]):
def pacejka_model_vector(
    state: List[sp.Symbol],
    input: List[sp.Symbol],
    params: dict,
) -> sp.Matrix:
    x = state[0]
    y = state[1]
    yaw = state[2]
    vx = state[3]
    vy = state[4]
    omega = state[5]

    T = input[0]
    delta = input[1]

    #   Slip Angles
    #   Approximation of arctan(w/x) with a polynomial of degree 3
    #   for the discontinuity at x=0. This is needed to avoid
    #   numerical problems with the atan function. The approximation
    #   is applied for x < eps and has the same value and derivative at
    #   x=eps as the atan function.

    #   The polynomial is given by:
    #   g(x) = b*x + b*x*x*x;
    #   b = 1/2 * w/(eps^2+w^2) + 3/(2*eps)*arctan(w/eps)
    #   c = -1/(2*eps^3)* arctan(w/eps) - w/(2eps^2) * 1/(w*w + eps*eps)

    w_r = -vy + params["lr"] * omega
    b_r = 0.5 * w_r / (params["eps"] * params["eps"] + w_r * w_r) + 3 / (
        2 * params["eps"]
    ) * sp.atan(w_r / params["eps"])
    c_r = -1 / (2 * params["eps"] * params["eps"] * params["eps"]) * sp.atan(
        w_r / params["eps"]
    ) - w_r / (2 * params["eps"] * params["eps"]) * 1 / (
        w_r * w_r + params["eps"] * params["eps"]
    )
    g_r = b_r * vx + c_r * vx * vx * vx

    w_f = -vy - params["lf"] * omega
    b_f = 0.5 * w_f / (params["eps"] * params["eps"] + w_f * w_f) + 3 / (
        2 * params["eps"]
    ) * sp.atan(w_f / params["eps"])
    c_f = -1 / (2 * params["eps"] * params["eps"] * params["eps"]) * sp.atan(
        w_f / params["eps"]
    ) - w_f / (2 * params["eps"] * params["eps"]) * 1 / (
        w_f * w_f + params["eps"] * params["eps"]
    )
    g_f = b_f * vx + c_f * vx * vx * vx

    ar = sp.Piecewise(
        (g_r, vx < params["eps"]), (sp.atan2(-vy + params["lr"] * omega, vx), True)
    )
    af = sp.Piecewise(
        (g_f, vx < params["eps"]),
        (delta + sp.atan2(-vy - params["lf"] * omega, vx), True),
    )

    # forces on the wheels
    Fm = (params["Cm1"] - params["Cm2"] * vx) * T  # motor force
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
        + Fx_f * sp.cos(delta)
        - Fy_f * sp.sin(delta)
        + params["m"] * vy * omega
        + Ffriction
    )
    Fy = Fy_r + Fx_f * sp.sin(delta) + Fy_f * sp.cos(delta) - params["m"] * vx * omega
    Mz = (
        Fy_f * params["lf"] * sp.cos(delta)
        + Fx_f * params["lf"] * sp.sin(delta)
        - Fy_r * params["lr"]
    )

    f = sp.Matrix(
        [
            vx * sp.cos(yaw) - vy * sp.sin(yaw),
            vx * sp.sin(yaw) + vy * sp.cos(yaw),
            omega,
            Fx / params["m"],
            Fy / params["m"],
            Mz / params["I"],
        ]
    )

    return f
