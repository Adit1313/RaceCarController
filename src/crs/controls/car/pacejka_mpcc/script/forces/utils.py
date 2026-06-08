"""Function (Model Prediction, Cost, ...) for the forces solver.

The variables x, u and p have the following content:

    # States
    xp = x[0]
    yp = x[1]
    yaw = x[2]
    vx = x[3]
    vy = x[4]
    omega = x[5]
    T = x[6]
    delta = x[7]
    theta = x[8]

    # Inputs
    dT = u[0]
    ddelta = u[1]
    dtheta = u[2]

    # Parameters
    xd = p[0]
    yd = p[1]
    grad_xd = p[2]
    grad_yd = p[3]
    theta_hat = p[4]
    phi_d = p[5]
    Q1 = p[6]
    Q2 = p[7]
    R1 = p[8]
    R2 = p[9]
    R3 = p[10]
    q = p[11]
    lr = p[12]
    lf = p[13]
    m = p[14]
    I = p[15]
    Df = p[16]
    Cf = p[17]
    Bf = p[18]
    Dr = p[19]
    Cr = p[20]
    Br = p[21]
    Cm1 = p[22]
    Cm2 = p[23]
    Cd = p[24]
    Croll = p[25]
"""

import casadi
import math
from casadi import atan, sin, cos, atan2


def compute_phi(z, s, m):
    harm_arg = 2 * math.pi * s * z
    c = casadi.cos(harm_arg)
    s = casadi.sin(harm_arg)
    phi = casadi.vertcat(c, s)

    return phi


def continuous_dynamics(x, u, p):
    """Cont. Dynamics. x are the states, u are the inputs p are the parameters."""
    # States
    yaw = x[2]
    vx = x[3]
    vy = x[4]
    omega = x[5]
    T = x[6]
    delta = x[7]

    # Inputs
    dT = u[0]
    ddelta = u[1]
    dtheta = u[2]

    lr = p[12]
    lf = p[13]
    m = p[14]
    I = p[15]
    Df = p[16]
    Cf = p[17]
    Bf = p[18]
    Dr = p[19]
    Cr = p[20]
    Br = p[21]
    Cm1 = p[22]
    Cm2 = p[23]
    Cd = p[24]
    Croll = p[25]

    # dynamics
    Fx = (Cm1 - Cm2 * vx) * T - Cd * vx * vx - Croll
    beta = atan(vy / vx)
    ar = atan2(-vy + lr * omega, vx)
    af = delta + atan2(-vy - lf * omega, vx)
    Fr = Dr * sin(Cr * atan(Br * ar))
    Ff = Df * sin(Cf * atan(Bf * af))

    f_expl = casadi.vertcat(
        vx * cos(yaw) - vy * sin(yaw),
        vx * sin(yaw) + vy * cos(yaw),
        omega,
        1 / m * (Fx - Ff * sin(delta) + m * vy * omega),
        1 / m * (Fr + Ff * cos(delta) - m * vx * omega),
        1 / I * (Ff * lf * cos(delta) - Fr * lr),
        dT,
        ddelta,
        dtheta,
    )

    return f_expl


def cost_function_pacejka(z, p):
    """Cost function for the model. Z are states and inputs stacked together, p are parameters."""
    # States
    xp = z[0]
    yp = z[1]
    theta = z[8]

    # Inputs
    dT = z[9]
    ddelta = z[10]
    dtheta = z[11]
    # Parameters
    xd = p[0]
    yd = p[1]
    grad_xd = p[2]
    grad_yd = p[3]
    theta_hat = p[4]
    phi_d = p[5]
    Q1 = p[6]
    Q2 = p[7]
    R1 = p[8]
    R2 = p[9]
    R3 = p[10]
    q = p[11]

    # cost
    eC = casadi.sin(phi_d) * (xp - xd - grad_xd * (theta - theta_hat)) - casadi.cos(
        phi_d
    ) * (yp - yd - grad_yd * (theta - theta_hat))
    eL = -casadi.cos(phi_d) * (xp - xd - grad_xd * (theta - theta_hat)) - casadi.sin(
        phi_d
    ) * (yp - yd - grad_yd * (theta - theta_hat))

    c_eC = eC * eC * Q1
    c_eL = eL * eL * Q2
    c_theta = -q * theta
    c_dT = dT * dT * R1
    c_ddelta = ddelta * ddelta * R2
    c_dtheta = dtheta * dtheta * R3

    return c_eC + c_eL + c_theta + c_dT + c_ddelta + c_dtheta


def dynamics_RK4(z, p, freq):
    """Runge Kutta 4 Approximation of car dynamics. z are stacked states and inputs, p the parameters and freq the frequency"""
    x = z[0:9]
    u = z[9:12]

    dt = 1 / freq
    k1 = continuous_dynamics(x, u, p)
    k2 = continuous_dynamics(x + (dt / 2.0) * k1, u, p)
    k3 = continuous_dynamics(x + (dt / 2.0) * k2, u, p)
    k4 = continuous_dynamics(x + dt * k3, u, p)

    next_state = x + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

    return next_state


def inequality_constraint(z, p):
    x = z[0]
    y = z[1]
    xd = p[0]
    yd = p[1]
    h = (x - xd) * (x - xd) + (y - yd) * (y - yd)
    return h
