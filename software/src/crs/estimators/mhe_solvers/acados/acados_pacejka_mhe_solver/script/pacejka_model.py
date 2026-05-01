import math
import numpy as np
from casadi import *
from typing import Dict
from acados_template import AcadosModel


def pacejka_model(constraints: Dict[str, float]):
    # define structs
    model = types.SimpleNamespace()
    constraint = types.SimpleNamespace()
    model_name = "pacejka_model"

    """ constraints """
    model.x_min = constraints["x_min"]
    model.x_max = constraints["x_max"]
    model.y_min = constraints["y_min"]
    model.y_max = constraints["y_max"]
    model.yaw_min = constraints["yaw_min"]
    model.yaw_max = constraints["yaw_max"]
    model.vx_min = constraints["vx_min"]
    model.vx_max = constraints["vx_max"]
    model.vy_min = constraints["vy_min"]
    model.vy_max = constraints["vy_max"]
    model.dyaw_min = constraints["dyaw_min"]
    model.dyaw_max = constraints["dyaw_max"]
    model.delta_min = constraints["delta_min"]
    model.delta_max = constraints["delta_max"]
    model.T_min = constraints["T_min"]
    model.T_max = constraints["T_max"]
    model.theta_min = constraints["theta_min"]
    model.theta_max = constraints["theta_max"]
    model.ddelta_min = constraints["ddelta_min"]
    model.ddelta_max = constraints["ddelta_max"]
    model.dT_min = constraints["dT_min"]
    model.dT_max = constraints["dT_max"]

    model.w_x_min = constraints["w_x_min"]
    model.w_x_max = constraints["w_x_max"]
    model.w_y_min = constraints["w_y_min"]
    model.w_y_max = constraints["w_y_max"]
    model.w_psi_min = constraints["w_psi_min"]
    model.w_psi_max = constraints["w_psi_max"]
    model.w_vx_min = constraints["w_vx_min"]
    model.w_vx_max = constraints["w_vx_max"]
    model.w_vy_min = constraints["w_vy_min"]
    model.w_vy_max = constraints["w_vy_max"]
    model.w_omega_min = constraints["w_omega_min"]
    model.w_omega_max = constraints["w_omega_max"]

    """ Dynamics """
    # CasADi - states
    xp = SX.sym("xp")
    yp = SX.sym("yp")
    yaw = SX.sym("yaw")
    vx = SX.sym("vx")
    vy = SX.sym("vy")
    omega = SX.sym("omega")
    x = vertcat(xp, yp, yaw, vx, vy, omega)

    # set up control input variable
    T = SX.sym("T")
    delta = SX.sym("delta")
    u = vertcat(T, delta)

    # xdot
    xpdot = SX.sym("xpdot")
    ypdot = SX.sym("ypdot")
    yawdot = SX.sym("yawdot")
    vxdot = SX.sym("vxdot")
    vydot = SX.sym("vydot")
    omegadot = SX.sym("omegadot")
    xdot = vertcat(xpdot, ypdot, yawdot, vxdot, vydot, omegadot)

    # algebraic variables
    z = vertcat([])

    # Regularization Params
    # TODO

    # Model params
    lr = SX.sym("lr")
    lf = SX.sym("lf")
    m = SX.sym("m")
    I = SX.sym("I")
    Df = SX.sym("Df")
    Cf = SX.sym("Cf")
    Bf = SX.sym("Bf")
    Dr = SX.sym("Dr")
    Cr = SX.sym("Cr")
    Br = SX.sym("Br")
    Cm1 = SX.sym("Cm1")
    Cm2 = SX.sym("Cm2")
    Cd0 = SX.sym("Cd0")
    Cd1 = SX.sym("Cd1")
    Cd2 = SX.sym("Cd2")

    car_width = SX.sym("car_width")
    wheel_radius = SX.sym("wheel_radius")
    gamma = SX.sym("gamma")
    eps = SX.sym("eps")

    # parameters
    p = vertcat(
        lr,
        lf,
        m,
        I,
        Df,
        Cf,
        Bf,
        Dr,
        Cr,
        Br,
        Cm1,
        Cm2,
        Cd0,
        Cd1,
        Cd2,
        car_width,
        wheel_radius,
        T,
        delta,
        gamma,
        eps,
    )

    #   Slip Angles
    #   Approximation of arctan(w/x) with a polynomial of degree 3
    #   to overcome the discontinuity at x=0. This is needed to avoid
    #   numerical problems with the atan function. The approximation
    #   is applied for x < eps and has the same value and derivative at
    #   x=eps as the atan function.

    #   The polynomial is given by:
    #   g(x) = b*x + c*x*x*x;
    #   b = 1/2 * w/(eps^2+w^2) + 3/(2*eps)*arctan(w/eps)
    #   c = -1/(2*eps^3)* arctan(w/eps) - w/(2eps^2) * 1/(w*w + eps*eps)

    w_r = -vy + lr * omega
    b_r = 0.5 * w_r / (eps * eps + w_r * w_r) + 3 / (2 * eps) * atan(w_r / eps)
    c_r = -1 / (2 * eps * eps * eps) * atan(w_r / eps) - w_r / (2 * eps * eps) * 1 / (
        w_r * w_r + eps * eps
    )
    g_r = b_r * vx + c_r * vx * vx * vx

    w_f = -vy - lf * omega
    b_f = 0.5 * w_f / (eps * eps + w_f * w_f) + 3 / (2 * eps) * atan(w_f / eps)
    c_f = -1 / (2 * eps * eps * eps) * atan(w_f / eps) - w_f / (2 * eps * eps) * 1 / (
        w_f * w_f + eps * eps
    )
    g_f = b_f * vx + c_f * vx * vx * vx

    ar = if_else(vx < eps, g_r, atan2(-vy + lr * omega, vx))
    af = if_else(vx < eps, g_f, delta + atan2(-vy - lf * omega, vx))

    # forces on the wheels
    Fm = (Cm1 - Cm2 * vx) * T  # motor force
    Ffriction = sign(vx) * (-Cd0 - Cd1 * vx - Cd2 * vx * vx)  # friction force

    Fx_f = Fm * (1 - gamma)  # front wheel force, x component
    Fx_r = Fm * gamma  # rear wheel force, x component

    Fy_f = Df * sin(Cf * atan(Bf * af))  # front wheel force, y component
    Fy_r = Dr * sin(Cr * atan(Br * ar))  # rear wheel force, y component

    # dynamics including apparent forces
    Fx = Fx_r + Fx_f * cos(delta) - Fy_f * sin(delta) + m * vy * omega + Ffriction
    Fy = Fy_r + Fx_f * sin(delta) + Fy_f * cos(delta) - m * vx * omega
    Mz = Fy_f * lf * cos(delta) + Fx_f * lf * sin(delta) - Fy_r * lr

    # set up noise param
    w_x = SX.sym("w_x")
    w_y = SX.sym("w_y")
    w_psi = SX.sym("w_psi")
    w_vx = SX.sym("w_vx")
    w_vy = SX.sym("w_vy")
    w_omega = SX.sym("w_omega")
    w = vertcat(w_x, w_y, w_psi, w_vx, w_vy, w_omega)

    # state dynamics
    f_expl = vertcat(
        vx * cos(yaw) - vy * sin(yaw) + w_x,
        vx * sin(yaw) + vy * cos(yaw) + w_y,
        omega + w_psi,
        Fx / m + w_vx,
        Fy / m + w_vy,
        Mz / I + w_omega,
    )

    f_impl = f_expl - xdot

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = w
    model.z = z
    model.p = p
    model.name = model_name

    return model, constraint
