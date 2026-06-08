import math
import numpy as np
from casadi import *
from typing import Dict


def kinematic_model_tracking_mpc(constraints: Dict[str, float]):
    # define structs
    model = types.SimpleNamespace()
    constraint = types.SimpleNamespace()
    model_name = "kinematic_model_tracking_mpc"

    """ constraints """
    model.x_min = constraints["x_min"]
    model.x_max = constraints["x_max"]
    model.y_min = constraints["y_min"]
    model.y_max = constraints["y_max"]
    model.yaw_min = constraints["yaw_min"]
    model.yaw_max = constraints["yaw_max"]
    model.v_min = constraints["v_min"]
    model.v_max = constraints["v_max"]
    model.delta_min = constraints["delta_min"]
    model.delta_max = constraints["delta_max"]
    model.T_min = constraints["T_min"]
    model.T_max = constraints["T_max"]
    model.ddelta_min = constraints["ddelta_min"]
    model.ddelta_max = constraints["ddelta_max"]
    model.dT_min = constraints["dT_min"]
    model.dT_max = constraints["dT_max"]

    """ Dynamics """
    # CasADi - states
    xp = SX.sym("xp")
    yp = SX.sym("yp")
    yaw = SX.sym("yaw")
    v = SX.sym("v")
    T = SX.sym("T")
    delta = SX.sym("delta")
    x = vertcat(xp, yp, yaw, v, T, delta)

    # CasADi - input
    dT = SX.sym("dT")
    ddelta = SX.sym("ddelta")
    u = vertcat(dT, ddelta)

    # xdot
    xpdot = SX.sym("xpdot")
    ypdot = SX.sym("ypdot")
    yawdot = SX.sym("yawdot")
    vdot = SX.sym("vdot")
    xdot = vertcat(xpdot, ypdot, yawdot, vdot, dT, ddelta)

    # algebraic variables
    z = vertcat([])

    # define params
    xd = SX.sym("xd")
    yd = SX.sym("yd")
    Q1 = SX.sym("Q1")
    Q2 = SX.sym("Q2")
    R1 = SX.sym("R1")
    R2 = SX.sym("R2")
    lr = SX.sym("lr")
    lf = SX.sym("lf")
    a = SX.sym("a")
    b = SX.sym("b")
    tau = SX.sym("tau")

    # parameters
    p = vertcat(
        xd,
        yd,
        Q1,
        Q2,
        R1,
        R2,
        lr,
        lf,
        a,
        b,
        tau,
    )

    beta = atan2(tan(delta) * lr, lf + lr)
    u_kin = a * T + b
    v_x = v * cos(yaw + beta)
    v_y = v * sin(yaw + beta)

    f_expl = vertcat(
        v_x,
        v_y,
        v * sin(beta) / lr,
        -1 / tau * v + 1 / tau * u_kin,
        dT,
        ddelta,
    )

    # cost
    eX = xp - xd
    eY = yp - yd

    c_eX = eX * eX * Q1
    c_eY = eY * eY * Q2
    c_dT = dT * dT * R1
    c_ddelta = ddelta * ddelta * R2

    model.cost_expr_ext_cost = c_eX + c_eY + c_dT + c_ddelta

    # nonlinear track constraints
    # constraint.expr = vertcat(radius_sq) - not required at the moment

    params = types.SimpleNamespace()
    params.xd = xd
    params.yd = yd
    params.Q1 = Q1
    params.Q2 = Q2
    params.R1 = R1
    params.R2 = R2
    params.lr = lr
    params.lf = lf
    params.a = a
    params.b = b
    params.tau = tau

    model.f_impl_expr = xdot - f_expl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    model.z = z
    model.p = p
    model.name = model_name
    model.params = params

    # return model, constraint
    return model, constraint
