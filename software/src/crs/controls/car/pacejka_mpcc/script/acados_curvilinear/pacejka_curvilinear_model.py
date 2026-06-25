import math

import numpy as np
from casadi import *
from typing import Dict


def pacejka_curvilinear_model(constraints: Dict[str, float]):
    # define structs
    model = types.SimpleNamespace()
    constraint = types.SimpleNamespace()
    model_name = "pacejka_curvilinear_model"

    """ constraints """
    model.s_min = constraints["s_min"]
    model.s_max = constraints["s_max"]
    model.mu_min = constraints["mu_min"]
    model.mu_max = constraints["mu_max"]
    model.vx_min = constraints["vx_min"]
    model.vx_max = constraints["vx_max"]
    model.vy_min = constraints["vy_min"]
    model.vy_max = constraints["vy_max"]
    model.dr_min = constraints["dr_min"]
    model.dr_max = constraints["dr_max"]
    model.T_min = constraints["T_min"]
    model.T_max = constraints["T_max"]
    model.delta_min = constraints["delta_min"]
    model.delta_max = constraints["delta_max"]
    model.dT_min = constraints["dT_min"]
    model.dT_max = constraints["dT_max"]
    model.ddelta_min = constraints["ddelta_min"]
    model.ddelta_max = constraints["ddelta_max"]

    """ Dynamics """
    # CasADi - states
    s = SX.sym("s")
    d = SX.sym("d")
    mu = SX.sym("mu")
    vx = SX.sym("vx")
    vy = SX.sym("vy")
    r = SX.sym("r")
    T = SX.sym("T")
    delta = SX.sym("delta")
    x = vertcat(s, d, mu, vx, vy, r, T, delta)

    # CasADi - input
    dT = SX.sym("dT")
    ddelta = SX.sym("ddelta")
    u = vertcat(dT, ddelta)

    # xdot
    sdot = SX.sym("sdot")
    ddot = SX.sym("ddot")
    mudot = SX.sym("mudot")
    vxdot = SX.sym("vxdot")
    vydot = SX.sym("vydot")
    rdot = SX.sym("rdot")
    xdot = vertcat(sdot, ddot, mudot, vxdot, vydot, rdot, dT, ddelta)

    # algebraic variables
    z = vertcat([])

    # define params
    kappad = SX.sym("kappad")
    Q1 = SX.sym("Q1")
    Q2 = SX.sym("Q2")
    R1 = SX.sym("R1")
    R2 = SX.sym("R2")
    R3 = SX.sym("R3")
    q = SX.sym("q")
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
    gamma = SX.sym("gamma")
    eps = SX.sym("eps")
    car_width = SX.sym("car_width")
    wheel_radius = SX.sym("wheel_radius")

    # parameters
    p = vertcat(
        kappad,
        Q1,
        Q2,
        R1,
        R2,
        R3,
        q,
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
        gamma,
        eps,
        car_width,
        wheel_radius,
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
    w_r = -vy + lr * r
    b_r = 0.5 * w_r / (eps * eps + w_r * w_r) + 3 / (2 * eps) * atan(w_r / eps)
    c_r = -1 / (2 * eps * eps * eps) * atan(w_r / eps) - w_r / (2 * eps * eps) * 1 / (
        w_r * w_r + eps * eps
    )
    g_r = b_r * vx + c_r * vx * vx * vx

    w_f = -vy - lf * r
    b_f = 0.5 * w_f / (eps * eps + w_f * w_f) + 3 / (2 * eps) * atan(w_f / eps)
    c_f = -1 / (2 * eps * eps * eps) * atan(w_f / eps) - w_f / (2 * eps * eps) * 1 / (
        w_f * w_f + eps * eps
    )
    g_f = b_f * vx + c_f * vx * vx * vx

    ar = if_else(vx < eps, g_r, atan2(-vy + lr * r, vx))
    af = if_else(vx < eps, g_f, delta + atan2(-vy - lf * r, vx))

    # forces on the wheels
    Fm = (Cm1 - Cm2 * vx) * T  # motor force
    Ffriction = sign(vx) * (-Cd0 - Cd1 * vx - Cd2 * vx * vx)  # friction force

    Fx_f = Fm * (1 - gamma)  # front wheel force, x component
    Fx_r = Fm * gamma  # rear wheel force, x component

    Fy_f = Df * sin(Cf * atan(Bf * af))  # front wheel force, y component
    Fy_r = Dr * sin(Cr * atan(Br * ar))  # rear wheel force, y component

    # dynamics including apparent forces
    Fx = Fx_r + Fx_f * cos(delta) - Fy_f * sin(delta) + m * vy * r + Ffriction
    Fy = Fy_r + Fx_f * sin(delta) + Fy_f * cos(delta) - m * vx * r
    Mz = Fy_f * lf * cos(delta) + Fx_f * lf * sin(delta) - Fy_r * lr

    sdot = (vx * cos(mu) - vy * sin(mu)) / (1 - d * kappad)

    f_expl = vertcat(
        sdot,  # sdot
        vx * sin(mu) + vy * cos(mu),  # ddot
        r - kappad * sdot,  # mudot
        Fx / m,  # vxdot
        Fy / m,  # vydot
        Mz / I,  # rdot
        dT,  # Tdot
        ddelta,  # deltadot
    )

    beta_dyn = atan(vy / vx)
    beta_kin = atan(delta * lr / (lf + lr))

    # cost
    c_d = d * d * Q1
    c_mu = mu * mu * Q2
    c_s = -q * sdot
    c_dT = dT * dT * R1
    c_ddelta = ddelta * ddelta * R2
    c_b = (beta_dyn - beta_kin) * (beta_dyn - beta_kin) * R3

    model.cost_expr_ext_cost = c_d + c_mu + c_s + c_dT + c_ddelta + c_b

    # expression for the constraint h
    # track width constraint
    mu_abs = if_else(mu < 0, -mu, mu)
    d_cog2center = sin(mu) * (lf / 2 - lr / 2)
    car_length = lf + lr  # Approximate car length until we add a separate parameter
    d_max = (
        d
        + d_cog2center
        + sign(d) * (car_width / 2 * cos(mu_abs) + car_length / 2 * sin(mu_abs))
    )
    constraint.expr = vertcat(d_max)

    params = types.SimpleNamespace()
    params.kappad = kappad
    params.Q1 = Q1
    params.Q2 = Q2
    params.R1 = R1
    params.R2 = R2
    params.R3 = R3
    params.q = q
    params.lr = lr
    params.lf = lf
    params.m = m
    params.I = I
    params.Df = Df
    params.Cf = Cf
    params.Bf = Bf
    params.Dr = Dr
    params.Cr = Cr
    params.Br = Br
    params.Cm1 = Cm1
    params.Cm2 = Cm2
    params.Cd0 = Cd0
    params.Cd1 = Cd1
    params.Cd2 = Cd2
    params.gamma = gamma
    params.eps = eps
    params.car_width = car_width
    params.gamma = gamma

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
