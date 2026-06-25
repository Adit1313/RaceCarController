import casadi as ca
from dataclasses import dataclass

@dataclass(frozen=True)
class PacejkaParams:
    lr: float; lf: float; m: float; I: float
    Df: float; Cf: float; Bf: float
    Dr: float; Cr: float; Br: float
    Cm1: float; Cm2: float
    Cd0: float; Cd1: float; Cd2: float
    gamma: float
    eps: float

def pacejka_rhs(x, u, p: PacejkaParams):
    # x = [pos_x, pos_y, yaw, v_x, v_y, yaw_rate]
    pos_x, pos_y, yaw, v_x, v_y, yaw_rate = x[0], x[1], x[2], x[3], x[4], x[5]
    torque, steer = u[0], u[1]

    lr, lf, m, I = p.lr, p.lf, p.m, p.I
    eps = p.eps

    # slip + forces 
    w_r = (-v_y + lr * yaw_rate)
    b_r = 0.5*w_r/(eps*eps + w_r*w_r) + 3/(2*eps)*ca.atan(w_r/eps)
    c_r = -1/(2*eps**3)*ca.atan(w_r/eps) - w_r/(2*eps*eps)*1/(w_r*w_r + eps*eps)
    g_r = b_r*v_x + c_r*v_x**3

    w_f = (-v_y - lf * yaw_rate)
    b_f = 0.5*w_f/(eps*eps + w_f*w_f) + 3/(2*eps)*ca.atan(w_f/eps)
    c_f = -1/(2*eps**3)*ca.atan(w_f/eps) - w_f/(2*eps*eps)*1/(w_f*w_f + eps*eps)
    g_f = b_f*v_x + c_f*v_x**3

    ar = ca.if_else(v_x < eps, g_r, ca.atan2(-v_y + lr*yaw_rate, v_x))
    af = ca.if_else(v_x < eps, g_f, steer + ca.atan2(-v_y - lf*yaw_rate, v_x))

    Fm = (p.Cm1 - p.Cm2 * v_x) * torque
    Ffriction = ca.sign(v_x) * (-p.Cd0 - p.Cd1*v_x - p.Cd2*v_x*v_x)

    Fx_f = (1 - p.gamma) * Fm
    Fx_r = p.gamma * Fm

    Fy_f = p.Df * ca.sin(p.Cf * ca.atan(p.Bf * af))
    Fy_r = p.Dr * ca.sin(p.Cr * ca.atan(p.Br * ar))

    Fx = Fx_r + Fx_f*ca.cos(steer) - Fy_f*ca.sin(steer) + m*v_y*yaw_rate + Ffriction
    Fy = Fy_r + Fx_f*ca.sin(steer) + Fy_f*ca.cos(steer) - m*v_x*yaw_rate
    Mz = Fy_f*lf*ca.cos(steer) + Fx_f*lf*ca.sin(steer) - Fy_r*lr

    pos_x_dot = v_x*ca.cos(yaw) - v_y*ca.sin(yaw)
    pos_y_dot = v_x*ca.sin(yaw) + v_y*ca.cos(yaw)
    yaw_dot = yaw_rate
    v_x_dot = Fx / m
    v_y_dot = Fy / m
    yaw_rate_dot = Mz / I

    return ca.vertcat(pos_x_dot, pos_y_dot, yaw_dot, v_x_dot, v_y_dot, yaw_rate_dot)

def rk4_step(f_rhs, x, u, dt):
    k1 = f_rhs(x, u)
    k2 = f_rhs(x + dt/2*k1, u)
    k3 = f_rhs(x + dt/2*k2, u)
    k4 = f_rhs(x + dt*k3, u)
    return x + dt/6*(k1 + 2*k2 + 2*k3 + k4)

def make_pacejka_functions(p: PacejkaParams):
    # CasADi's Python type stubs are incomplete; these calls are correct at runtime.
    x = ca.MX.sym("x", 6)  # type: ignore[call-arg]
    u = ca.MX.sym("u", 2)  # type: ignore[call-arg]
    dt = ca.MX.sym("dt")   # type: ignore[call-arg]

    rhs_expr = pacejka_rhs(x, u, p)
    f_rhs = ca.Function("pacejka_rhs", [x, u], [rhs_expr])

    x_next = rk4_step(lambda X,U: f_rhs(X,U), x, u, dt)
    f_rk4 = ca.Function("pacejka_rk4", [x, u, dt], [x_next])
    return f_rhs, f_rk4