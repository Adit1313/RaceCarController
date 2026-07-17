from __future__ import annotations
from dataclasses import asdict
from typing import Dict
import casadi as ca
import numpy as np
import pacejka_model as pm
from pacejka_track import build_track_interpolants, build_track_geometry, load_track


def solve_time_optimal_pacejka_lap(
    track_file: str,
    params: pm.Params,
    *,
    N: int = 300,
    track_width: float = 0.6,
    vehicle_width: float = 0.05,
    warm_up: bool= False,
    torque_min: float = 0,
    torque_max: float = 1.0,
    steer_min: float = -0.3,
    steer_max: float = 0.3,
    vx_max: float = 3.5,
    vx_min = 0.4,
    vy_abs_max: float = 1,
    yaw_rate_abs_max: float = 4,
    T_min: float = 0.5,
    T_max: float = 30.0,
    w_u: float = 1e-4,
    w_du: float = 1e-3,
    hessian_approximation: str = "limited-memory",
    nlp_scaling_method: str = "gradient-based",
    ipopt_print: int = 3,
    max_iter: int = 1000,
) -> Dict:
    """Solve a periodic fastest-lap OCP using Pacejka dynamics.

    Returns a dict with arrays for reference export.
    """

    # build track geometry
    track = load_track(track_file)
    geo = build_track_geometry(track, track_width=track_width)
    L = float(geo["L"])

    margin = vehicle_width / 2.0
    n_max = float(track_width / 2.0 - margin)
    if n_max <= 0:
        raise ValueError("Track too narrow for vehicle_width")

    track_f = build_track_interpolants(geo)
    f_rhs, _ = pm.make_pacejka_functions(params)


    # optimization problem
    opti = ca.Opti()

    X = opti.variable(7, N + 1)  # [x,y,yaw,vx,vy,omega,theta]
    U = opti.variable(2, N)      # [torque, steer]
    T = opti.variable()
    dt = T / N

    # unpack state and inputs
    x = X[0, :]
    y = X[1, :]
    yaw = X[2, :]
    vx = X[3, :]
    vy = X[4, :]
    omega = X[5, :]
    theta = X[6, :]

    torque = U[0, :]
    steer = U[1, :]

    # state and input bounds
    opti.subject_to(opti.bounded(T_min, T, T_max))
    opti.subject_to(opti.bounded(0.0, theta, L))
    opti.subject_to(opti.bounded(vx_min, vx, vx_max))
    opti.subject_to(opti.bounded(-vy_abs_max, vy, vy_abs_max))
    opti.subject_to(opti.bounded(-yaw_rate_abs_max, omega, yaw_rate_abs_max))
    opti.subject_to(opti.bounded(torque_min, torque, torque_max))
    opti.subject_to(opti.bounded(steer_min, steer, steer_max))

    # bound on throttle delta:
    if N > 1:
        dtorque = torque[1:] - torque[:-1]
        opti.subject_to(opti.bounded(-0.2, dtorque, 0.2)) # +/- 0.2 as suggested by Yunfan


    # initial constraints
    opti.subject_to(theta[0] == 0.0)
    opti.subject_to(x[0] == float(geo["rx"][0]))
    psi0 = float(np.arctan2(geo["ty"][0], geo["tx"][0]))
    opti.subject_to(yaw[0] == psi0)    
    
    # Lap completion in theta
    opti.subject_to(theta[-1] == L)

    # Distinguish between fast periodic or warm-up lap

    # for fast periodic lap:
    if warm_up==False:
        opti.subject_to(y[0] >= 2.5)     # keep y[0] free
        opti.subject_to(y[0] <= 3.5)

        # Periodicity of states
        opti.subject_to(x[-1] == x[0])
        opti.subject_to(y[-1] == y[0])
        opti.subject_to(vx[0] == vx[-1])
        opti.subject_to(vy[0] == vy[-1])
        opti.subject_to(omega[-1] == omega[0])

        # Yaw periodicity
        opti.subject_to(ca.sin(yaw[-1]) == ca.sin(yaw[0]))
        opti.subject_to(ca.cos(yaw[-1]) == ca.cos(yaw[0]))

        # Time optimality
        cost = T

    # for warm up lap
    else:
        # initial constraints
        opti.subject_to(y[0] == 3) # fix y[0] for warmup lap
        opti.subject_to(vx[0] == 0.4)
        opti.subject_to(vy[0] == 0.0)

        # terminal constraints
        opti.subject_to(vx[-1] == 2.75)
        opti.subject_to(y[-1] == 3.125)
        opti.subject_to(y[0] == 3.0)
        opti.subject_to(vy[-1] == 0.19)

        opti.subject_to(x[-1] == x[0])

        # No time optimality:
        cost = 0

    def lateral_offset_and_heading(theta_k, x_k, y_k):
        xc_k = track_f["xc"](theta_k)
        yc_k = track_f["yc"](theta_k)
        tx_k = track_f["tx"](theta_k)
        ty_k = track_f["ty"](theta_k)
        kappa_k = track_f["kappa"](theta_k) # curvature
        psi_c_k = ca.atan2(ty_k, tx_k) # yaw of centerline
        n_k = (x_k - xc_k) * (-ty_k) + (y_k - yc_k) * (tx_k) # lateral offset
        return n_k, psi_c_k, kappa_k

    # extend existing dynamic bicyle model by progress state theta
    def rhs7(Xk, Uk):
        x6 = Xk[0:6]
        theta_k = Xk[6]

        x6_dot = f_rhs(x6, Uk)

        n_k, psi_c_k, kappa_k = lateral_offset_and_heading(theta_k, Xk[0], Xk[1])
        e_psi = Xk[2] - psi_c_k # heading error
        theta_dot = (Xk[3] * ca.cos(e_psi) - Xk[4] * ca.sin(e_psi)) #/ (1 - kappa_k * n_k) removed for faster convergence

        return ca.vertcat(x6_dot, theta_dot)

    # new rk4 step for the new 7-state model
    def rk4_step(Xk, Uk, dt_):
        k1 = rhs7(Xk, Uk)
        k2 = rhs7(Xk + dt_ / 2 * k1, Uk)
        k3 = rhs7(Xk + dt_ / 2 * k2, Uk)
        k4 = rhs7(Xk + dt_ * k3, Uk)
        return Xk + dt_ / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

    # Dynamics + track constraints
    for k in range(N):
        opti.subject_to(X[:, k + 1] == rk4_step(X[:, k], U[:, k], dt))
        n_k, _, _ = lateral_offset_and_heading(theta[k], x[k], y[k])
        opti.subject_to(opti.bounded(-n_max, n_k, n_max))  
    n_N, _, _ = lateral_offset_and_heading(theta[-1], x[-1], y[-1])
    opti.subject_to(opti.bounded(-n_max, n_N, n_max))  

    # Objective regularization
    cost += w_u * dt * ca.sumsqr(U)
    if N > 1:
        cost += w_du * dt * ca.sumsqr(U[:, 1:] - U[:, :-1])
    opti.minimize(cost)

    # Initial guess: centerline with vx = 0.8*vx_max in T=5s
    theta_guess = np.linspace(0.0, L, N + 1)

    # Use periodic interpolants for a consistent closed-loop guess
    x_guess = np.array([float(track_f["xc"](i)) for i in theta_guess], dtype=float) 
    y_guess = np.array([float(track_f["yc"](i)) for i in theta_guess], dtype=float)
    tx_guess = np.array([float(track_f["tx"](i)) for i in theta_guess], dtype=float)
    ty_guess = np.array([float(track_f["ty"](i)) for i in theta_guess], dtype=float)
    yaw_guess = np.unwrap(np.arctan2(ty_guess, tx_guess))

    vx_guess = np.ones(N + 1) * 0.8 * vx_max
    vy_guess = np.zeros(N + 1)
    kap_guess = np.array([float(track_f["kappa"](i)) for i in theta_guess], dtype=float)
    omega_guess = kap_guess * vx_guess

    opti.set_initial(theta, theta_guess)
    opti.set_initial(x, x_guess)
    opti.set_initial(y, y_guess)
    opti.set_initial(yaw, yaw_guess)
    opti.set_initial(vx, vx_guess)
    opti.set_initial(vy, vy_guess)
    opti.set_initial(omega, omega_guess)
    opti.set_initial(T, 5)

    opti.solver(
        "ipopt",
        {},
        {
            # suggestions by copilot to improve convergence time
            "max_iter": int(max_iter),
            "tol": 1e-6,
            "acceptable_tol": 1e-3,
            "acceptable_iter": 5,
            "mu_strategy": "adaptive",
            "nlp_scaling_method": str(nlp_scaling_method),
            "hessian_approximation": str(hessian_approximation),
            "bound_push": 1e-5,
            "bound_frac": 1e-5,
            "slack_bound_push": 1e-5,
            "slack_bound_frac": 1e-5,
            "print_level": int(ipopt_print),
        },
    )

    try:
        sol = opti.solve()
    
    # return best iterate if solver fails
    except RuntimeError as e:
        print(f"Using best iterate: {e}")
        sol = opti.debug

    T_opt = float(sol.value(T))
    dt_opt = T_opt / N
    X_opt = np.array(sol.value(X))
    U_opt = np.array(sol.value(U))

    ref = {
        "track_file": track_file,
        "N": int(N),
        "T": T_opt,
        "dt": float(dt_opt),
        "t": (np.arange(N + 1) * dt_opt).tolist(),
        "x": X_opt[0, :].tolist(),
        "y": X_opt[1, :].tolist(),
        "yaw": X_opt[2, :].tolist(),
        "vx": X_opt[3, :].tolist(),
        "vy": X_opt[4, :].tolist(),
        "yaw_rate": X_opt[5, :].tolist(),
        "theta": X_opt[6, :].tolist(),
        "torque": U_opt[0, :].tolist(),
        "steer": U_opt[1, :].tolist(),
        "x_init": float(X_opt[0, 0]),
        "y_init": float(X_opt[1, 0]),

    }
    extra = {
        "geo": geo,
        "n_max": n_max,
    }
    return {"reference": ref, **extra}