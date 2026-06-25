# pyright: reportGeneralTypeIssues=false

"""pacejka_opt_reference.py
==========================

Time-domain, periodic, time-optimal reference generation using a Pacejka bicycle model.

This replaces the spatial/Frenet OCP in time_optimal_line.py with a global (x,y,...) OCP.

State
-----
  X = [pos_x, pos_y, yaw, v_x, v_y, yaw_rate, s]

Input
-----
  U = [torque, steer]

Progress state
--------------
  s is arc-length progress along the track centerline. We enforce:
    s(0) = 0, s(T) = L
and periodicity of the physical states to obtain a steady periodic lap.

Track constraints
-----------------
We build CasADi interpolants of the centerline and curvature versus s, and enforce
the lateral offset to be within track half-width.
"""


from __future__ import annotations

import argparse
from dataclasses import asdict
from pathlib import Path
from typing import Dict, Tuple

import casadi as ca
import matplotlib.pyplot as plt
import numpy as np
from scipy.ndimage import uniform_filter1d
import yaml

import pacejka_model as pm


# ==========================================================
# TRACK DATA & GEOMETRY
# ==========================================================

def load_track(track_file: str) -> Tuple[np.ndarray, np.ndarray]:
    """Load and clean track coordinates from YAML file."""
    base_dir = Path(__file__).resolve().parent
    track_path = base_dir / "tracks" / track_file

    print(f"Loading track: {track_path}")
    with open(track_path, "r") as f:
        track_data = yaml.safe_load(f)

    rx = np.array(track_data["track"]["xCoords"], dtype=float)
    ry = np.array(track_data["track"]["yCoords"], dtype=float)

    # Remove duplicate loop if data stored twice
    n_half = len(rx) // 2
    if n_half > 5 and np.allclose(rx[:n_half], rx[n_half:], atol=1e-6):
        print(f"Duplicate track data detected — trimming to {n_half} points")
        rx = rx[:n_half]
        ry = ry[:n_half]

    # If the track already contains a repeated endpoint, drop it.
    # We will handle periodic closure explicitly in the geometry/interpolants.
    if len(rx) > 3 and np.hypot(rx[0] - rx[-1], ry[0] - ry[-1]) < 1e-9:
        rx = rx[:-1]
        ry = ry[:-1]

    print(f"Track points: {len(rx)}")
    return rx, ry


def build_track_geometry(rx: np.ndarray, ry: np.ndarray, track_width: float = 0.5) -> Dict:
    """Compute track normals, curvature, and arc length.

    Uses periodic (wrap-around) finite differences and defines arc-length progress
    that includes the closing segment from the last point back to the first.
    """
    half_w = track_width / 2.0

    rx = np.asarray(rx, dtype=float)
    ry = np.asarray(ry, dtype=float)
    if rx.ndim != 1 or ry.ndim != 1 or rx.shape != ry.shape:
        raise ValueError("rx/ry must be 1D arrays of equal length")
    if len(rx) < 5:
        raise ValueError("Track must have at least 5 points")

    # Periodic (wrap) central differences on the point index grid
    dx = 0.5 * (np.roll(rx, -1) - np.roll(rx, 1))
    dy = 0.5 * (np.roll(ry, -1) - np.roll(ry, 1))
    ddx = np.roll(rx, -1) - 2.0 * rx + np.roll(rx, 1)
    ddy = np.roll(ry, -1) - 2.0 * ry + np.roll(ry, 1)

    # Smooth derivatives with wrap-around to avoid end artifacts
    dx = np.asarray(uniform_filter1d(dx, size=11, mode="wrap"), dtype=float)
    dy = np.asarray(uniform_filter1d(dy, size=11, mode="wrap"), dtype=float)
    ddx = np.asarray(uniform_filter1d(ddx, size=11, mode="wrap"), dtype=float)
    ddy = np.asarray(uniform_filter1d(ddy, size=11, mode="wrap"), dtype=float)

    vnorm = np.sqrt(dx**2 + dy**2)
    vnorm[vnorm == 0] = 1e-9
    tx, ty = dx / vnorm, dy / vnorm
    nx, ny = -ty, tx

    curvature = (dx * ddy - dy * ddx) / (dx**2 + dy**2 + 1e-9) ** 1.5

    # Arc-length that includes closing segment (i -> i+1 with wrap)
    seg_dx = np.roll(rx, -1) - rx
    seg_dy = np.roll(ry, -1) - ry
    ds = np.sqrt(seg_dx**2 + seg_dy**2)
    if np.any(ds <= 0):
        ds = np.maximum(ds, 1e-9)

    s_grid = np.concatenate([[0.0], np.cumsum(ds)])

    print(f"Track length:    {s_grid[-1]:.3f} units")
    print(f"Curvature range: {curvature.min():.3f} to {curvature.max():.3f}")

    L = float(s_grid[-1])
    # Values at s=L should match the first sample for periodic continuity.
    s = s_grid[:-1]

    return dict(
        rx=rx,
        ry=ry,
        nx=nx,
        ny=ny,
        tx=tx,
        ty=ty,
        curvature=curvature,
        s=s,
        s_ext=s_grid,
        L=L,
        half_w=half_w,
        left_rx=rx + half_w * nx,
        left_ry=ry + half_w * ny,
        right_rx=rx - half_w * nx,
        right_ry=ry - half_w * ny,
    )


def build_track_interpolants(geo: Dict) -> Dict[str, ca.Function]:
    """Build CasADi interpolants for centerline and curvature vs arc-length s.

    For periodic continuity we drop the last sample and append the first sample at s=L.
    """
    s = np.asarray(geo["s"], dtype=float)
    L = float(geo["L"])
    if L <= 0:
        raise ValueError("Track length L must be positive")

    if not np.all(np.diff(s) > 0):
        raise ValueError("Track s grid is not strictly increasing")

    # Periodic extension: append the first sample at s=L.
    def _ext(arr: np.ndarray) -> np.ndarray:
        arr = np.asarray(arr, dtype=float)
        return np.concatenate([arr, [float(arr[0])]])

    rx_ext = _ext(geo["rx"])
    ry_ext = _ext(geo["ry"])
    tx_ext = _ext(geo["tx"])
    ty_ext = _ext(geo["ty"])
    kap_ext = _ext(geo["curvature"])

    s_ext = np.concatenate([s, [L]])
    if not np.all(np.diff(s_ext) > 0):
        raise ValueError("Track s grid (extended) is not strictly increasing")

    # CasADi interpolants (linear is robust; bspline is smoother but can overshoot)
    xc = ca.interpolant("xc", "linear", [s_ext], rx_ext)
    yc = ca.interpolant("yc", "linear", [s_ext], ry_ext)
    tx = ca.interpolant("tx", "linear", [s_ext], tx_ext)
    ty = ca.interpolant("ty", "linear", [s_ext], ty_ext)
    kappa = ca.interpolant("kappa", "linear", [s_ext], kap_ext)

    # CasADi's Python type stubs are incomplete; this call is correct at runtime.
    s_sym = ca.MX.sym("s")  # type: ignore[call-arg]
    out = {
        "xc": ca.Function("xc_f", [s_sym], [xc(s_sym)]),
        "yc": ca.Function("yc_f", [s_sym], [yc(s_sym)]),
        "tx": ca.Function("tx_f", [s_sym], [tx(s_sym)]),
        "ty": ca.Function("ty_f", [s_sym], [ty(s_sym)]),
        "kappa": ca.Function("kappa_f", [s_sym], [kappa(s_sym)]),
    }
    return out


# ==========================================================
# OCP
# ==========================================================

def solve_time_optimal_pacejka_lap(
    track_file: str,
    params: pm.PacejkaParams,
    *,
    # Use N=345 so the exported reference has exactly N+1=346 points.
    N: int = 345,
    track_width: float = 0.4,
    vehicle_width: float = 0.05,
    torque_min: float = 0,
    torque_max: float = 1.0,
    steer_min: float = -0.3,
    steer_max: float = 0.3,
    vx_min: float = 0.5,
    vx_max: float = 3.5,
    vy_abs_max: float = 1,
    yaw_rate_abs_max: float = 4,
    T_min: float = 0.5,
    T_max: float = 30.0,
    w_u: float = 1e-4,
    w_du: float = 1e-3,
    enforce_monotonic_s: bool = False,
    hessian_approximation: str = "limited-memory",
    nlp_scaling_method: str = "gradient-based",
    ipopt_print: int = 3,
    max_iter: int = 4000,
) -> Dict:
    """Solve a periodic fastest-lap OCP using Pacejka dynamics.

    Returns a dict with arrays for reference export.
    """
    rx, ry = load_track(track_file)
    geo = build_track_geometry(rx, ry, track_width=track_width)
    L = float(geo["L"])

    margin = vehicle_width / 2.0
    n_max = float(geo["half_w"] - margin)
    if n_max <= 0:
        raise ValueError("Track too narrow for vehicle_width")

    track_f = build_track_interpolants(geo)
    f_rhs, _ = pm.make_pacejka_functions(params)

    opti = ca.Opti()

    X = opti.variable(7, N + 1)  # [x,y,yaw,vx,vy,r,s]
    U = opti.variable(2, N)      # [torque, steer]
    T = opti.variable()
    dt = T / N

    # Convenience slices
    x = X[0, :]
    y = X[1, :]
    yaw = X[2, :]
    vx = X[3, :]
    vy = X[4, :]
    r = X[5, :]
    s = X[6, :]

    torque = U[0, :]
    steer = U[1, :]

    # Bounds
    # CasADi Opti stubs can confuse Pylance; runtime behavior is correct.
    opti.subject_to(opti.bounded(T_min, T, T_max))  # type: ignore[call-arg]
    opti.subject_to(opti.bounded(0.0, s, L))  # type: ignore[call-arg]
    opti.subject_to(opti.bounded(vx_min, vx, vx_max))  # type: ignore[call-arg]
    opti.subject_to(opti.bounded(-vy_abs_max, vy, vy_abs_max))  # type: ignore[call-arg]
    opti.subject_to(opti.bounded(-yaw_rate_abs_max, r, yaw_rate_abs_max))  # type: ignore[call-arg]
    opti.subject_to(opti.bounded(torque_min, torque, torque_max))  # type: ignore[call-arg]
    opti.subject_to(opti.bounded(steer_min, steer, steer_max))  # type: ignore[call-arg]



    # limit throttle delta:
    if N > 1:
        dtorque = torque[1:] - torque[:-1]
        opti.subject_to(opti.bounded(-0.2, dtorque, 0.2))

    # Fixed start (phase fixing)
    opti.subject_to(s[0] == 0.0)
    opti.subject_to(x[0] == float(geo["rx"][0]))
    opti.subject_to(y[0] == 3.0)
    # opti.subject_to(y[-1] == 3.0)
    psi0 = float(np.arctan2(geo["ty"][0], geo["tx"][0]))
    opti.subject_to(yaw[0] == psi0)

    # Lap completion in s
    opti.subject_to(s[-1] == L)

    # Periodicity of physical states
    opti.subject_to(x[-1] == x[0])
    opti.subject_to(y[-1] == y[0])
    opti.subject_to(vx[0] == vx[-1])
    opti.subject_to(vy[0] == vy[-1])
    opti.subject_to(r[-1] == r[0])
    # Yaw periodicity without 2pi ambiguity
    opti.subject_to(ca.sin(yaw[-1]) == ca.sin(yaw[0]))
    opti.subject_to(ca.cos(yaw[-1]) == ca.cos(yaw[0]))

    # for warm up lap
    # opti.subject_to(vx[0] == 0.4)
    # opti.subject_to(vy[0] == 0.0)
    # opti.subject_to(vx[-1] == 2.54)
    # opti.subject_to(vy[-1] == 0.19)

    # # for cooldown lap
    # opti.subject_to(vx[0] == 2.54)
    # opti.subject_to(vy[0] == 0.19)
    # opti.subject_to(vx[-1] == 0.6)
    # opti.subject_to(vy[-1] == 0.0)

    def lateral_offset_and_heading(s_k, x_k, y_k):
        xc_k = track_f["xc"](s_k)
        yc_k = track_f["yc"](s_k)
        tx_k = track_f["tx"](s_k)
        ty_k = track_f["ty"](s_k)
        kappa_k = track_f["kappa"](s_k)
        psi_c_k = ca.atan2(ty_k, tx_k)
        # normal = [-ty, tx]
        n_k = (x_k - xc_k) * (-ty_k) + (y_k - yc_k) * (tx_k)  # type: ignore
        return n_k, psi_c_k, kappa_k

    def rhs7(Xk, Uk):
        x6 = Xk[0:6]
        s_k = Xk[6]

        x6_dot = f_rhs(x6, Uk)

        n_k, psi_c_k, kappa_k = lateral_offset_and_heading(s_k, Xk[0], Xk[1])
        epsi = Xk[2] - psi_c_k
        # Tangential progress rate (robust approximation; avoids stiff 1/(1-kappa*n))
        s_dot = Xk[3] * ca.cos(epsi) - Xk[4] * ca.sin(epsi)

        return ca.vertcat(x6_dot, s_dot)

    def rk4_step7(Xk, Uk, dt_):
        k1 = rhs7(Xk, Uk)
        k2 = rhs7(Xk + dt_ / 2 * k1, Uk)
        k3 = rhs7(Xk + dt_ / 2 * k2, Uk)
        k4 = rhs7(Xk + dt_ * k3, Uk)
        return Xk + dt_ / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

    # Dynamics + track constraints
    for k in range(N):
        opti.subject_to(X[:, k + 1] == rk4_step7(X[:, k], U[:, k], dt))

        # Monotonic progress can help avoid pathological s-dynamics during solve,
        # but can make large-N problems harder; keep it configurable.
        if enforce_monotonic_s:
            opti.subject_to(s[k + 1] >= s[k])

        n_k, _, _ = lateral_offset_and_heading(s[k], x[k], y[k])
        opti.subject_to(opti.bounded(-n_max, n_k, n_max))  # type: ignore[call-arg]

    # Enforce track constraint at terminal node as well
    n_N, _, _ = lateral_offset_and_heading(s[-1], x[-1], y[-1])
    opti.subject_to(opti.bounded(-n_max, n_N, n_max))  # type: ignore[call-arg]

    # Objective: minimize time + small regularization
    cost = T
    #cost = 0
    cost += w_u * dt * ca.sumsqr(U)
    if N > 1:
        cost += w_du * dt * ca.sumsqr(U[:, 1:] - U[:, :-1])
    opti.minimize(cost)

    # Initial guess (centerline, moderate speed)
    s_guess = np.linspace(0.0, L, N + 1)

    # Use periodic interpolants for a consistent closed-loop guess
    x_guess = np.array([float(track_f["xc"](si)) for si in s_guess], dtype=float)  # type: ignore
    y_guess = np.array([float(track_f["yc"](si)) for si in s_guess], dtype=float)  # type: ignore
    tx_guess = np.array([float(track_f["tx"](si)) for si in s_guess], dtype=float)  # type: ignore
    ty_guess = np.array([float(track_f["ty"](si)) for si in s_guess], dtype=float)  # type: ignore
    yaw_guess = np.unwrap(np.arctan2(ty_guess, tx_guess))

    vx_guess = np.ones(N + 1) * min(2.0, vx_max)
    vy_guess = np.zeros(N + 1)
    kap_guess = np.array([float(track_f["kappa"](si)) for si in s_guess], dtype=float)  # type: ignore
    r_guess = kap_guess * vx_guess

    opti.set_initial(s, s_guess)
    opti.set_initial(x, x_guess)
    opti.set_initial(y, y_guess)
    opti.set_initial(yaw, yaw_guess)
    opti.set_initial(vx, vx_guess)
    opti.set_initial(vy, vy_guess)
    opti.set_initial(r, r_guess)
    opti.set_initial(torque, 0.0)
    # Rough steering guess from curvature (kinematic bicycle approximation)
    steer_guess = np.arctan((params.lf + params.lr) * kap_guess[:-1])
    steer_guess = np.clip(steer_guess, steer_min, steer_max)
    opti.set_initial(steer, steer_guess)
    # Rough time guess based on length and speed
    opti.set_initial(T, max(T_min, min(T_max, L / max(1.0, float(np.mean(vx_guess))))))

    opti.solver(
        "ipopt",
        {},
        {
            "max_iter": int(max_iter),
            "tol": 1e-6,
            "acceptable_tol": 1e-4,
            "acceptable_iter": 50,
            "mu_strategy": "adaptive",
            "nlp_scaling_method": str(nlp_scaling_method),
            "hessian_approximation": str(hessian_approximation),
            # A few numerics tweaks that often reduce "Error in step computation"
            "bound_push": 1e-8,
            "bound_frac": 1e-8,
            "slack_bound_push": 1e-8,
            "slack_bound_frac": 1e-8,
            "print_level": int(ipopt_print),
        },
    )

    try:
        sol = opti.solve()
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
        "s": X_opt[6, :].tolist(),
        "torque": U_opt[0, :].tolist(),
        "steer": U_opt[1, :].tolist(),
        "x_init": float(X_opt[0, 0]),
        "y_init": float(X_opt[1, 0]),
        "params": asdict(params),
    }
    extra = {
        "geo": geo,
        "n_max": n_max,
    }
    return {"reference": ref, **extra}



def compute_reference_geometry(x_ref: np.ndarray, y_ref: np.ndarray) -> Dict:
    """
    Compute CRS-style geometric reference quantities from optimized x/y trajectory.

    Returns:
        xRate, yRate:
            normalized tangent directions of the optimized reference line

        tangentAngle:
            atan2(yRate, xRate)

        arcLength:
            cumulative arc length along the optimized reference

        curvature:
            curvature computed from periodic finite differences

        trackLength:
            total length of the optimized reference
    """
    x_ref = np.asarray(x_ref, dtype=float)
    y_ref = np.asarray(y_ref, dtype=float)

    if x_ref.ndim != 1 or y_ref.ndim != 1 or x_ref.shape != y_ref.shape:
        raise ValueError("x_ref and y_ref must be 1D arrays of equal length.")

    if len(x_ref) < 5:
        raise ValueError("Reference must contain at least 5 points.")

    # If the last point repeats the first point, remove it for periodic geometry.
    has_repeated_endpoint = (
        np.hypot(x_ref[-1] - x_ref[0], y_ref[-1] - y_ref[0]) < 1e-9
    )

    if has_repeated_endpoint:
        x_base = x_ref[:-1]
        y_base = y_ref[:-1]
    else:
        x_base = x_ref
        y_base = y_ref

    # Periodic finite differences.
    dx = 0.5 * (np.roll(x_base, -1) - np.roll(x_base, 1))
    dy = 0.5 * (np.roll(y_base, -1) - np.roll(y_base, 1))

    ddx = np.roll(x_base, -1) - 2.0 * x_base + np.roll(x_base, 1)
    ddy = np.roll(y_base, -1) - 2.0 * y_base + np.roll(y_base, 1)

    norm = np.sqrt(dx * dx + dy * dy)
    norm[norm < 1e-12] = 1.0

    x_rate_base = dx / norm
    y_rate_base = dy / norm

    tangent_base = np.arctan2(y_rate_base, x_rate_base)

    curvature_base = (dx * ddy - dy * ddx) / (dx * dx + dy * dy + 1e-9) ** 1.5

    # Arc length including closing segment.
    seg_dx = np.roll(x_base, -1) - x_base
    seg_dy = np.roll(y_base, -1) - y_base
    ds = np.sqrt(seg_dx * seg_dx + seg_dy * seg_dy)
    ds = np.maximum(ds, 1e-12)

    arc_base = np.concatenate([[0.0], np.cumsum(ds[:-1])])
    track_length = float(np.sum(ds))

    if has_repeated_endpoint:
        x_rate = np.append(x_rate_base, x_rate_base[0])
        y_rate = np.append(y_rate_base, y_rate_base[0])
        tangent_angle = np.append(tangent_base, tangent_base[0])
        curvature = np.append(curvature_base, curvature_base[0])
        arc_length = np.append(arc_base, track_length)
    else:
        x_rate = x_rate_base
        y_rate = y_rate_base
        tangent_angle = tangent_base
        curvature = curvature_base
        arc_length = arc_base

    return {
        "xRate": x_rate,
        "yRate": y_rate,
        "tangentAngle": tangent_angle,
        "arcLength": arc_length,
        "curvature": curvature,
        "trackLength": track_length,
    }


def export_reference(ref: Dict, out_path: Path, geo: Dict | None = None) -> None:
    """
    Export optimized reference in a CRS-like YAML format.

    Important:
        xRate/yRate/tangentAngle/arcLength/curvature are geometric quantities
        of the optimized reference line. They are not the dynamic body-frame
        velocities vx/vy. The dynamic velocities are stored separately as vx/vy.
    """
    x_ref = np.asarray(ref["x"], dtype=float)
    y_ref = np.asarray(ref["y"], dtype=float)

    ref_geo = compute_reference_geometry(x_ref, y_ref)

    # Prefer original track metadata if available.
    if geo is not None:
        track_width = float(2.0 * geo["half_w"])
        density = int(round(len(geo["rx"]) / float(geo["L"])))
    else:
        track_width = float(ref.get("trackWidth", 0.5))
        density = int(round(len(x_ref) / float(ref_geo["trackLength"])))

    yaw_init = float(ref["yaw"][0]) if "yaw" in ref else float(ref_geo["tangentAngle"][0])

    out = {
        "reference": {
            # CRS-style geometric reference
            "xCoords": list(map(float, ref["x"])),
            "yCoords": list(map(float, ref["y"])),

            # "xRate": list(map(float, ref_geo["xRate"])),
            # "yRate": list(map(float, ref_geo["yRate"])),
            # "tangentAngle": list(map(float, ref_geo["tangentAngle"])),
            # "arcLength": list(map(float, ref_geo["arcLength"])),
            # "curvature": list(map(float, ref_geo["curvature"])),

            # "trackLength": float(ref_geo["trackLength"]),
            # "trackWidth": float(track_width),
            # "density": int(density),

            "x_init": float(ref["x_init"]),
            "y_init": float(ref["y_init"]),
            "yaw_init": float(yaw_init),

            # # Dynamic reference quantities
            # "yaw": list(map(float, ref["yaw"])),
            # "vx": list(map(float, ref["vx"])),
            # "vy": list(map(float, ref["vy"])),
            # "yaw_rate": list(map(float, ref["yaw_rate"])),

            # Progress and timing
            # "s": list(map(float, ref["s"])),
            # "T": float(ref["T"]),
            # "dt": float(ref["dt"]),
            # "N": int(ref["N"]),
            # "t": list(map(float, ref["t"])),

            # Inputs
            # "torque": list(map(float, ref["torque"])),
            # "steer": list(map(float, ref["steer"])),

            # Parameters for reproducibility
            # "params": ref["params"],
        }
    }

    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w") as f:
        yaml.safe_dump(out, f, default_flow_style=False, sort_keys=False)
    print(f"Reference exported to {out_path}")


def plot_reference(sol: Dict) -> None:
    geo = sol["geo"]
    ref = sol["reference"]

    fig, ax = plt.subplots(figsize=(10, 7))
    ax.plot(geo["rx"], geo["ry"], "k--", linewidth=1, label="Centerline")
    ax.plot(geo["left_rx"], geo["left_ry"], "b", linewidth=1)
    ax.plot(geo["right_rx"], geo["right_ry"], "b", linewidth=1)
    ax.plot(ref["x"], ref["y"], "g", linewidth=2, label="Pacejka optimal")
    ax.plot(ref["x"][0], ref["y"][0], "kx", markersize=14, markeredgewidth=3, label="Start")
    ax.set_aspect("equal")
    ax.grid(True)
    ax.legend()
    ax.set_title(f"Pacejka time-optimal lap  |  T={ref['T']:.2f}s")
    plt.show()


# ==========================================================
# MAIN
# ==========================================================

def _default_params() -> pm.PacejkaParams:
    # Defaults copied from your original pacejka script values
    return pm.PacejkaParams(
        lr=0.038,
        lf=0.052,
        m=0.201,
        I=0.000705,

        Df=0.65,
        Cf=1.5,
        Bf=5.2,
        Dr=1.0,
        Cr=1.45,
        Br=8.5,
        
        Cm1=0.98028992,
        Cm2=0.01814131,

        Cd0=0.08518052,
        Cd1=0.01,
        Cd2=0.02750696,

        gamma=1,
        eps=0.2,
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Pacejka time-optimal periodic lap")

    parser.add_argument("--track", type=str, default="FREIBURG_L_TRACK.yaml")

    parser.add_argument("--N", type=int, default=350)

    parser.add_argument('--plot_results', type=bool, default=True,
                        help='Plot optimization results (default: False)')

    parser.add_argument(
        "--out",
        type=str,
        default="FB_L_30HZ.yaml"
    )
    parser.add_argument("--max_iter", type=int, default=2000,
                        help="IPOPT max iterations")
    
    parser.add_argument("--ipopt_print", type=int, default=3,
                        help="IPOPT print level")
    
    parser.add_argument("--hessian", type=str, default="limited-memory",
                        choices=["limited-memory", "exact"],
                        help="IPOPT Hessian approximation")
    
    parser.add_argument("--nlp_scaling", type=str, default="gradient-based",
                        choices=["gradient-based", "none"],
                        help="IPOPT NLP scaling method")
    
    parser.add_argument("--monotonic_s", action="store_true",
                        help="Enable monotonic s constraints (can help for small N)")
    parser.add_argument("--no_monotonic_s", action="store_true",
                        help="Disable monotonic s constraints")
    
    args = parser.parse_args()

    params = _default_params()
    sol = solve_time_optimal_pacejka_lap(
        args.track,
        params,
        N=int(args.N),
        max_iter=int(args.max_iter),
        ipopt_print=int(args.ipopt_print),
        hessian_approximation=str(args.hessian),
        nlp_scaling_method=str(args.nlp_scaling),
        enforce_monotonic_s=(bool(args.monotonic_s) and not bool(args.no_monotonic_s)),
        track_width=0.4,
        vehicle_width=0.06
    )

    ref = sol["reference"]
    print(f"Solved: T={ref['T']:.3f}s  dt={ref['dt']:.4f}s  N={ref['N']}")
    print(f"Start: x={ref['x_init']:.3f}  y={ref['y_init']:.3f}")
    print(f"Speed: vx min={min(ref['vx']):.3f}  max={max(ref['vx']):.3f}")

    if True:
        out_path = Path(args.out)
        # If user passes a relative path, interpret it relative to this script.
        if not out_path.is_absolute():
            out_path = Path(__file__).resolve().parent / out_path
        export_reference(ref, out_path, sol["geo"])

    if args.plot_results:
        plot_reference(sol)


#### new parameters?
        # lr=0.038,
        # lf=0.052,
        # m=0.181,
        # I=0.000505,

        # Df=0.65,
        # Cf=1.5,
        # Bf=5.2,
        # Dr=1,
        # Cr=1.5,
        # Br=8.5,

        # Cm1=98028992,
        # Cm2=0.01814131,

        # Cd0=0.08518052,
        # Cd1=0.01,
        # Cd2=0.02750696,

        # gamma=1,
        # eps=0.2,