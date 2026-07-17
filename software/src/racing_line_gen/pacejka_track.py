import numpy as np
import yaml
from pathlib import Path
from typing import Dict
import casadi as ca


def load_track(track_file: str) -> Dict[str, np.ndarray | float]:
    """Load and clean track geometry arrays from YAML file."""
    base_dir = Path(__file__).resolve().parent
    track_path = base_dir / "tracks" / track_file

    print(f"Loading track: {track_path}")
    with open(track_path, "r") as f:
        track_data = yaml.safe_load(f)

    track = track_data["track"]
    rx = np.array(track["xCoords"], dtype=float)
    ry = np.array(track["yCoords"], dtype=float)
    s = np.array(track["arcLength"], dtype=float)
    curvature = np.array(track["curvature"], dtype=float)
    tangent_angle = np.array(track["tangentAngle"], dtype=float)
    loop_length = float(s[-1])

    # Remove duplicate loop since data is stored twice.
    n_half = len(rx) // 2
    if n_half > 5 and np.allclose(rx[:n_half], rx[n_half:], atol=1e-6):
        print(f"Duplicate track data detected — trimming to {n_half} points")
        loop_length = float(s[n_half] - s[0])
        rx = rx[:n_half]
        ry = ry[:n_half]
        s = s[:n_half]
        curvature = curvature[:n_half]
        tangent_angle = tangent_angle[:n_half]

    print(f"Track points: {len(rx)}")
    return dict(
        rx=rx,
        ry=ry,
        s=s,
        L=loop_length,
        curvature=curvature,
        tangent_angle=tangent_angle,
    )



def build_track_geometry(track: Dict[str, np.ndarray | float], track_width: float = 0.6) -> Dict:
    """Assemble track normals, curvature, and arc length from YAML geometry."""
    half_w = track_width / 2.0

    rx = np.asarray(track["rx"], dtype=float)
    ry = np.asarray(track["ry"], dtype=float)
    s = np.asarray(track["s"], dtype=float)
    curvature = np.asarray(track["curvature"], dtype=float)
    tangent_angle = np.asarray(track["tangent_angle"], dtype=float)

    # tangents and normals 
    tx = np.cos(tangent_angle)
    ty = np.sin(tangent_angle)
    nx, ny = -ty, tx

    s_grid = np.asarray(s, dtype=float)
    L = float(track["L"])
    s_ext = np.concatenate([s_grid, [L]])

    print(f"Track length:    {L:.3f} units")
    print(f"Curvature range: {curvature.min():.3f} to {curvature.max():.3f}")

    return dict(
        rx=rx,
        ry=ry,
        nx=nx,
        ny=ny,
        tx=tx,
        ty=ty,
        curvature=curvature,
        s=s_grid,
        s_ext=s_ext,
        L=L,
        half_w=half_w,
        left_rx=rx + half_w * nx,
        left_ry=ry + half_w * ny,
        right_rx=rx - half_w * nx,
        right_ry=ry - half_w * ny,
    )



def build_track_interpolants(geo: Dict[str, np.ndarray | float]) -> Dict[str, ca.Function]:
    """Build CasADi interpolants for centerline and curvature vs arc-length s.
    """
    s = np.asarray(geo["s"], dtype=float)
    L = float(geo["L"])

    # Periodic extension: append the first sample at s=L.
    def _ext(arr: np.ndarray) -> np.ndarray:
        arr = np.asarray(arr, dtype=float)
        return np.concatenate([arr, [float(arr[0])]])

    rx_ext = _ext(np.asarray(geo["rx"], dtype=float))
    ry_ext = _ext(np.asarray(geo["ry"], dtype=float))
    tx_ext = _ext(np.asarray(geo["tx"], dtype=float))
    ty_ext = _ext(np.asarray(geo["ty"], dtype=float))
    kap_ext = _ext(np.asarray(geo["curvature"], dtype=float))
    s_ext = np.concatenate([s, [L]])


    # CasADi interpolants
    xc = ca.interpolant("xc", "linear", [s_ext], rx_ext)
    yc = ca.interpolant("yc", "linear", [s_ext], ry_ext)
    tx = ca.interpolant("tx", "linear", [s_ext], tx_ext)
    ty = ca.interpolant("ty", "linear", [s_ext], ty_ext)
    kappa = ca.interpolant("kappa", "linear", [s_ext], kap_ext)

    s_sym = ca.MX.sym("s")
    out = {
        "xc": ca.Function("xc_f", [s_sym], [xc(s_sym)]),
        "yc": ca.Function("yc_f", [s_sym], [yc(s_sym)]),
        "tx": ca.Function("tx_f", [s_sym], [tx(s_sym)]),
        "ty": ca.Function("ty_f", [s_sym], [ty(s_sym)]),
        "kappa": ca.Function("kappa_f", [s_sym], [kappa(s_sym)]),
    }
    return out




def export_reference(ref: Dict, out_path: Path, geo: Dict | None = None) -> None:
    """
    Export optimized reference as YAML file.
    """
    yaw_init = float(ref["yaw"][0])

    out = {
        "reference": {
            # state references
            "xCoords": list(map(float, ref["x"])),
            "yCoords": list(map(float, ref["y"])),
            "yaw": list(map(float, ref["yaw"])),
            "vx": list(map(float, ref["vx"])),
            "vy": list(map(float, ref["vy"])),
            "yaw_rate": list(map(float, ref["yaw_rate"])),

            # initial values
            "x_init": float(ref["x_init"]),
            "y_init": float(ref["y_init"]),
            "yaw_init": float(yaw_init),

            # input references
            "torque": list(map(float, ref["torque"])),
            "steer": list(map(float, ref["steer"])),
        }
    }

    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w") as f:
        yaml.safe_dump(out, f, default_flow_style=False, sort_keys=False)
    print(f"Reference exported to {out_path}")