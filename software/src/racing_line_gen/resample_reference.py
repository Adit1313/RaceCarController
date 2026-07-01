#!/usr/bin/env python3
"""
resample_racing_line.py
-----------------------
Resamples an optimal racing line stored in a YAML file to a desired number
of evenly-spaced points along the arc length of the path.

Expected YAML structure:
    reference:
        xCoords:
            - 1.0
            - 2.0
            ...
        yCoords:
            - 3.0
            - 4.0
            ...

Usage:
    python resample_racing_line.py input.yaml output.yaml --num_points 400
    python resample_racing_line.py input.yaml output.yaml --num_points 190 --closed
    python resample_racing_line.py input.yaml output.yaml --num_points 400 --closed --plot
"""

import sys
from pathlib import Path
from typing import Tuple

import matplotlib.pyplot as plt
import numpy as np
import yaml
from scipy.interpolate import splev, splprep


# ── I/O ───────────────────────────────────────────────────────────────────────

def load_racing_line(path: Path) -> Tuple[np.ndarray, np.ndarray]:
    """Load and clean racing-line coordinates from a YAML file.

    Handles:
    - Duplicate data (track stored twice end-to-end).
    - Repeated endpoint (closed loop with first == last point).
    """
    with open(path, "r") as f:
        data = yaml.safe_load(f)

    try:
        ref = data["reference"]
        x = np.array(ref["xCoords"], dtype=float)
        y = np.array(ref["yCoords"], dtype=float)
    except KeyError as e:
        sys.exit(f"[ERROR] Missing key in YAML: {e}")

    if len(x) != len(y):
        sys.exit(f"[ERROR] xCoords ({len(x)}) and yCoords ({len(y)}) lengths differ.")
    if len(x) < 4:
        sys.exit("[ERROR] Need at least 4 points to fit a spline.")

    # Drop repeated endpoint; periodic closure is handled explicitly in the spline
    if len(x) > 3 and np.hypot(x[0] - x[-1], y[0] - y[-1]) < 1e-9:
        x = x[:-1]
        y = y[:-1]

    print(f"Loaded {len(x)} points from {path.name}")
    return x, y


def save_racing_line(
    path: Path,
    x: np.ndarray,
    y: np.ndarray,
    original_data: dict,
) -> None:
    """Write resampled coordinates back into the original YAML structure."""
    original_data["reference"]["xCoords"] = x.tolist()
    original_data["reference"]["yCoords"] = y.tolist()

    with open(path, "w") as f:
        yaml.dump(original_data, f, default_flow_style=False, sort_keys=False)

    print(f"[OK] Saved {len(x)} points → {path}")


# ── Core algorithm ────────────────────────────────────────────────────────────

def resample(
    x: np.ndarray,
    y: np.ndarray,
    num_points: int,
    closed: bool = False,
) -> Tuple[np.ndarray, np.ndarray]:
    """Resample a 2-D path to *num_points* arc-length-uniform samples.

    Parameters
    ----------
    x, y       : Original coordinate arrays (no repeated endpoint).
    num_points : Desired number of output points.
    closed     : Treat path as a closed loop (circuit). The output contains
                 *num_points* unique points (start != end).
    """
    if closed:
        x = np.append(x, x[0])
        y = np.append(y, y[0])

    k = min(3, len(x) - 1)          # cubic spline, or lower for very short paths
    tck, _ = splprep([x, y], s=0, k=k, per=closed)

    u_new = (
        np.linspace(0.0, 1.0, num_points, endpoint=False)
        if closed
        else np.linspace(0.0, 1.0, num_points)
    )

    x_new, y_new = splev(u_new, tck)
    return x_new, y_new


# ── Plotting ──────────────────────────────────────────────────────────────────

def plot_resampled(x: np.ndarray, y: np.ndarray, source_file: Path, num_points: int) -> None:
    """Plot the resampled racing line with sample-point markers."""
    fig, ax = plt.subplots(figsize=(10, 7))

    # Line
    # Close the visual loop for display only
    x_plot = np.append(x, x[0])
    y_plot = np.append(y, y[0])
    ax.plot(x_plot, y_plot, color="green", linewidth=2, label="Resampled racing line")

    # Sample points
    ax.scatter(x, y, s=12, color="green", zorder=3, label=f"Sample points ({num_points})")

    # Start marker
    ax.plot(x[0], y[0], "kx", markersize=14, markeredgewidth=3, label="Start")

    ax.set_aspect("equal")
    ax.grid(True)
    ax.legend()
    ax.set_title(f"Resampled racing line  |  {num_points} points  |  {source_file.name}")
    plt.tight_layout()
    plt.show()


# ── Configuration ─────────────────────────────────────────────────────────────

CLOSED       = True   # True for a closed circuit, False for open path
PLOT         = True   # show plot after saving

# Points from controller frequency:
#   NUM_POINTS = round(LAP_TIME * CONTROL_FREQ)
#   -> one waypoint per control cycle for the full lap
LAP_TIME     = 5   # estimated lap time in seconds
CONTROL_FREQ = 30     # controller frequency in Hz  (e.g. 15, 30, 50)
NUM_POINTS   = round(LAP_TIME * CONTROL_FREQ)
# NUM_POINTS   = 50
INPUT_FILE   = Path("references/FREIBURG_FULL_TRACK_REFERENCE.yaml")           # input YAML file
OUTPUT_FILE  = Path(INPUT_FILE.parent / f"{INPUT_FILE.stem}_RESAMPLED_{NUM_POINTS}.yaml")  # output YAML file

# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    if NUM_POINTS < 2:
        sys.exit("[ERROR] NUM_POINTS must be at least 2.")

    # Load
    print(f"[..] Loading   : {INPUT_FILE}")
    x_orig, y_orig = load_racing_line(INPUT_FILE)
    print(f"     Original   : {len(x_orig)} points")

    with open(INPUT_FILE, "r") as f:
        full_data = yaml.safe_load(f)

    # Resample
    mode = "closed loop" if CLOSED else "open path"
    print(f"[..] Resampling to {NUM_POINTS} points ({mode}) …")
    x_new, y_new = resample(x_orig, y_orig, NUM_POINTS, closed=CLOSED)

    # Save
    OUTPUT_FILE.parent.mkdir(parents=True, exist_ok=True)
    save_racing_line(OUTPUT_FILE, x_new, y_new, full_data)

    # Plot
    if PLOT:
        plot_resampled(x_new, y_new, INPUT_FILE, NUM_POINTS)


if __name__ == "__main__":
    main()