import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import Normalize
from typing import Dict

def plot_reference(sol: Dict, lap_type) -> None:
    geo = sol["geo"]
    ref = sol["reference"]

    fig, ax = plt.subplots(figsize=(18, 12))

    # Track geometry
    ax.plot(geo["rx"], geo["ry"], "k--", linewidth=1, label="Centerline")
    ax.plot(geo["left_rx"], geo["left_ry"], "b", linewidth=1)
    ax.plot(geo["right_rx"], geo["right_ry"], "b", linewidth=1)

    # Reference trajectory colored by vx
    x = np.asarray(ref["x"], dtype=float)
    y = np.asarray(ref["y"], dtype=float)
    vx = np.asarray(ref["vx"], dtype=float)

    points = np.column_stack((x, y))
    segments = np.stack([points[:-1], points[1:]], axis=1)

    norm = Normalize(vmin=0.0, vmax=3.5)
    lc = LineCollection(
        segments,
        cmap="viridis",
        norm=norm,
        linewidth=2.5,
        label="trajectory"
    )

    # One color value per segment
    lc.set_array(vx[:-1])

    ax.add_collection(lc)

    # Colorbar
    cbar = fig.colorbar(lc, ax=ax)
    cbar.set_label(r"$v_x$ [m/s]")

    # Start marker
    ax.plot(
        ref["x"][0],
        ref["y"][0],
        "kx",
        markersize=14,
        markeredgewidth=3,
        label="Start"
    )

    ax.set_aspect("equal")
    ax.grid(True)
    ax.legend()
    ax.set_title(f"{lap_type}  |  T={ref['T']:.2f}s")

    plt.show()