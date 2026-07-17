import argparse
import pacejka_model as pm
from pacejka_track import export_reference
from pacejka_plots import plot_reference
from pacejka_ocp import solve_time_optimal_pacejka_lap
from pathlib import Path

# settings
TRACK = "FREIBURG_L_TRACK.yaml"
N = 300
PLOT_RESULTS=True
OUT="L_TRACK_REFERENCE.yaml"
MAX_ITER = 1000
PRINT_LEVEL = 3
TRACK_WIDTH = 0.6
WARM_UP = False

def _default_params() -> pm.Params:
    return pm.Params(
        lr=0.038,
        lf=0.052,
        m=0.181,
        I=0.000505,

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

    params = _default_params()
    sol = solve_time_optimal_pacejka_lap(
        track_file=TRACK,
        params= params,
        N=N,
        max_iter=MAX_ITER,
        ipopt_print=PRINT_LEVEL,
        track_width=TRACK_WIDTH,
        warm_up = WARM_UP
    )

    ref = sol["reference"]
    print(f"Solved: T={ref['T']:.3f}s  dt={ref['dt']:.4f}s  N={ref['N']}")
    print(f"Start: x={ref['x_init']:.3f}  y={ref['y_init']:.3f}")
    print(f"Speed: vx min={min(ref['vx']):.3f}  max={max(ref['vx']):.3f}")

    out_path = Path(OUT)
    export_reference(ref, out_path, sol["geo"])

    if PLOT_RESULTS:
        if WARM_UP:
            lap_type="warm-up lap"
        else:
            lap_type="time optimal lap"
        plot_reference(sol, lap_type)
        