# Racing Line Toolkit

Scripts for generating, resampling, and stitching together time-optimal racing-line references for a Pacejka tire-model vehicle.

## Pipeline

```
track definition (.yaml)
        │
        ▼
pacejka_main.py   →  solves the time-optimal lap, exports L_TRACK_REFERENCE.yaml
        │
        ▼
resample_racing_line.py →  re-samples the reference to N evenly-spaced points
        │
        ▼
stitch_references.py    →  concatenates multiple references into one combined file
```

Each stage reads/writes a YAML file shaped like:

```yaml
reference:
  xCoords: [1.0, 2.0, ...]
  yCoords: [3.0, 4.0, ...]
```

---

## 1. `pacejka_main.py`

Solves a **time-optimal lap** for a vehicle with a Pacejka tire model on a given track, then exports the resulting racing line as a reference YAML.

### What it does
1. Builds the vehicle parameters (`pm.Params`) — mass, inertia, wheelbase, and Pacejka tire coefficients (front/rear stiffness, shape, peak factors), plus drivetrain/drag coefficients.
2. Calls `solve_time_optimal_pacejka_lap()` (from `pacejka_ocp`) to solve the optimal-control problem over `N` collocation points, given the track file and a track-width constraint.
3. Prints a summary: lap time `T`, timestep `dt`, point count `N`, start position, and min/max longitudinal speed.
4. Exports the solved reference via `export_reference()` (from `pacejka_track`) to an output YAML.
5. Optionally plots the result with `plot_reference()` (from `pacejka_plots`).

### Configuration (edit constants at the top of the file)

| Variable | Meaning | Default |
|---|---|---|
| `TRACK` | Input track definition file | `FREIBURG_L_TRACK.yaml` |
| `N` | Number of discretization points for the OCP | `300` |
| `OUT` | Output reference file | `L_TRACK_REFERENCE.yaml` |
| `MAX_ITER` | Max solver iterations | `1000` |
| `PRINT_LEVEL` | Solver verbosity (IPOPT print level) | `3` |
| `TRACK_WIDTH` | Track width constraint (m) | `0.6` |
| `WARM_UP` | Solve a warm-up (non-time-optimal) lap instead | `False` |
| `PLOT_RESULTS` | Show plots after solving | `True` |

Vehicle/tire parameters (mass, wheelbase, Pacejka `B`/`C`/`D` coefficients, drivetrain `Cm1`/`Cm2`, drag `Cd0-2`, etc.) are set in `_default_params()`.

### Usage
```bash
python pacejka_main.py
```
There are no CLI arguments — adjust the constants at the top of the file and re-run. Requires `pacejka_model.py`, `pacejka_track.py`, `pacejka_plots.py`, and `pacejka_ocp.py` to be importable (same directory or on `PYTHONPATH`), along with the track YAML referenced by `TRACK`.

---

## 2. `resample_racing_line.py`

Re-samples an existing racing line to a chosen number of **arc-length-evenly-spaced** points, using cubic spline interpolation (`scipy.interpolate.splprep`/`splev`).

### What it does
- Loads `xCoords`/`yCoords` from `reference:` in the input YAML.
- Cleans the data: drops a duplicated closing point if the path already loops back on itself.
- Fits a spline through the points (periodic if `closed=True`) and evaluates it at `NUM_POINTS` evenly spaced parameter values.
- Writes the result back into the same YAML structure.
- Optionally plots the resampled line with start-point and sample markers.

### Configuration (edit constants at the top of the file)

| Variable | Meaning | Default |
|---|---|---|
| `CLOSED` | Treat the path as a closed circuit | `True` |
| `PLOT` | Show plot after saving | `True` |
| `LAP_TIME` / `CONTROL_FREQ` | Used to derive `NUM_POINTS = round(LAP_TIME * CONTROL_FREQ)`, i.e. one waypoint per control cycle | `3.5 s`, `30 Hz` |
| `NUM_POINTS` | Target point count (override directly if not deriving from lap time/frequency) | computed |
| `INPUT_FILE` | Source reference YAML | `references/L_TRACK_REFERENCE.yaml` |
| `OUTPUT_FILE` | Destination file (auto-named `<stem>_RESAMPLED_<N>.yaml`) | derived |

### Usage
```bash
python resample_racing_line.py
```
The docstring also documents an argparse-style CLI (`input.yaml output.yaml --num_points 400 [--closed] [--plot]`), but the current `main()` runs off the module-level constants above rather than `sys.argv` — edit the constants or wire up `argparse` if you need CLI control.

Requires at least 4 input points to fit a spline.

---

## 3. `stitch_references.py`

Interactively combines two or more reference YAML files (e.g. a straight + a hairpin + a chicane) into a single stitched reference, by concatenating their coordinate arrays in a user-chosen order.

### What it does
1. Scans `references/` for `*.yaml`/`*.yml` files and lists them with their point counts.
2. Prompts for a sequence of file numbers (space- or comma-separated), which may repeat a file or reorder freely.
3. Loads and concatenates `xCoords`/`yCoords` from the selected files in that order.
4. Saves the combined path to `references/stitched.yaml`, recording the source filenames under `reference.stitchedFrom`.

### Usage
```bash
python stitch_references.py
```
```
Available reference files:
────────────────────────────────────────
   1.  L_TRACK_REFERENCE.yaml            (300 pts)
   2.  L_TRACK_REFERENCE_RESAMPLED_105.yaml (105 pts)
────────────────────────────────────────

Enter the sequence of numbers to stitch (space- or comma-separated).
Example:  2 1 3   or   2, 1, 3

Sequence: 1 2 1
```

Note: the script does not smooth or blend the seams between concatenated segments — points are simply appended in sequence, so segment endpoints should already be reasonably close if a continuous path is desired. It also excludes any pre-existing `references/stitched.yaml` from the pickable list, to avoid stitching a file into itself.

---

## Typical end-to-end workflow

```bash
# 1. Solve the time-optimal lap for a track
python pacejka_main.py                     # → references/L_TRACK_REFERENCE.yaml

# 2. Resample to match your controller's cycle time
python resample_racing_line.py                    # → *_RESAMPLED_<N>.yaml

# 3. (Optional) stitch multiple references together
python stitch_references.py                       # → references/stitched.yaml
```
