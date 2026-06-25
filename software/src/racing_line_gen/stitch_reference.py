#!/usr/bin/env python3
"""
stitch_references.py
--------------------
Queries the references folder, lists available YAML files, then lets the user
pick a sequence of them to stitch into a single combined reference YAML.

Usage:
    python stitch_references.py
"""

import sys
from pathlib import Path
from typing import List, Tuple

import numpy as np
import yaml

# ── Configuration ─────────────────────────────────────────────────────────────

REFERENCES_DIR = Path(__file__).resolve().parent / "references"

# ── I/O ───────────────────────────────────────────────────────────────────────

def find_yaml_files(directory: Path) -> List[Path]:
    """Return all YAML files in the given directory (non-recursive), sorted by name."""
    if not directory.exists():
        sys.exit(f"[ERROR] References directory not found: {directory}")
    files = sorted(directory.glob("*.yaml")) + sorted(directory.glob("*.yml"))
    if not files:
        sys.exit(f"[ERROR] No YAML files found in {directory}")
    return files


def make_output_path(directory: Path, selected_files: List[Path]) -> Path:
    """Build the output filename by concatenating the stems of the selected files."""
    combined_name = "_".join(f.stem for f in selected_files) + ".yaml"
    return directory / combined_name


def load_coords(path: Path) -> Tuple[np.ndarray, np.ndarray]:
    """Load xCoords and yCoords from a reference YAML file."""
    with open(path, "r") as f:
        data = yaml.safe_load(f)
    try:
        ref = data["reference"]
        x = np.array(ref["xCoords"], dtype=float)
        y = np.array(ref["yCoords"], dtype=float)
    except KeyError as e:
        sys.exit(f"[ERROR] Missing key {e} in {path.name}")
    return x, y


def save_stitched(path: Path, x: np.ndarray, y: np.ndarray,
                  source_files: List[Path]) -> None:
    """Save stitched coordinates into a reference YAML file."""
    # Record which files were stitched to produce this output
    source_names = [f.name for f in source_files]
    data = {
        "reference": {
            "stitchedFrom": source_names,
            "xCoords": x.tolist(),
            "yCoords": y.tolist(),
        }
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)
    print(f"\n[OK] Stitched reference saved → {path}")


# ── UI helpers ────────────────────────────────────────────────────────────────

def print_menu(files: List[Path]) -> None:
    """Print the numbered list of available YAML files."""
    print("\nAvailable reference files:")
    print("─" * 40)
    for i, f in enumerate(files, start=1):
        with open(f, "r") as fh:
            data = yaml.safe_load(fh)
        try:
            n = len(data["reference"]["xCoords"])
        except (KeyError, TypeError):
            n = "?"
        print(f"  {i:>2}.  {f.name:<35}  ({n} pts)")
    print("─" * 40)


def parse_sequence(raw: str, num_files: int) -> List[int]:
    """Parse a whitespace- or comma-separated string of 1-based indices."""
    tokens = raw.replace(",", " ").split()
    if not tokens:
        sys.exit("[ERROR] No numbers entered.")
    indices = []
    for tok in tokens:
        if not tok.isdigit():
            sys.exit(f"[ERROR] '{tok}' is not a valid number.")
        idx = int(tok)
        if idx < 1 or idx > num_files:
            sys.exit(f"[ERROR] {idx} is out of range (1–{num_files}).")
        indices.append(idx - 1)   # convert to 0-based
    return indices


# ── Core ──────────────────────────────────────────────────────────────────────

def stitch(files: List[Path], indices: List[int]) -> Tuple[np.ndarray, np.ndarray]:
    """Load and concatenate coords for the selected files in order."""
    all_x, all_y = [], []
    for idx in indices:
        path = files[idx]
        x, y = load_coords(path)
        all_x.append(x)
        all_y.append(y)
        print(f"  + {path.name}  ({len(x)} pts)")
    return np.concatenate(all_x), np.concatenate(all_y)


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    files = find_yaml_files(REFERENCES_DIR)

    print_menu(files)

    print("\nEnter the sequence of numbers to stitch (space- or comma-separated).")
    print("Example:  2 1 3   or   2, 1, 3")
    raw = input("\nSequence: ").strip()

    indices = parse_sequence(raw, len(files))

    # Resolve selected file paths and derive the output filename from them
    selected_files = [files[i] for i in indices]
    output_file = make_output_path(REFERENCES_DIR, selected_files)

    # Exclude the output file from the available list to avoid self-stitching
    if output_file.resolve() in {f.resolve() for f in files}:
        files = [f for f in files if f.resolve() != output_file.resolve()]

    print(f"\nStitching {len(indices)} file(s) in order:")
    x_out, y_out = stitch(files, indices)

    print(f"\nTotal points: {len(x_out)}")
    save_stitched(output_file, x_out, y_out, selected_files)


if __name__ == "__main__":
    main()