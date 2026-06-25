#! /usr/bin/python3
import hashlib
import importlib
import os
import shutil
import sys

from typing import List

import yaml

from tap import Tap


class CliArguments(Tap):
    """Create a new acados MPC solver with the specified config."""

    config: str
    """Name of the configuration file."""
    lib_dir: str
    """Where to store the generated code."""
    script_dir: str
    """where the ocp script"""
    dirs_to_hash: List[str] = ["config", "script"]
    """Directories and files which are considered in the hashing process."""
    additional_args: List[str] = []
    """Additional solver specific args to be passed to the solver creation."""


def hash_string(text: str) -> str:
    return hashlib.md5(text.encode()).hexdigest()


def generate_path_hash(path) -> str:
    hash_str = ""
    if os.path.isfile(path):
        return hash_string(path)

    for file in os.listdir(path):
        if os.path.isfile(file):
            with open(file, "r") as f:
                hash_str += hash_string(f.read())

    # hash the concatenated hashes once more
    return hash_string(hash_str)


def generate_current_hash(args: CliArguments) -> str:
    hash_str = ""
    for dir in args.dirs_to_hash:
        hash_str += generate_path_hash(dir)

    argstr = ""
    for key, val in sorted(args.as_dict().items()):
        argstr += str(key) + str(val)

    hash_str += hash_string(argstr)
    # Hash once more for good measure
    return hash_string(hash_str)


if __name__ == "__main__":
    args = CliArguments().parse_args()

    # Funky stuff so we can load the solver generation module from anywhere
    sys.path.append(args.script_dir)

    try:
        with open(os.path.join(args.lib_dir, "hash.txt"), "r") as last_hash_file:
            last_hash = last_hash_file.read()
    except FileNotFoundError:
        last_hash = ""

    current_hash = generate_current_hash(args)

    if current_hash == last_hash:
        print("No changes detected. Not recreating solver.")
        exit(0)

    if os.path.exists(args.lib_dir):
        shutil.rmtree(args.lib_dir)
        os.mkdir(args.lib_dir)

    with open(os.path.join(args.config)) as f:
        cfg = yaml.load(f, Loader=yaml.loader.SafeLoader)

    ocp_module = importlib.import_module("generate_solver")
    ocp_module.generate_solver(cfg, args.lib_dir, args.additional_args)

    with open(os.path.join(args.lib_dir, "hash.txt"), "w") as hash_file:
        hash_file.write(current_hash)
