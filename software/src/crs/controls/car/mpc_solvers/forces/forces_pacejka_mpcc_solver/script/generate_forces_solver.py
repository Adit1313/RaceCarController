import argparse
import os
import numpy as np
import rospkg
import yaml

import numpy as np
import forcespro
import forcespro.nlp

import utils


def build_solver(N: int, Ts: float, cfg: dict):
    """
    Creates the required C-Code for the acados solver

    Parameters
    ----------
    N : int
        Time Horizon
    Ts : float
        Sampling Time
    acados_source_path : str
        Path to acados source location
    model: Callable
        Model function e.g. pacejka_model

    Returns
    -------
    None
    """
    npar = 26  # number of parameters per stage
    nvar = 12
    car_dim = 0.07  # move to config?
    half_track_width = 0.46 / 2  # move to config?

    # dimensions
    model = forcespro.nlp.SymbolicModel()
    model.N = N  # set horizon length
    model.nvar = nvar  # number of variables
    model.npar = npar  # set number of parameters per stage for solver
    model.neq = 9  # number of equality constraints
    model.nh = 1  # number of nonlinear inequality constraints

    # objective
    model.objective = utils.cost_function_pacejka

    # equalities
    def rk4_with_freq(z, p):
        return utils.dynamics_RK4(z, p, freq=1 / Ts)

    model.eq = rk4_with_freq
    model.E = np.block([np.eye(9), np.zeros([9, 3])])

    # inequalities
    model.ineq = utils.inequality_constraint
    model.hu = (half_track_width - car_dim) * (half_track_width - car_dim)
    model.hl = 0

    # initial state indeces (9:11 are inputs to the system)
    model.xinitidx = range(0, 9)

    # inequalities lower and upper bounds for state vector defined at head of
    # this subscript
    # z =              ([   x,   y,     yaw,   vx,    vy,  dyaw,  torque,  steer,  Theta,     dtorque,    dsteer,    dtheta])
    constraints = cfg["model_bounds"]

    model.lb = np.array(
        [
            constraints["x_min"],
            constraints["y_min"],
            constraints["yaw_min"],
            constraints["vx_min"],
            constraints["vy_min"],
            constraints["dyaw_min"],
            constraints["T_min"],
            constraints["delta_min"],
            constraints["theta_min"],
            constraints["dT_min"],
            constraints["ddelta_min"],
            constraints["dtheta_min"],
        ]
    )

    model.ub = np.array(
        [
            constraints["x_max"],
            constraints["y_max"],
            constraints["yaw_max"],
            constraints["vx_max"],
            constraints["vy_max"],
            constraints["dyaw_max"],
            constraints["T_max"],
            constraints["delta_max"],
            constraints["theta_max"],
            constraints["dT_max"],
            constraints["ddelta_max"],
            constraints["dtheta_max"],
        ]
    )
    # Define solver options.
    codeoptions = forcespro.CodeOptions("FORCESNLPsolver")

    codeoptions.maxit = 150  # Maximum number of iterations
    # codeoptions.solver_timeout = 2 # timeout enabled
    # codeoptions.timeout_estimate_coeff = Ts / 2 # Set timeout to half the sampling time

    codeoptions.printlevel = (
        0  # Use printlevel = 2 to print progress (but not for timings)
    )
    codeoptions.optlevel = 2
    # 0: no optimization, 1: optimize for size, 2: optimize for speed, 3: optimize for size & speed
    codeoptions.nlp.checkFunctions = 0
    codeoptions.nlp.linear_solver = "symm_indefinite_fast"
    codeoptions.noVariableElimination = 1
    codeoptions.overwrite = 1  # 1: overwrite existing solver
    codeoptions.cleanup = 0
    codeoptions.BuildSimulinkBlock = 0
    codeoptions.nohash = 0
    codeoptions.parallel = 1
    codeoptions.sse = 1
    codeoptions.license.use_floating_license = 1
    codeoptions.platform = "Docker-Gnu-x86_64"
    codeoptions.license.floating_license_server = "host.docker.internal"
    codeoptions.license.floating_license_port = "53135"

    # define outputs and generate solver
    outputs = [
        ("s0", 0, range(0, 12)),
        ("theta", range(0, N), 8),
        ("dtheta", range(0, N), 11),
        ("x", range(0, N), 0),
        ("y", range(0, N), 1),
        ("horizon", range(0, N), range(0, 12)),
    ]
    # Generate c code
    model.generate_solver(codeoptions, outputs)

    # Generate external function
    generate_FORCSEPRO_information(model, codeoptions)


def generate_FORCSEPRO_information(model, codeoptions):
    with open(codeoptions["name"] + "_info.h", "w") as fp:
        fp.write("#ifndef FORCESNLP_SOLVER_INFO_H\n")
        fp.write("#define FORCESNLP_SOLVER_INFO_H\n")
        fp.write("\n")
        fp.write(f"struct data_{codeoptions.name}\n")
        fp.write("{\n")
        fp.write("    int N;\n")
        fp.write("    double sampling_time;\n")
        fp.write("};\n")
        fp.write("\n")
        fp.write(f"struct data_{codeoptions.name} get_data_{codeoptions.name}();\n")
        fp.write("\n")
        fp.write("#endif // FORCESNLP_SOLVER_INFO_H")

    with open(codeoptions["name"] + "_info.cpp", "w") as fp:
        fp.write(f'#include "{codeoptions.name}_info.h"\n')
        fp.write("\n")
        fp.write(f"struct data_{codeoptions.name} get_data_{codeoptions.name}()\n")
        fp.write("{\n")
        fp.write(f"    struct data_{codeoptions.name} current_data;\n")
        fp.write(f"    current_data.N = {model.N};\n")
        fp.write(f"    current_data.sampling_time = {codeoptions.nlp.integrator.Ts};\n")
        fp.write("    return current_data;\n")
        fp.write("};\n")
        fp.write("\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Create a new Forces MPCC solver for the pacejka model"
    )
    parser.add_argument(
        "--config",
        type=str,
        help="Name of the configuration file",
        default="solver.yaml",
    )
    args = parser.parse_args()

    rospack = rospkg.RosPack()
    # commented out to be able to run it without ROS/Docker.
    # TODO, change this back
    # controller_path = rospack.get_path('forces_pacejka_mpcc_solver')
    # os.chdir( controller_path + "/src" )

    with open(f"../config/{args.config}") as f:
        cfg = yaml.load(f, Loader=yaml.loader.SafeLoader)
        build_solver(cfg["solver_creation"]["N"], cfg["solver_creation"]["Ts"], cfg)
