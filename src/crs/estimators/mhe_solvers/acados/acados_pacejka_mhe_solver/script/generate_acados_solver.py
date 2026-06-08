import argparse
import os
import numpy as np
import rospkg
import yaml
from scipy.linalg import block_diag
from casadi import SX, vertcat, mtimes, reshape, sqrt, fabs
from typing import List
import shutil

from casadi import vertcat

from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
from pacejka_model import pacejka_model
from sensor_models.wheel_encoder_model import wheel_encoder_model
from sensor_models.imu_model import imu_model
from sensor_models.lighthouse_sweep_1_model import lighthouse_sweep_1_model
from sensor_models.lighthouse_sweep_2_model import lighthouse_sweep_2_model


def build_acados_solver(
    N: int,
    Ts: float,
    cost_type: str,
    sensors: List[str],
    acados_source_path: str,
    model,
):
    """
    Creates the required C-Code for the acados solver

    Parameters
    ----------
    N : int
        Time Horizon
    Ts : float
        Sampling Time
    cost_type : str
                Type of cost function. Must be one of "LINEAR_LS" or "NONLINEAR_LS"
    sensors : [str]
              List of sensors used for the MHE
    acados_source_path : str
        Path to acados source location
    model: Callable
        Model function e.g. pacejka_model

    Returns
    -------
    None
    """

    for sensor in sensors:
        assert sensor in [
            "mocap",
            "imu",
            "imu_yaw_rate",
            "wheel_encoders",
            "lighthouse",
        ], f"Sensor type {sensor} not supported."

    # create render arguments
    ocp = AcadosOcp()

    # export model
    model, constraint = model()

    # define acados ODE
    model_ac = AcadosModel()
    model_ac.f_impl_expr = model.f_impl_expr
    model_ac.f_expl_expr = model.f_expl_expr
    model_ac.x = model.x
    model_ac.xdot = model.xdot
    model_ac.u = model.u
    model_ac.z = model.z
    model_ac.name = model.name
    model_ac.p = model.p
    ocp.model = model_ac
    model_ac.con_h_expr = None

    # Mocap measurement model
    R_mocap = np.eye(3)
    H_mocap = np.concatenate([np.eye(3), np.zeros((3, 3))], axis=1)  # dim(H) = 3x6

    # IMU measurement model
    R_imu = np.eye(3)
    f_imu = imu_model(ocp.model)  # dim(f_imu) = 3x1

    # IMU yaw rate measurement model
    R_imu_yaw_rate = np.eye(1)
    H_imu_yaw_rate = np.array([[0, 0, 0, 0, 0, 1]])  # dim(H) = 1x6

    # Wheel encoder measurement model
    R_wheel_encoders = np.eye(4)
    f_wheel_encoders = wheel_encoder_model(ocp.model)  # dim(f_wheel_encoders) = 4x1

    # Lighthouse measurement model
    R_lighthouse_sweep_1 = np.eye(4)
    f_lighthouse_sweep_1, model.p = lighthouse_sweep_1_model(ocp.model)  # 4x1

    R_lighthouse_sweep_2 = np.eye(4)
    f_lighthouse_sweep_2, model.p = lighthouse_sweep_2_model(ocp.model)  # 4x1

    model_ac.p = model.p
    nparam = model.p.size()[0]

    x = ocp.model.x  # dim(x) = 6
    # the inputs to the problem is the process noise w for each state -> dim(u) = 6
    u = ocp.model.u
    nx = x.size()[0]
    nu = u.size()[0]

    nz = 0
    if "mocap" in sensors:
        # number of scalar measurements (observation matrix for linear measurement model)
        nz += np.size(H_mocap, 0)
    if "imu" in sensors:
        nz += 3
    if "imu_yaw_rate" in sensors:
        nz += np.size(H_imu_yaw_rate, 0)
    if "wheel_encoders" in sensors:
        nz += 4
    if "lighthouse" in sensors:
        nz += 8  # 4 angles per lighthouse

    ny_0 = nx + nu + nz  # f(x), w and nr of measurements
    ny = nu + nz  # f(x), w

    P = np.eye(nx)  # dim(P) = 6x6
    Q = np.eye(nx)  # dim(Q) = 6x6

    ocp.dims.nx = nx
    ocp.dims.ny = ny
    ocp.dims.np = model.p.size()[0]
    ocp.dims.nbx = nx  # number of state bounds
    ocp.dims.nbu = nu
    ocp.dims.nu = nu
    ocp.dims.nh = 0  # number of nonlinear constraints
    ocp.dims.nsh = 0  # number of soft constraints
    ocp.dims.ns = nx + nu + 0  # num of slack vars

    time_steps = np.zeros(N)
    for i in range(N):
        time_steps[i] = Ts
    print("Time steps: ", time_steps)
    ocp.solver_options.time_steps = time_steps

    ocp.dims.nsbx = nx
    ocp.dims.nsbu = nu

    if cost_type == "LINEAR_LS":
        assert (
            "imu" not in sensors
            and "wheel_encoders" not in sensors
            and "lighthouse" not in sensors
        ), "The linear least squares cost function does not support the imu, wheel encoders or lighthouse sensors."

        # arrival cost
        ocp.cost.cost_type_0 = "LINEAR_LS"
        R_mat = []
        nz = 0
        if "mocap" in sensors:
            R_mat.append(np.linalg.inv(R_mocap))
        if "imu_yaw_rate" in sensors:
            R_mat.append(np.linalg.inv(R_imu_yaw_rate))
        R = block_diag(*R_mat)
        ocp.cost.W_0 = block_diag(np.linalg.inv(P), np.linalg.inv(Q), R)

        ocp.cost.Vx_0 = np.zeros((ny_0, nx))
        ocp.cost.Vx_0[:nx, :] = np.eye(nx)
        ocp.cost.Vx_0[(nx + nu) : (nx + nu + np.size(H_mocap, 0)), :] = H_mocap
        ocp.cost.Vx_0[(nx + nu + np.size(H_mocap, 0)) :, :] = H_imu_yaw_rate
        # remember: 'u' corresponds to the process noise w!
        ocp.cost.Vu_0 = np.zeros((ny_0, nu))
        ocp.cost.Vu_0[nx : nx + nu, :] = np.eye(nu)
        ocp.cost.yref_0 = np.zeros((ny_0,))

        ocp.cost.cost_type = "LINEAR_LS"

        ocp.cost.W = block_diag(np.linalg.inv(Q), R)
        ocp.cost.Vx = np.zeros((ny, nx))
        ocp.cost.Vx[nu : (nu + np.size(H_mocap, 0)), :] = H_mocap
        ocp.cost.Vx[(nu + np.size(H_mocap, 0)) :, :] = H_imu_yaw_rate
        # remember: 'u' correspondsto the process noise w!
        ocp.cost.Vu = np.zeros((ny, nu))
        ocp.cost.Vu[:nu, :] = np.eye(nu)
        ocp.cost.yref = np.zeros((ny,))
        # ocp.model.cost_y_expr = vertcat(u, mtimes(H_mocap, reshape(x, nx, 1)))
        cost = []
        if "mocap" in sensors:
            cost.append(mtimes(H_mocap, reshape(x, nx, 1)))
        if "imu_yaw_rate" in sensors:
            cost.append(mtimes(H_imu_yaw_rate, reshape(x, nx, 1)))

        # ocp.model.cost_y_expr = vertcat(u, *cost)
        ocp.parameter_values = np.zeros((nparam,))

    elif cost_type == "NONLINEAR_LS":
        # arrival cost
        ocp.cost.cost_type_0 = "NONLINEAR_LS"
        R_mat = []
        nz = 0
        if "mocap" in sensors:
            R_mat.append(np.linalg.inv(R_mocap))
        if "imu" in sensors:
            R_mat.append(np.linalg.inv(R_imu))
        if "imu_yaw_rate" in sensors:
            R_mat.append(np.linalg.inv(R_imu_yaw_rate))
        if "wheel_encoders" in sensors:
            R_mat.append(np.linalg.inv(R_wheel_encoders))
        if "lighthouse" in sensors:
            R_mat.append(np.linalg.inv(R_lighthouse_sweep_1))
            R_mat.append(np.linalg.inv(R_lighthouse_sweep_2))

        R = block_diag(*R_mat)
        ocp.cost.W_0 = block_diag(np.linalg.inv(P), np.linalg.inv(Q), R)

        cost = []
        if "mocap" in sensors:
            cost.append(mtimes(H_mocap, reshape(x, nx, 1)))
        if "imu" in sensors:
            cost.append(f_imu)
        if "imu_yaw_rate" in sensors:
            cost.append(mtimes(H_imu_yaw_rate, reshape(x, nx, 1)))
        if "wheel_encoders" in sensors:
            cost.append(f_wheel_encoders)
        if "lighthouse" in sensors:
            cost.append(f_lighthouse_sweep_1)
            cost.append(f_lighthouse_sweep_2)

        ocp.model.cost_y_expr_0 = vertcat(x, u, *cost)  # TODO: check, if correct
        ocp.cost.yref_0 = np.zeros((ny_0,))

        # intermediate cost
        ocp.cost.cost_type = "NONLINEAR_LS"
        ocp.cost.W = block_diag(np.linalg.inv(Q), R)
        ocp.model.cost_y_expr = vertcat(u, *cost)
        ocp.parameter_values = np.zeros((nparam,))
        ocp.cost.yref = np.zeros((ny,))

    else:
        Exception("Unknown cost type")

    # terminal cost
    ocp.cost.cost_type_e = "LINEAR_LS"
    ocp.cost.W_e = np.zeros((0, 0))
    ocp.cost.Vx_e = np.zeros((0, 0))
    ocp.cost.yref_e = np.zeros((0,))

    ocp.cost.zl = 100 * np.ones(nu + nx + 0)
    ocp.cost.zu = 100 * np.ones(nu + nx + 0)
    ocp.cost.Zl = np.zeros(nu + nx + 0)
    ocp.cost.Zu = np.zeros(nu + nx + 0)

    # set slacks at initial stage (only for input bounds)
    ocp.cost.zl_0 = 100 * np.ones(nu + 0)
    ocp.cost.zu_0 = 100 * np.ones(nu + 0)
    ocp.cost.Zl_0 = np.zeros(nu + 0)
    ocp.cost.Zu_0 = np.zeros(nu + 0)

    # set constraints
    ocp.constraints.lsh = np.zeros(ocp.dims.nsh)
    ocp.constraints.ush = np.zeros(ocp.dims.nsh)
    ocp.constraints.idxsh = np.array([])

    ocp.constraints.lsbx = np.zeros(ocp.dims.nsbx)
    ocp.constraints.usbx = np.zeros(ocp.dims.nsbx)
    ocp.constraints.idxsbx = np.arange(nx)

    ocp.constraints.lsbu = np.zeros(ocp.dims.nsbu)
    ocp.constraints.usbu = np.zeros(ocp.dims.nsbu)
    ocp.constraints.idxsbu = np.arange(nu)

    # State Constraints
    ocp.constraints.lbx = np.array(  # lower bounds on x
        [
            model.x_min,
            model.y_min,
            model.yaw_min,
            model.vx_min,
            model.vy_min,
            model.dyaw_min,
        ]
    )

    ocp.constraints.ubx = np.array(  # upper bounds on x
        [
            model.x_max,
            model.y_max,
            model.yaw_max,
            model.vx_max,
            model.vy_max,
            model.dyaw_max,
        ]
    )

    ocp.constraints.idxbx = np.arange(nx)  # indices of bounds on x

    # Input Constraints (Reminder: Inputs = Process noise w!)
    ocp.constraints.lbu = np.array(  # lower bounds on u
        [
            model.w_x_min,
            model.w_y_min,
            model.w_psi_min,
            model.w_vx_min,
            model.w_vy_min,
            model.w_omega_min,
        ]
    )
    ocp.constraints.ubu = np.array(  # upper bounds on u
        [
            model.w_x_max,
            model.w_y_max,
            model.w_psi_max,
            model.w_vx_max,
            model.w_vy_max,
            model.w_omega_max,
        ]
    )

    ocp.constraints.idxbu = np.arange(nu)  # indices of bounds on u

    # set intial condition
    ocp.parameter_values = np.zeros(nparam)

    # set QP solver and integration
    ocp.solver_options.N_horizon = N
    ocp.solver_options.Tsim = Ts
    ocp.solver_options.tf = Ts * N
    ocp.solver_options.qp_solver = (
        "PARTIAL_CONDENSING_HPIPM"  # Other options: "FULL_CONDENSING_HPIPM"
    )
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"  # Other options: "SQP"

    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"

    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 3
    ocp.solver_options.levenberg_marquardt = 1e-5
    ocp.solver_options.print_stats = 0
    ocp.solver_options.tol = 1e-3
    ocp.solver_options.nlp_solver_max_iter = 100
    ocp.solver_options.qp_solver_iter_max = 150
    ocp.solver_options.qp_solver_warm_start = 1

    ocp.solver_options.nlp_solver_step_length = 0.75

    ocp.acados_include_path = acados_source_path + "/include"
    ocp.acados_lib_path = acados_source_path + "/lib"

    ocp.code_export_directory = "lib"

    # create solver with agent specific code files
    filename = "lib/acados_pacejka_mhe_solver_config.json"
    AcadosOcpSolver.generate(ocp, json_file=filename)
    # This generates the file "c_generated_code/acados_solver_pacejka_model.h", we patch it to include the compiled sensor list.
    # This is a hack, but it works.
    with open("lib/acados_solver_pacejka_model.h", "r") as f:
        lines = f.readlines()
    # Find start of struct definition
    try:
        location = lines.index("typedef struct pacejka_model_solver_capsule\n")
        sensors_as_string = "{" + ", ".join([f'"{sensor}"' for sensor in sensors]) + "}"
        lines.insert(
            location,
            f"// AUTO-GENERATED. Added sensor list for sanity check. \nextern const char* SOLVER_MHE_SENSOR_LIST[{len(sensors)}];\n",
        )
        lines.insert(location, f"extern const int SOLVER_MHE_NUM_SENSORS;\n")
        # Overwrite file with new content
        with open("lib/acados_solver_pacejka_model.h", "w") as f:
            f.writelines(lines)
    except ValueError as e:
        print(
            "Could not find struct definition in acados_solver_pacejka_model.h. Will not be able to add sensor list to created .h file. Did you change the acados template?"
        )
        raise e

    with open("lib/acados_solver_pacejka_model.c", "r") as f:
        lines = f.readlines()

    # Definitions of the custom variables in the source files to prevent multiple definitions error
    try:
        location = 81
        lines.insert(location, f"const int SOLVER_MHE_NUM_SENSORS = {len(sensors)};\n")
        lines.insert(
            location + 1,
            f"// AUTO-GENERATED. Added sensor list for sanity check. \nconst char* SOLVER_MHE_SENSOR_LIST[{len(sensors)}] = {sensors_as_string};\n",
        )
        # Overwrite file with new content
        with open("lib/acados_solver_pacejka_model.c", "w") as f:
            f.writelines(lines)
    except ValueError as e:
        print("Failed to add the custom definitions to the auto generated file.")
        raise e


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Create a new Acados MHE solver for the pacejka model"
    )
    parser.add_argument(
        "--config",
        type=str,
        help="Name of the configuration file",
        default="solver.yaml",
    )
    parser.add_argument(
        "--acados_source",
        type=str,
        help="Path to the acados source",
        default=os.environ["ACADOS_SOURCE_DIR"],
    )
    args = parser.parse_args()

    try:
        rospack = rospkg.RosPack()
        controller_path = rospack.get_path("acados_pacejka_mhe_solver")
        os.chdir(controller_path)
    except:
        print(
            "Did not find rospackage acados_pacejka_mhe_solver. Assuming script is called from the scripts folder."
        )
        os.chdir("../src")
        controller_path = "../"

    if os.path.exists(os.path.join(controller_path, "lib")):
        shutil.rmtree(os.path.join(controller_path, "lib"))

    with open(os.path.join(controller_path, "config", args.config)) as f:
        cfg = yaml.load(f, Loader=yaml.loader.SafeLoader)

        def loadModel():
            return pacejka_model(cfg["model_bounds"])

        build_acados_solver(
            cfg["solver_creation"]["N"],
            cfg["solver_creation"]["Ts"],
            cfg["solver_creation"]["cost_type"],
            cfg["solver_creation"]["sensors"],
            args.acados_source,
            loadModel,
        )
