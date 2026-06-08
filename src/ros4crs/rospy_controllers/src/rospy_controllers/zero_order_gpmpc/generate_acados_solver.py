import argparse
from subprocess import check_output
import os
import numpy as np
import rospkg
import yaml

import shutil

from acados_template import (
    AcadosModel,
    AcadosOcp,
    AcadosSim,
    ZoroDescription,
)
from .pacejka_model_zogpmpc import (
    pacejka_model_acados_gpzoro,
    USE_RADIUS_CONSTRAINT,
    USE_RADIUS_CONSTRAINT_SQRT,
    USE_LINEAR_THETA_CORRECTION,
    USE_CONVEX_OVER_NONLINEAR,
)

from scipy.stats import norm
from scipy.linalg import block_diag
import casadi as ca


def generate_acados_ocp(
    N: int, Ts: float, acados_source_path: str, model, config: dict, ctrl_params: dict
):
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

    rospy_controllers_dir = "/code/src/ros4crs/rospy_controllers/src/rospy_controllers"
    print(f"switching working directory to: {rospy_controllers_dir}")
    os.chdir(rospy_controllers_dir)

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
    model_ac.p = model.p
    model_ac.name = model.name
    print(f"Pacejka model name: {model_ac.name}")
    ocp.model = model_ac

    # set dimensions
    nx = model.x.size()[0]
    nu = model.u.size()[0]

    ocp.dims.nx = nx
    ocp.dims.np = model.p.size()[0]
    ocp.dims.nbx = nx
    ocp.dims.nbu = nu
    ocp.dims.nu = nu
    ocp.dims.N = N

    nsbx = nx - nu
    ocp.dims.nsbx = nsbx
    ocp.dims.nsbu = 0

    ny_0 = 6
    ny = 6
    ny_e = 0
    # ny_e = 2
    ocp.dims.ny_0 = ny_0
    ocp.dims.ny = ny
    ocp.dims.ny_e = ny_e
    ocp.cost.cost_type_0 = "NONLINEAR_LS"
    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.cost.cost_type_e = "NONLINEAR_LS"

    # Q1_C = model.p[6]
    # Q2_L = model.p[7]
    # R1_dT = model.p[8]
    # R2_ddelta = model.p[9]
    # R3_dtheta = model.p[10]
    # q_dtheta = model.p[11]
    Q1_C = ctrl_params["Q1"]
    Q2_L = ctrl_params["Q2"]
    R1_dT = ctrl_params["R1"]
    R2_ddelta = ctrl_params["R2"]
    R3_dtheta = ctrl_params["R3"]
    q_dtheta = ctrl_params["q"]
    eps = q_dtheta**2 / 4.0 / R3_dtheta + 1e-1

    # NOTE: cost needs to be defined with numpy arrays, not casadi
    ocp_cost_W_dtheta = np.array([[R3_dtheta, -q_dtheta / 2.0], [-q_dtheta / 2.0, eps]])
    ocp.cost.W_0 = block_diag(Q1_C, Q2_L, R1_dT, R2_ddelta, ocp_cost_W_dtheta)
    ocp.cost.W = block_diag(Q1_C, Q2_L, R1_dT, R2_ddelta, ocp_cost_W_dtheta)
    ocp.cost.W_e = np.zeros((ny_e, ny_e))
    # ocp.cost.W_e = block_diag(Q1_C, Q2_L)

    print(f"ctrl_params: {ctrl_params}")
    print(f"ocp cost W: {ocp.cost.W}")

    ocp.cost.yref_0 = np.zeros((ny_0,))
    ocp.cost.yref = np.zeros((ny,))
    ocp.cost.yref_e = np.zeros((ny_e,))
    ocp.model.cost_y_expr_0 = model.cost_y_expr_0
    ocp.model.cost_y_expr = model.cost_y_expr
    ocp.model.cost_y_expr_e = model.cost_y_expr_e

    # define constraint
    car_width = 0.12
    track_width = config["track"]["track_width"]
    safety_margin = config["track"]["safety_margin"]
    w = 0.5 * (track_width - car_width) - safety_margin

    if USE_RADIUS_CONSTRAINT:
        if USE_RADIUS_CONSTRAINT_SQRT:
            lh_val = np.array([0.0])
            uh_val = np.array([w])
        else:
            lh_val = np.array([0.0])
            uh_val = np.array([w * w])
    else:
        lh_val = np.array([-w])
        uh_val = np.array([w])

    if USE_CONVEX_OVER_NONLINEAR:
        nphi = constraint.con_phi_expr.size()[0]
        ocp.dims.nphi = nphi
        ocp.dims.nsphi = nphi
        ocp.dims.ns = ocp.dims.nsbx + ocp.dims.nsphi

        model_ac.con_r_expr = constraint.con_r_expr
        model_ac.con_phi_expr = constraint.con_phi_expr
        model_ac.con_r_in_phi = constraint.con_r_in_phi

        ocp.constraints.lphi = lh_val
        ocp.constraints.uphi = uh_val
        ocp.constraints.idxsphi = np.array(range(ocp.dims.nsphi))
    else:
        nh = constraint.expr.size()[0]
        ocp.dims.nh = nh
        ocp.dims.nsh = nh
        ocp.dims.ns = ocp.dims.nsbx + ocp.dims.nsh

        model_ac.con_h_expr = constraint.expr

        ocp.constraints.lh = lh_val
        ocp.constraints.uh = uh_val
        ocp.constraints.idxsh = np.array(range(ocp.dims.nsh))

    enable_terminal_constr = True
    if enable_terminal_constr:
        if USE_CONVEX_OVER_NONLINEAR:
            model_ac.con_phi_expr_e = constraint.con_phi_expr
            model_ac.con_r_expr_e = constraint.con_r_expr
            model_ac.con_r_in_phi_e = constraint.con_r_in_phi

            ocp.dims.nphi_e = ocp.dims.nphi
            ocp.dims.nsphi_e = ocp.dims.nsphi
            ocp.dims.ns_e = ocp.dims.nsphi_e + ocp.dims.nsbx

            ocp.constraints.lphi_e = ocp.constraints.lphi
            ocp.constraints.uphi_e = ocp.constraints.uphi
            ocp.constraints.idxsphi_e = np.array(range(ocp.dims.nsphi_e))
            ocp.constraints.constr_type = "BGP"
        else:
            model_ac.con_h_expr_e = constraint.expr

            ocp.dims.nh_e = ocp.dims.nh
            ocp.dims.nsh_e = ocp.dims.nsh
            ocp.dims.ns_e = ocp.dims.nsh_e + ocp.dims.nsbx

            ocp.constraints.lh_e = ocp.constraints.lh
            ocp.constraints.uh_e = ocp.constraints.uh
            ocp.constraints.idxsh_e = np.array(range(ocp.dims.nsh_e))
            ocp.constraints.constr_type = "BGH"

        ocp.cost.Zl_e = config["solver_creation"]["cost"]["Zl"] * np.ones(
            ocp.dims.ns_e
        )  # 4 12
        ocp.cost.Zu_e = config["solver_creation"]["cost"]["Zu"] * np.ones(ocp.dims.ns_e)
        ocp.cost.zl_e = config["solver_creation"]["cost"]["zl"] * np.ones(ocp.dims.ns_e)
        ocp.cost.zu_e = config["solver_creation"]["cost"]["zu"] * np.ones(ocp.dims.ns_e)

    ocp.constraints.idxsbx = np.arange(
        nsbx
    )  # hard constraints on last 3 states = inputs
    ocp.constraints.idxsbx_e = np.arange(
        nsbx
    )  # hard constraints on last 3 states = inputs

    # works for robust
    ocp.cost.Zl_0 = config["solver_creation"]["cost"]["Zl"] * np.ones(
        ocp.dims.ns_0
    )  # 4 12
    ocp.cost.Zu_0 = config["solver_creation"]["cost"]["Zu"] * np.ones(ocp.dims.ns_0)
    ocp.cost.zl_0 = config["solver_creation"]["cost"]["zl"] * np.ones(ocp.dims.ns_0)
    ocp.cost.zu_0 = config["solver_creation"]["cost"]["zu"] * np.ones(ocp.dims.ns_0)
    ocp.cost.Zl = config["solver_creation"]["cost"]["Zl"] * np.ones(ocp.dims.ns)  # 4 12
    ocp.cost.Zu = config["solver_creation"]["cost"]["Zu"] * np.ones(ocp.dims.ns)
    ocp.cost.zl = config["solver_creation"]["cost"]["zl"] * np.ones(ocp.dims.ns)
    ocp.cost.zu = config["solver_creation"]["cost"]["zu"] * np.ones(ocp.dims.ns)

    # setting constraints
    ocp.constraints.lbx = np.array(  # lower bounds on x
        [
            model.x_min,
            model.y_min,
            model.yaw_min,
            model.vx_min,
            model.vy_min,
            model.dyaw_min,
            model.T_min,
            model.delta_min,
            model.theta_min,
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
            model.T_max,
            model.delta_max,
            model.theta_max,
        ]
    )
    # setting constraints
    ocp.constraints.lbx_e = np.array(  # lower bounds on x
        [
            model.x_min,
            model.y_min,
            model.yaw_min,
            model.vx_min,
            model.vy_min,
            model.dyaw_min,
            model.T_min,
            model.delta_min,
            model.theta_min,
        ]
    )

    ocp.constraints.ubx_e = np.array(  # upper bounds on x
        [
            model.x_max,
            model.y_max,
            model.yaw_max,
            model.vx_max,
            model.vy_max,
            model.dyaw_max,
            model.T_max,
            model.delta_max,
            model.theta_max,
        ]
    )

    ocp.constraints.idxbx = np.arange(nx)  # indices of bounds on x
    ocp.constraints.idxbx_e = np.arange(nx)  # indices of bounds on x

    ocp.constraints.lbu = np.array(  # lower bounds on u
        [model.dT_min, model.ddelta_min, model.dtheta_min]
    )
    ocp.constraints.ubu = np.array(  # upper bounds on u
        [model.dT_max, model.ddelta_max, model.dtheta_max]
    )
    ocp.constraints.idxbu = np.arange(nu)  # indices of bounds on u

    # set intial condition
    ocp.constraints.x0 = np.zeros(nx)
    ocp.constraints.idxbx_0 = np.arange(nx)
    ocp.parameter_values = np.zeros(ocp.dims.np)
    # set QP solver and integration
    ocp.solver_options.Tsim = Ts
    ocp.solver_options.tf = Ts * N
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    # ocp.solver_options.hessian_approx = "EXACT"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 3
    ocp.solver_options.print_level = 0
    # ocp.solver_options.regularize_method = "MIRROR"  #  "PROJECT"  #
    # ocp.solver_options.reg_epsilon = 5e-2
    ocp.solver_options.levenberg_marquardt = config["solver_creation"]["solver_opts"][
        "levenberg_marquardt"
    ]

    ocp.solver_options.tol = 1e-4

    ocp.acados_include_path = acados_source_path + "/include"
    ocp.acados_lib_path = acados_source_path + "/lib"

    # add zoro_description
    ocp.zoro_description = generate_zoro_description(
        config["solver_creation"]["zoro_description"], ocp
    )

    return ocp


def generate_zoro_description(cfg_zoro: dict, ocp: AcadosOcp):
    B = np.diag(cfg_zoro["unc_jac_G_mat_idx"])
    B = B[:, ~np.all(B == 0, axis=0)]

    zoro_description = ZoroDescription()
    zoro_description.backoff_scaling_gamma = float(cfg_zoro["backoff_scaling_gamma"])
    zoro_description.P0_mat = np.diag(cfg_zoro["P0_mat_diag"])
    zoro_description.W_mat = np.diag(cfg_zoro["W_mat_diag"])
    zoro_description.fdbk_K_mat = np.zeros((ocp.dims.nu, ocp.dims.nx))
    zoro_description.unc_jac_G_mat = B
    zoro_description.input_P0_diag = True
    zoro_description.input_P0 = False
    zoro_description.input_W_diag = True
    zoro_description.input_W_add_diag = True
    zoro_description.output_P_matrices = True

    if USE_CONVEX_OVER_NONLINEAR:
        if USE_RADIUS_CONSTRAINT:
            zoro_description.idx_uphi_t = [0]
        else:
            zoro_description.idx_uphi_t = [0]
            zoro_description.idx_lphi_t = [0]
    else:
        if USE_RADIUS_CONSTRAINT:
            zoro_description.idx_uh_t = [0]
        else:
            zoro_description.idx_uh_t = [0]
            zoro_description.idx_lh_t = [0]

    zoro_description.idx_lbx_t = list(range(ocp.dims.nx))
    zoro_description.idx_ubx_t = list(range(ocp.dims.nx))

    return zoro_description


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Create a new Acados MPCC solver for the pacejka model"
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

    rospack = rospkg.RosPack()
    controller_path = rospack.get_path("rospy_controllers")
    os.chdir(controller_path + "/src")

    with open(f"../config/{args.config}") as f:
        cfg = yaml.load(f, Loader=yaml.loader.SafeLoader)

        def loadModel():
            return pacejka_model_acados_gpzoro(cfg["model_bounds"])

        build_acados_solver(
            cfg["solver_creation"]["N"],
            cfg["solver_creation"]["Ts"],
            args.acados_source,
            cfg,
            loadModel,
        )
