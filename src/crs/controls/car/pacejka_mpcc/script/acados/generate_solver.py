import os

import numpy as np

from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
from pacejka_model import pacejka_model


def generate_solver(config, lib_dir, additional_args):
    """
    Creates the required C-Code for the acados solver

    Parameters
    ----------
    config: Dict
        Dictionary containing the configuration parameters. Usually loaded from config/solver.yaml
    lib_dir: str
        Directory where code should be generated into
    model: Callable
        Model function e.g. pacejka_model

    Returns
    -------
    None
    """
    N = config["solver_creation"]["N"]
    Ts = config["solver_creation"]["Ts"]
    acados_source_path = os.environ["ACADOS_SOURCE_DIR"]
    use_linear_constraint = "use_linear_constraint" in additional_args

    # create render arguments
    ocp = AcadosOcp()

    # export model
    model, constraint = pacejka_model(config["model_bounds"], use_linear_constraint)

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
    ocp.model = model_ac

    # define constraint
    model_ac.con_h_expr = constraint.expr

    # set dimensions
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = 0
    ny_e = 0

    ocp.dims.nx = nx
    ocp.dims.np = model.p.size()[0]
    ocp.dims.ny = ny
    ocp.dims.ny_e = ny_e
    ocp.dims.nbx = nx
    ocp.dims.nbu = nu
    ocp.dims.nu = nu
    ocp.dims.nh = 1
    ocp.dims.nsh = 1
    ocp.dims.ns = nx + nu + 1

    ocp.dims.nsbx = nx
    ocp.dims.nsbu = nu

    ocp.cost.cost_type = "EXTERNAL"
    ocp.cost.cost_type_e = "EXTERNAL"

    ocp.model.cost_expr_ext_cost = model.cost_expr_ext_cost
    ocp.model.cost_expr_ext_cost_e = 0

    car_width = 0.12 # FIXME!
    track_width = config["track"]["track_width"]
    safety_margin = config["track"]["safety_margin"]
    w = 0.5 * (track_width - car_width) - safety_margin
    lh, uh = constraint.bound_generator(w)
    ocp.constraints.lh = np.array([lh])
    ocp.constraints.uh = np.array([uh])

    ocp.constraints.lsh = np.zeros(ocp.dims.nsh)
    ocp.constraints.ush = np.zeros(ocp.dims.nsh)
    ocp.constraints.idxsh = np.array([0])

    ocp.constraints.lsbx = np.zeros(ocp.dims.nsbx)
    ocp.constraints.usbx = np.zeros(ocp.dims.nsbx)
    ocp.constraints.idxsbx = np.arange(nx)

    ocp.constraints.lsbu = np.zeros(ocp.dims.nsbu)
    ocp.constraints.usbu = np.zeros(ocp.dims.nsbu)
    ocp.constraints.idxsbu = np.arange(nu)

    ocp.cost.zl = 100 * np.ones(nu + nx + 1)
    ocp.cost.zu = 100 * np.ones(nu + nx + 1)
    ocp.cost.Zl = np.zeros(nu + nx + 1)
    ocp.cost.Zu = np.zeros(nu + nx + 1)

    # Configure slacks on initial state
    ocp.cost.zl_0 = 100 * np.ones(nu)
    ocp.cost.zu_0 = 100 * np.ones(nu)
    ocp.cost.Zl_0 = np.zeros(nu)
    ocp.cost.Zu_0 = np.zeros(nu)

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

    ocp.constraints.idxbx = np.arange(nx)  # indices of bounds on x

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
    ocp.solver_options.N_horizon = N
    ocp.solver_options.Tsim = Ts
    ocp.solver_options.tf = Ts * N
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 3
    ocp.solver_options.print_level = 0

    ocp.solver_options.tol = 1e-4

    ocp.acados_include_path = acados_source_path + "/include"
    ocp.acados_lib_path = acados_source_path + "/lib"

    ocp.code_export_directory = lib_dir

    # create solver with agent specific code files
    filename = os.path.join(lib_dir, "acados_pacejka_mpcc_solver_config.json")

    AcadosOcpSolver.generate(ocp, json_file=filename)
