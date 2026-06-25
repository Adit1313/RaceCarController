import os
import yaml
import math
import csv
import numpy as np
import casadi as cs
from acados_template import AcadosOcpSolver, AcadosSimSolver, ZoroDescription
from typing import Tuple
from time import perf_counter
from datetime import datetime
from scipy.stats import norm
import torch
import gpytorch

import rospy
from crs_msgs.msg import car_state_cart, car_input

from rospy_controllers.rospy_controller import RosPyController
from .generate_acados_solver import (
    generate_acados_ocp,
)
from .pacejka_model_zogpmpc import (
    pacejka_model_acados_gpzoro,
)
from .rviz_publisher_mpc import RvizPublisherMPC
from rospy_controllers.utils import (
    unwrap_yaw_angle,
    get_closest_track_point_idx,
    RecursiveAverager,
    MovingAverager,
    ThrottledPrinter,
    DictRecursiveAverager,
    check_file_changes,
)

from l4acados.models import (
    PyTorchFeatureSelector,
    GPyTorchResidualModel,
)
from l4acados.models.pytorch_models.gpytorch_models.gpytorch_gp import (
    BatchIndependentMultitaskGPModel,
    BatchIndependentInducingPointGpModel,
)
from l4acados.models.pytorch_models.gpytorch_models.gpytorch_data_processing_strategy import (
    VoidDataStrategy,
    RecordDataStrategy,
    OnlineLearningStrategy,
)
from l4acados.controllers import ZeroOrderGPMPC

# from l4acados.gpytorch_utils.gp_hyperparam_training import train_gp_model
from .train_gp_hyperparams import train_gp_model

WORKDIR = "/code/src/ros4crs/rospy_controllers"
rospy_controllers_dir = os.path.join(WORKDIR, "src", "rospy_controllers")
zero_order_gpmpc_dir = os.path.join(rospy_controllers_dir, "zero_order_gpmpc")


class MPCCCPacejkaControllerZOGPMPC(RosPyController):
    def __init__(self, track, pacejka_params, controller_params):
        np.set_printoptions(precision=2)
        rospy.loginfo(os.getcwd())
        self.track = track
        self.pacejka_params = pacejka_params
        self.pacejka_params_list = [
            pacejka_params["model_params"][key]
            for key in [
                "lr",
                "lf",
                "m",
                "I",
                "Df",
                "Cf",
                "Bf",
                "Dr",
                "Cr",
                "Br",
                "Cm1",
                "Cm2",
                "Cd0",
                "Cd1",
                "Cd2",
                "gamma",
                "eps",
                "car_width",
                "wheel_radius",
            ]
        ]
        self.controller_params = controller_params
        self.controller_params_list = [
            controller_params[key]
            for key in [
                "Q1",
                "Q2",
                "R1",
                "R2",
                "R3",
                "q",
            ]
        ]

        print("controller params: ", self.controller_params)
        print(f"controller params list: {self.controller_params_list}")

        with open(
            f"/code/src/ros4crs/rospy_controllers/src/rospy_controllers/zero_order_gpmpc/solver.yaml"
        ) as f:
            self.cfg = yaml.load(f, Loader=yaml.loader.SafeLoader)

        if rospy.get_param("~num_rti_iter") == 1:
            self.cfg["solver_creation"]["solver_opts"]["levenberg_marquardt"] = (
                self.cfg["solver_creation"]["solver_opts"]["levenberg_marquardt_rti"]
            )
        else:
            self.cfg["solver_creation"]["solver_opts"]["levenberg_marquardt"] = (
                self.cfg["solver_creation"]["solver_opts"]["levenberg_marquardt_sqp"]
            )
        print(
            "Setting LM to: ",
            self.cfg["solver_creation"]["solver_opts"]["levenberg_marquardt"],
        )

        self.ocp = generate_acados_ocp(
            self.cfg["solver_creation"]["N"],
            self.cfg["solver_creation"]["Ts"],
            os.environ["ACADOS_SOURCE_DIR"],
            lambda: pacejka_model_acados_gpzoro(
                constraints=self.cfg["model_bounds"],
            ),
            self.cfg,
            self.controller_params,
        )
        self.ocp.code_export_directory = os.path.join(
            zero_order_gpmpc_dir, "c_generated_code"
        )

        """B maps noise to state"""
        self.B = self.ocp.zoro_description.unc_jac_G_mat
        self.B_d_pinv = np.linalg.pinv(self.B)

        # self._cost_function = cs.Function(
        #     "cost_function",
        #     [self.ocp.model.x, self.ocp.model.u, self.ocp.model.p],
        #     [self.ocp.model.cost_expr_ext_cost],
        # )
        self._cost_function_W_flat = np.array(
            self.ocp.cost.W.flatten()
        )  # NOTE: this does not depend on model.p
        self._cost_function_y = cs.Function(
            "cost_function_y",
            [self.ocp.model.x, self.ocp.model.u, self.ocp.model.p],
            [self.ocp.model.cost_y_expr],
        )
        self._cost_function = cs.Function(
            "cost_function",
            [self.ocp.model.x, self.ocp.model.u, self.ocp.model.p],
            [
                self.ocp.model.cost_y_expr.T
                @ self.ocp.cost.W
                @ self.ocp.model.cost_y_expr
            ],
        )
        self.set_problem_dimensions()

        self.x_init_cov = np.zeros(self.nx)
        self.last_input = {"steer": 0, "torque": 0.5}

        # unwrap yaw angle from track
        self.track_yaw_angle_unwrapped = unwrap_yaw_angle(self.track["tangentAngle"])

        # initial arc length
        idx_track = get_closest_track_point_idx(
            track["x_init"], track["y_init"], self.track_xCoords, self.track_yCoords
        )
        self.theta_ = self.track["arcLength"][idx_track]

        self.last_solution = {
            "states": np.tile(
                np.zeros(
                    self.nx,
                ),
                (self.N + 1, 1),
            ),
            "inputs": np.tile(
                np.zeros(
                    self.nu,
                ),
                (self.N, 1),
            ),
            "reference_on_track": np.tile(
                np.zeros(
                    3,
                ),
                (self.N, 1),
            ),
        }
        self.update_params = np.zeros((self.np, self.N + 1))

        self.input_apply_index = 1
        self.rti_initialized = False
        self.theta_shift_index = 0
        self._sqp_iter = 0

        self.rti_initial_iter = 200
        self.rti_run_iter = rospy.get_param("~num_rti_iter")
        self._hewing_sqp = rospy.get_param("~cautious_gp")
        """Whether the covariances should be kept fixed if multiple SQP iterations are performed."""

        self._realtime_factor = rospy.get_param("/clock_publisher_node/clock_rate", 1.0)
        print(f"realtime factor: {self._realtime_factor}")
        """Adjust forward simulation time of initial input to account for realitme factor"""

        input_selection = controller_params["gp_params"]["input_feature_selection"]
        input_feature_selector = PyTorchFeatureSelector(input_selection)
        x_data_path = os.path.join(WORKDIR, "data", "x_data.csv")
        y_data_path = os.path.join(WORKDIR, "data", "y_data.csv")

        load_data = rospy.get_param("~load_data")
        x_train_data, y_train_data = None, None
        if load_data:
            torch.random.manual_seed(1234)

            indices_to_keep = controller_params["gp_params"]["load_data_keep_idxs"]
            x_train_data, y_train_data = self.load_tensor_data(x_data_path, y_data_path)
            x_train_data, y_train_data = (
                x_train_data[indices_to_keep[0] : indices_to_keep[1], :],
                y_train_data[indices_to_keep[0] : indices_to_keep[1], :],
            )
            indices = torch.randperm(x_train_data.size(0))
            x_train_data = x_train_data[indices]
            y_train_data = y_train_data[indices]

            max_num_datapoints = controller_params["gp_params"]["max_num_data_points"]
            x_train_data = x_train_data[:max_num_datapoints, :]
            y_train_data = y_train_data[:max_num_datapoints, :]

            x_train_data = input_feature_selector(x_train_data)

        if not rospy.get_param("~inducing_gp"):
            gpytorch_gp_model = BatchIndependentMultitaskGPModel(
                train_x=x_train_data,
                train_y=y_train_data,
                likelihood=gpytorch.likelihoods.MultitaskGaussianLikelihood(
                    num_tasks=self.nw,
                ),
                residual_dimension=self.nw,
                input_dimension=sum(input_selection),
                use_ard=True,
            )
            hyperparameter_names_dict = {
                "likelihood.task_noises": "likelihood.task_noises",
                "kernel.lengthscale": "covar_module.base_kernel.lengthscale",
                "kernel.outputscale": "covar_module.outputscale",
            }
        elif rospy.get_param("~inducing_gp"):
            gpytorch_gp_model = BatchIndependentInducingPointGpModel(
                train_x=x_train_data,
                train_y=y_train_data,
                likelihood=gpytorch.likelihoods.MultitaskGaussianLikelihood(
                    num_tasks=self.nw,
                ),
                residual_dimension=self.nw,
                input_dimension=sum(input_selection),
                inducing_points=controller_params["gp_params"][
                    "num_inducing_points_if_inducing_gp"
                ],
                use_ard=True,
            )
            hyperparameter_names_dict = {
                "likelihood.task_noises": "likelihood.task_noises",
                "kernel.lengthscale": "covar_module.base_kernel.base_kernel.lengthscale",
                "kernel.outputscale": "covar_module.base_kernel.outputscale",
            }

        gpytorch_gp_model.initialize(
            **{
                hyperparameter_names_dict[key]: torch.Tensor(
                    controller_params["gp_params"]["hyperparameters"][key]
                )
                for key in hyperparameter_names_dict.keys()
            }
        )

        # train GP model
        if rospy.get_param("~train_gp_params") == "only_inducing_points":
            for p_name, p in gpytorch_gp_model.named_parameters():
                if p_name != "covar_module.inducing_points":
                    p.requires_grad = False

        elif rospy.get_param("~train_gp_params") == "only_inducing_points_with_kmeans":
            print("Setting inducing points with kmeans clustering")
            from scipy.cluster.vq import kmeans

            # whiten data
            x_train_data_whitened = (
                x_train_data - x_train_data.mean(dim=0)
            ) / x_train_data.std(dim=0)

            # use kmeans to find inducing points
            inducing_point_locations, _ = kmeans(
                x_train_data_whitened.numpy(),
                controller_params["gp_params"]["num_inducing_points_if_inducing_gp"],
                iter=20,
            )
            gpytorch_gp_model.initialize(
                **{
                    "covar_module.inducing_points": torch.Tensor(
                        inducing_point_locations
                    )
                }
            )

        if (
            rospy.get_param("~train_gp_params") != "none"
            and rospy.get_param("~train_gp_params")
            != "only_inducing_points_with_kmeans"
        ):
            print("Optimizing hyper-parameters...")
            # gpytorch_gp_model.initialize(**{"covar_module.inducing_points": 2*torch.rand(
            #     (controller_params["gp_params"]["num_inducing_points_if_inducing_gp"],
            #     x_train_data.shape[1])
            # ) - 1})
            gpytorch_gp_model, likelihood = train_gp_model(
                gpytorch_gp_model,
                torch_seed=5678,
                training_iterations=300,
                learning_rate=0.1,
            )

        # torch.save(gpytorch_gp_model.state_dict(), "gp_model.pt")
        # torch.save(likelihood.state_dict(), "likelihood.pt")

        gpytorch_gp_model.eval()
        gpytorch_gp_model.likelihood.eval()

        self.sample_inducing_gp_locations = isinstance(
            gpytorch_gp_model,
            BatchIndependentInducingPointGpModel,
        ) and (rospy.get_param("~inducing_gp_locations") == "transductive")

        if rospy.get_param("~data_processing_mode") == "none":
            data_processing_strategy = VoidDataStrategy()
        elif rospy.get_param("~data_processing_mode") == "record":
            data_processing_strategy = RecordDataStrategy(x_data_path, y_data_path)
        elif rospy.get_param("~data_processing_mode") == "learn":
            data_processing_strategy = OnlineLearningStrategy(
                controller_params["gp_params"]["max_num_data_points"]
            )
        else:
            raise ValueError(
                f"Invalid data processing mode: {rospy.get_param('~data_processing_mode')}. Should be one of ['none', 'record', 'learn'])"
            )

        self.residual_gp = GPyTorchResidualModel(
            gp_model=gpytorch_gp_model,
            feature_selector=input_feature_selector,
            data_processing_strategy=data_processing_strategy,
        )
        """Residual Gaussian Process which predicts model mismatch"""

        build_c_code = check_file_changes(
            [
                os.path.join(zero_order_gpmpc_dir, "generate_acados_solver.py"),
                os.path.join(zero_order_gpmpc_dir, "pacejka_model_zogpmpc.py"),
                os.path.join(zero_order_gpmpc_dir, "solver.yaml"),
            ],
            os.path.join(WORKDIR, "data", "build_c_code_hash.md5"),
        )

        self.zogpmpc = ZeroOrderGPMPC(
            ocp=self.ocp,
            B=self.B,
            residual_model=self.residual_gp if rospy.get_param("~load_gp") else None,
            use_cython=True,
            path_json_ocp=os.path.join(
                zero_order_gpmpc_dir, "zoro_ocp_solver_config.json"
            ),
            path_json_sim=os.path.join(
                zero_order_gpmpc_dir, "zoro_sim_solver_config.json"
            ),
            build_c_code=build_c_code,
        )

        # soft constraint adaptive penalty
        zl_arr = np.linspace(
            self.cfg["solver_creation"]["cost"]["zl_0"],
            self.cfg["solver_creation"]["cost"]["zl_e"],
            self.N,
        )
        zu_arr = np.linspace(
            self.cfg["solver_creation"]["cost"]["zu_0"],
            self.cfg["solver_creation"]["cost"]["zu_e"],
            self.N,
        )
        Zl_arr = np.linspace(
            self.cfg["solver_creation"]["cost"]["Zl_0"],
            self.cfg["solver_creation"]["cost"]["Zl_e"],
            self.N,
        )
        Zu_arr = np.linspace(
            self.cfg["solver_creation"]["cost"]["Zu_0"],
            self.cfg["solver_creation"]["cost"]["Zu_e"],
            self.N,
        )
        for i in range(self.N):
            if i == 0:
                dims_ns = self.ocp.dims.ns_0
            else:
                dims_ns = self.ocp.dims.ns

            a = zl_arr[i] * np.ones(dims_ns)
            if dims_ns > 0:
                self.zogpmpc.ocp_solver.cost_set(i, "zl", zl_arr[i] * np.ones(dims_ns))
                self.zogpmpc.ocp_solver.cost_set(i, "zu", zu_arr[i] * np.ones(dims_ns))
                self.zogpmpc.ocp_solver.cost_set(i, "Zl", Zl_arr[i] * np.ones(dims_ns))
                self.zogpmpc.ocp_solver.cost_set(i, "Zu", Zu_arr[i] * np.ones(dims_ns))

            self.zogpmpc.ocp_solver.cost_set(
                i, "W", self.ocp.cost.W
            )  # NOTE: needs to be done, otherwise rebuild c code when controller.yaml changes

        # get sim solver from zogpmpc
        self.sim_solver = self.zogpmpc.sim_solver
        self.sim_solver.set("T", self.cfg["solver_creation"]["Ts"])

        self.lag_compensation_time_averager = MovingAverager(30)
        self.lag_compensation_time_averager.update(
            self.controller_params["lag_compensation_time"]
        )

        self.prediction_sim_params = np.array(
            [0.0] * 6 + self.controller_params_list + self.pacejka_params_list
        )
        """Default parameters for the residual state prediction simulator"""

        self._lap_start = None
        """Start time of the lap. Is initialized after MPC is jump started"""

        self._solve_times_averager = DictRecursiveAverager()
        self._solve_time_printer = ThrottledPrinter(20)
        self.visualizer = RvizPublisherMPC(self.N)

        rospy.logwarn("starting acados based zoro mpcc")
        rospy.logwarn(f"GP Type: {type(self.residual_gp.gp_model)}")
        rospy.logwarn(f"GP mode: {type(self.residual_gp._data_processing_strategy)}")
        try:
            rospy.logwarn(
                f"Num datapoints: {self.residual_gp.gp_model.train_inputs[0].shape[0]}"
            )
        except:
            rospy.logwarn(f"GP does not use any datapoitns.")
        rospy.logwarn(
            f"num sqp iter: {self.rti_run_iter}, hewing style: {self._hewing_sqp}"
        )

    def set_problem_dimensions(self) -> None:
        """Sets up a bunch of problem parameters for later access"""

        self.nx = self.ocp.dims.nx
        self.nu = self.ocp.dims.nu
        self.nw = self.B.shape[1]
        self.np = self.ocp.dims.np
        self.N = self.cfg["solver_creation"]["N"]
        self.idx_x = 0
        self.idx_y = 1
        self.idx_yaw = 2
        self.idx_vxb = 3
        self.idx_vyb = 4
        self.idx_vyaw = 5
        self.idx_torque = 6
        self.idx_steer = 7
        self.idx_theta = 8
        self.track_N = len(self.track["xCoords"])
        self.track_maxArcLength = max(self.track["arcLength"])
        self.track_xCoords = np.array(self.track["xCoords"])
        self.track_yCoords = np.array(self.track["yCoords"])
        self.laps = 0

    # --------------------------- REAL-TIME COMPUTE ---------------------------

    def predict_next_state(self, x_curr, u_curr):
        """uses the state_prediction_sim_solver to estimate the next state"""
        self.sim_solver.set("T", self.cfg["solver_creation"]["Ts"])
        return self.sim_solver.simulate(
            x=x_curr, u=u_curr, p=self.prediction_sim_params
        )

    def compute_track_reference(self, theta_i):
        distance_on_track = theta_i - self.laps * self.track_maxArcLength
        reference_track_index = int(distance_on_track * self.track["density"])

        return {
            "x": self.track["xCoords"][reference_track_index % self.track_N],
            "y": self.track["yCoords"][reference_track_index % self.track_N],
            "grad_x": self.track["xRate"][reference_track_index % self.track_N],
            "grad_y": self.track["yRate"][reference_track_index % self.track_N],
            "theta": distance_on_track + self.laps * self.track_maxArcLength,
            "phi": self.track_yaw_angle_unwrapped[
                reference_track_index % (2 * self.track_N)
            ]
            + self.laps * 2 * math.pi,
        }

    def update_track_points(self, theta_i):
        track_point = self.compute_track_reference(theta_i)
        reference_on_track = np.array(
            [track_point["x"], track_point["y"], track_point["phi"]]
        )
        update_params = np.array(
            [
                track_point["x"],  # xd,
                track_point["y"],  # yd,
                track_point["grad_x"],  # grad_xd,
                track_point["grad_y"],  # grad_yd,
                track_point["theta"],  # theta_hat,
                track_point["phi"],  # phi_d,
            ]
            + self.controller_params_list
            + self.pacejka_params_list
        )

        return update_params, reference_on_track

    def lag_compensation_x_init(self):
        """Compensates for MPC solve time by forward simulating the initial state of the problem
        to when the MPC will have finished solving.
        """

        if self.lag_compensation_time_averager.average > 0 and self.rti_initialized:
            self.sim_solver.set(
                "T", self.lag_compensation_time_averager.average * self._realtime_factor
            )
            # TODO: lag compensation with GP model
            x_init_lag = self.sim_solver.simulate(
                x=self.x_init, u=np.array([0.0, 0.0, 0.0]), p=self.update_params[:, 1]
            )

            self.x_init[self.idx_x] = x_init_lag[self.idx_x]
            self.x_init[self.idx_y] = x_init_lag[self.idx_y]
            self.x_init[self.idx_yaw] = x_init_lag[self.idx_yaw]
            self.x_init[self.idx_vxb] = x_init_lag[self.idx_vxb]
            self.x_init[self.idx_vyb] = x_init_lag[self.idx_vyb]
            self.x_init[self.idx_vyaw] = x_init_lag[self.idx_vyaw]
            # (input unchanged)
            self.x_init[self.idx_theta] = x_init_lag[self.idx_theta]

    def sample_inducing_points(self):
        """Sample new inducing points along the open loop prediction"""
        time_before_update = perf_counter()
        inducing_indices = np.linspace(
            0,
            self.N,
            self.residual_gp.gp_model.num_inducing_points,
            dtype=int,
            endpoint=False,
        )
        inducing_points = np.hstack(
            (
                self.last_solution["states"][inducing_indices],
                self.last_solution["inputs"][inducing_indices],
            )
        )
        self.residual_gp.gp_model.train()
        self.residual_gp.gp_model.initialize(
            **{
                "covar_module.inducing_points": self.residual_gp._feature_selector(
                    torch.Tensor(inducing_points)
                )
            }
        )
        self.residual_gp.gp_model.eval()
        time_update = perf_counter() - time_before_update
        print(f"gp update time: {1e3*(time_update)}")

    def rti_preparation_phase(self, j=0):
        """Runs the preparation phase of the controller

        Queries the GP, sets all the parameters, updates the initial guess
        and computes the jacobians.
        """

        # If we use hewing sqp and inducing point gp, then we update the inducing points in
        # the first sqp iteration
        if self.sample_inducing_gp_locations and j == 0 and self.rti_initialized:
            self.sample_inducing_points()

        for i in range(self.N):
            i_next = i

            if j == 0 and self.rti_initialized:
                i_next = min(i_next + self.theta_shift_index, self.N - 1)

            theta_next = self.last_solution["states"][i_next][self.idx_theta]

            (
                self.update_params[:, i],
                self.last_solution["reference_on_track"][i],
            ) = self.update_track_points(theta_next)

            self.zogpmpc.p_hat_nonlin[i, :] = self.update_params[:, i]

            # initial guess
            if j == 0 and self.rti_initialized:
                self.zogpmpc.ocp_solver.set(
                    i, "x", self.last_solution["states"][i_next]
                )
                self.zogpmpc.ocp_solver.set(
                    i, "u", self.last_solution["inputs"][i_next]
                )

        theta_last = self.last_solution["states"][self.N, self.idx_theta]
        (
            self.update_params[:, self.N],
            _,
        ) = self.update_track_points(theta_last)
        self.zogpmpc.p_hat_nonlin[self.N, :] = self.update_params[:, self.N]

        if j == 0 and self.rti_initialized:
            self.zogpmpc.ocp_solver.set(
                self.N, "x", self.last_solution["states"][self.N]
            )

        skip_variances = self._hewing_sqp and j > 0
        with gpytorch.settings.skip_posterior_variances(skip_variances):
            self.zogpmpc.sim_solver.set("T", self.cfg["solver_creation"]["Ts"])
            self.zogpmpc.preparation()

        if not skip_variances:
            self.zogpmpc.do_custom_update()

        if self.rti_initialized and j == 0:
            res_outptut = self.compute_residual(self._x_init_copy)
            if res_outptut is not None:
                x_train, residual = res_outptut
                self.residual_gp.record_datapoint(x_train, residual)

    def update_track_and_solve(self, n_rti, advanced_step=False):
        feedback_start_time = perf_counter()

        # lag compensation
        self.lag_compensation_x_init()

        # set initial condition
        self.zogpmpc.ocp_solver.set(0, "lbx", self.x_init)
        self.zogpmpc.ocp_solver.set(0, "ubx", self.x_init)

        self.last_solution_copy = self.last_solution.copy()

        j = 0
        break_sqp = False
        while j < n_rti:
            status_fdbk = self.rti_feedback_phase()
            j += 1

            if status_fdbk != 0:
                print(
                    f"WARNING: feedback phase failed with status {status_fdbk}: aborting SQP iteration at {j} out of {n_rti} iterations"
                )
                self.last_solution = self.last_solution_copy
                break_sqp = True

            if not self.rti_initialized or n_rti - j > 1:
                self.rti_preparation_phase(j)

            if j == 0 and advanced_step:
                self.zogpmpc.ocp_solver.options_set("rti_phase", 2)
                status_fdbk = self.zogpmpc.ocp_solver.solve()
                self.zogpmpc.ocp_solver.options_set("rti_phase", 1)
                self.zogpmpc.ocp_solver.solve()

            residuals = self.zogpmpc.ocp_solver.get_residuals(recompute=True)

            if (
                self.rti_initialized
                and max(residuals) < self.cfg["solver_creation"]["solver_opts"]["tol"]
            ):
                print(
                    f'solver reached required tolerance of {self.cfg["solver_creation"]["solver_opts"]["tol"]} in {j} out of {n_rti} iterations'
                )
                break_sqp = True
                self._sqp_iter = j

            if break_sqp:
                break

        print(f"residuals after prep: {residuals}")

        if status_fdbk == 0:
            for i in range(self.N):
                self.last_solution["states"][i] = self.zogpmpc.ocp_solver.get(i, "x")
                self.last_solution["inputs"][i] = self.zogpmpc.ocp_solver.get(i, "u")
            self.last_solution["states"][self.N] = self.zogpmpc.ocp_solver.get(
                self.N, "x"
            )

        if self.rti_initialized:
            feedback_execution_time = perf_counter() - feedback_start_time
            self.lag_compensation_time_averager.update(feedback_execution_time)

        return status_fdbk

    def get_input_init(self):
        # self.x_init[self.idx_vxb] = np.clip(self.x_init[self.idx_vxb], 0.5, np.Inf)
        rospy.loginfo(f"Controller starting from inital state {self.x_init}")
        # set initial guess
        v_init = 0.5
        ts = 1 / 30
        distance_on_track = 0
        # self.x_init[3] = 0.5
        for i in range(self.N + 1):
            curr_vel = (1.5 - v_init) * i / self.N + v_init
            distance_on_track += curr_vel * ts
            reference_track_index = int(distance_on_track * self.track["density"])

            track_point = {
                "x": self.track["xCoords"][reference_track_index % self.track_N],
                "y": self.track["yCoords"][reference_track_index % self.track_N],
                "grad_x": self.track["xRate"][reference_track_index % self.track_N],
                "grad_y": self.track["yRate"][reference_track_index % self.track_N],
                "theta": distance_on_track + self.laps * self.track_maxArcLength,
                "phi": self.track_yaw_angle_unwrapped[
                    reference_track_index % (2 * self.track_N)
                ]
                + self.laps * 2 * math.pi,
            }

            self.last_solution["states"][i] = self.x_init
            self.last_solution["states"][i][self.idx_x] = track_point["x"]
            self.last_solution["states"][i][self.idx_y] = track_point["y"]
            self.last_solution["states"][i][self.idx_yaw] = track_point["phi"]
            self.last_solution["states"][i][self.idx_vxb] = curr_vel
            self.last_solution["states"][i][self.idx_vyb] = 0
            self.last_solution["states"][i][self.idx_theta] = distance_on_track

            self.zogpmpc.ocp_solver.set(i, "x", self.last_solution["states"][i])

        # SQP LM setting for initial solve
        # TODO: wait for this to be incorporated in Cython interface
        # self.zogpmpc.ocp_solver.options_set(
        #     "levenberg_marquardt",
        #     self.cfg["solver_creation"]["solver_opts"]["levenberg_marquardt_sqp"],
        # )

        self.rti_preparation_phase()
        self.update_track_and_solve(self.rti_initial_iter)

        # Restore LM setting
        # TODO: wait for this to be incorporated in Cython interface
        # self.zogpmpc.ocp_solver.options_set(
        #     "levenberg_marquardt",
        #     self.cfg["solver_creation"]["solver_opts"]["levenberg_marquardt_rti"],
        # )

        # will be _sqp or _rti, depending on num_iter
        self.rti_initialized = True

    def get_input(self, state: car_state_cart) -> car_input:
        self.x_init = np.array(
            [
                state.x,
                state.y,
                state.yaw,
                state.vx_b,
                # np.clip(state.vx_b, 0.1, np.Inf),
                state.vy_b,
                state.dyaw,
                self.last_input["torque"],
                self.last_input["steer"],
                self.theta_,
            ]
        )

        # Create a copy of x_init which is not affected by lag compensation.
        # if self.rti_initialized:
        if hasattr(self, "_x_init_copy"):
            self._previous_initial_state = self._x_init_copy.copy()
        self._x_init_copy = self.x_init.copy()

        # # initialized?
        if not self.rti_initialized:
            self.get_input_init()
            self._lap_start = perf_counter()

        # lap counter
        if self.theta_ > (self.laps + 1) * self.track_maxArcLength:
            curr_time = perf_counter()
            self.laps += 1
            print(f"lap {self.laps}, lap time: {curr_time - self._lap_start}")
            self._lap_start = curr_time

        self.update_track_and_solve(self.rti_run_iter)

        # apply
        x_pred = self.last_solution["states"][self.input_apply_index]

        self.last_input["steer"] = x_pred[self.idx_steer]
        self.last_input["torque"] = x_pred[self.idx_torque]
        self.theta_ = x_pred[self.idx_theta]

        input = car_input()
        input.steer = self.last_input["steer"]
        input.torque = self.last_input["torque"]

        return input

    def rti_feedback_phase(self):
        """Solves the QP which was generated in the preparation phase"""
        return self.zogpmpc.feedback()

    def publish_visualization(self):
        self.visualizer.publish_init_position(
            self.current_state  # without lag compensation
        )
        self.visualizer.publish_trajectory(
            self.planned_trajectory,
            self.reference_trajectory,
        )
        self.visualizer.publish_uncertainties(
            self.planned_trajectory,
            self.planned_covariances,
        )

    # --------------------------- STATIC ---------------------------
    @staticmethod
    def load_tensor_data(
        x_data_path: str, y_data_path: str
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        try:
            with open(x_data_path, "r") as file:
                train_x_tensor = torch.from_numpy(
                    np.genfromtxt(file, delimiter=",")
                ).type(torch.float32)
        except FileNotFoundError:
            print(f"File {x_data_path} does not exist.")
        except Exception as e:
            raise e

        # If the data only has one feature, we need to add a batch dimension
        if train_x_tensor.dim() == 1:
            train_x_tensor = torch.unsqueeze(train_x_tensor, -1)

        try:
            with open(y_data_path, "r") as file:
                train_y_tensor = torch.from_numpy(
                    np.genfromtxt(file, delimiter=",")
                ).type(torch.float32)
        except FileNotFoundError:
            print(f"File {y_data_path} does not exist.")
        except Exception as e:
            raise e

        # If the data only has one feature, we need to add a batch dimension
        if train_y_tensor.dim() == 1:
            train_y_tensor = torch.unsqueeze(train_y_tensor, -1)

        return train_x_tensor, train_y_tensor

    # --------------------------- LOGGING ---------------------------

    @property
    def current_constraint_value(self):
        return self._compute_constraint_value(self._x_init_copy)

    @property
    def current_control_input(self):
        return self.last_input["torque"], self.last_input["steer"]

    @property
    def current_optimizer_input(self):
        return self.last_solution["inputs"][0]

    @property
    def current_optimizer_state(self):
        return self.last_solution["states"][0]

    @property
    def current_optimizer_ref(self):
        return self.last_solution["reference_on_track"][0]

    @property
    def current_sqp_iterations(self):
        return self._sqp_iter

    @property
    def current_residuals(self):
        return self.zogpmpc.ocp_solver.get_residuals(recompute=True)

    @property
    def current_stage_cost_value(self):
        return np.array(self._compute_cost_value()).flatten()

    @property
    def current_stage_cost_value_y(self):
        return np.array(self._compute_cost_value_y()).flatten()

    @property
    def current_stage_cost_value_W(self):
        return self._cost_function_W_flat

    @property
    def current_state(self):
        return self._x_init_copy  # current state without lag compensation

    @property
    def current_state_x(self):
        return self._x_init_copy[self.idx_x]

    @property
    def current_state_y(self):
        return self._x_init_copy[self.idx_y]

    @property
    def current_track_ref_x(self):
        return self.compute_track_reference(self._x_init_copy[self.idx_theta])["x"]

    @property
    def current_track_ref_y(self):
        return self.compute_track_reference(self._x_init_copy[self.idx_theta])["y"]

    @property
    def planned_covariances(self):
        if not hasattr(self.zogpmpc, "covariances_array"):
            return None
        gamma_sq = (
            self.cfg["solver_creation"]["zoro_description"]["backoff_scaling_gamma"]
            ** 2
        )
        return [
            gamma_sq
            * self.zogpmpc.covariances_array[
                self.nx**2 * i : self.nx**2 * (i + 1)
            ].reshape(self.nx, self.nx)
            for i in range(self.N + 1)
        ]

    @property
    def planned_trajectory(self):
        return self.last_solution["states"]

    @property
    def reference_trajectory(self):
        return self.last_solution["reference_on_track"]

    def _compute_constraint_value(self, x_state):
        track_point = self.compute_track_reference(x_state[self.idx_theta])
        return (
            (x_state[self.idx_x] - track_point["x"]) ** 2
            + (x_state[self.idx_y] - track_point["y"]) ** 2
            - (
                0.5
                * (
                    self.cfg["track"]["track_width"]
                    - self.pacejka_params["model_params"]["car_width"]
                )
                - self.cfg["track"]["safety_margin"]
            )
            ** 2
        )

    def _compute_cost_value_y(self):
        return self._cost_function_y(
            self._x_init_copy,
            self.last_solution["inputs"][0],
            self.update_params[:, 0],
        )

    def _compute_cost_value(self):
        return self._cost_function(
            self._x_init_copy,
            self.last_solution["inputs"][0],
            self.update_params[:, 0],
        )

    def compute_residual(self, new_state) -> Tuple[np.ndarray, np.ndarray]:
        """Compute residual between actual state and expected state
        y(k) = B^{+}(x(k) - f(x(k-1), u(k-1)))
        """
        if not hasattr(self, "_previous_initial_state"):
            rospy.logwarn("no prev initial state, skipping residual comp")
            return None

        x_prev = self._previous_initial_state
        u_prev = self.last_solution["inputs"][0]
        x_predicted = self.predict_next_state(x_prev, u_prev)
        x_train = np.hstack((x_prev, u_prev))

        residual = self.B_d_pinv @ (new_state - x_predicted)
        return x_train, residual
