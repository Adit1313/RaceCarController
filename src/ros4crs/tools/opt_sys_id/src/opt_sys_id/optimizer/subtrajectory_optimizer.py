import casadi as ca
import numpy as np

from .optimizer import Optimizer

from opt_sys_id.model.awd_model import AwdBicycleModel
from opt_sys_id.tools import Vectorizer, to_vector


class SubtrajectoryOptimizer(Optimizer):
    def __init__(self, model: AwdBicycleModel, dt: float, N: int, M: int) -> None:
        self.M = M
        self.x_vec = Vectorizer()
        super().__init__(model, dt, N)

    def formulate_solver(self):
        # Optimization Variables
        theta = ca.SX.sym("th", self.model.num_params)
        x = ca.SX.sym("x", self.model.num_states, self.N + 1)
        w = ca.SX.sym("w", self.model.num_states, self.N)
        v = ca.SX.sym("v", self.model.num_measurements, self.N + 1)

        # Known Parameters
        y = ca.SX.sym("y", self.model.num_measurements, self.N + 1)
        u = ca.SX.sym("u", self.model.num_inputs, self.N)
        dt = ca.SX.sym("dt", self.N)
        theta0 = ca.SX.sym("th0", self.model.num_params)
        x0 = ca.SX.sym("x0", self.model.num_states, int(np.ceil(self.N / self.M)))
        P = ca.SX.sym("P", self.model.num_params, self.model.num_params)
        Q = ca.SX.sym("Q", self.model.num_states, self.model.num_states)
        R = ca.SX.sym("R", self.model.num_measurements, self.model.num_measurements)
        S = ca.SX.sym("S", self.model.num_states, self.model.num_states)

        f = (theta - theta0).T @ P @ (theta - theta0)
        g = []
        self.lbg = []
        self.ubg = []

        # Fixed parameters
        for param in self.model.fixed_params:
            g += [theta[param] - theta0[param]]
            self.lbg += [0]
            self.ubg += [0]

        for k in range(self.N):
            # State error cost
            f += w[:, k].T @ Q @ w[:, k]

            # Dynamic constraints
            if k % self.M != self.M - 1 or k == self.N - 1:
                g += [
                    x[:, k]
                    + self.model.calc_step_rk4(x[:, k], u[:, k], theta, dt[k])
                    + w[:, k]
                    - x[:, k + 1]
                ]
                self.lbg += [0] * self.model.num_states
                self.ubg += [0] * self.model.num_states

            # prior weighting on state
            if k % self.M == 0:
                f += (
                    (x[:, k] - x0[:, int(k / self.M)]).T
                    @ S
                    @ (x[:, k] - x0[:, int(k / self.M)])
                )

        for k in range(self.N + 1):
            # Observation error cost
            f += v[:, k].T @ R @ v[:, k]

            # Observation constraints
            if k < self.N:
                g += [
                    self.model.calc_y(x[:, k], u[:, k], False, theta)
                    + v[:, k]
                    - y[:, k]
                ]
            else:
                # At N, we don't have a defined input. Re-use the last one
                g += [
                    self.model.calc_y(x[:, k], u[:, -1], False, theta)
                    + v[:, k]
                    - y[:, k]
                ]

            self.lbg += [0] * self.model.num_measurements
            self.ubg += [0] * self.model.num_measurements

        self.nlp = {}
        self.nlp["x"] = self.x_vec.to_vector(theta, x, w, v)
        self.nlp["p"] = to_vector(y, u, dt, theta0, x0, P, Q, R, S)
        self.nlp["f"] = f
        self.nlp["g"] = ca.vertcat(*g)

        options = {"ipopt": {"max_iter": 10000}}
        self.solver = ca.nlpsol("solver", "ipopt", self.nlp, options)

    def find_params(
        self,
        y: np.ndarray,
        u: np.ndarray,
        theta0: np.ndarray,
        P: np.ndarray,
        Q: np.ndarray,
        R: np.ndarray,
        S: np.ndarray,
        x0: np.ndarray = None,
        xws: np.ndarray = None,
        t: np.ndarray = None,
    ):
        """
        Solve the subtrajectory optimization problem.

        Args:
            y (np.ndarray): Measurements
            u (np.ndarray): Inputs
            theta0 (np.ndarray): Initial guess for the parameters
            P (np.ndarray): Deviation from prior parameter regularization cost
            Q (np.ndarray): Process noise regularization cost
            R (np.ndarray): Measurement regularization cost
            S (np.ndarray): State regularization cost
            x0 (np.ndarray, optional): Initial state guess. If not set, xws is used to initialize the states. Defaults to None.
            xws (np.ndarray, optional): Warm start for the states. Defaults to None.
            t (np.ndarray, optional): Time vector. Defaults to None.
        """
        if xws is None:
            xws = self.model.initial_estimate(y, u, self.dt)

        if x0 is None:
            x0 = np.zeros((self.model.num_states, int(np.ceil(self.N / self.M))))
            for i in range(int(np.ceil(self.N / self.M))):
                x0[:, i] = xws[:, i * self.M]

        if t is None:
            dt = np.ones(self.N) * self.dt
        else:
            dt = t[1:] - t[:-1]

        w0 = np.zeros((self.model.num_states, self.N))
        v0 = np.zeros((self.model.num_measurements, self.N + 1))

        self.solver_in = {}
        self.solver_in["x0"] = to_vector(theta0, xws, w0, v0)
        self.solver_in["p"] = to_vector(y, u, dt, theta0, x0, P, Q, R, S)
        self.solver_in["lbg"] = self.lbg
        self.solver_in["ubg"] = self.ubg

        print("Solving optimization problem...")
        print(f"x0: {self.solver_in['x0']}")
        print(f"Shape x0: {self.solver_in['x0'].shape}")
        print(f"p: {self.solver_in['p']}")
        print(f"Shape p: {self.solver_in['p'].shape}")

        self.solution = self.solver(**self.solver_in)

        self.theta_hat, self.x_hat, self.w_hat, self.v_hat = self.x_vec.from_vector(
            self.solution["x"]
        )

        return self.solution

    def get_parameter_cov(self):
        f = self.nlp["f"]
        g = self.nlp["g"]
        x = self.nlp["x"]
        p = self.nlp["p"]

        x_hat = self.solution["x"]
        p_hat = self.solver_in["p"]
        lam_g = self.solution["lam_g"]

        dg_dx = ca.Function("dg_dx", [x, p], [ca.jacobian(g, x)])(x_hat, p_hat).full()

        L = f + g.T @ lam_g
        ddL_ddx = ca.Function("ddL_ddx", [x, p], [ca.hessian(L, x)[0]])(
            x_hat, p_hat
        ).full()

        # Assemble KKT matrix
        zeros = np.zeros((g.shape[0], g.shape[0]))
        K = np.block([[ddL_ddx, dg_dx.T], [dg_dx, zeros]])

        K11 = K[: self.model.num_params, : self.model.num_params]
        K21 = K[self.model.num_params :, : self.model.num_params]
        K22 = K[self.model.num_params :, self.model.num_params :]

        K1 = K11 - K21.T @ np.linalg.solve(K22, K21)

        return np.linalg.solve(K1, np.eye(self.model.num_params))
