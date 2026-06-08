import casadi as ca
import numpy as np

from opt_sys_id.model import Model
from opt_sys_id.tools import Vectorizer, to_vector


class StateEstimator:
    solver = ca.Function()

    def __init__(self, model: Model, dt: float, N: int) -> None:
        self.model = model
        self.dt = dt
        self.N = N
        self.x_vec = Vectorizer()

        self.formulate_solver()

    def formulate_solver(self):
        # Optimization Variables
        x = ca.SX.sym("x", self.model.num_states, self.N + 1)
        w = ca.SX.sym("w", self.model.num_states, self.N)
        v = ca.SX.sym("v", self.model.num_measurements, self.N + 1)

        # Known Parameters
        theta = ca.SX.sym("th", self.model.num_params)
        y = ca.SX.sym("y", self.model.num_measurements, self.N + 1)
        u = ca.SX.sym("u", self.model.num_inputs, self.N)
        dt = ca.SX.sym("dt", self.N)
        Q = ca.SX.sym("Q", self.model.num_states, self.model.num_states)
        R = ca.SX.sym("R", self.model.num_measurements, self.model.num_measurements)
        S = ca.SX.sym("S", self.model.num_states, self.model.num_states)

        # Initial state
        x0 = ca.SX.sym("x0", self.model.num_states)

        # f = (x0 - x[:,0]).T @ S @ (x0 - x[:,0])
        f = 0
        g = []
        self.lbg = []
        self.ubg = []

        for k in range(self.N):
            # State error cost
            f += w[:, k].T @ Q @ w[:, k]

            # Dynamic constraints
            g += [
                x[:, k]
                + self.model.calc_step_rk4(x[:, k], u[:, k], theta, dt[k])
                + w[:, k]
                - x[:, k + 1]
            ]
            self.lbg += [0] * self.model.num_states
            self.ubg += [0] * self.model.num_states

        # Initial state cost
        f += (x[:, 0] - x0).T @ S @ (x[:, 0] - x0)

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
                # At the last time step, we don't have a control input, re-use the last one
                g += [
                    self.model.calc_y(x[:, k], u[:, -1], False, theta)
                    + v[:, k]
                    - y[:, k]
                ]
            self.lbg += [0] * self.model.num_measurements
            self.ubg += [0] * self.model.num_measurements

        self.nlp = {}
        self.nlp["x"] = self.x_vec.to_vector(x, w, v)
        self.nlp["p"] = to_vector(y, u, dt, theta, x0, Q, R, S)
        self.nlp["f"] = f
        self.nlp["g"] = ca.vertcat(*g)

        self.solver = ca.nlpsol("solver", "ipopt", self.nlp)

    def find_states(
        self,
        y: np.array,
        u: np.array,
        theta: np.array,
        Q: np.array,
        R: np.array,
        S: np.array,
        x0: np.array = None,
        xws: np.array = None,
        t=None,
    ):
        if xws is None:
            xws = self.model.initial_estimate(y, u, self.dt)

        if x0 is None:
            x0 = xws[:, 0]

        if t is None:
            dt = np.ones(self.N) * self.dt
        else:
            dt = t[1:] - t[:-1]

        w0 = np.zeros((self.model.num_states, self.N))
        v0 = np.zeros((self.model.num_measurements, self.N + 1))

        self.solver_in = {}
        self.solver_in["x0"] = to_vector(xws, w0, v0)
        self.solver_in["p"] = to_vector(y, u, dt, theta, x0, Q, R, S)
        self.solver_in["lbg"] = self.lbg
        self.solver_in["ubg"] = self.ubg

        self.solution = self.solver(**self.solver_in)

        self.x_hat, self.w_hat, self.v_hat = self.x_vec.from_vector(self.solution["x"])

        return self.solution
