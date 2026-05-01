from typing import List, Tuple

import numpy as np

from opt_sys_id.model import Model
from opt_sys_id.tools import gaussian_vector


class Simulator:
    model = Model()

    def __init__(self, params: np.array, dt: float) -> None:
        self.params = params
        self.dt = dt
        self.initial = InitialsGenerator(self.model, params)
        self.input = InputGenerator(self.model, params, dt)

    def simulate(
        self,
        x0: np.array,
        u: np.array,
        var_w: np.array = None,
        var_v: np.array = None,
        seed: str = "",
        w: np.array = None,
        v: np.array = None,
        t: np.array = None,
    ) -> Tuple[np.array, np.array]:
        N = np.shape(u)[1]

        x = np.zeros((self.model.num_states, N + 1))
        y = np.zeros((self.model.num_measurements, N + 1))

        if t is None:
            dt = np.ones(N) * self.dt
        else:
            dt = t[1:] - t[:-1]

        if w is not None:
            pass
        elif var_w is not None:
            w = gaussian_vector(var_w, self.model.num_states, N, seed + "w")
        else:
            w = np.zeros((self.model.num_states, N))

        if v is not None:
            pass
        elif var_v is not None:
            v = gaussian_vector(var_v, self.model.num_measurements, N + 1, seed + "v")
        else:
            v = np.zeros((self.model.num_measurements, N + 1))

        x[:, 0] = x0
        y[:, 0] = self.model.calc_y(x0, u[:, 0], True, self.params) + v[:, 0]

        for k in range(N):
            x[:, k + 1] = self.predict(x[:, k], u[:, k], True, dt[k]) + w[:, k]
            y[:, k + 1] = (
                self.model.calc_y(x[:, k + 1], u[:, k], True, self.params) + v[:, k + 1]
            )

        return x, y, w, v

    def predict(
        self, x_k: np.array, u_k: np.array, numerical=False, dt=None
    ) -> np.array:
        if dt is None:
            dt = self.dt
        return x_k + self.model.calc_step_rk4(x_k, u_k, self.params, dt, numerical)


class InitialsGenerator:
    def __init__(self, model: Model, params: np.array) -> None:
        self.model = model
        self.params = params

    def zero(self) -> np.array:
        return np.zeros(self.model.num_states)


class InputGenerator:
    def __init__(self, model: Model, params: np.array, dt: float) -> None:
        self.model = model
        self.params = params
        self.dt = dt

    def zero(self, N: int) -> np.array:
        return np.zeros((self.model.num_inputs, N))

    def ramp_input(self, v1: float, v2: float, input: int, N: int) -> np.array:
        u = self.zero(N)
        u[input, :] = np.linspace(v1, v2, N)

        return u

    def sin(self, amp: float, period: float, input: int, N: int) -> np.array:
        t = np.arange(0, N * self.dt, self.dt)
        omega = 2 * np.pi / period

        u = self.zero(N)
        u[input, :] = amp * np.sin(t * omega)

        return u

    def random(self, var: List[float], N: int, seed: str = "u") -> np.array:
        return gaussian_vector(var, self.model.num_inputs, N, seed)
