import numpy as np

from abc import abstractmethod


class Model:
    num_params = 0

    num_states = 0
    num_inputs = 0
    num_measurements = 0

    fixed_params = []

    @abstractmethod
    def calc_dx(
        self, x: np.array, u: np.array, params: np.array, numerical: bool = False
    ):
        pass

    def calc_y(
        self,
        x: np.array,
        u: np.array,
        numerical: bool = False,
        params: np.ndarray = None,
    ):
        """
        Method is used to specify the measurement model.

        This method can be omitted in sub-classes if the full state vector is observed.
        Args:
            x (np.array): state vector
            u (np.array): input vector
            numerical (bool, optional): If `numerical` is set to `False`, the calculations should be done using `casadi` commands, so they can be used by the optimizer.
            params (np.ndarray, optional): Parameters of the model. Defaults to None.
        """
        return x

    def initial_estimate(self, y: np.array, u: np.array, dt: float):
        N = u.shape[1]
        x0 = np.zeros((self.num_states, N + 1))
        return x0

    def calc_step_ee(
        self,
        x: np.array,
        u: np.array,
        params: np.array,
        dt: float,
        numerical: bool = False,
    ):
        return dt * self.calc_dx(x, u, params, numerical)

    def calc_step_rk4(
        self,
        x: np.array,
        u: np.array,
        params: np.array,
        dt: float,
        numerical: bool = False,
    ):
        k1 = self.calc_dx(x, u, params, numerical)
        k2 = self.calc_dx(x + dt / 2.0 * k1, u, params, numerical)
        k3 = self.calc_dx(x + dt / 2.0 * k2, u, params, numerical)
        k4 = self.calc_dx(x + dt * k3, u, params, numerical)

        out = dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
        return out
