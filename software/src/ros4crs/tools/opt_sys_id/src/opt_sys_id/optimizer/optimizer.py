import casadi as ca

from abc import abstractmethod

from opt_sys_id.model import Model


class Optimizer:
    solver = ca.Function()

    def __init__(self, model: Model, dt: float, N: int) -> None:
        self.model = model
        self.dt = dt
        self.N = N

        self.formulate_solver()

    @abstractmethod
    def formulate_solver(self):
        pass

    @abstractmethod
    def find_params(self, *args):
        pass
