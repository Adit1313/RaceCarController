import numpy as np

from abc import abstractmethod


class SensorModel:
    """
    Base class for a sensor model.

    A sensor model calculates the measurement vector `y` from the state vector `x`.
    It is distinct from a model in that it does not have a dynamic model, only a measurement model.
    """

    num_measurements = 0

    @abstractmethod
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
        pass
