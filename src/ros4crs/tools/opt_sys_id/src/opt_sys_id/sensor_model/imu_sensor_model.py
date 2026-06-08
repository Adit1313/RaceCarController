import numpy as np
import casadi as ca

from .sensor_model import SensorModel


class YawRateSensorModel(SensorModel):
    """
    Sensor model for an inertial measurement unit (IMU).

    In the current configuration, we only use the yaw rate from the IMU. Therefore, the
    sensor model is linear and only has an output size of one.
    """

    num_measurements = 1

    def calc_y(
        self,
        x: np.array,
        u: np.array,
        numerical: bool = False,
        params: np.ndarray = None,
    ):
        dyaw = x[5]
        if numerical:
            return np.array([dyaw])
        else:
            return ca.vertcat(*[dyaw])
