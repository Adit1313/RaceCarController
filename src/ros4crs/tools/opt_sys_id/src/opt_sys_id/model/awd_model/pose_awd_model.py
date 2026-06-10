import numpy as np

from .awd_model import AwdBicycleModel
from .structures import States

from opt_sys_id.tools import finite_differences, bartlett_smoothing, unwrap_mod2pi


class PoseAwdBicycleModel(AwdBicycleModel):
    """An AwdBicycleModel that only considers the pose states, not including twist."""

    smoothing_scale = 20

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(States.pose(), *args, **kwargs)

    def initial_estimate(self, y: np.array, u: np.array, dt: float):
        x0 = super().initial_estimate(y, u, dt)

        # Unwrap yaw
        yaw = unwrap_mod2pi(x0[States.yaw, :])

        # Use finite differences to calculate velocities
        pxdt = finite_differences(x0, dt, States.px)
        pydt = finite_differences(x0, dt, States.py)
        vx0 = pxdt * np.cos(yaw[:-1]) + pydt * np.sin(yaw[:-1])

        omega0 = finite_differences(x0, dt, States.yaw)

        # Use smoothed velocities
        x0[States.vx, :-1] = bartlett_smoothing(vx0, self.smoothing_scale)
        x0[States.omega, :-1] = bartlett_smoothing(omega0, self.smoothing_scale)

        # Complete terminal velocities, which are missing due to finite difference calculation
        x0[States.vx, -1] = x0[States.vx, -2]
        x0[States.omega, -1] = x0[States.omega, -2]

        return x0
