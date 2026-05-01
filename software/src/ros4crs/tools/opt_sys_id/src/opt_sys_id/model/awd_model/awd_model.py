import casadi as ca
import numpy as np

from typing import List

from opt_sys_id.model import Model
from opt_sys_id.sensor_model import SensorModel

from .structures import Params, States, Inputs


class AwdBicycleModel(Model):
    """
    All-wheel drive bicycle model.

    In addition to the standard bicycle model, this model splits the longitudinal force between the front and rear wheels.
    """

    num_params = len(Params)

    def __init__(
        self,
        states: List[States] = States.full(),
        sensor_models: List[SensorModel] = None,
    ) -> None:
        """
        Initialize the AWD bicycle model.

        Args:
            states: List of states to be measured. Defaults to all states.
            sensor_models: List of sensor models to be used for measurement. Defaults to None.
        """
        super().__init__()

        self.num_states = 6
        self.num_inputs = 2
        self.measured_states = states

        # Register the sensor models and count the number of measurements. Later, in calc_y, stack
        # the measurement vectors according to the order of the sensor models.
        if sensor_models is not None:
            self.num_measurements = sum(
                [model.num_measurements for model in sensor_models]
            )
            print(f"Number of measurements: {self.num_measurements}")
            self.sensor_models = sensor_models
        else:
            print("NO measurement models added")
            self.num_measurements = 0
            self.sensor_models = []

    def calc_dx(
        self, x: np.array, u: np.array, params: np.array, numerical: bool = False
    ):
        # size params
        p_lr = params[Params.lr]  # [m]       distance from CoG to rear axis
        p_lf = params[Params.lf]  # [m]       distance from CoG to front axis
        p_m = params[Params.m]  # [kg]      vehicle mass
        p_I = params[Params.I]  # [kg m^2]  moment of inertia around vertical axis

        # lateral force params
        p_Df = params[Params.Df]  #           front Pacejka peak value
        p_Cf = params[Params.Cf]  #           front Pacejka shape factor
        p_Bf = params[Params.Bf]  #           front Pacejka stiffness factor
        p_Dr = params[Params.Dr]  #           rear Pacejka peak value
        p_Cr = params[Params.Cr]  #           rear Pacejka shape factor
        p_Br = params[Params.Br]  #           rear Pacejka stiffness factor

        # longitudinal force params
        p_Cm1 = params[Params.Cm1]  #           motor parameter 1
        p_Cm2 = params[Params.Cm2]  #           motor parameter 2
        p_Cd0 = params[Params.Cd0]  #           0th friction parameter
        p_Cd1 = params[Params.Cd1]  #           1st friction parameter
        p_Cd2 = params[Params.Cd2]  #           2nd friction parameter

        # force distribution parameter
        p_gamma = params[Params.gamma]  #           force distribution parameter

        yaw = x[States.yaw]  #           yaw
        vx = x[States.vx]  #           longitudinal velocity component
        vy = x[States.vy]  #           lateral velocity component
        omega = x[States.omega]  #           yaw rate

        T = u[Inputs.T]  #           torque input
        delta = u[Inputs.delta]  #           steering

        # System Model
        Ff = ca.sign(vx) * (-p_Cd2 * vx * vx - p_Cd1 * vx - p_Cd0)
        Fm = (p_Cm1 - p_Cm2 * vx) * T
        Fx_r = p_gamma * Fm
        Fx_f = (1 - p_gamma) * Fm

        ar = ca.arctan2(-vy + p_lr * omega, vx)
        af = delta + ca.arctan2(-vy - p_lf * omega, vx)
        Fy_r = p_Dr * ca.sin(p_Cr * ca.arctan(p_Br * ar))
        Fy_f = p_Df * ca.sin(p_Cf * ca.arctan(p_Bf * af))

        if numerical:
            # disable formatter because these lines are easier to read even if too long
            # fmt: off
            return np.array(
                [
                    vx * np.cos(yaw) - vy * np.sin(yaw),
                    vx * np.sin(yaw) + vy * np.cos(yaw),
                    omega,
                    1 / p_m * (Fx_r + Fx_f * np.cos(delta) - Fy_f * np.sin(delta) + p_m * vy * omega + Ff),
                    1 / p_m * (Fy_r + Fx_f * np.sin(delta) + Fy_f * np.cos(delta) - p_m * vx * omega),
                    1 / p_I * (Fy_f * p_lf * np.cos(delta) + Fx_f * p_lf * np.sin(delta) - Fy_r * p_lr),
                ]
            )
            # fmt: on
        else:
            # fmt: off
            return ca.vertcat(
                vx * ca.cos(yaw) - vy * ca.sin(yaw),
                vx * ca.sin(yaw) + vy * ca.cos(yaw),
                omega,
                1 / p_m * (Fx_r + Fx_f * ca.cos(delta) - Fy_f * ca.sin(delta) + p_m * vy * omega + Ff),
                1 / p_m * (Fy_r + Fx_f * ca.sin(delta) + Fy_f * ca.cos(delta) - p_m * vx * omega),
                1 / p_I * (Fy_f * p_lf * ca.cos(delta) + Fx_f * p_lf * ca.sin(delta) - Fy_r * p_lr),
            )
            # fmt: on

    def calc_y(
        self,
        x: np.array,
        u: np.array,
        numerical: bool = False,
        params: np.ndarray = None,
    ):
        # Calculate the measurements for each sensor model and stack them
        # according to the order passed at initialization.
        if numerical:
            first_sensor = True
            for model in self.sensor_models:
                if first_sensor:
                    y = model.calc_y(x, u, numerical, params).reshape([-1, 1])
                    first_sensor = False
                else:
                    y = np.vstack(
                        (y, model.calc_y(x, u, numerical, params).reshape([-1, 1]))
                    )
            return np.squeeze(y)
        else:
            return ca.vertcat(
                *[model.calc_y(x, u, numerical, params) for model in self.sensor_models]
            )

    def initial_estimate(self, y: np.array, u: np.array, dt: float):
        x0 = super().initial_estimate(y, u, dt)

        # Avoid singularity for low vx
        x0[States.vx, :] = 3

        # Use measured states
        for i in range(len(self.measured_states)):
            state = self.measured_states[i]
            x0[state, :] = y[i, :]

        return x0

    def convert_vx_to_T(self, vx: float, params: np.array) -> float:
        p_Cm1 = params[Params.Cm1]
        p_Cm2 = params[Params.Cm2]
        p_Cd = params[Params.Cd]
        p_Croll = params[Params.Croll]

        return (p_Cd * vx * vx + p_Croll) / (p_Cm1 - p_Cm2 * vx)
