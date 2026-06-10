import numpy as np

from enum import IntEnum

from opt_sys_id.tools import enum2list


class Params(IntEnum):
    lr = 0
    lf = 1
    m = 2
    I = 3
    Df = 4
    Cf = 5
    Bf = 6
    Dr = 7
    Cr = 8
    Br = 9
    Cm1 = 10
    Cm2 = 11
    Cd0 = 12
    Cd1 = 13
    Cd2 = 14
    gamma = 15

    # Parameters for the wheel encoder sensor model
    wheel_radius = 16
    car_width = 17

    def get_default():
        params = np.zeros(len(Params))

        params[Params.lr] = 0.05
        params[Params.lf] = 0.05
        params[Params.m] = 0.242
        params[Params.I] = 0.0016

        params[Params.Df] = 0.65
        params[Params.Cf] = 1.5
        params[Params.Bf] = 5.2
        params[Params.Dr] = 1.0
        params[Params.Cr] = 1.45
        params[Params.Br] = 8.5

        params[Params.Cm1] = 0.98028992
        params[Params.Cm2] = 0.01814131
        params[Params.Cd0] = 0.014
        params[Params.Cd1] = 0.014
        params[Params.Cd2] = 0.06
        params[Params.gamma] = 0.5

        params[Params.wheel_radius] = 0.0175
        params[Params.car_width] = 0.088

        return params

    def print_params(params: np.ndarray):
        """Print a formatted list of the model parameters."""
        print("model_params:")
        for i, p in enumerate(params):
            print(f"  {Params(i).name}: {p:.8f}")

    def get_model1():
        params = Params.get_default()

        return params


class States(IntEnum):
    px = 0
    py = 1
    yaw = 2
    vx = 3
    vy = 4
    omega = 5

    def full():
        return enum2list(States)

    def pose():
        return [States.px, States.py, States.yaw]


class Inputs(IntEnum):
    T = 0
    delta = 1
