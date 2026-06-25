# Calculates the gt jacobians using simpy.
import sympy as sp
import yaml
import numpy as np
import os
import shutil
import random
from typing import List, Callable
from imu.get_measurement_vec import get_measurement_vector as imu_get_measurement_vector
from wheel_encoders.get_measurement_vec import (
    get_measurement_vector as wheel_encoders_get_measurement_vector,
)
from lighthouse.get_measurement_vec import (
    get_measurement_vector as lighthouse_get_measurement_vector,
)


def generate_jacobian(
    measurement_function: Callable[[List[sp.Symbol], List[sp.Symbol], dict], sp.Matrix],
    state: List[sp.Symbol],
    input: List[sp.Symbol],
    parameters: dict,
    sensor_name: str,
):
    random.seed(42)

    # Get Jacobian of measurement_function wrt state
    f_jac_state = measurement_function.jacobian(state)

    # Load parameters
    params = yaml.load(
        open(
            os.path.dirname(os.path.abspath(__file__))
            + "/params/example_pacejka_params.yaml",
            "r",
        ),
        yaml.FullLoader,
    )

    lr_val = params["lr"]
    lf_val = params["lf"]
    m_val = params["m"]
    I_val = params["I"]
    Df_val = params["Df"]
    Cf_val = params["Cf"]
    Bf_val = params["Bf"]
    Dr_val = params["Dr"]
    Cr_val = params["Cr"]
    Br_val = params["Br"]
    Cm1_val = params["Cm1"]
    Cm2_val = params["Cm2"]
    Cd0_val = params["Cd0"]
    Cd1_val = params["Cd1"]
    Cd2_val = params["Cd2"]
    gamma_val = params["gamma"]
    eps_val = params["eps"]
    car_width_val = params["car_width"]
    wheel_radius_val = params["wheel_radius"]

    if sensor_name == "lighthouse":
        lighthouse_params = yaml.load(
            open(
                os.path.dirname(os.path.abspath(__file__))
                + "/params/example_lighthouse_params.yaml",
                "r",
            ),
            yaml.FullLoader,
        )
        # Lighthouse parameters
        bs_position_val = lighthouse_params["bs_position"]
        bs_rotation_val = lighthouse_params["bs_rotation"]
        light_plane_tilt_val = lighthouse_params["light_plane_tilt"]
        sensor_positions_val = lighthouse_params["sensor_positions"]

    if os.path.exists(
        os.path.join(os.path.dirname(os.path.abspath(__file__)), sensor_name, "data")
    ) and os.path.isdir(
        os.path.join(os.path.dirname(os.path.abspath(__file__)), sensor_name, "data")
    ):
        shutil.rmtree(
            os.path.join(
                os.path.dirname(os.path.abspath(__file__)), sensor_name, "data"
            )
        )

    # create testcases
    for i in range(10):
        # Load random state and control input
        state_val = np.random.rand(6) * 2 - 0.5
        control_input_val = np.random.rand(2)
        # Make sure vx is well defined
        state_val[3] += 1.2
        # evaluate jacobian
        if sensor_name == "lighthouse":
            f_jac_state_val = f_jac_state.subs(
                [
                    (parameters["lr"], lr_val),
                    (parameters["lf"], lf_val),
                    (parameters["m"], m_val),
                    (parameters["I"], I_val),
                    (parameters["Df"], Df_val),
                    (parameters["Cf"], Cf_val),
                    (parameters["Bf"], Bf_val),
                    (parameters["Dr"], Dr_val),
                    (parameters["Cr"], Cr_val),
                    (parameters["Br"], Br_val),
                    (parameters["Cm1"], Cm1_val),
                    (parameters["Cm2"], Cm2_val),
                    (parameters["Cd0"], Cd0_val),
                    (parameters["Cd1"], Cd1_val),
                    (parameters["Cd2"], Cd2_val),
                    (parameters["gamma"], gamma_val),
                    (parameters["eps"], eps_val),
                    (parameters["car_width"], car_width_val),
                    (parameters["wheel_radius"], wheel_radius_val),
                    (parameters["bs_position"], sp.Matrix(bs_position_val)),
                    (parameters["bs_rotation"], sp.Matrix(bs_rotation_val)),
                    (parameters["light_plane_tilt"], light_plane_tilt_val),
                    (parameters["sensor_positions"], sp.Matrix(sensor_positions_val)),
                    (state[0], state_val[0]),
                    (state[1], state_val[1]),
                    (state[2], state_val[2]),
                    (state[3], state_val[3]),
                    (state[4], state_val[4]),
                    (state[5], state_val[5]),
                    (input[0], control_input_val[0]),
                    (input[1], control_input_val[1]),
                ]
            )
        else:
            f_jac_state_val = f_jac_state.subs(
                [
                    (parameters["lr"], lr_val),
                    (parameters["lf"], lf_val),
                    (parameters["m"], m_val),
                    (parameters["I"], I_val),
                    (parameters["Df"], Df_val),
                    (parameters["Cf"], Cf_val),
                    (parameters["Bf"], Bf_val),
                    (parameters["Dr"], Dr_val),
                    (parameters["Cr"], Cr_val),
                    (parameters["Br"], Br_val),
                    (parameters["Cm1"], Cm1_val),
                    (parameters["Cm2"], Cm2_val),
                    (parameters["Cd0"], Cd0_val),
                    (parameters["Cd1"], Cd1_val),
                    (parameters["Cd2"], Cd2_val),
                    (parameters["gamma"], gamma_val),
                    (parameters["eps"], eps_val),
                    (parameters["car_width"], car_width_val),
                    (parameters["wheel_radius"], wheel_radius_val),
                    (state[0], state_val[0]),
                    (state[1], state_val[1]),
                    (state[2], state_val[2]),
                    (state[3], state_val[3]),
                    (state[4], state_val[4]),
                    (state[5], state_val[5]),
                    (input[0], control_input_val[0]),
                    (input[1], control_input_val[1]),
                ]
            )

        os.makedirs(
            os.path.join(
                os.path.dirname(os.path.abspath(__file__)), sensor_name, "data"
            ),
            exist_ok=True,
        )

        # save state and control input as .csv
        np.savetxt(
            os.path.dirname(os.path.abspath(__file__))
            + "/"
            + sensor_name
            + "/data/state_"
            + str(i)
            + ".csv",
            state_val,
            delimiter=",",
            fmt="%f",
        )
        np.savetxt(
            os.path.dirname(os.path.abspath(__file__))
            + "/"
            + sensor_name
            + "/data/control_input_"
            + str(i)
            + ".csv",
            control_input_val,
            delimiter=",",
            fmt="%f",
        )
        # save jacobian as .csv
        np.savetxt(
            os.path.dirname(os.path.abspath(__file__))
            + "/"
            + sensor_name
            + "/data/jacobian_"
            + str(i)
            + ".csv",
            np.array(f_jac_state_val).astype(np.float64),
            delimiter=",",
            fmt="%f",
        )


def main():
    # Pacjeka parameters
    (
        lr,
        lf,
        m,
        I,
        Df,
        Cf,
        Bf,
        Dr,
        Cr,
        Br,
        Cm1,
        Cm2,
        Cd0,
        Cd1,
        Cd2,
        gamma,
        eps,
        car_width,
        wheel_radius,
    ) = sp.symbols(
        "lr lf m I Df Cf Bf Dr Cr Br Cm1 Cm2 Cd0 Cd1 Cd2 gamma eps car_width wheel_radius",
        real=True,
    )

    x, y, yaw, v_x, v_y, yaw_rate, torque, steer = sp.symbols(
        "x y yaw v_x v_y yaw_rate torque steer", real=True
    )
    state = [x, y, yaw, v_x, v_y, yaw_rate]
    control_input = [torque, steer]
    params = {
        "lr": lr,
        "lf": lf,
        "m": m,
        "I": I,
        "Df": Df,
        "Cf": Cf,
        "Bf": Bf,
        "Dr": Dr,
        "Cr": Cr,
        "Br": Br,
        "Cm1": Cm1,
        "Cm2": Cm2,
        "Cd0": Cd0,
        "Cd1": Cd1,
        "Cd2": Cd2,
        "gamma": gamma,
        "eps": eps,
        "car_width": car_width,
        "wheel_radius": wheel_radius,
    }

    # IMU
    imu_meas_fnc = imu_get_measurement_vector(state, control_input, params)
    generate_jacobian(imu_meas_fnc, state, control_input, params, "imu")

    # WHEEL ENCODERS
    wheel_meas_fnc = wheel_encoders_get_measurement_vector(state, control_input, params)
    generate_jacobian(wheel_meas_fnc, state, control_input, params, "wheel_encoders")

    # LIGHTHOUSE
    bs_position = sp.MatrixSymbol("bs_position", 3, 1)
    bs_rotation = sp.MatrixSymbol("bs_rotation", 3, 3)
    sensor_positions = sp.MatrixSymbol("sensor_positions", 2, 4)
    light_plane_tilt = sp.symbols("light_plane_tilt", real=True)

    params_ligthhouse = {
        "lr": lr,
        "lf": lf,
        "m": m,
        "I": I,
        "Df": Df,
        "Cf": Cf,
        "Bf": Bf,
        "Dr": Dr,
        "Cr": Cr,
        "Br": Br,
        "Cm1": Cm1,
        "Cm2": Cm2,
        "Cd0": Cd0,
        "Cd1": Cd1,
        "Cd2": Cd2,
        "gamma": gamma,
        "eps": eps,
        "car_width": car_width,
        "wheel_radius": wheel_radius,
        "bs_position": bs_position,
        "bs_rotation": bs_rotation,
        "light_plane_tilt": light_plane_tilt,
        "sensor_positions": sensor_positions,
    }
    lighthouse_meas_fnc = lighthouse_get_measurement_vector(
        state, control_input, params_ligthhouse
    )

    generate_jacobian(
        lighthouse_meas_fnc, state, control_input, params_ligthhouse, "lighthouse"
    )


if __name__ == "__main__":
    main()
