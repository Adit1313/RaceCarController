import sympy as sp
import yaml
import numpy as np
import os
from typing import List, Callable

from pacejka_model import pacejka_model_vector


def generate_jacobian(
    dynamic_function: Callable[[List[sp.Symbol], List[sp.Symbol], dict], sp.Matrix],
    state: List[sp.Symbol],
    input: List[sp.Symbol],
    parameters: dict,
):
    # Seed the numpy random generator
    np.random.seed(41)

    # Get Jacobian of dynamic_function wrt state
    f_jac_state_A = dynamic_function.jacobian(state)
    # In the EKF B is not used. If at some point it is needed, uncomment the following lines
    # f_jac_input_B = dynamic_function.jacobian(input)

    # Load parameters
    params = yaml.load(
        open(
            os.path.dirname(os.path.abspath(__file__))
            + "/params/test_pacejka_params.yaml",
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

    # create testcases
    for i in range(10):
        # Load random state and control input
        state_val = np.random.rand(6) * 2 - 0.5
        control_input_val = np.random.rand(2)
        # Make sure vx is well defined
        state_val[3] += 1.2

        f_jac_state_val = f_jac_state_A.subs(
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

        # In the EKF B is not used. If at some point it is needed, uncomment the following lines
        # f_jac_input_val = f_jac_input_B.subs(
        #     [
        #         (parameters["lr"], lr_val),
        #         (parameters["lf"], lf_val),
        #         (parameters["m"], m_val),
        #         (parameters["I"], I_val),
        #         (parameters["Df"], Df_val),
        #         (parameters["Cf"], Cf_val),
        #         (parameters["Bf"], Bf_val),
        #         (parameters["Dr"], Dr_val),
        #         (parameters["Cr"], Cr_val),
        #         (parameters["Br"], Br_val),
        #         (parameters["Cm1"], Cm1_val),
        #         (parameters["Cm2"], Cm2_val),
        #         (parameters["Cd0"], Cd0_val),
        #         (parameters["Cd1"], Cd1_val),
        #         (parameters["Cd2"], Cd2_val),
        #         (parameters["gamma"], gamma_val),
        #         (parameters["eps"], eps_val),
        #         (parameters["car_width"], car_width_val),
        #         (parameters["wheel_radius"], wheel_radius_val),
        #         (state[0], state_val[0]),
        #         (state[1], state_val[1]),
        #         (state[2], state_val[2]),
        #         (state[3], state_val[3]),
        #         (state[4], state_val[4]),
        #         (state[5], state_val[5]),
        #         (input[0], control_input_val[0]),
        #         (input[1], control_input_val[1]),
        #     ]
        # )

        # save state and control input as .csv
        np.savetxt(
            os.path.dirname(os.path.abspath(__file__))
            + "/data/state_"
            + str(i)
            + ".csv",
            state_val,
            delimiter=",",
            fmt="%f",
        )
        np.savetxt(
            os.path.dirname(os.path.abspath(__file__))
            + "/data/control_input_"
            + str(i)
            + ".csv",
            control_input_val,
            delimiter=",",
            fmt="%f",
        )
        # save jacobian A as .csv
        np.savetxt(
            os.path.dirname(os.path.abspath(__file__))
            + "/data/jacobian_A_"
            + str(i)
            + ".csv",
            np.array(f_jac_state_val).astype(np.float64),
            delimiter=",",
            fmt="%f",
        )
        # save jacobian B as .csv
        # In the EKF B is not used. If at some point it is needed, uncomment the following lines
        # np.savetxt(
        #     os.path.dirname(os.path.abspath(__file__))
        #     + "/data/jacobian_B_"
        #     + str(i)
        #     + ".csv",
        #     np.array(f_jac_input_val).astype(np.float64),
        #     delimiter=",",
        #     fmt="%f",
        # )


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

    x, y, yaw, vx, vy, omega, T, delta = sp.symbols(
        "x y yaw vx vy omega T delta", real=True
    )
    state = [x, y, yaw, vx, vy, omega]
    control_input = [T, delta]
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

    dyn_fnc = pacejka_model_vector(state, control_input, params)
    generate_jacobian(dyn_fnc, state, control_input, params)


if __name__ == "__main__":
    main()
