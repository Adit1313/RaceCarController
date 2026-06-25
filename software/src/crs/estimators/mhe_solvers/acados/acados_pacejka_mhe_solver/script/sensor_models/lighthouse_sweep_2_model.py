from casadi import vertcat, cos, sin, tan, asin, atan, sqrt, power, pi
from casadi import *
import math
import numpy as np
from acados_template import AcadosModel


def lighthouse_sweep_2_model(pacejka_model: AcadosModel):
    x = pacejka_model.x[0]
    y = pacejka_model.x[1]
    yaw = pacejka_model.x[2]

    sensor_pos_1x = pacejka_model.p[21]
    sensor_pos_2x = pacejka_model.p[22]
    sensor_pos_3x = pacejka_model.p[23]
    sensor_pos_4x = pacejka_model.p[24]

    sensor_pos_1y = pacejka_model.p[25]
    sensor_pos_2y = pacejka_model.p[26]
    sensor_pos_3y = pacejka_model.p[27]
    sensor_pos_4y = pacejka_model.p[28]

    bs_position_x = pacejka_model.p[29]
    bs_position_y = pacejka_model.p[30]
    bs_position_z = pacejka_model.p[31]

    bs_rotation_00 = pacejka_model.p[32]
    bs_rotation_01 = pacejka_model.p[33]
    bs_rotation_02 = pacejka_model.p[34]
    bs_rotation_10 = pacejka_model.p[35]
    bs_rotation_11 = pacejka_model.p[36]
    bs_rotation_12 = pacejka_model.p[37]
    bs_rotation_20 = pacejka_model.p[38]
    bs_rotation_21 = pacejka_model.p[39]
    bs_rotation_22 = pacejka_model.p[40]

    light_plane_tilt_2 = SX.sym("light_plane_tilt_2")

    sensor_positions = vertcat(
        horzcat(sensor_pos_1x, sensor_pos_2x, sensor_pos_3x, sensor_pos_4x),
        horzcat(sensor_pos_1y, sensor_pos_2y, sensor_pos_3y, sensor_pos_4y),
    )
    bs_position = vertcat(bs_position_x, bs_position_y, bs_position_z)
    bs_rotation = vertcat(
        horzcat(bs_rotation_00, bs_rotation_01, bs_rotation_02),
        horzcat(bs_rotation_10, bs_rotation_11, bs_rotation_12),
        horzcat(bs_rotation_20, bs_rotation_21, bs_rotation_22),
    )

    rot = vertcat(
        horzcat(cos(yaw), -sin(yaw)), horzcat(sin(yaw), cos(yaw)), horzcat(SX(0), SX(0))
    )
    pos_car = vertcat(x, y, 0)

    pos_sensor_w = np.matmul(rot, sensor_positions)  # rot * sensor_positions
    for i in range(4):
        pos_sensor_w[:, i] += pos_car

    rot_bs = bs_rotation.T
    for i in range(3):
        pos_sensor_w[i, :] -= bs_position[i]

    sbs = np.matmul(rot_bs, pos_sensor_w)
    alphas = atan(sbs[1, :] / sbs[0, :])

    # 4 angle measurements from the second sweep plane
    alpha = alphas + asin(
        (sbs[2, :] * tan(light_plane_tilt_2))
        / (sqrt(power(sbs[0, :], 2) + power(sbs[1, :], 2)))
    )

    pacejka_model.p = vertcat(pacejka_model.p, light_plane_tilt_2)

    return (
        vertcat(
            alpha[0],
            alpha[1],
            alpha[2],
            alpha[3],
        ),
        pacejka_model.p,
    )
