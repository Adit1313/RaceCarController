from casadi import vertcat, cos, sin, tan, asin, atan, sqrt, power, pi
from casadi import *
import math
import numpy as np
from acados_template import AcadosModel


def lighthouse_sweep_1_model(pacejka_model: AcadosModel):
    x = pacejka_model.x[0]
    y = pacejka_model.x[1]
    yaw = pacejka_model.x[2]

    sensor_pos_1x = SX.sym("sensor_pos_1x")
    sensor_pos_2x = SX.sym("sensor_pos_2x")
    sensor_pos_3x = SX.sym("sensor_pos_3x")
    sensor_pos_4x = SX.sym("sensor_pos_4x")

    sensor_pos_1y = SX.sym("sensor_pos_1y")
    sensor_pos_2y = SX.sym("sensor_pos_2y")
    sensor_pos_3y = SX.sym("sensor_pos_3y")
    sensor_pos_4y = SX.sym("sensor_pos_4y")

    bs_position_x = SX.sym("bs_position_x")
    bs_position_y = SX.sym("bs_position_y")
    bs_position_z = SX.sym("bs_position_z")

    bs_rotation_00 = SX.sym("bs_rotation_00")
    bs_rotation_01 = SX.sym("bs_rotation_01")
    bs_rotation_02 = SX.sym("bs_rotation_02")
    bs_rotation_10 = SX.sym("bs_rotation_10")
    bs_rotation_11 = SX.sym("bs_rotation_11")
    bs_rotation_12 = SX.sym("bs_rotation_12")
    bs_rotation_20 = SX.sym("bs_rotation_20")
    bs_rotation_21 = SX.sym("bs_rotation_21")
    bs_rotation_22 = SX.sym("bs_rotation_22")

    light_plane_tilt_1 = SX.sym("light_plane_tilt_1")

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

    # 4 angle measurements from the first sweep plane
    alpha = alphas + asin(
        (sbs[2, :] * tan(light_plane_tilt_1))
        / (sqrt(power(sbs[0, :], 2) + power(sbs[1, :], 2)))
    )

    pacejka_model.p = vertcat(
        pacejka_model.p,
        sensor_pos_1x,
        sensor_pos_2x,
        sensor_pos_3x,
        sensor_pos_4x,
        sensor_pos_1y,
        sensor_pos_2y,
        sensor_pos_3y,
        sensor_pos_4y,
        bs_position_x,
        bs_position_y,
        bs_position_z,
        bs_rotation_00,
        bs_rotation_01,
        bs_rotation_02,
        bs_rotation_10,
        bs_rotation_11,
        bs_rotation_12,
        bs_rotation_20,
        bs_rotation_21,
        bs_rotation_22,
        light_plane_tilt_1,
    )

    return (
        vertcat(
            alpha[0],
            alpha[1],
            alpha[2],
            alpha[3],
        ),
        pacejka_model.p,
    )
