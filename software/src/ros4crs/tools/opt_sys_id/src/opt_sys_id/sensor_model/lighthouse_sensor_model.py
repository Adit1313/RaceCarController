import copy

import numpy as np
import casadi as ca

from .sensor_model import SensorModel


class LighthouseSensorModel(SensorModel):
    """Sensor model for a Lighthouse sensor with all 8 angles unified."""

    def __init__(
        self, T_bs: np.ndarray, sensor_offsets: np.ndarray, dt1: float, dt2: float
    ):
        """
        Initialize the Lighthouse sensor model.

        Args:
            T_bs (np.ndarray): Pose of the base station in world frame (Transform BS->World)
            sensor_offsets (np.ndarray): Position of the sensors in the car frame
            dt1 (float): Tilt offset of the first sweep plane
            dt2 (float): Tilt offset of the second sweep plane
        """
        self.num_measurements = 8

        assert T_bs.shape == (4, 4)
        assert sensor_offsets.shape == (2, 4)

        self.Rot_bs = T_bs[:3, :3]
        self.p_bs = T_bs[:3, 3]
        self.sensor_offsets = sensor_offsets
        self.dt1 = dt1
        self.dt2 = dt2

    def calc_y(
        self,
        x: np.array,
        u: np.array,
        numerical: bool = False,
        params: np.ndarray = None,
    ):
        xp, yp, yaw = x[0], x[1], x[2]

        # All 8 angle measurements from both sweep planes
        if numerical:
            # Calculate positions of the sensors in world frame
            Rot_car = np.vstack(
                (
                    np.hstack((np.cos(yaw), -np.sin(yaw))),
                    np.hstack((np.sin(yaw), np.cos(yaw))),
                    np.hstack((0, 0)),
                )
            )
            p_car = np.vstack((xp, yp, 0))

            pos_sensor_w = Rot_car @ self.sensor_offsets + p_car

            # Calculate the angles at which the sensors are hit by the base station

            rot_bs = self.Rot_bs.T  # Rot_bs is BS -> World, we need World -> BS

            # colwise subtract p_bs
            for i in range(3):
                pos_sensor_w[i, :] -= self.p_bs[i]

            sbs = rot_bs @ pos_sensor_w

            # alphas are the angles
            alphas = np.arctan(sbs[1, :] / sbs[0, :])

            light_plane_tilt_1 = -np.pi / 6 - self.dt1
            light_plane_tilt_2 = np.pi / 6 - self.dt2

            # 4 angle measurements from the first sweep plane
            alpha_1 = copy.deepcopy(
                alphas
            )  # make sure to copy the array, not just the reference
            alpha_2 = copy.deepcopy(alphas)

            for i in range(4):
                alpha_1[i] += np.arcsin(
                    sbs[2, i] * np.tan(light_plane_tilt_1) / np.linalg.norm(sbs[0:2, i])
                )
                alpha_2[i] += np.arcsin(
                    sbs[2, i] * np.tan(light_plane_tilt_2) / np.linalg.norm(sbs[0:2, i])
                )

            return np.vstack(
                (
                    alpha_1[0],
                    alpha_1[1],
                    alpha_1[2],
                    alpha_1[3],
                    alpha_2[0],
                    alpha_2[1],
                    alpha_2[2],
                    alpha_2[3],
                )
            )
        else:
            # Calculate positions of the sensors in world frame
            Rot_car = ca.vertcat(
                ca.horzcat(ca.cos(yaw), -ca.sin(yaw)),
                ca.horzcat(ca.sin(yaw), ca.cos(yaw)),
                ca.horzcat(0, 0),
            )
            p_car = ca.vertcat(xp, yp, 0)

            pos_sensor_w = Rot_car @ self.sensor_offsets + p_car

            # Calculate the angles at which the sensors are hit by the base station

            rot_bs = self.Rot_bs.T  # Rot_bs is BS -> World, we need World -> BS

            # colwise subtract p_bs
            for i in range(3):
                pos_sensor_w[i, :] -= self.p_bs[i]

            sbs = rot_bs @ pos_sensor_w

            # alphas are the angles
            alphas = ca.atan(sbs[1, :] / sbs[0, :])

            light_plane_tilt_1 = -np.pi / 6 - self.dt1
            light_plane_tilt_2 = np.pi / 6 - self.dt2

            # 4 angle measurements from the first sweep plane
            alpha_1 = copy.deepcopy(
                alphas
            )  # make sure to copy the array, not just the reference
            alpha_2 = copy.deepcopy(alphas)

            for i in range(4):
                alpha_1[i] += ca.asin(
                    sbs[2, i] * ca.tan(light_plane_tilt_1) / ca.norm_2(sbs[0:2, i])
                )
                alpha_2[i] += ca.asin(
                    sbs[2, i] * ca.tan(light_plane_tilt_2) / ca.norm_2(sbs[0:2, i])
                )

            alpha = ca.vertcat(
                alpha_1[0],
                alpha_1[1],
                alpha_1[2],
                alpha_1[3],
                alpha_2[0],
                alpha_2[1],
                alpha_2[2],
                alpha_2[3],
            )

        return alpha
