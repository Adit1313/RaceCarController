import numpy as np
import rospy

from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA


class RvizPublisherMPC:
    def __init__(self, N_horizon):
        self.N = N_horizon

        # controller planned trajectory indices
        self.pt_idx_x = 0
        self.pt_idx_y = 1
        self.pt_idx_yaw = 2
        self.pt_idx_vxb = 3
        self.pt_idx_vyb = 4
        self.pt_idx_vyaw = 5
        self.pt_idx_torque = 6
        self.pt_idx_steer = 7
        self.pt_idx_theta = 8

        # controller reference trajectory indices
        self.rt_idx_x = 0
        self.rt_idx_y = 1
        self.rt_idx_phi = 2

        self.publish_init_position_init()
        self.publish_trajectory_init()
        self.publish_uncertainty_init()

    def publish_trajectory_init(self):
        # print(rospy.get_param_names())
        use_arrows = rospy.get_param("~visualizer/use_arrows")
        frame_id = rospy.get_param("~visualizer/frame_id")
        self.vis_min_velocity = rospy.get_param("~visualizer/min_velocity")
        self.vis_max_velocity = rospy.get_param("~visualizer/max_velocity")
        namespace = rospy.get_param("~visualizer/namespace")

        planned_size_x = rospy.get_param("~visualizer/planned/size_x")
        planned_size_y = rospy.get_param("~visualizer/planned/size_y")
        planned_size_z = rospy.get_param("~visualizer/planned/size_z")

        reference_r = rospy.get_param("~visualizer/reference/r")
        reference_g = rospy.get_param("~visualizer/reference/g")
        reference_b = rospy.get_param("~visualizer/reference/b")
        reference_a = rospy.get_param("~visualizer/reference/a")
        reference_size_x = rospy.get_param("~visualizer/reference/size_x")
        reference_size_y = rospy.get_param("~visualizer/reference/size_y")
        reference_size_z = rospy.get_param("~visualizer/reference/size_z")

        color = ColorRGBA()
        color.a = 1
        color.r = 0
        color.b = 1
        color.g = 0

        self.marker_planned = Marker()
        self.marker_planned.header.frame_id = frame_id
        self.marker_planned.ns = namespace + "_planned_trajectory"
        self.marker_planned.id = 0
        self.marker_planned.scale.x = planned_size_x
        self.marker_planned.scale.y = planned_size_y
        self.marker_planned.scale.z = planned_size_z
        self.marker_planned.action = Marker.ADD

        self.marker_planned.color.r = color.r
        self.marker_planned.color.g = color.g
        self.marker_planned.color.b = color.b
        self.marker_planned.color.a = color.a

        if use_arrows:
            self.marker_planned.type = Marker.ARROW
        else:
            self.marker_planned.type = Marker.POINT

        self.marker_reference = Marker()
        self.marker_reference.header.frame_id = frame_id
        self.marker_reference.ns = namespace + "_reference_trajectory"
        self.marker_reference.id = 0
        self.marker_reference.scale.x = reference_size_x
        self.marker_reference.scale.y = reference_size_y
        self.marker_reference.scale.z = reference_size_z
        self.marker_reference.action = Marker.ADD

        self.marker_reference.color.r = reference_r
        self.marker_reference.color.g = reference_g
        self.marker_reference.color.b = reference_b
        self.marker_reference.color.a = reference_a

        self.prediction_pub = rospy.Publisher(
            "~visualizer/controller_visualization",
            Marker,
            queue_size=3 * self.N + 1,
        )

        self.reference_pub = rospy.Publisher(
            "~visualizer/controller_visualization",
            Marker,
            queue_size=3 * self.N + 1,
        )

    def publish_uncertainty_init(self):
        frame_id = rospy.get_param("~visualizer/frame_id")
        namespace = rospy.get_param("~visualizer/namespace")

        color = ColorRGBA()
        color.a = 0.3
        color.r = 0
        color.b = 1
        color.g = 0

        # initialize ellipses
        self.marker_planned_ell = Marker()
        self.marker_planned_ell.header.frame_id = frame_id
        self.marker_planned_ell.ns = namespace + "_planned_trajectory_ell"
        self.marker_planned_ell.id = 0
        self.marker_planned_ell.scale.x = 1.0
        self.marker_planned_ell.scale.y = 1.0
        self.marker_planned_ell.scale.z = 0.01
        self.marker_planned_ell.color.r = color.r
        self.marker_planned_ell.color.g = color.g
        self.marker_planned_ell.color.b = color.b
        self.marker_planned_ell.color.a = color.a
        self.marker_planned_ell.type = Marker.CYLINDER
        self.marker_planned_ell.action = Marker.ADD

        self.prediction_ell_pub = rospy.Publisher(
            "~visualizer/controller_visualization",
            Marker,
            queue_size=3 * self.N + 1,
        )

    def publish_init_position_init(self):
        frame_id = rospy.get_param("~visualizer/frame_id")
        namespace = rospy.get_param("~visualizer/namespace")
        planned_size_x = rospy.get_param("~visualizer/planned/size_x")
        planned_size_y = rospy.get_param("~visualizer/planned/size_y")
        planned_size_z = rospy.get_param("~visualizer/planned/size_z")

        self.marker_x_init = Marker()
        self.marker_x_init.header.frame_id = frame_id
        self.marker_x_init.ns = namespace + "_x_init"
        self.marker_x_init.id = 0
        self.marker_x_init.color.r = 1
        self.marker_x_init.color.g = 0
        self.marker_x_init.color.b = 0
        self.marker_x_init.color.a = 1
        self.marker_x_init.action = Marker.ADD
        self.marker_x_init.type = Marker.ARROW
        self.marker_x_init.scale.x = planned_size_x
        self.marker_x_init.scale.y = planned_size_y
        self.marker_x_init.scale.z = planned_size_z

        self.prediction_x_init_pub = rospy.Publisher(
            "~visualizer/controller_visualization", Marker, queue_size=1
        )

    def publish_init_position(self, current_state):
        time = rospy.Time.now()
        x = current_state[self.pt_idx_x]
        y = current_state[self.pt_idx_y]
        yaw = current_state[self.pt_idx_yaw]
        vxb = current_state[self.pt_idx_vxb]
        vyb = current_state[self.pt_idx_vyb]

        # publish initial condition
        self.marker_x_init.pose.position.x = x
        self.marker_x_init.pose.position.y = y
        self.marker_x_init.pose.position.z = 0.0
        self.marker_x_init.pose.orientation.w = np.cos(0.5 * yaw)
        self.marker_x_init.pose.orientation.x = 0.0
        self.marker_x_init.pose.orientation.y = 0.0
        self.marker_x_init.pose.orientation.z = np.sin(0.5 * yaw)

        velocity = np.sqrt(vxb**2 + vyb**2)
        normalized_velocity = max(
            0.0,
            min(
                1.0,
                (velocity - self.vis_min_velocity)
                / (self.vis_max_velocity - self.vis_min_velocity),
            ),
        )
        if normalized_velocity < 0.5:
            self.marker_x_init.color.r = normalized_velocity
            self.marker_x_init.color.g = 0
            self.marker_x_init.color.b = 1 - normalized_velocity
        else:
            self.marker_x_init.color.r = 1 - normalized_velocity
            self.marker_x_init.color.g = normalized_velocity
            self.marker_x_init.color.b = 0
        self.marker_x_init.color.a = 1
        self.marker_x_init.id = 0
        self.prediction_x_init_pub.publish(self.marker_x_init)

    def publish_trajectory(self, planned_trajectory, reference_trajectory):
        time = rospy.Time.now()
        self.marker_planned.header.stamp = time
        self.marker_reference.header.stamp = time
        self.marker_x_init.header.stamp = time

        # TODO: get from controller:
        # - last_solution (states, reference_on_track)
        # - N
        # - theta_shift_index
        # - zogpmpc.covariances_array
        # - gamma_factor
        # -> just give full object and tie to it?
        for i in range(self.N):
            self.marker_planned.pose.position.x = planned_trajectory[i][self.pt_idx_x]
            self.marker_planned.pose.position.y = planned_trajectory[i][self.pt_idx_y]
            self.marker_planned.pose.position.z = 0.0
            self.marker_planned.pose.orientation.w = np.cos(
                planned_trajectory[i][self.pt_idx_yaw] * 0.5
            )
            self.marker_planned.pose.orientation.x = 0
            self.marker_planned.pose.orientation.y = 0
            self.marker_planned.pose.orientation.z = np.sin(
                planned_trajectory[i][self.pt_idx_yaw] * 0.5
            )

            velocity = np.sqrt(
                planned_trajectory[i][self.pt_idx_vxb] ** 2
                + planned_trajectory[i][self.pt_idx_vyb] ** 2
            )
            normalized_velocity = max(
                0.0,
                min(
                    1.0,
                    (velocity - self.vis_min_velocity)
                    / (self.vis_max_velocity - self.vis_min_velocity),
                ),
            )
            if normalized_velocity < 0.5:
                self.marker_planned.color.r = normalized_velocity
                self.marker_planned.color.g = 0
                self.marker_planned.color.b = 1 - normalized_velocity
            else:
                self.marker_planned.color.r = 1 - normalized_velocity
                self.marker_planned.color.g = normalized_velocity
                self.marker_planned.color.b = 0
            self.marker_planned.color.a = 1
            self.marker_planned.id = i

            self.marker_reference.pose.position.x = reference_trajectory[i][
                self.rt_idx_x
            ]
            self.marker_reference.pose.position.y = reference_trajectory[i][
                self.rt_idx_y
            ]
            self.marker_reference.pose.position.z = 0.0
            self.marker_reference.pose.orientation.w = np.cos(
                reference_trajectory[i][self.rt_idx_phi] * 0.5
            )
            self.marker_reference.pose.orientation.x = 0
            self.marker_reference.pose.orientation.y = 0
            self.marker_reference.pose.orientation.z = np.sin(
                reference_trajectory[i][self.rt_idx_phi] * 0.5
            )
            self.marker_reference.id = i

            self.prediction_pub.publish(self.marker_planned)
            self.prediction_pub.publish(self.marker_reference)

    def publish_uncertainties(self, planned_trajectory, planned_covariances):
        time = rospy.Time.now()
        # print(time)
        self.marker_planned_ell.header.stamp = time
        self.marker_planned_ell.points.clear()
        self.marker_planned_ell.colors.clear()

        # print(f"self.N = {self.N}")
        for i in range(self.N):
            self.marker_planned_ell.pose.position.x = planned_trajectory[i][
                self.pt_idx_x
            ]
            self.marker_planned_ell.pose.position.y = planned_trajectory[i][
                self.pt_idx_y
            ]
            self.marker_planned_ell.pose.position.z = 0.0

            if planned_covariances is None:
                return

            cov_x = planned_covariances[i][self.pt_idx_x, self.pt_idx_x]
            cov_y = planned_covariances[i][self.pt_idx_y, self.pt_idx_y]
            cov_xy = planned_covariances[i][self.pt_idx_x, self.pt_idx_y]
            current_covariance = np.array(
                [
                    [cov_x, cov_xy],
                    [cov_xy, cov_y],
                ]
            )

            self.set_ellipsoid(current_covariance)
            self.marker_planned_ell.id = i
            self.prediction_ell_pub.publish(self.marker_planned_ell)

        # TODO: final covariance missing?

    def set_ellipsoid(self, covariance_matrix):
        if not np.all(np.isfinite(covariance_matrix)):
            rospy.logwarn("covariance matrix is not finite. not visualizing")
            return

        # prject matrix to x-y-plane
        eigenvalues, eigenvectors = np.linalg.eig(covariance_matrix)

        # Compute the semi-axis lengths based on the eigenvalues
        # semi_axes_lengths = 0.99 * np.sqrt(eigenvalues)
        semi_axes_lengths = np.sqrt(eigenvalues)  # TODO: get this from ctrl file?

        angle_xy = np.arctan2(eigenvectors[1, 0], eigenvectors[0, 0])
        # if np.isnan(angle_xy):
        #     return
        self.marker_planned_ell.pose.orientation.x = 0.0
        self.marker_planned_ell.pose.orientation.y = 0.0
        self.marker_planned_ell.pose.orientation.z = np.sin(angle_xy * 0.5)
        self.marker_planned_ell.pose.orientation.w = np.cos(angle_xy * 0.5)

        # Update the scale of the self.marker_planned_ell based on the semi-axis lengths
        self.marker_planned_ell.scale.x = semi_axes_lengths[0]
        self.marker_planned_ell.scale.y = semi_axes_lengths[1]
