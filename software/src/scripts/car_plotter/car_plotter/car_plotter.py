import os
from typing import List

import matplotlib.pyplot as plt
import numpy as np
import oyaml
import rosbag
import rospkg
from matplotlib.collections import LineCollection


def get_states_from_bag(filename: str, data_topic_name: str, list_topics: bool = False):
    """Load the car data from a bag

    Args:
        - filename: Filepath to the bag file.
        - data_topic_name: Name of the car topic.
        - list_topics: Whether to print information about the bag content.
    """

    bag = rosbag.Bag(filename)  # read the bag

    topics = bag.get_type_and_topic_info()[1]  # get the topics in bag
    if list_topics:
        for topic_name, topic_tuple in topics.items():
            print(topic_name)
            print(
                f" Type:\t{topic_tuple.msg_type} \n"
                f"Count:\t{topic_tuple.message_count} \n"
                f" Freq:\t{topic_tuple.frequency} \n"
            )

    if data_topic_name not in topics.keys():
        raise RuntimeError(
            f"The topic{data_topic_name} is not contianed in "
            f"the rosbag {filename} use the -l option to list the topics."
        )

    state_list: List[np.ndarray] = []
    for _, (_, msg, _) in enumerate(bag.read_messages(topics=[data_topic_name])):
        curr_state = np.array([msg.x, msg.y, np.sqrt(msg.vx_b**2 + msg.vy_b**2)])
        state_list.append(curr_state)

    states = np.array(state_list)
    return states


def plot_track(ax, track_name: str = "DEMO_TRACK.yaml", plot_centerline: bool = False):
    """Plot the track on the specified axis

    Args:
        - ax: The mpl axis where the track should be plotted.
        - track_name: The filename of the track used.
        - plot_centerline: Whether the track centerline should be shown.
    """

    rospack = rospkg.RosPack()
    package_path = rospack.get_path("track_generation")
    track_path = os.path.join(package_path, "tracks", track_name)
    half_track_width = 0.23

    with open(track_path) as file:
        track = oyaml.load(file, Loader=oyaml.FullLoader)

    track = track["track"]

    xtrack = np.array(track["xCoords"])
    ytrack = np.array(track["yCoords"])
    xrate = np.array(track["xRate"])
    yrate = np.array(track["yRate"])

    if plot_centerline:
        ax.plot(xtrack, ytrack, "k", alpha=0.3)
    ax.plot(xtrack + half_track_width * yrate, ytrack - half_track_width * xrate, "k")
    ax.plot(xtrack - half_track_width * yrate, ytrack + half_track_width * xrate, "k")


def plot_trajectory(states: np.ndarray, ax, color=None, alpha=1):
    """Plot the car trajectory

    Args:
        - states: The car trajectory consisting of x, y, and total velocity.
        - ax: The mpl axis where the trajectory should be plotted.
        - color: If set, the trajectory will have a solid color instead of being
          colored according to the velocity.
        - alpha: Alpha value for the trajectory.

    Returns:
        - The line object of the trajectory
    """

    points = np.array([states[:, 0], states[:, 1]]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    norm = plt.Normalize(1.5, 2.5)

    if color is None:
        lc = LineCollection(segments, cmap="viridis", norm=norm, alpha=alpha)
        lc.set_array(states[:, 2])
    else:
        lc = LineCollection(segments, color=[color], alpha=alpha)

    lc.set_linewidth(2)
    line = ax.add_collection(lc)
    return line
