import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from evaluate_rosbag import resample
import os
import os.path
import yaml
import csv
import glob


# Make sure the track.yaml file is available. It can be found in /code/src/ros4crs/tools/track_generation/tracks.
def plot_track(
    plot_name: str,
    x_data,
    y_data,
    use_tunnel=False,
    start_tunnel=-300,
    end_tunnel=-1,
    use_color_gradient=False,
    var_for_color=0,
    max=0,
    min=0,
):

    center_line_x = None
    center_line_y = None
    rows_to_write = []

    # Get track centerline coordinates from .yaml
    yaml_file_names = glob.glob("./*.yaml")
    for i, each_yaml_file in enumerate(yaml_file_names):
        print(
            "Processing file {} of {} file name: {}".format(
                i + 1, len(yaml_file_names), each_yaml_file
            )
        )

        with open(each_yaml_file) as file:
            track_data = yaml.safe_load(file)
            center_line_x = np.array(track_data["track"]["xCoords"])
            center_line_y = np.array(track_data["track"]["yCoords"])

    # Compute Track
    track_width = track_data["track"]["width_of_track"]
    xRate = np.array(track_data["track"]["xRate"])
    yRate = np.array(track_data["track"]["yRate"])
    bounds = np.stack([center_line_x, center_line_y], -1)

    lower_bound_x = center_line_x + yRate * track_width / 2
    lower_bound_y = center_line_y - xRate * track_width / 2

    upper_bound_x = center_line_x - yRate * track_width / 2
    upper_bound_y = center_line_y + xRate * track_width / 2

    # Plot
    fig, ax = plt.subplots(1, 2)
    fig.set_figheight(6.66)
    fig.set_figwidth(20)

    if use_color_gradient:
        c = np.clip((var_for_color - min) / (max - min), 0, 1)
        ax[0].scatter(x_data, y_data, c=c, s=2, cmap="viridis")
        ax[1].plot(x_data, y_data)
    else:
        plt.scatter(x_data, y_data, s=2, cmap="viridis")
    ax[0].plot(lower_bound_x, lower_bound_y, "k")
    ax[0].plot(upper_bound_x, upper_bound_y, "k")
    ax[1].plot(lower_bound_x, lower_bound_y, "k")
    ax[1].plot(upper_bound_x, upper_bound_y, "k")

    # Labels
    ax[0].set_xlabel("x_p [m]")
    ax[0].set_ylabel("y_p [m]")
    ax[0].title.set_text(plot_name)

    # Labels
    ax[1].set_xlabel("x")
    ax[1].set_ylabel("y")
    ax[1].title.set_text(plot_name)

    # Tunnel
    if use_tunnel:
        plt.fill_between(
            center_line_x[start_tunnel:end_tunnel],
            center_line_y[start_tunnel:end_tunnel] + track_width / 2,
            center_line_y[start_tunnel:end_tunnel] - track_width / 2,
            where=(upper_bound_y[start_tunnel:end_tunnel] < -0.8),
            color="gray",
            alpha=0.6,
        )

    ax[0].plot(center_line_x, center_line_y, "tab:red", alpha=0.8)
    ax[1].plot(center_line_x, center_line_y, "tab:red", alpha=0.8)
    name = plot_name + ".png"
    plt.savefig(name, format="png")
    print("Saved to", name)
