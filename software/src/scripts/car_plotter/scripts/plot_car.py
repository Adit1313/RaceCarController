from argparse import ArgumentParser

import matplotlib.pyplot as plt

from car_plotter import *


def main():
    parser = ArgumentParser(
        prog="car_plotter.py",
        description="Super cool and extremely awesome cli tool which "
        + "plots car trajectories from a bag file in python.",
    )
    parser.add_argument(
        "-f",
        "--file",
        dest="filename",
        help="Filename of the bagfile",
    )
    parser.add_argument(
        "-t",
        "--topic",
        dest="topic",
        default="/SABRINA/estimation_node/best_state",
        help="Full topic name of the car state message",
    )
    parser.add_argument(
        "-l",
        dest="list_topics",
        action="store_true",
        default=False,
        help="If set, information about the rosbag will be printed",
    )
    parser.add_argument(
        "--track",
        dest="track",
        default="DEMO_TRACK.yaml",
        help="Name of the track used (e.g. DEMO_TRACK.yaml)",
    )
    parser.add_argument(
        "-s",
        dest="savefig",
        action="store_true",
        default=False,
        help="Whether to save the figure to disk. "
        "Figure is saved with at the same location and the same "
        "name as the bag file",
    )
    parser.add_argument(
        "-c",
        dest="centerline",
        action="store_true",
        default=False,
        help="Whether to plot the centerline",
    )
    parser.add_argument(
        "-q",
        dest="quiet",
        action="store_true",
        default=False,
        help="Whether plot is shown in the end. Set this flag "
        "if you want to compose this tool with other scripts",
    )

    args = parser.parse_args()
    print(f"Plotting trajectory for bag file {args.filename}")

    states = get_states_from_bag(args.filename, args.topic, args.list_topics)

    fig, ax = plt.subplots(1, 1)
    fig.tight_layout()
    ax.axis("off")
    ax.set_aspect("equal")
    plot_track(ax, args.track, args.centerline)
    line = plot_trajectory(states, ax)
    fig.colorbar(line, label="Velocity [m/s]", shrink=0.75)

    if args.savefig:
        filename = f"{args.filename.split('.')[0]}.pdf"
        fig.savefig(filename)

    if not args.quiet:
        plt.show()
