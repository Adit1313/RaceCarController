import numpy as np
import bagpy
from bagpy import bagreader
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib as mpl
from evaluate_rosbag import resample
from scipy.spatial.transform import Rotation as R
import json
import os
import argparse
import shutil


def _get_valid_timestamps(dataframes):
    min_ts = -np.inf
    max_ts = np.inf
    for df in dataframes:
        min_ts = max(min_ts, df.Time.min())
        max_ts = min(max_ts, df.Time.max())
    return min_ts, max_ts


def _cut_dataframes(dataframes, min_ts, max_ts):
    for df in dataframes:
        valid = (df.Time >= min_ts) & (df.Time <= max_ts)
        df = df[valid]
    return dataframes


def _clean_dataframes(dataframes):
    # Drop all columns which are note numeric
    data = []
    for df in dataframes:
        df = df.select_dtypes(include=[np.number])
        data.append(df)
    return data


def _resample_dataframes(dataframes, ts, min_ts, max_ts, method):
    # resample(dataframes[0], ts, min_ts, max_ts, method)
    # import pdb; pdb.set_trace()
    return [resample(df, ts, min_ts, max_ts, method) for df in dataframes]


def _yaw_from_quat(x):
    return (
        R.from_quat(x).as_euler("XYZ")[-1]
        if not np.isnan(x).all() and np.linalg.norm(x) != 0
        else np.NaN
    )


def apply_se2_transform(dataframe, position, rotation, columns=[["x", "y"]]):
    for cols in columns:
        df = dataframe[cols]
        if rotation is not None:
            df = df.dot(rotation)
        if position is not None:
            df = df + position
        dataframe[cols] = df
    return dataframe


def _compare_and_plot_series(
    gt_df, pred_df, fields=[], n_cols=2, out_file=None, shade_diff=True, plot=False
):
    n_plts = len(fields)
    n_rows = n_plts // n_cols + 1
    fig, axs = plt.subplots(n_rows, n_cols)
    # gs = fig.add_gridspec(n_rows,n_cols)

    # plt.plot(gt_df["x"], gt_df["y"], "-", label="Ground truth")
    # plt.plot(pred_df["x"], pred_df["y"], "-", label="Prediction")
    # # Create grid
    # axs[n_rows - 1, 0] = fig.add_subplot(gs[n_rows - 1, :])
    # axs[n_rows - 1, 0].minorticks_on()
    # axs[n_rows - 1, 0].grid(which='major', linestyle='-', alpha=0.6)
    # axs[n_rows - 1, 0].grid(which='minor', linestyle='-', alpha=0.2)
    # axs[n_rows - 1, 0].plot(gt_df["x"], gt_df["y"], "-", label="Ground truth")
    # axs[n_rows - 1, 0].plot(pred_df["x"], pred_df["y"], "-", label="Prediction")
    # for idx, field in enumerate(fields):
    idx = 0
    for row in range(n_rows - 1):
        for col in range(n_cols):
            field = fields[idx]
            idx += 1
            gt_field = gt_df[field]
            pred_field = pred_df[field]
            # plt.subplot(n_rows, n_cols, idx + 1)

            # Create grid
            axs[row, col].minorticks_on()
            axs[row, col].grid(which="major", linestyle="-", alpha=0.6)
            axs[row, col].grid(which="minor", linestyle="-", alpha=0.2)

            axs[row, col].plot(
                gt_df["Time"] - gt_df["Time"][0], gt_field, "-", label="Ground truth"
            )
            axs[row, col].plot(
                pred_df["Time"] - gt_df["Time"][0], pred_field, "-", label="Prediction"
            )

            if shade_diff:
                axs[row, col].fill_between(
                    gt_df["Time"] - gt_df["Time"][0],
                    gt_field,
                    pred_field,
                    where=(gt_field > pred_field),
                    color="red",
                    alpha=0.3,
                )
                axs[row, col].fill_between(
                    gt_df["Time"] - gt_df["Time"][0],
                    gt_field,
                    pred_field,
                    where=(gt_field < pred_field),
                    color="green",
                    alpha=0.3,
                )
            # plt.grid()
            axs[row, col].title.set_text(field)
            axs[row, col].legend()

    plt.gcf().set_size_inches(n_rows * 10, n_cols * 10)
    if out_file is not None:
        plt.savefig(out_file)
    if plot:
        plt.show(block=False)


def _plt_2d(gt_df, pred_df, out_file=None, plot=False):
    fig, axs = plt.subplots()
    # Create grid
    axs.minorticks_on()
    axs.grid(which="major", linestyle="-", alpha=0.6)
    axs.grid(which="minor", linestyle="-", alpha=0.2)

    plt.plot(gt_df["x"], gt_df["y"], "-", label="Ground truth")
    plt.plot(pred_df["x"], pred_df["y"], "-", label="Prediction")

    plt.title("Trajectory")
    axs.set_xlabel("x positon")
    axs.set_ylabel("y position")
    plt.legend()
    plt.gcf().set_size_inches(10, 10)
    plt.tight_layout()
    if out_file is not None:
        plt.savefig(out_file)
    if plot:
        plt.show(block=False)


def _calc_error(gt_df, pred_df, fields=[], out_file=None):
    data = {}
    for field in fields:
        error = gt_df[field] - pred_df[field]
        mse = np.mean(error**2)
        rmse = np.sqrt(mse)
        l1_error = np.mean(np.abs(error))
        bias = np.mean(error)
        std = np.std(error)
        data[field] = {
            "mse": mse,
            "rmse": rmse,
            "l1_error": l1_error,
            "bias": bias,
            "std": std,
        }

        # pretty print
        # print(f"Error for {field}:")
        # print(f"\t MSE: {mse:.4f}")
        # print(f"\t RMSE: {rmse:.4f}")
        # print(f"\t L1 Error: {l1_error:.4f}")
        # print(f"\t Bias: {bias:.4f}")
        # print(f"\t Std: {std:.4f}")
        print("Done Processing run. Used bag:", args.bag)
        print("Saved results to:", args.output_folder)
    # save data to json file
    if out_file is not None:
        with open(out_file, "w") as f:
            json.dump(data, f)


def main(args):
    print("Comparing rosbag", args.bag)

    #######################################
    # Subsampling and cleaning the data
    #######################################

    # open bag
    bag = bagreader(args.bag)

    # output folder
    plots_folder = os.path.join(args.output_folder, "plots")
    results_folder = os.path.join(args.output_folder, "results")
    data_folder = os.path.join(args.output_folder, "data")

    # qs_path = bag.message_by_topic("/qualisys/PAPER/pose")

    # qs_pose_df = pd.read_csv(qs_path)
    qs_pose_df = pd.read_csv(bag.message_by_topic("/qualisys/PAPER/pose"))
    qs_vel_df = pd.read_csv(bag.message_by_topic("/qualisys/PAPER/velocity"))
    lh_df = pd.read_csv(bag.message_by_topic("/car1/lighthouse"))

    gt_df = qs_pose_df.join(qs_vel_df, rsuffix="_vel")
    # dorp nans
    gt_df = gt_df.dropna()

    dataframes = [gt_df, lh_df]

    first_estimator_idx = None
    ## data to be compared
    if "/car1/estimation_node/best_state" in bag.topics:
        print("Found estimation node data")
        dataframes.append(
            pd.read_csv(bag.message_by_topic("/car1/estimation_node/best_state"))
        )
        first_estimator_idx = len(dataframes) - 1
    else:
        print(
            "[WARNING] No estimation node data found in the rosbag. Skipping comparison."
        )

    second_estimator_idx = None
    if "/car1/second_estimation_node/best_state" in bag.topics:
        print("Found second estimation node data")
        dataframes.append(
            pd.read_csv(bag.message_by_topic("/car1/second_estimation_node/best_state"))
        )
        second_estimator_idx = len(dataframes) - 1
    else:
        print(
            "[WARNING] No second estimation node data found in the rosbag. Skipping comparison."
        )

    # define dataframes to be compared. First one is the reference for timestamps if no frequency is given
    min_ts, max_ts = _get_valid_timestamps(dataframes)
    print(f"Found data spanning {(max_ts - min_ts):.2f} seconds in the rosbag.")
    # dataframes = _cut_dataframes(dataframes, min_ts, max_ts)
    dataframes = _clean_dataframes(dataframes)
    ss_freq = args.ss

    if ss_freq == -1:
        print("Resampling to valid qualisys timestamps")
        ts = dataframes[0].Time.to_numpy()
        ts = ts[ts > min_ts]

    else:
        print(f"Resampling to {ss_freq} Hz")
        ts = 1 / ss_freq

    dataframes = _resample_dataframes(dataframes, ts, min_ts, max_ts, "nearest")
    print("Length of resampled dataframes:", [len(df) for df in dataframes])

    gt_df = dataframes[0]

    # only extract important columns
    # create a new column in gt

    gt_df["yaw"] = gt_df[
        [
            "pose.orientation.x",
            "pose.orientation.y",
            "pose.orientation.z",
            "pose.orientation.w",
        ]
    ].apply(_yaw_from_quat, axis=1)

    gt_df = gt_df[
        [
            "Time",
            "pose.position.x",
            "pose.position.y",
            "yaw",
            "twist.linear.x",
            "twist.linear.y",
            "twist.angular.z",
        ]
    ]
    # rename columns
    gt_df.columns = ["Time", "x", "y", "yaw", "vx_w", "vy_w", "dyaw"]
    gt_df["yaw"] = np.unwrap(gt_df["yaw"])

    #######################################
    #  Plotting the data
    #######################################

    sty = "tableau-colorblind10"
    mpl.style.use(sty)

    if first_estimator_idx is not None:
        df = dataframes[first_estimator_idx]
        df = df[["Time", "x", "y", "yaw", "vx_b", "vy_b", "dyaw"]].copy()
        # create new columns
        yaw = df["yaw"].to_numpy()
        df["vx_w"] = df["vx_b"] * np.cos(yaw) - df["vy_b"] * np.sin(yaw)
        df["vy_w"] = df["vx_b"] * np.sin(yaw) + df["vy_b"] * np.cos(yaw)

        # Remove first 6s of data
        # starting_bag_at_ts = 6
        # gt_df_ = gt_df[gt_df['Time'] >= gt_df['Time'].min() + starting_bag_at_ts]
        # df_ = df[df['Time'] >= df['Time'].min() + starting_bag_at_ts]

        _calc_error(
            gt_df,
            df,
            ["x", "y", "yaw", "vx_w", "vy_w", "dyaw"],
            out_file=os.path.join(results_folder, "error.json"),
        )
        _plt_2d(
            gt_df,
            df,
            out_file=os.path.join(plots_folder, "trajectory.png"),
            plot=args.plot,
        )
        _compare_and_plot_series(
            gt_df,
            df,
            ["x", "y", "yaw", "vx_w", "vy_w", "dyaw"],
            out_file=os.path.join(plots_folder, "comparison.png"),
            plot=args.plot,
        )
        # save df to file
        df.to_csv(os.path.join(data_folder, "estimated.csv"), index=False)
        gt_df.to_csv(os.path.join(data_folder, "gt.csv"), index=False)

    if second_estimator_idx is not None:
        df = dataframes[second_estimator_idx]
        df = df[["Time", "x", "y", "yaw", "vx_b", "vy_b", "dyaw"]].copy()
        # create new columns
        yaw = df["yaw"].to_numpy()
        df["vx_w"] = df["vx_b"] * np.cos(yaw) - df["vy_b"] * np.sin(yaw)
        df["vy_w"] = df["vx_b"] * np.sin(yaw) + df["vy_b"] * np.cos(yaw)
        _calc_error(
            gt_df,
            df,
            ["x", "y", "yaw", "vx_w", "vy_w", "dyaw"],
            out_file=os.path.join(results_folder, "2nd_est_error.json"),
        )
        _plt_2d(
            gt_df,
            df,
            out_file=os.path.join(plots_folder, "2nd_est_trajectory.png"),
            plot=args.plot,
        )
        _compare_and_plot_series(
            gt_df,
            df,
            ["x", "y", "yaw", "vx_w", "vy_w", "dyaw"],
            out_file=os.path.join(plots_folder, "2nd_est_comparison.png"),
            plot=args.plot,
        )

        # save df to file
        df.to_csv(os.path.join(data_folder, "2nd_estimated.csv"), index=False)
        gt_df.to_csv(os.path.join(data_folder, "gt.csv"), index=False)

    if args.plot:
        plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Compare rosbags")

    parser.add_argument("bag", type=str, help="Path to first rosbag")
    parser.add_argument(
        "experiment_name", type=str, help="Name of the experiment", default="test"
    )

    parser.add_argument(
        "-output_folder",
        type=str,
        help="Output Folder",
        default="/home/sabodmer/crs-2.0/src/scripts/estimator_comparison/data",
    )

    parser.add_argument(
        "-ss",
        type=float,
        help="Resampling step size. Use -1 to resample to valid qualisys timestamps.",
        default=-1,
        required=False,
    )

    parser.add_argument(
        "-force",
        help="Overwrite existing output folder",
        action="store_true",
        required=False,
    )
    parser.add_argument(
        "-plot",
        help="Plot the results",
        action="store_true",
        required=False,
    )
    args = parser.parse_args()

    config_folder_name = "_".join(os.path.basename(args.bag).split("_")[:3])
    # Prepare output folder
    if not os.path.exists(args.bag):
        print(f"Bag {args.bag} does not exist")
        exit(1)

    target_folder = os.path.join(args.output_folder, args.experiment_name)
    if os.path.exists(target_folder) and not args.force:
        print(f"Folder {target_folder} already exists")
        exit(1)
    os.makedirs(target_folder, exist_ok=True)
    os.makedirs(os.path.join(target_folder, "bag"), exist_ok=True)
    print("Copying bag to output folder", os.path.join(target_folder, "run.bag"))
    shutil.copy(args.bag, os.path.join(target_folder, "bag", "run.bag"))
    print("Copying config files to output folder")
    config_path = os.path.join(os.path.dirname(args.bag), config_folder_name)
    if os.path.exists(config_path):
        # remove
        shutil.rmtree(os.path.join(target_folder, "config"), ignore_errors=True)
    shutil.copytree(config_path, os.path.join(target_folder, "config"))
    # create plots folder
    os.makedirs(os.path.join(target_folder, "plots"), exist_ok=True)
    # Create results folder
    os.makedirs(os.path.join(target_folder, "results"), exist_ok=True)
    os.makedirs(os.path.join(target_folder, "data"), exist_ok=True)
    # update args paths
    args.bag = os.path.join(target_folder, "bag", "run.bag")
    args.output_folder = target_folder

    main(args)
