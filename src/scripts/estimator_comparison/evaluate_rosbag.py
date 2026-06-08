import numpy as np
import bagpy
from bagpy import bagreader
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib as mpl


def load_csv_from_bag(bagfile, namespace, is_sim):
    b = bagreader(bagfile)
    print(b.topic_table)

    # Extract messages for each topic
    if not is_sim:
        estimate_msg = sim_b.message_by_topic(
            "/" + namespace + "/estimation_node/best_state"
        )
        qualisys_vel_msg = b.message_by_topic(
            "/" + namespace + "/qualisys/PAPER/velocity"
        )
        qualisys_pos_msg = b.message_by_topic("/" + namespace + "/qualisys/PAPER/pose")
        qualisys_odom_msg = b.message_by_topic("/" + namespace + "/qualisys/PAPER/odom")
        qualisys_vel_demo_msg = b.message_by_topic(
            "/" + namespace + "/qualisys/DEMO_TRACK/velocity"
        )
        qualisys_pos_demo_msg = b.message_by_topic(
            "/" + namespace + "/qualisys/DEMO_TRACK/pose"
        )
        qualisys_odom_demo_msg = b.message_by_topic(
            namespace + "/qualisys/DEMO_TRACK/odom"
        )
        imu_msg = b.message_by_topic("/" + namespace + "/imu")
        lighthouse_msg = b.message_by_topic("/" + namespace + "/lighthouse")
        wheel_encoders_msg = b.message_by_topic("/" + namespace + "/wheel_encoders")
        tf_msg = b.message_by_topic("/tf")
        input_msg = b.message_by_topic("/" + namespace + "/input")

    if is_sim:
        estimate_msg = sim_b.message_by_topic(
            "/" + namespace + "/estimation_node/best_state"
        )
        gt_msg = sim_b.message_by_topic("/" + namespace + "/ros_simulator/gt_state")
        imu_msg = b.message_by_topic("/" + namespace + "/imu")
        wheel_encoders_msg = b.message_by_topic("/" + namespace + "/wheel_encoders")
        qualisys_vel_demo_msg = b.message_by_topic(
            "/" + namespace + "/qualisys/DEMO_TRACK/velocity"
        )
        qualisys_pos_demo_msg = b.message_by_topic(
            "/" + namespace + "/qualisys/DEMO_TRACK/pose"
        )
        qualisys_odom_demo_msg = b.message_by_topic(
            "/" + namespace + "/qualisys/DEMO_TRACK/odom"
        )
        qualisys_vel_msg = b.message_by_topic("/qualisys/" + namespace + "/velocity")
        qualisys_pos_msg = b.message_by_topic("/qualisys/" + namespace + "/pose")
        qualisys_odom_msg = b.message_by_topic("/qualisys/" + namespace + "/odom")
        tf_msg = b.message_by_topic("/tf")
        input_msg = b.message_by_topic("/" + namespace + "/input")

    # Extract from CSV
    if not is_sim:
        df_estimation = pd.read_csv(estimate_msg)
        df_qualisys_vel = pd.read_csv(qualisys_vel_msg)
        df_qualisys_pos = pd.read_csv(qualisys_pos_msg)
        df_qualisys_odom = pd.read_csv(qualisys_odom_msg)
        df_qualisys_demo_vel = pd.read_csv(qualisys_vel_demo_msg)
        df_qualisys_demo_pos = pd.read_csv(qualisys_pos_demo_msg)
        df_qualisys_demo_odom = pd.read_csv(qualisys_odom_demo_msg)
        df_imu = pd.read_csv(imu_msg)
        df_lighthouse = pd.read_csv(lighthouse_msg)
        df_wheel_encoders = pd.read_csv(wheel_encoders_msg)
        df_tf = pd.read_csv(tf_msg)
        df_input = pd.read_csv(input_msg)

    if is_sim:
        df_gt = pd.read_csv(gt_msg)
        df_estimation = pd.read_csv(estimate_msg)
        df_qualisys_demo_vel = pd.read_csv(qualisys_vel_demo_msg)
        df_qualisys_demo_pos = pd.read_csv(qualisys_pos_demo_msg)
        df_qualisys_demo_odom = pd.read_csv(qualisys_odom_demo_msg)
        df_qualisys_vel = pd.read_csv(qualisys_vel_msg)
        df_qualisys_pos = pd.read_csv(qualisys_pos_msg)
        df_qualisys_odom = pd.read_csv(qualisys_odom_msg)
        df_lighthouse = pd.read_csv(lighthouse_msg)
        df_wheel_encoders = pd.read_csv(wheel_encoders_msg)
        df_tf = pd.read_csv(tf_msg)
        df_input = pd.read_csv(input_msg)


def plot_state_data(data_to_compare, data_names):
    sty = "seaborn-v0_8-paper"
    sty = "seaborn-v0_8-deep"
    mpl.style.use(sty)

    fig, axs = plt.subplots(3, 2, figsize=(10, 8))
    plt_idx_1 = 1
    plt_idx_2 = 0

    data = data_to_compare[0]
    time = data["Time"]
    # start_time = time.iloc[0]
    final_time = time.iloc[-1]
    min_final_time = final_time

    for idx, data in enumerate(data_to_compare):
        time = data["Time"]
        start_time = time.iloc[0]
        final_time = time.iloc[-1]
        if final_time <= min_final_time:
            min_final_time = time.iloc[-1]

        # ---------- Plot Position ----------
        major_ticks_x = np.arange(-1.7, 1.7, 0.5)
        major_ticks_y = np.arange(-1.3, 1.3, 0.5)
        minor_ticks_x = np.arange(-1.7, 1.7, 0.1)
        minor_ticks_y = np.arange(-1.3, 1.3, 0.1)
        axs[0, 0].set_xticks(major_ticks_x)
        axs[0, 0].set_xticks(minor_ticks_x, minor=True)
        axs[0, 0].set_yticks(major_ticks_y)
        axs[0, 0].set_yticks(minor_ticks_y, minor=True)

        # And a corresponding grid
        axs[0, 0].grid(which="both")
        axs[0, 0].grid(which="minor", alpha=0.2)
        axs[0, 0].grid(which="major", alpha=0.6, linestyle="--")

        plt_pos = axs[0, 0].plot(data["x"], data["y"])
        legend = axs[0, 0].legend(data_names, loc="upper right")
        axs[0, 0].set_xlabel("x positon")
        axs[0, 0].set_ylabel("y position")
        axs[0, 0].title.set_text("Position")

        # ---------- Plot yaw ----------
        # time = data['Time']
        # start_time = time.iloc[0]
        # final_time = time.iloc[-1]
        major_ticks_x = np.arange(start_time, min_final_time, 10)
        major_ticks_y = np.arange(-4, 4, 1)
        minor_ticks_x = np.arange(start_time, min_final_time, 5)
        minor_ticks_y = np.arange(-4, 4, 0.5)
        axs[1, 0].set_xticks(major_ticks_x)
        axs[1, 0].set_xticks(minor_ticks_x, minor=True)
        axs[1, 0].set_yticks(major_ticks_y)
        axs[1, 0].set_yticks(minor_ticks_y, minor=True)

        # And a corresponding grid
        axs[1, 0].grid(which="both")
        axs[1, 0].grid(which="minor", alpha=0.2)
        axs[1, 0].grid(which="major", alpha=0.6, linestyle="--")

        # Wrap angle
        yaw_wrapped = (data["yaw"] + np.pi) % (2 * np.pi) - np.pi
        plt_yaw = axs[1, 0].plot(data["Time"], yaw_wrapped)
        legend = axs[1, 0].legend(data_names, loc="upper right")
        axs[1, 0].set_xlabel("Time")
        axs[1, 0].set_ylabel("yaw")
        axs[1, 0].title.set_text("Yaw")

        # ---------- Plot Velocity x ----------
        # time = data['Time']
        # start_time = time.iloc[0]
        # final_time = time.iloc[-1]
        major_ticks_x = np.arange(start_time, final_time, 10)
        major_ticks_y = np.arange(-1.3, 1.3, 0.5)
        minor_ticks_x = np.arange(start_time, final_time, 5)
        minor_ticks_y = np.arange(-1.3, 1.3, 0.1)
        axs[2, 0].set_xticks(major_ticks_x)
        axs[2, 0].set_xticks(minor_ticks_x, minor=True)
        axs[2, 0].set_yticks(major_ticks_y)
        axs[2, 0].set_yticks(minor_ticks_y, minor=True)

        # And a corresponding grid
        axs[2, 0].grid(which="both")
        axs[2, 0].grid(which="minor", alpha=0.2)
        axs[2, 0].grid(which="major", alpha=0.6, linestyle="--")

        plt_vel_x = axs[2, 0].plot(data["Time"], data["vx_w"])
        legend = axs[2, 0].legend(data_names, loc="upper right")
        axs[2, 0].set_xlabel("Time")
        axs[2, 0].set_ylabel("vel x")
        axs[2, 0].title.set_text("Velocity x")

        # ---------- Plot Velocity y ----------
        # time = data['Time']
        # start_time = time.iloc[0]
        # final_time = time.iloc[-1]
        major_ticks_x = np.arange(start_time, final_time, 10)
        major_ticks_y = np.arange(-1.3, 1.5, 0.5)
        minor_ticks_x = np.arange(start_time, final_time, 5)
        minor_ticks_y = np.arange(-1.3, 1.5, 0.1)
        axs[0, 1].set_xticks(major_ticks_x)
        axs[0, 1].set_xticks(minor_ticks_x, minor=True)
        axs[0, 1].set_yticks(major_ticks_y)
        axs[0, 1].set_yticks(minor_ticks_y, minor=True)

        # And a corresponding grid
        axs[0, 1].grid(which="both")
        axs[0, 1].grid(which="minor", alpha=0.2)
        axs[0, 1].grid(which="major", alpha=0.6, linestyle="--")

        plt_vel_y = axs[0, 1].plot(data["Time"], data["vy_w"])
        legend = axs[0, 1].legend(data_names, loc="upper right")
        axs[0, 1].set_xlabel("Time")
        axs[0, 1].set_ylabel("vel y")
        axs[0, 1].title.set_text("Velocity y")

        # ---------- Plot dyaw ----------
        # time = data['Time']
        # start_time = time.iloc[0]
        # final_time = time.iloc[-1]
        major_ticks_x = np.arange(start_time, final_time, 10)
        major_ticks_y = np.arange(-5, 4, 1)
        minor_ticks_x = np.arange(start_time, final_time, 5)
        minor_ticks_y = np.arange(-5, 4, 0.5)
        axs[1, 1].set_xticks(major_ticks_x)
        axs[1, 1].set_xticks(minor_ticks_x, minor=True)
        axs[1, 1].set_yticks(major_ticks_y)
        axs[1, 1].set_yticks(minor_ticks_y, minor=True)

        # And a corresponding grid
        axs[1, 1].grid(which="both")
        axs[1, 1].grid(which="minor", alpha=0.2)
        axs[1, 1].grid(which="major", alpha=0.6, linestyle="--")

        plt_dyaw = axs[1, 1].plot(data["Time"], data["dyaw"])
        legend = axs[1, 1].legend(data_names, loc="upper right")
        axs[1, 1].set_xlabel("Time")
        axs[1, 1].set_ylabel("dyaw")
        axs[1, 1].title.set_text("Yaw Rate")

    fig.tight_layout()
    plt.show()


def compute_rmse(data_1, data_2, title):
    x_mse = np.sqrt(np.mean((data_1["x"] - data_2["x"]) ** 2))
    y_mse = np.sqrt(np.mean((data_1["y"] - data_2["y"]) ** 2))
    yaw_mse = np.sqrt(np.mean((data_1["yaw"] - data_2["yaw"]) ** 2))
    vx_mse = np.sqrt(np.mean((data_1["vx_w"] - data_2["vx_w"]) ** 2))
    vy_mse = np.sqrt(np.mean((data_1["vy_w"] - data_2["vy_w"]) ** 2))
    dyaw_mse = np.sqrt(np.mean((data_1["dyaw"] - data_2["dyaw"]) ** 2))

    print(title)
    print("---------------------------")
    print("pos x: ", x_mse)
    print("pos y: ", y_mse)
    print("yaw:   ", yaw_mse)
    print("vel x: ", vx_mse)
    print("vel y: ", vy_mse)
    print("dyaw:  ", dyaw_mse)


def resample(df, Ts, start_ts: float, end_ts: float = None, method="nearest"):
    """Simple function to resample a dataframe with a given timesteps/frequency.

    Args:
        df (pd.DataFrame): Dataframe to resample. Must have a column "Time", all other columns are resampled and interpolated
        Ts (float|np.ndarray): Timestep or array of timesteps to resample to.
        start_ts (float): Start time of the resampling
        end_ts (float, optional): End time of the resampling. If None, the end time of the dataframe is used. Defaults to None.
        method (str, optional): Interpolation method. "nearest" or "linear". Defaults to "nearest".
    """

    time = df.Time.to_numpy()
    data = df.iloc[:, 1:].to_numpy()
    if end_ts is None:
        end_ts = time.max()

    valid = time >= start_ts
    valid = valid * (time < end_ts)

    data = data[valid, :]
    time = time[valid]
    subsampled = []
    subsampled_ts = []
    if isinstance(Ts, float):
        num_steps = np.ceil((end_ts - start_ts) / Ts) + 1
        timestamps = np.arange(num_steps) * Ts + start_ts
    else:
        timestamps = Ts
        Ts = np.mean(np.diff(Ts))

    timestamp_idx = 0
    start_ts = timestamps[timestamp_idx]

    while start_ts < end_ts:

        if method == "nearest":
            closest_idx = np.argmin(np.abs(time - start_ts))
            closest = time[closest_idx]
            if np.abs(closest - start_ts) < Ts:
                subsampled.append(data[closest_idx, :])
            else:
                # print(f"Could not Interpolate for Time:  {start_ts} closest timestep too far away ({np.abs(closest - start_ts):.4f}s)")
                subsampled.append(data[closest_idx, :] * np.NaN)
        elif method == "linear":
            closest_idxs = np.argsort(np.abs(time - start_ts))[:2]
            closest_idxs = np.sort(closest_idxs)
            closest_pts = time[closest_idxs]
            t0 = closest_pts[0]
            t1 = closest_pts[1]
            y0 = data[closest_idxs[0], :]
            y1 = data[closest_idxs[1], :]

            # bilinear interpolate
            value = y0 + (start_ts - t0) * (y1 - y0) / (t1 - t0)

            if np.abs(closest_pts - start_ts).max() < 1 * Ts:
                subsampled.append(value)
            else:
                print(
                    f"Could not Interpolate for Time:  {start_ts} closest timestep too far away ({np.abs(closest_pts - start_ts).max():.4f}s)"
                )
                subsampled.append(data[closest_idxs[0], :] * np.NaN)
        subsampled_ts.append(start_ts)

        timestamp_idx += 1
        if timestamp_idx >= len(timestamps):
            break
        start_ts = timestamps[timestamp_idx]

    data_for_dataframe = {"Time": time}
    for idx, c in enumerate(df.columns):
        if c == "Time":
            data_for_dataframe["Time"] = np.array(subsampled_ts)
        else:
            data_for_dataframe[c] = np.array(subsampled)[:, idx - 1]

    # make to dataset again
    return pd.DataFrame(data_for_dataframe)
