import pandas as pd
import glob
import numpy as np
import os
import json
import matplotlib.pyplot as plt
from scipy import signal
from scipy.spatial.transform import Rotation as R
from bagpy import bagreader


def eval_data(
    folder,
    selection,
    Names,
    time_delay,
    run_time,
    plot_integrated_error=False,
    R_imu_yaw=None,
    R_imu=None,
    R_we=None,
    Q=None,
    Threshold=None,
    with_std=False,
):
    data = glob.glob(folder + "/**/*.json", recursive=True)

    recalc_mse = True
    use_selection = True
    use_exclude = False

    order = 3
    cutoff = 0.1

    pd_data = []
    nan_idx = []

    datasets = []
    gt = []
    for idx, file in enumerate(data):
        name = os.path.dirname(os.path.dirname(file))
        nan_idx = []
        if use_exclude:
            if name.split("/")[-1] in exclude:
                print("Skipping", name.split("/")[-1])
                continue

        if use_selection:
            if name.split("/")[-1] not in selection:
                # print("Skipping",  name.split("/")[-1])
                continue
            else:
                sort_idx = selection.index(name.split("/")[-1])

        print(name.split("/")[-1])
        errors = json.load(open(file, "r"))
        pred = pd.read_csv(
            os.path.join(
                os.path.dirname(os.path.dirname(file)), "data", "estimated.csv"
            )
        )
        error_clean = {"name": name.split("/")[-1]}
        error_std = {"name": name.split("/")[-1]}

        pd_data.append(error_clean)
        gt_data = pd.read_csv(
            os.path.join(os.path.dirname(os.path.dirname(file)), "data", "gt.csv")
        )

        # Filter yaw rate for gt
        b, a = signal.butter(order, cutoff, analog=False)
        dyaw_filt = signal.filtfilt(b, a, gt_data["dyaw"])
        gt_data["dyaw"] = dyaw_filt

        pred = pred[pred.Time >= pred.Time[0] + time_delay].reset_index()
        gt_data = gt_data[gt_data.Time >= gt_data.Time[0] + time_delay].reset_index()
        if run_time != -1:
            pred = pred[pred.Time <= pred.Time[0] + run_time]
            gt_data = gt_data[gt_data.Time <= gt_data.Time[0] + run_time]

        # Remove Nans
        gt_nans = list(set(np.argwhere(np.isnan(((gt_data.to_numpy()))))[:, 0]))
        for i in range(len(gt_nans)):
            nan_idx.append(gt_nans[i])

        pred_nans = list(set(np.argwhere(np.isnan(((pred.to_numpy()))))[:, 0]))
        for i in range(len(pred_nans)):
            nan_idx.append(pred_nans[i])

        # Remove duplicates
        nan_idx = list(set(nan_idx))

        gt_data = gt_data.drop(nan_idx)
        pred = pred.drop(nan_idx)
        if len(nan_idx) != 0:
            print("Dropped rows ", nan_idx, " due to nans")

        if recalc_mse:
            samples = np.inf
            sum_1 = 0
            sum_2 = 0
            for field, vals in errors.items():
                data = pred[field]
                mask = ~np.isnan(data)
                rmse = round(
                    np.sqrt(
                        np.mean(((data[mask] - gt_data[field][mask]).to_numpy()) ** 2)
                    )
                    * 1e3,
                    2,
                )
                std = round(np.std(data[mask] - gt_data[field][mask]), 2)
                if field in ["x", "y", "yaw", "vx_w", "vy_w", "dyaw"]:
                    if with_std:
                        error_clean[field] = f"{rmse} +/- {std}"
                    else:
                        error_clean[field] = rmse
                    sum_1 += rmse
                    if field == "dyaw":
                        continue
                    else:
                        sum_2 += rmse
                else:
                    error_clean[field] = rmse
                samples = np.min([samples, mask.sum()])

            error_clean["Sum"] = sum_1
            error_clean["Sum w/o Rate"] = sum_2
            error_clean["sort index"] = sort_idx
            error_clean["samples"] = samples
            error_clean["start_ts"] = pred["Time"].min()
            error_clean["duration"] = pred["Time"].max() - pred["Time"].min()

        else:
            for field, vals in errors.items():
                error_clean[field] = vals["rmse"]

        datasets.append({"name": name, "data": pred})
        gt.append({"name": name, "data": gt_data})

    df = pd.DataFrame.from_records(pd_data).round(2).sort_values("sort index")
    if R_imu is not None:
        df["R_imu"] = R_imu
    if R_imu_yaw is not None:
        df["R_imu_yaw"] = R_imu_yaw
    if R_we is not None:
        df["R_we"] = R_we
    if Q is not None:
        df["Q"] = Q
    if Threshold is not None:
        df["Threshold"] = Threshold

    df.name = Names

    return df.style.apply(
        lambda col: ["font-weight:bold" if x == col.min() else "" for x in col]
    ).format(precision=2)


def plot_interagted_error(folder, selection, time_delay, run_time):
    data = glob.glob(folder + "/**/*.json", recursive=True)

    recalc_mse = True
    use_selection = True
    use_exclude = False

    order = 3
    cutoff = 0.1

    pd_data = []
    nan_idx = []

    datasets = []
    gt = []
    for idx, file in enumerate(data):
        name = os.path.dirname(os.path.dirname(file))
        nan_idx = []
        if use_exclude:
            if name.split("/")[-1] in exclude:
                print("Skipping", name.split("/")[-1])
                continue

        if use_selection:
            if name.split("/")[-1] not in selection:
                # print("Skipping",  name.split("/")[-1])
                continue
            else:
                sort_idx = selection.index(name.split("/")[-1])

        print(name.split("/")[-1])
        errors = json.load(open(file, "r"))
        pred = pd.read_csv(
            os.path.join(
                os.path.dirname(os.path.dirname(file)), "data", "estimated.csv"
            )
        )
        error_clean = {"name": name.split("/")[-1]}
        error_std = {"name": name.split("/")[-1]}

        pd_data.append(error_clean)
        gt_data = pd.read_csv(
            os.path.join(os.path.dirname(os.path.dirname(file)), "data", "gt.csv")
        )

        # Filter yaw rate for gt
        b, a = signal.butter(order, cutoff, analog=False)
        dyaw_filt = signal.filtfilt(b, a, gt_data["dyaw"])
        gt_data["dyaw"] = dyaw_filt

        pred = pred[pred.Time >= pred.Time[0] + time_delay].reset_index()
        gt_data = gt_data[gt_data.Time >= gt_data.Time[0] + time_delay].reset_index()
        if run_time != -1:
            pred = pred[pred.Time <= pred.Time[0] + run_time]
            gt_data = gt_data[gt_data.Time <= gt_data.Time[0] + run_time]

        # Remove Nans
        gt_nans = list(set(np.argwhere(np.isnan(((gt_data.to_numpy()))))[:, 0]))
        for i in range(len(gt_nans)):
            nan_idx.append(gt_nans[i])

        pred_nans = list(set(np.argwhere(np.isnan(((pred.to_numpy()))))[:, 0]))
        for i in range(len(pred_nans)):
            nan_idx.append(pred_nans[i])

        # Remove duplicates
        nan_idx = list(set(nan_idx))

        gt_data = gt_data.drop(nan_idx)
        pred = pred.drop(nan_idx)
        if len(nan_idx) != 0:
            print("Dropped rows ", nan_idx, " due to nans")

        datasets.append({"name": name, "data": pred})
        gt.append({"name": name, "data": gt_data})

    n_fields = len(gt[0]["data"].columns[1:])
    plt_idx = 1
    for field in gt[0]["data"].columns[1:]:
        plt.subplot(n_fields, 2, plt_idx)
        plt.title(field)
        for idx, (data, gt_data) in enumerate(zip(datasets, gt)):
            df = data["data"]
            gt_df = gt_data["data"]
            if idx == 0:
                plt.plot(
                    gt_df["Time"] - gt_df["Time"][0], gt_df[field], "--", label="GT"
                )
            plt.plot(
                df["Time"] - gt_df["Time"][0],
                df[field],
                "--",
                label=data["name"] + "_" + str(idx),
            )
        plt.legend()
        plt.grid()
        plt.subplot(n_fields, 2, plt_idx + 1)
        plt.title(field + "   rmse Integral")
        for idx, (data, gt_data) in enumerate(zip(datasets, gt)):
            df = data["data"]
            gt_df = gt_data["data"]
            plt.plot(
                df["Time"] - gt_df["Time"][0],
                (gt_df[field] - df[field]).abs(),
                label=data["name"] + "_" + str(idx),
            )
            plt.fill_between(
                df["Time"] - gt_df["Time"][0],
                (gt_df[field] - df[field]).abs().cumsum(),
                alpha=0.4,
            )

        plt.legend()
        plt.grid()
        plt_idx += 2
    plt.gcf().set_size_inches(50, 20)
    # plt.savefig("/home/sabodmer/crs-2.0/src/scripts/estimator_comparison/compare.png")
    plt.show()
