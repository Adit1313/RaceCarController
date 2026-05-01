import casadi as ca
import hashlib
import numpy as np

from enum import IntEnum
from typing import Tuple


def finite_differences(x, dt, idx=-1):
    if idx == -1:
        if x.ndim == 1:
            return (x[1:] - x[:-1]) / dt
        else:
            return (x[:, 1:] - x[:, :-1]) / dt
    else:
        return (x[idx, 1:] - x[idx, :-1]) / dt


def to_vector(*arrays):
    vectors = []
    for array in arrays:
        if isinstance(array, list):
            vectors += [ca.reshape(subarray, (-1, 1)) for subarray in array]
        elif isinstance(array, np.ndarray):
            vectors += [np.reshape(array, (-1, 1), order="F")]
        else:
            vectors += [ca.reshape(array, (-1, 1))]
    return ca.vertcat(*vectors)


def from_vector(vector: np.array, *shapes):
    arrays = []
    for shape in shapes:
        l = np.prod(shape)
        arrays += [np.array(ca.reshape(vector[:l], shape))]
        vector = vector[l:]

    return arrays


class Vectorizer:
    def to_vector(self, *arrays):
        self.shapes = [array.shape for array in arrays]

        return to_vector(*arrays)

    def from_vector(self, vector: np.array):
        return from_vector(vector, *self.shapes)


def gaussian_vector(var, n: int = 1, N: int = 1, seed: str = ""):
    set_seed(seed)
    return np.random.multivariate_normal(np.zeros(n), np.diag(var), N).T


def uniform_vector(low: float, high: float, shape: Tuple[int], seed: str = ""):
    set_seed(seed)
    return np.random.uniform(low, high, shape)


def bartlett_smoothing(x: np.array, N: int) -> np.array:
    kernel = np.array(list(range(1, N + 2)) + list(range(N, 0, -1)))
    kernel = kernel / np.sum(kernel)

    x = np.hstack([x[0]] * N + [x] + [x[-1]] * N)
    return np.convolve(x, kernel, "valid")


def unwrap_mod2pi(phi):
    non_nans = np.nonzero(~np.isnan(phi))[0]
    for i in range(len(non_nans) - 1):
        while phi[non_nans[i + 1]] - phi[non_nans[i]] > np.pi:
            phi[non_nans[i + 1]] -= 2 * np.pi
        while phi[non_nans[i + 1]] - phi[non_nans[i]] < -np.pi:
            phi[non_nans[i + 1]] += 2 * np.pi

    return phi


def set_seed(seed_str) -> None:
    hash_bytes = hashlib.sha256(seed_str.encode("utf-8")).digest()
    seed = int.from_bytes(hash_bytes[:4], byteorder="big")

    np.random.seed(seed)


def enum2list(enum: IntEnum):
    return [item for item in enum]
