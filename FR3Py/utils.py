import sys
from dataclasses import dataclass

import numpy as np


def addROSPath(installation_path):
    """
    Add the ROS installation path to the python path.

    Parameters:
        installation_path (str): Path to the ROS installation directory.
    """
    sys.path.append(installation_path + "/lib/python3/dist-packages")


class NumpyMemMapDataPipe:
    def __init__(self, channel_name, force=False, dtype="uint8", shape=(640, 480, 3)):
        self.channel_name = channel_name
        self.force = force
        self.dtype = dtype
        self.shape = shape
        if force:
            self.shm = np.memmap(
                "/dev/shm/" + channel_name, dtype=self.dtype, mode="w+", shape=shape
            )
        else:
            self.shm = np.memmap(
                "/dev/shm/" + channel_name, dtype=self.dtype, mode="r+", shape=shape
            )

    def write(self, data, match_length=True):
        if match_length:
            self.shm[: data.shape[0], ...] = data
        else:
            assert (
                data.shape == self.shape
            ), "The data and the shape of the shared memory must match"
            self.shm[:] = data

    def read(self):
        return self.shm.copy()

