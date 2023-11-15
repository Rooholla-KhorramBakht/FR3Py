import xml.etree.ElementTree as ET
from dataclasses import dataclass

import imageio.v3 as iio
import numpy as np
from scipy.spatial.transform import Rotation


@dataclass
class CalibrationData:
    pos: np.ndarray
    quat: np.ndarray
    rot_mat: np.ndarray
    trans_mat: np.array
    pose: np.ndarray
    parent_frame: str
    child_frame: str


def moveitHandEyeLaunchResultParser(filename):
    """Load and parse the calibration result from the Moveit library and returns
    the contained pose in various forms"""
    tree = ET.parse(filename)
    root = tree.getroot()

    node = root.findall("node")[0]
    arguments = node.get("args").split(" ")

    pos = np.array([float(arguments[0]), float(arguments[1]), float(arguments[2])])
    quat = np.array(
        [
            float(arguments[5]),
            float(arguments[6]),
            float(arguments[7]),
            float(arguments[8]),
        ]
    )
    rot_mat = Rotation.from_quat(quat).as_matrix()

    trans_mat = np.eye(4)
    trans_mat[:3, :3] = rot_mat
    trans_mat[:3, 3] = pos

    parent_frame = arguments[9]
    child_frame = arguments[10]

    return CalibrationData(pos, quat, rot_mat, trans_mat, parent_frame, child_frame)
