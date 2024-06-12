import sys
from dataclasses import dataclass
import numpy as np
import copy
import numpy as np
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
from scipy.spatial.transform import Rotation
from FR3Py import getDataPath

class PinocchioModel:
    def __init__(self):
        package_directory = getDataPath()
        robot_URDF = package_directory + "/assets/urdf/fr3.urdf"
        self.robot = RobotWrapper.BuildFromURDF(robot_URDF, package_directory)
        # get fr3 frame ids
        self.FR3_LINK3_FRAME_ID = 8
        self.FR3_LINK4_FRAME_ID = 10
        self.FR3_LINK5_FRAME_ID = 12
        self.FR3_LINK6_FRAME_ID = 14
        self.FR3_LINK7_FRAME_ID = 16
        self.FR3_HAND_FRAME_ID = 20
        self.EE_FRAME_ID = 26
        # Get frame ID for grasp target
        self.jacobian_frame = pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        self.q_nominal = np.array(
            [
                [0.0],
                [-np.pi / 4],
                [0.0],
                [-3 * np.pi / 4],
                [0.0],
                [np.pi / 2],
                [np.pi / 4],
                [0.001],
                [0.001],
            ]
        )

    def compute_crude_location(self, R_offset, p_offset, frame_id):
        # get link orientation and position
        _p = self.robot.data.oMf[frame_id].translation
        _Rot = self.robot.data.oMf[frame_id].rotation

        # compute link transformation matrix
        _T = np.hstack((_Rot, _p[:, np.newaxis]))
        T = np.vstack((_T, np.array([[0.0, 0.0, 0.0, 1.0]])))

        # compute link offset transformation matrix
        _TB = np.hstack((R_offset, p_offset))
        TB = np.vstack((_TB, np.array([[0.0, 0.0, 0.0, 1.0]])))

        # get transformation matrix
        T_mat = T @ TB

        # compute crude model location
        p = (T_mat @ np.array([[0.0], [0.0], [0.0], [1.0]]))[:3, 0]

        # compute crude model orientation
        Rot = T_mat[:3, :3]

        # quaternion
        q = Rotation.from_matrix(Rot).as_quat()

        return p, Rot, q

    def getInfo(self, q, dq):
        """
        gets the q, dq from the robot and compute the robot kinematic and
        dynamic parameters and returns to the user.

        state contains:
        -------------------------------------
        q: joint position
        dq: joint velocity
        f(x): drift
        g(x): control influence matrix
        G: gravitational vector
        J_EE: end-effector Jacobian
        dJ_EE: time derivative of end-effector Jacobian
        pJ_EE: pseudo-inverse of end-effector Jacobian
        R_EE: end-effector rotation matrix
        P_EE: end-effector position vector
        """
        _q = q.squeeze()
        _dq = dq.squeeze()

        assert _q.shape == (9,) or _q.shape == (7,), "q vector should be 9 or 7"
        assert _dq.shape == (9,) or _dq.shape == (7,), "dq vector should be 9 or 7,"

        if q.shape[0] == 7:
            _q = np.hstack([_q, np.zeros(2)])
        if dq.shape[0] == 7:
            _dq = np.hstack([_dq, np.zeros(2)])

        self.robot.computeJointJacobians(_q)
        self.robot.framesForwardKinematics(_q)
        self.robot.centroidalMomentum(_q, _dq)
        # Get Jacobian from grasp target frame
        # preprocessing is done in get_state_update_pinocchio()
        jacobian = self.robot.getFrameJacobian(self.EE_FRAME_ID, self.jacobian_frame)

        # Get pseudo-inverse of frame Jacobian
        pinv_jac = np.linalg.pinv(jacobian)

        dJ = pin.getFrameJacobianTimeVariation(
            self.robot.model, self.robot.data, self.EE_FRAME_ID, self.jacobian_frame
        )

        jacobian_link3 = self.robot.getFrameJacobian(
            self.FR3_LINK3_FRAME_ID, self.jacobian_frame
        )
        jacobian_link4 = self.robot.getFrameJacobian(
            self.FR3_LINK4_FRAME_ID, self.jacobian_frame
        )
        jacobian_link5_1 = self.robot.getFrameJacobian(
            self.FR3_LINK5_FRAME_ID, self.jacobian_frame
        )
        jacobian_link5_2 = self.robot.getFrameJacobian(
            self.FR3_LINK5_FRAME_ID, self.jacobian_frame
        )
        jacobian_link6 = self.robot.getFrameJacobian(
            self.FR3_LINK6_FRAME_ID, self.jacobian_frame
        )
        jacobian_link7 = self.robot.getFrameJacobian(
            self.FR3_LINK7_FRAME_ID, self.jacobian_frame
        )
        jacobian_hand = self.robot.getFrameJacobian(
            self.EE_FRAME_ID, self.jacobian_frame
        )

        # Get pseudo-inverse of hand Jacobian
        pinv_jacobian_hand = np.linalg.pinv(jacobian_hand)

        # compute the position and rotation of the crude models
        p_link3, R_link3, q_LINK3 = self.compute_crude_location(
            np.eye(3), np.array(([0.0], [0.0], [-0.145])), self.FR3_LINK3_FRAME_ID
        )

        p_link4, R_link4, q_LINK4 = self.compute_crude_location(
            np.eye(3), np.array(([0.0], [0.0], [0.0])), self.FR3_LINK4_FRAME_ID
        )

        p_link5_1, R_link5_1, q_LINK5_1 = self.compute_crude_location(
            np.eye(3), np.array(([0.0], [0.0], [-0.26])), self.FR3_LINK5_FRAME_ID
        )

        p_link5_2, R_link5_2, q_LINK5_2 = self.compute_crude_location(
            np.eye(3), np.array(([0.0], [0.08], [-0.13])), self.FR3_LINK5_FRAME_ID
        )

        p_link6, R_link6, q_LINK6 = self.compute_crude_location(
            np.eye(3), np.array(([0.0], [0.0], [-0.03])), self.FR3_LINK6_FRAME_ID
        )

        p_link7, R_link7, q_LINK7 = self.compute_crude_location(
            np.eye(3), np.array(([0.0], [0.0], [0.01])), self.FR3_LINK7_FRAME_ID
        )

        p_hand, R_hand, q_HAND = self.compute_crude_location(
            np.eye(3), np.array(([0.0], [0.0], [0.06])), self.FR3_HAND_FRAME_ID
        )
        # Get dynamics
        Minv = pin.computeMinverse(self.robot.model, self.robot.data, _q)
        M = self.robot.mass(_q)
        nle = self.robot.nle(_q, _dq)
        f = np.vstack((_dq[:, np.newaxis], -Minv @ nle[:, np.newaxis]))
        g = np.vstack((np.zeros((9, 9)), Minv))
        robot_states = {
            "q": _q,
            "dq": _dq,
            "f(x)": f[:7],
            "g(x)": g[:7],
            "M(q)": M[:7,:7],
            "M(q)^{-1}": Minv[:7,:7],
            "nle": nle[:7],
            "G": self.robot.gravity(_q)[:7],
            "R_LINK3": copy.deepcopy(R_link3),
            "P_LINK3": copy.deepcopy(p_link3),
            "q_LINK3": copy.deepcopy(q_LINK3),
            "J_LINK3": jacobian_link3,
            "R_LINK4": copy.deepcopy(R_link4),
            "P_LINK4": copy.deepcopy(p_link4),
            "q_LINK4": copy.deepcopy(q_LINK4),
            "J_LINK4": jacobian_link4,
            "R_LINK5_1": copy.deepcopy(R_link5_1),
            "P_LINK5_1": copy.deepcopy(p_link5_1),
            "q_LINK5_1": copy.deepcopy(q_LINK5_1),
            "J_LINK5_1": jacobian_link5_1,
            "R_LINK5_2": copy.deepcopy(R_link5_2),
            "P_LINK5_2": copy.deepcopy(p_link5_2),
            "q_LINK5_2": copy.deepcopy(q_LINK5_2),
            "J_LINK5_2": jacobian_link5_2,
            "R_LINK6": copy.deepcopy(R_link6),
            "P_LINK6": copy.deepcopy(p_link6),
            "q_LINK6": copy.deepcopy(q_LINK6),
            "J_LINK6": jacobian_link6,
            "R_LINK7": copy.deepcopy(R_link7),
            "P_LINK7": copy.deepcopy(p_link7),
            "q_LINK7": copy.deepcopy(q_LINK7),
            "J_LINK7": jacobian_link7,
            "R_HAND": copy.deepcopy(R_hand),
            "P_HAND": copy.deepcopy(p_hand),
            "q_HAND": copy.deepcopy(q_HAND),
            "J_HAND": jacobian_hand[...,:7],
            "pJ_HAND": pinv_jacobian_hand[:7,...],
            "R_EE": copy.deepcopy(self.robot.data.oMf[self.EE_FRAME_ID].rotation),
            "P_EE": copy.deepcopy(self.robot.data.oMf[self.EE_FRAME_ID].translation),
            "J_EE": jacobian[...,:7],
            "dJ_EE": dJ,
            "pJ_EE": pinv_jac[:7,...],
            "R_EE": copy.deepcopy(self.robot.data.oMf[self.EE_FRAME_ID].rotation),
            "P_EE": copy.deepcopy(self.robot.data.oMf[self.EE_FRAME_ID].translation),
        }
        return robot_states

