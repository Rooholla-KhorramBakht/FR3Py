from FR3Py import getDataPath
import numpy as np
import os
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
import copy
from scipy.spatial.transform import Rotation


class RobotModel:
    def __init__(self):
        package_directory = getDataPath()
        robot_URDF = package_directory + "/robots/fr3.urdf"
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
        assert q.shape == (9,), "q vector should be 9,"
        assert dq.shape == (9,), "dq vector should be 9,"
        self.robot.computeJointJacobians(q)
        self.robot.framesForwardKinematics(q)
        self.robot.centroidalMomentum(q, dq)
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
        Minv = pin.computeMinverse(self.robot.model, self.robot.data, q)
        M = self.robot.mass(q)
        nle = self.robot.nle(q, dq)
        f = np.vstack((dq[:, np.newaxis], -Minv @ nle[:, np.newaxis]))
        g = np.vstack((np.zeros((9, 9)), Minv))
        robot_states = {
            "q": q,
            "dq": dq,
            "f(x)": f,
            "g(x)": g,
            "M(q)": M,
            "M(q)^{-1}": Minv,
            "nle": nle,
            "G": self.robot.gravity(q),
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
            "J_HAND": jacobian_hand,
            "pJ_HAND": pinv_jacobian_hand,
            "R_EE": copy.deepcopy(self.robot.data.oMf[self.EE_FRAME_ID].rotation),
            "P_EE": copy.deepcopy(self.robot.data.oMf[self.EE_FRAME_ID].translation),
            "J_EE": jacobian,
            "dJ_EE": dJ,
            "pJ_EE": pinv_jac,
            "R_EE": copy.deepcopy(self.robot.data.oMf[self.EE_FRAME_ID].rotation),
            "P_EE": copy.deepcopy(self.robot.data.oMf[self.EE_FRAME_ID].translation),
        }
        return robot_states
