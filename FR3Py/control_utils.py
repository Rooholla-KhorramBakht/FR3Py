from FR3Py import getDataPath
import numpy as np
import os
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
import copy

class RobotModel():
    def __init__(self):
        package_directory = getDataPath()
        robot_URDF = package_directory + "/robots/fr3.urdf"
        self.robot = RobotWrapper.BuildFromURDF(robot_URDF, package_directory)
        #End-effector frame id
        self.EE_FRAME_ID = 26
        # Get frame ID for grasp target
        self.jacobian_frame = pin.ReferenceFrame.LOCAL_WORLD_ALIGNED

    def getInfo(self, q, dq):
        '''
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
        '''
        assert q.shape == (9,), 'q vector should be 9,'
        assert dq.shape == (9,), 'dq vector should be 9,'
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
                        "J_EE": jacobian,
                        "dJ_EE": dJ,
                        "pJ_EE": pinv_jac,
                        "R_EE": copy.deepcopy(self.robot.data.oMf[self.EE_FRAME_ID].rotation),
                        "P_EE": copy.deepcopy(self.robot.data.oMf[self.EE_FRAME_ID].translation),
                      }
        return robot_states
