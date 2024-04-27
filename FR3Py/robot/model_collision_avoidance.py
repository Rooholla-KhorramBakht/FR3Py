from dataclasses import dataclass
import numpy as np
import numpy as np
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
from scipy.spatial.transform import Rotation
from FR3Py import ASSETS_PATH

class PinocchioModel:
    def __init__(self, base_pos=[0,0,0], base_quat=[0,0,0,1]):
        """ 
        Initialize the Pinocchio model of the robot.
        base_pos: [x,y,z]
        base_quat: [qx,qy,qz,qw]
        """
        package_directory = ASSETS_PATH
        robot_URDF = package_directory + "/mujoco/fr3_with_camera_and_bounding_boxes.urdf"
        self.robot = RobotWrapper.BuildFromURDF(robot_URDF, package_directory)
        
        # Define base position and orientation
        self.base_p_offset = np.array(base_pos).reshape(-1,1)
        self.base_R_offset = Rotation.from_quat(base_quat).as_matrix()
        
        # Get frame ID for world
        self.jacobian_frame = pin.ReferenceFrame.LOCAL_WORLD_ALIGNED

        # Get frame ids for bounding boxes
        self.FR3_LINK3_BB_FRAME_ID = self.robot.model.getFrameId("fr3_link3_bounding_box")
        self.FR3_LINK4_BB_FRAME_ID = self.robot.model.getFrameId("fr3_link4_bounding_box")
        self.FR3_LINK5_1_BB_FRAME_ID = self.robot.model.getFrameId("fr3_link5_1_bounding_box")
        self.FR3_LINK5_2_BB_FRAME_ID = self.robot.model.getFrameId("fr3_link5_2_bounding_box")
        self.FR3_LINK6_BB_FRAME_ID = self.robot.model.getFrameId("fr3_link6_bounding_box")
        self.FR3_LINK7_BB_FRAME_ID = self.robot.model.getFrameId("fr3_link7_bounding_box")
        self.FR3_HAND_BB_FRAME_ID = self.robot.model.getFrameId("fr3_hand_bounding_box")

        # Get frame ID for links
        self.FR3_LINK3_FRAME_ID = self.robot.model.getFrameId("fr3_link3")
        self.FR3_LINK4_FRAME_ID = self.robot.model.getFrameId("fr3_link4")
        self.FR3_LINK5_FRAME_ID = self.robot.model.getFrameId("fr3_link5")
        self.FR3_LINK5_FRAME_ID = self.robot.model.getFrameId("fr3_link5")
        self.FR3_LINK6_FRAME_ID = self.robot.model.getFrameId("fr3_link6")
        self.FR3_LINK7_FRAME_ID = self.robot.model.getFrameId("fr3_link7")
        self.FR3_HAND_FRAME_ID = self.robot.model.getFrameId("fr3_hand")
        self.EE_FRAME_ID = self.robot.model.getFrameId("fr3_hand_tcp")
        self.FR3_CAMERA_FRAME_ID = self.robot.model.getFrameId("fr3_camera")

        # Choose the useful frame names with frame ids 
        self.frame_names_and_ids = {
            # "LINK3": self.FR3_LINK3_FRAME_ID,
            # "LINK4": self.FR3_LINK4_FRAME_ID,
            # "LINK5": self.FR3_LINK5_FRAME_ID,
            # "LINK6": self.FR3_LINK6_FRAME_ID,
            # "LINK7": self.FR3_LINK7_FRAME_ID,
            # "HAND": self.FR3_HAND_FRAME_ID,
            "EE": self.EE_FRAME_ID,
            # "CAMERA": self.FR3_CAMERA_FRAME_ID,

            "LINK3_BB": self.FR3_LINK3_BB_FRAME_ID,
            "LINK4_BB": self.FR3_LINK4_BB_FRAME_ID,
            "LINK5_1_BB": self.FR3_LINK5_1_BB_FRAME_ID,
            "LINK5_2_BB": self.FR3_LINK5_2_BB_FRAME_ID,
            "LINK6_BB": self.FR3_LINK6_BB_FRAME_ID,
            "LINK7_BB": self.FR3_LINK7_BB_FRAME_ID,
            "HAND_BB": self.FR3_HAND_BB_FRAME_ID,
        }

        # Define nominal joint position
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


    def compute_crude_location(self, base_R_offset, base_p_offset, frame_id):
        # get link orientation and position
        _p = self.robot.data.oMf[frame_id].translation
        _Rot = self.robot.data.oMf[frame_id].rotation

        # compute link transformation matrix
        _T = np.hstack((_Rot, _p[:, np.newaxis]))
        T = np.vstack((_T, np.array([[0.0, 0.0, 0.0, 1.0]])))

        # compute link offset transformation matrix
        _TW = np.hstack((base_R_offset, base_p_offset))
        TW = np.vstack((_TW, np.array([[0.0, 0.0, 0.0, 1.0]])))
        
        # get transformation matrix
        T_mat = TW @ T 

        # compute crude model location
        p = (T_mat @ np.array([[0.0], [0.0], [0.0], [1.0]]))[:3, 0]

        # compute crude model orientation
        Rot = T_mat[:3, :3]

        # quaternion
        q = Rotation.from_matrix(Rot).as_quat()

        return p, Rot, q

    def getInfo(self, q, dq):
        """
        info contains:
        -------------------------------------
        q: joint position
        dq: joint velocity
        J_{frame_name}: jacobian of frame_name
        P_{frame_name}: position of frame_name
        R_{frame_name}: orientation of frame_name
        quat_{frame_name}: quaternion of frame_name
        """
        assert q.shape == (9,), "q vector should be 9,"
        assert dq.shape == (9,), "dq vector should be 9,"
        self.robot.computeJointJacobians(q)
        self.robot.framesForwardKinematics(q)
        self.robot.centroidalMomentum(q, dq)
        
        info = {"q": q,
                "dq": dq}

        for frame_name, frame_id in self.frame_names_and_ids.items():
            # Frame jacobian
            info[f"J_{frame_name}"] = self.robot.getFrameJacobian(frame_id, self.jacobian_frame)

            # Frame position and orientation
            (   info[f"P_{frame_name}"],
                info[f"R_{frame_name}"],
                info[f"quat_{frame_name}"],
            ) = self.compute_crude_location(
                self.base_R_offset, self.base_p_offset, frame_id
            )

            # Advanced calculation
            # dJdq = dJ/dt * dq, shape (6,)
            info[f"dJdq_{frame_name}"] = np.array(pin.getFrameClassicalAcceleration(
                self.robot.model, self.robot.data, frame_id, self.jacobian_frame
            ))

        # Get dynamics
        info["M"] = self.robot.mass(q)
        info["Minv"] = pin.computeMinverse(self.robot.model, self.robot.data, q)
        info["nle"] = self.robot.nle(q, dq)
        info["G"] = self.robot.gravity(q)
        
        return info


class BoundingShapeCoef(object):

    def __init__(self):
        super(BoundingShapeCoef, self).__init__()

        self.coefs = {}
        self.coefs["LINK3_BB"] = np.array([[1/(0.09*0.09), 0.0, 0.0],
                                             [0.0, 1/(0.09*0.09), 0.0],
                                             [0.0, 0.0, 1/(0.165*0.165)]])
        
        self.coefs["LINK4_BB"] = np.array([[1/(0.09*0.09), 0.0, 0.0],
                                             [0.0, 1/(0.09*0.09), 0.0],
                                             [0.0, 0.0, 1/(0.15*0.15)]])
        
        self.coefs["LINK5_1_BB"] = np.array([[1/(0.09*0.09), 0.0, 0.0],
                                             [0.0, 1/(0.09*0.09), 0.0],
                                             [0.0, 0.0, 1/(0.14*0.14)]])
        
        self.coefs["LINK5_2_BB"] = np.array([[1/(0.055*0.055), 0.0, 0.0],
                                             [0.0, 1/(0.055*0.055), 0.0],
                                             [0.0, 0.0, 1/(0.125*0.125)]])
        
        self.coefs["LINK6_BB"] = np.array([[1/(0.08*0.08), 0.0, 0.0],
                                             [0.0, 1/(0.08*0.08), 0.0],
                                             [0.0, 0.0, 1/(0.11*0.11)]])
        
        self.coefs["LINK7_BB"] = np.array([[1/(0.07*0.07), 0.0, 0.0],
                                             [0.0, 1/(0.07*0.07), 0.0],
                                             [0.0, 0.0, 1/(0.14*0.14)]])
        
        self.coefs["HAND_BB"] = np.array([[1/(0.07*0.07), 0.0, 0.0],
                                             [0.0, 1/(0.12*0.12), 0.0],
                                             [0.0, 0.0, 1/(0.10*0.10)]])
        
        # self.coefs["HAND_BB"] = np.array([[1/(0.12*0.12), 0.0, 0.0],
        #                                      [0.0, 1/(0.12*0.12), 0.0],
        #                                      [0.0, 0.0, 1/(0.12*0.12)]])

        self.coefs_sqrt = {}
        self.coefs_sqrt["LINK3_BB"] = np.array([[1/0.09, 0.0, 0.0],
                                             [0.0, 1/0.09, 0.0],
                                             [0.0, 0.0, 1/0.165]])
        
        self.coefs_sqrt["LINK4_BB"] = np.array([[1/0.09, 0.0, 0.0],
                                             [0.0, 1/0.09, 0.0],
                                             [0.0, 0.0, 1/0.15]])
        
        self.coefs_sqrt["LINK5_1_BB"] = np.array([[1/0.09, 0.0, 0.0],
                                                [0.0, 1/0.09, 0.0],
                                                [0.0, 0.0, 1/0.14]])
        
        self.coefs_sqrt["LINK5_2_BB"] = np.array([[1/0.055, 0.0, 0.0],
                                                [0.0, 1/0.055, 0.0],
                                                [0.0, 0.0, 1/0.125]])
        
        self.coefs_sqrt["LINK6_BB"] = np.array([[1/0.08, 0.0, 0.0],
                                                [0.0, 1/0.08, 0.0],
                                                [0.0, 0.0, 1/0.11]])
        
        self.coefs_sqrt["LINK7_BB"] = np.array([[1/0.07, 0.0, 0.0],
                                                [0.0, 1/0.07, 0.0],
                                                [0.0, 0.0, 1/0.14]])
        
        self.coefs_sqrt["HAND_BB"] = np.array([[1/0.07, 0.0, 0.0],
                                             [0.0, 1/0.12, 0.0],
                                             [0.0, 0.0, 1/0.10]])
        
        # self.coefs_sqrt["HAND_BB"] = np.array([[1/0.12, 0.0, 0.0],
        #                                      [0.0, 1/0.12, 0.0],
        #                                      [0.0, 0.0, 1/0.12]])
        