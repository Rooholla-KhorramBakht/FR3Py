from typing import Optional
import numpy as np
import pybullet as p
import pybullet_data
from gymnasium import Env, spaces
from FR3Py import getDataPath
from FR3Py.controllers.utils import RobotModel


class FR3Sim(Env):
    """
    A simulation class for the Franka Research 3 robot. The class is a subclass of gymnasium environment.
    The class allows to simulate the robot in two control modes in the joint space: velocity and torque.
    """
    metadata = {
        "render_modes": ["human", "rgb_array"],
    }

    def __init__(
        self,
        mode="velocity",
        render_mode: Optional[str] = None,
        record_path=None,
        Ts=0.001,
    ):
        """
        Class constructor that sets the control mode, the client and initializes the robot and its environment.
        
        @param mode: (str) Control mode for the robot, should be either "velocity" or "torque".
        @param render_mode: (str, optional) Mode for rendering. If set to "human", uses GUI for rendering. Default is None.
        @param record_path: (str, optional) Path to save recording. Default is None.
        @param Ts: (float) Time step for simulation. Default is 0.001.
        """
        assert mode in [
            "velocity",
            "torque",
        ], "Control mode should be velocity or torque"
        self.mode = mode
        print(f"interface control mode is: {mode}")
        if render_mode == "human":
            self.client = p.connect(p.GUI)
            # Improves rendering performance on M1 Macs
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        else:
            self.client = p.connect(p.DIRECT)

        self.record_path = record_path

        p.setGravity(0, 0, -9.81)
        p.setTimeStep(Ts)

        # Load plane
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.planeID = p.loadURDF("plane.urdf")

        # Load Franka Research 3 Robot
        model_name = "fr3"
        package_directory = getDataPath()
        robot_URDF = package_directory + "/robots/{}.urdf".format(model_name)
        urdf_search_path = package_directory + "/robots"
        p.setAdditionalSearchPath(urdf_search_path)
        self.robotID = p.loadURDF("{}.urdf".format(model_name), useFixedBase=True)

        # Get active joint ids
        self.active_joint_ids = [0, 1, 2, 3, 4, 5, 6, 10, 11]

        # Disable the velocity control on the joints as we use torque control.
        p.setJointMotorControlArray(
            self.robotID,
            self.active_joint_ids,
            p.VELOCITY_CONTROL,
            forces=np.zeros(9),
        )
        self.robot = RobotModel()

        # Get number of joints
        self.n_j = p.getNumJoints(self.robotID)

        # Set observation and action space
        obs_low_q = []
        obs_low_dq = []
        obs_high_q = []
        obs_high_dq = []
        _act_low = []
        _act_high = []

        for i in range(self.n_j):
            _joint_infos = p.getJointInfo(self.robotID, i)  # get info of each joint

            if _joint_infos[2] != p.JOINT_FIXED:
                obs_low_q.append(_joint_infos[8])
                obs_high_q.append(_joint_infos[9])
                obs_low_dq.append(-_joint_infos[11])
                obs_high_dq.append(_joint_infos[11])
                _act_low.append(-_joint_infos[10])
                _act_high.append(_joint_infos[10])

        obs_low = np.array(obs_low_q + obs_low_dq, dtype=np.float32)
        obs_high = np.array(obs_high_q + obs_high_dq, dtype=np.float32)
        act_low = np.array(_act_low, dtype=np.float32)
        act_high = np.array(_act_high, dtype=np.float32)

        self.observation_space = spaces.Box(obs_low, obs_high, dtype=np.float32)
        self.action_space = spaces.Box(act_low, act_high, dtype=np.float32)

    def reset(
        self,
        *,
        seed: Optional[int] = None,
        options: Optional[dict] = None,
        cameraDistance=1.4,
        cameraYaw=66.4,
        cameraPitch=-16.2,
        lookat=[0.0, 0.0, 0.0],
    ):
        """
        Resets the robot to its initial configuration and starts recording if record path is set.

        @param seed: (int, optional) Seed for random number generator. Default is None.
        @param options: (dict, optional) Optional parameters. Default is None.
        @param cameraDistance: (float) Distance of the camera for recording. Default is 1.4.
        @param cameraYaw: (float) Yaw of the camera for recording. Default is 66.4.
        @param cameraPitch: (float) Pitch of the camera for recording. Default is -16.2.
        @param lookat: (list) Focus point of the camera for recording. Default is [0.0, 0.0, 0.0].
        """
        super().reset(seed=seed)

        target_joint_angles = [
            0.0,
            -0.785398163,
            0.0,
            -2.35619449,
            0.0,
            1.57079632679,
            0.785398163397,
            0.001,
            0.001,
        ]

        self.q_nominal = np.array(target_joint_angles)

        for i, joint_ang in enumerate(target_joint_angles):
            p.resetJointState(self.robotID, self.active_joint_ids[i], joint_ang, 0.0)

        if self.record_path is not None:
            p.resetDebugVisualizerCamera(cameraDistance, cameraYaw, cameraPitch, lookat)

            self.loggingId = p.startStateLogging(
                p.STATE_LOGGING_VIDEO_MP4, self.record_path
            )

    def step(self, action):
        """
        Applies the action command to the robot and steps the simulation forward.

        @param action: (numpy.ndarray, shape=(9,)) The action to be applied on the robot in joint space.
        @return: The joint angles (numpy.ndarray, shape=(9,)) and velocities (numpy.ndarray, shape=(9,)) of the robot.
        """
        self.send_joint_command(action)
        p.stepSimulation()
        q, dq = self.get_state()
        return q, dq

    def close(self):
        """
        Closes the simulation environment and stops the recording if it is running.
        """
        if self.record_path is not None:
            p.stopStateLogging(self.loggingId)
        p.disconnect()

    def get_state(self):
        """
        Fetches the state of the robot.

        @return: Joint angles (numpy.ndarray, shape=(9,)) and velocities (numpy.ndarray, shape=(9,)) of the robot.
        """
        q = np.zeros(9)
        dq = np.zeros(9)

        for i, id in enumerate(self.active_joint_ids):
            _joint_state = p.getJointState(self.robotID, id)
            q[i], dq[i] = _joint_state[0], _joint_state[1]

        return q, dq

    def send_joint_command(self, cmd):
        """
        Sends joint commands to the robot depending on the control mode.

        @param cmd: (numpy.ndarray, shape=(9,)) The command to be sent to the robot in joint space.
        """
        zeroGains = cmd.shape[0] * (0.0,)
        if self.mode == "torque":
            p.setJointMotorControlArray(
                self.robotID,
                self.active_joint_ids,
                p.TORQUE_CONTROL,
                forces=cmd,
                positionGains=zeroGains,
                velocityGains=zeroGains,
            )
        else:
            # Run a PD with gravity compensation
            q, dq = self.get_state()
            info = self.robot.getInfo(q, dq)
            G = info["G"][:, np.newaxis]
            # compute torque command
            τ = (
                5.0 * (cmd[:, np.newaxis] - dq[:, np.newaxis])
                + G
                - 0.5 * dq[:, np.newaxis]
            )

            p.setJointMotorControlArray(
                self.robotID,
                self.active_joint_ids,
                p.TORQUE_CONTROL,
                forces=τ,
                positionGains=zeroGains,
                velocityGains=zeroGains,
            )
