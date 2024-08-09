from collections import deque
from typing import Optional

import numpy as np
import pypose as pp
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.prims import define_prim
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.sensor import ContactSensor, IMUSensor

import FR3Py

# from FR3Py.lcm_types.unitree_lowlevel import UnitreeLowState
from FR3Py.lcm_msgs.fr3_states import fr3_state


class FR3(Articulation):
    def __init__(
        self,
        prim_path: str,
        name: str = "fr3",
        physics_dt: Optional[float] = 1 / 200.0,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
        usd_path: Optional[str] = None,
    ) -> None:
        """
        [Summary]

        initialize robot, set up sensors and interfaces

        Args:
            prim_path {str} -- prim path of the robot on the stage
            name {str} -- name of the quadruped
            physics_dt {float} -- physics downtime of the controller
            position {np.ndarray} -- position of the robot
            orientation {np.ndarray} -- orientation of the robot
            usd_path {str} -- path to the usd file of the robot
        """
        self._prim_path = prim_path
        self.usd_path = usd_path
        prim = define_prim(self._prim_path, "Xform")
        if self.usd_path is None:
            prim.GetReferences().AddReference(FR3Py.FR3_USD_PATH)
        else:
            prim.GetReferences().AddReference(self.usd_path)

        super().__init__(
            prim_path=self._prim_path,
            name=name,
            position=position,
            orientation=orientation,
        )

        self.bullet_joint_order = [
            "fr3_joint1",
            "fr3_joint2",
            "fr3_joint3",
            "fr3_joint4",
            "fr3_joint5",
            "fr3_joint6",
            "fr3_joint7",
            "fr3_finger_joint1",
            "fr3_finger_joint2",
        ]

        self.isaac_joint_order = [
            "fr3_joint1",
            "fr3_joint2",
            "fr3_joint3",
            "fr3_joint4",
            "fr3_joint5",
            "fr3_joint6",
            "fr3_joint7",
            "fr3_finger_joint1",
            "fr3_finger_joint2",
        ]

        self.isaac_name_2_index = {s: i for i, s in enumerate(self.isaac_joint_order)}
        self.bullet_name_2_index = {s: i for i, s in enumerate(self.bullet_joint_order)}

        self.to_bullet_index = np.array(
            [self.isaac_name_2_index[id] for id in self.bullet_joint_order]
        )
        self.to_isaac_index = np.array(
            [self.bullet_name_2_index[id] for id in self.isaac_joint_order]
        )
        self.tau_est = np.zeros((12,))
        self.state = fr3_state()
        self.init_pos = np.array([0.0, 0.0, 0.6])
        self.init_quat = np.array([0.0, 0.0, 0.0, 1.0])
        self.init_joint_pos = np.array(
            [
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
        )
        return

    def toIsaacOrder(self, x):
        if x.shape[0] == 7:
            x = np.append(x, [0.0, 0.0])
        return x[self.to_isaac_index, ...]

    def toBulletOrder(self, x):
        return x[self.to_bullet_index, ...]

    def setState(self, pos, quat, q, q_dot=np.zeros((9,))) -> None:
        """[Summary]

        Set the kinematic state of the robot.

        Args:
            pos  {ndarray} -- The position of the robot (x, y, z)
            quat {ndarray} -- The orientation of the robot (qx, qy, qz, qw)
            q    {ndarray} -- Joint angles of the robot in standard Pinocchio order

        Raises:
            RuntimeError: When the DC Toolbox interface has not been configured.
        """
        # self.set_world_pose(position=pos, orientation=quat[[3, 0, 1, 2]])
        self.set_linear_velocity(np.zeros((3,)))
        self.set_angular_velocity(np.zeros((3,)))

        self.set_joint_positions(
            positions=np.asarray(self.toIsaacOrder(q), dtype=np.float32)
        )
        self.set_joint_velocities(
            velocities=np.asarray(self.toIsaacOrder(q_dot), dtype=np.float32)
        )
        self.set_joint_efforts(np.zeros((9,)))
        return

    def initialize(self, physics_sim_view=None) -> None:
        """[summary]
        initialize dc interface, set up drive mode and initial robot state
        """
        super().initialize(physics_sim_view=physics_sim_view)
        self.get_articulation_controller().set_effort_modes("force")
        self.get_articulation_controller().switch_control_mode("velocity")
        self.setState(self.init_pos, self.init_quat, self.init_joint_pos)

    def readStates(self):
        # joint pos and vel from the DC interface
        joint_state = super().get_joints_state()
        joint_pos = self.toBulletOrder(joint_state.positions)
        joint_vel = self.toBulletOrder(joint_state.velocities)
        joint_effort = self.toBulletOrder(super().get_applied_joint_efforts())
        # base frame
        base_pose = self.get_world_pose()
        base_pos = base_pose[0]
        base_quat = base_pose[1][[1, 2, 3, 0]]
        base_lin_vel = self.get_linear_velocity()
        base_ang_vel = self.get_angular_velocity()

        # assign to state objects
        self.state.q = joint_pos.tolist()
        self.state.dq = joint_vel.tolist()
        self.state.T = joint_effort.tolist()
        return self.state

    def setCommands(self, cmd):
        """[summary]
        sets the joint torques
        Argument:
            action {np.ndarray} -- Joint torque command
        """
        self.apply_action(
            ArticulationAction(joint_velocities=self.toIsaacOrder(np.array(cmd.cmd)))
        )
        return

    def step(self, cmd):
        self.readStates()
        self.setCommands(cmd)
        return self.state
