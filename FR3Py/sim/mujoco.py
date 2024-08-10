import time
from copy import deepcopy
import mujoco
import mujoco.viewer
import numpy as np
from FR3Py import ASSETS_PATH
import os
from scipy.spatial.transform import Rotation

class FR3Sim:
    def __init__(self, interface_type = 'torque', render=True, dt=0.002, xml_path=None):
        assert interface_type in ['torque', 'velocity'], 'The interface should be velocity or torque'
        self.interface_type = interface_type
        if xml_path is not None:
            self.model = mujoco.MjModel.from_xml_path(xml_path)
        else:
            self.model = mujoco.MjModel.from_xml_path(
                os.path.join(ASSETS_PATH,'mujoco/fr3.xml')
            )
        self.simulated = True
        self.data = mujoco.MjData(self.model)
        self.dt = dt
        _render_dt = 1/60
        self.render_ds_ratio = max(1, _render_dt//dt)

        if render:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self.render = True
            self.viewer.cam.distance = 3.0
            self.viewer.cam.azimuth = 90
            self.viewer.cam.elevation = -45
            self.viewer.cam.lookat[:] = np.array([0.0, -0.25, 0.824])
        else:
            self.render = False

        self.model.opt.gravity[2] = -9.81
        self.model.opt.timestep = dt
        self.renderer = None
        self.render = render
        self.step_counter = 0
        

        self.q0 = np.array([0.0, -0.785398163, 0.0, -2.35619449, 0.0, 1.57079632679, 0.785398163397])
        self.reset()
        mujoco.mj_step(self.model, self.data)
        self.viewer.sync()
        self.nv = self.model.nv
        self.jacp = np.zeros((3, self.nv))
        self.jacr = np.zeros((3, self.nv))
        self.M = np.zeros((self.nv, self.nv))
        self.latest_command_stamp = time.time()
        self.actuator_tau = np.zeros(7)
        self.tau_ff = np.zeros(7)
        self.dq_des = np.zeros(7)

    def reset(self):
        self.data.qpos[:7] = self.q0
        self.data.qvel[:7] = np.zeros(7)
        mujoco.mj_step(self.model, self.data)
        # Render every render_ds_ratio steps (60Hz GUI update)
        if self.render and (self.step_counter%self.render_ds_ratio)==0:
            self.viewer.sync()
        

    def getJointStates(self):
        return {"q":self.data.qpos[:7], 
               "dq":self.data.qvel[:7],
               'tau_est':(self.data.qfrc_constraint.squeeze()+self.data.qfrc_smooth.squeeze())[0:7]}

    def setCommands(self, cmd, finger_pos=None):
        self.dq_des = cmd
        self.tau_ff = cmd
        self.latest_command_stamp = time.time()
        self.step(finger_pos)
        
    def step(self, finger_pos=None):
        if self.interface_type =='torque':
            nle = self.getDynamicsParams()['nle']
            tau = nle[:7].squeeze()+self.tau_ff
            self.actuator_tau = tau
        else:
            state = self.getJointStates()
            q, dq = state['q'], state['dq']
            nle = self.getDynamicsParams()['nle']
            tau = nle[:7].squeeze()+20*(self.dq_des-dq)
            self.actuator_tau = tau

        if finger_pos is not None:
            tau = np.append(tau, finger_pos)
            self.data.ctrl[:8] = tau.squeeze()
        else:
            self.data.ctrl[:7] = tau.squeeze()
        self.step_counter += 1
        mujoco.mj_step(self.model, self.data)
        # Render every render_ds_ratio steps (60Hz GUI update)
        if self.render and (self.step_counter%self.render_ds_ratio)==0:
            self.viewer.sync()

    def getSiteJacobian(self, site_name):
        id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE,site_name)
        assert id>0, 'The requested site could not be found'
        mujoco.mj_jacSite(self.model, self.data, self.jacp, self.jacr, id)
        return self.jacp, self.jacr

    def getDynamicsParams(self):
        mujoco.mj_fullM(self.model, self.M, self.data.qM)
        nle = self.data.qfrc_bias.reshape(self.nv,1)
        return {
            'M':self.M,
            'nle':nle
        }

    def close(self):
        self.viewer.close()