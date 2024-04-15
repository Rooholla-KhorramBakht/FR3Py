import time
from copy import deepcopy
import mujoco
import mujoco.viewer
import numpy as np
from FR3Py import ASSETS_PATH
import os
from scipy.spatial.transform import Rotation

class FR3Sim:
    def __init__(self, render=True, dt=0.002):
        
        self.model = mujoco.MjModel.from_xml_path(
            os.path.join(ASSETS_PATH, 'mujoco/fr3.xml')
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
        self.actuator_tau = np.zeros(12)

    def reset(self):
        self.data.qpos[:7] = self.q0
        self.data.qvel[:7] = np.zeros(7)

    def getJointStates(self):
        return {"q":self.data.qpos[:7], 
               "dq":self.data.qvel[:7],
               'tau_est':self.actuator_tau[:7]}

    def setCommands(self, cmd):
        self.dq_des = cmd
        self.tau_ff = cmd
        self.latest_command_stamp = time.time()
        
    def step(self):
        state = self.getJointStates()
        q, dq = state['q'], state['dq']
        nle = self.getDynamicsParams()['nle']
        tau = nle[:7]
        # tau = np.diag(self.kp)@(self.q_des-q).reshape(12,1)+ \
        #       np.diag(self.kv)@(self.dq_des-dq).reshape(12,1)+self.tau_ff.reshape(12,1)
        # self.actuator_tau = tau
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