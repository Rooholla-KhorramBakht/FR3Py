import time
import mujoco
import mujoco.viewer
import numpy as np

class FR3Sim:
    def __init__(self, interface_type = 'torque', render=True, dt=0.002, xml_path=None):
        assert interface_type in ['torque', 'velocity'], 'The interface should be velocity or torque'
        self.interface_type = interface_type
        if xml_path is not None:
            self.model = mujoco.MjModel.from_xml_path(xml_path)
        else:
            raise NotImplementedError('The xml_path should be provided')
        
        self.simulated = True
        self.data = mujoco.MjData(self.model)
        self.dt = dt
        _render_dt = 1/60
        self.render_ds_ratio = max(1, _render_dt//dt)

        if render:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data, show_left_ui=False, show_right_ui=False)
            self.render = True
            self.viewer.cam.distance = 3.0
            self.viewer.cam.azimuth = 180
            self.viewer.cam.elevation = -45
            self.viewer.cam.lookat[:] = np.array([0.0, 0, 0.824])
        else:
            self.render = False

        self.model.opt.gravity[2] = -9.81
        self.model.opt.timestep = dt
        self.renderer = None
        self.render = render
        self.step_counter = 0

        self.viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
        self.viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True
        # self.viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = True
        
        self.model.vis.scale.contactwidth = 0.1
        self.model.vis.scale.contactheight = 0.03
        self.model.vis.scale.forcewidth = 0.05
        self.model.vis.map.force = 0.3

        self.ngeom = 0
        self.maxgeom = 1000
        
        # self.q0 = np.array([0.0, -0.785398163, 0.0, -2.35619449, 0.0, 1.57079632679, 0.785398163397, 0.04, 0.04])
        # self.reset()
        # mujoco.mj_step(self.model, self.data)
        # self.viewer.sync()
        self.nv = self.model.nv
        self.jacp = np.zeros((3, self.nv))
        self.jacr = np.zeros((3, self.nv))
        self.M = np.zeros((self.nv, self.nv))
        self.Minv = np.zeros((self.nv, self.nv))
        self.latest_command_stamp = time.time()
        self.actuator_tau = np.zeros(7)
        self.tau_ff = np.zeros(7)
        self.dq_des = np.zeros(7)

    def reset(self, q0):
        self.data.qpos[:] = q0
        self.data.qvel[:] = np.zeros(9)

    def getJointStates(self):
        return {"q":self.data.qpos[:7], 
               "dq":self.data.qvel[:7],
               'tau_est':(self.data.qfrc_constraint.squeeze()+self.data.qfrc_smooth.squeeze())[0:7]}
    
    def getFingerStates(self):
        return {"q":self.data.qpos[7:9], 
               "dq":self.data.qvel[7:9],
               'tau_est':(self.data.qfrc_constraint.squeeze()+self.data.qfrc_smooth.squeeze())[7:9]}

    def setCommands(self, cmd):
        self.dq_des = cmd
        self.tau_ff = cmd
        self.latest_command_stamp = time.time()
        
    def step(self):
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
        mujoco.mj_solveM(self.model, self.data, self.Minv, np.eye(self.Minv.shape[0]))
        nle = self.data.qfrc_bias.reshape(self.nv,1)
        return {
            'M':self.M,
            'nle':nle,
            'Minv':self.Minv
        }

    def close(self):
        self.viewer.close()

    def add_visual_capsule(self, point1, point2, radius, rgba, id_geom_offset=0, limit_num=False):
        """Adds one capsule to an mjvScene."""
        scene = self.viewer.user_scn
        if limit_num:
            if self.ngeom >= self.maxgeom:
                id_geom = self.ngeom % self.maxgeom + id_geom_offset
            else:
                scene.ngeom += 1
                id_geom = self.ngeom + id_geom_offset
            self.ngeom += 1
        else:
            id_geom = scene.ngeom
            scene.ngeom += 1
        # initialise a new capsule, add it to the scene using mjv_makeConnector
        mujoco.mjv_initGeom(scene.geoms[id_geom],
                            mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3),
                            np.zeros(3), np.zeros(9), np.array(rgba).astype(np.float32))
        mujoco.mjv_makeConnector(scene.geoms[id_geom],
                                mujoco.mjtGeom.mjGEOM_CAPSULE, radius,
                                point1[0], point1[1], point1[2],
                                point2[0], point2[1], point2[2])
        
        return 