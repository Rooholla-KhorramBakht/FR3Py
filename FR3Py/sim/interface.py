from FR3Py.lcm_msgs.fr3_commands import fr3_cmd
from FR3Py.lcm_msgs.fr3_states import fr3_state
from FR3Py.sim.utils import LCMBridgeClient
from FR3Py.sim.utils import NumpyMemMapDataPipe
import numpy as np
from FR3Py.sim.utils import load_config
from FR3Py import FR3_ISAACSIM_CFG_PATH
import time

class FR3IsaacSim:
    """
    Class for communication with the simulated Franka Emika FR3 robot in Isaac Sim.
    It uses LCM to send joint velocity to the simulated robot and receive joint states 
    (joint angle, velocity, and torque) and camera images from the robot.
    """
    def __init__(self, robot_id= "fr3"):
        '''
        @param robot_id: (str) The name of the robot in the Isaac Sim scene.
        '''
        self.lcm_bridge = LCMBridgeClient(robot_name='fr3')
        self.cfg = load_config(FR3_ISAACSIM_CFG_PATH)
        self.camera_pipes = {}
        for camera in self.cfg['cameras']:
            for type in camera['type']:
                if type == 'rgb':
                    shape = (camera['resolution'][1], camera['resolution'][0], 4)
                else:
                    shape = (camera['resolution'][1], camera['resolution'][0])
                
                self.camera_pipes[camera['name']+'_'+type] = \
                NumpyMemMapDataPipe(camera['name']+'_'+type, force = False, shape= shape)
                print(camera['name']+'_'+type)

    def LCMThreadFunc(self):
        """
        Function that runs on a separate thread and handles LCM communication.
        `lc.handle()` function is called when there are data available to be processed.
        """
        while self.running:
            rfds, wfds, efds = select.select([self.lc.fileno()], [], [], 0.5)
            if rfds:  # Handle only if there are data in the interface file
                self.lc.handle()

    def getStates(self):
        """
        Get the current joint angle, velocity, and torque of the robot.

        @return: (dict or None) The joint state {'q': (numpy.ndarray, shape=(n,)),
        'dq': (numpy.ndarray, shape=(n,)), 'T': (numpy.ndarray, shape=(n,))}
        if the latest update was received less than 0.2 seconds ago;
        otherwise, return None.
        """
        state = self.lcm_bridge.getStates(timeout=0.1)
        if state is not None:
            q = np.array(state.q)
            dq = np.array(state.dq)
            T = np.array(state.T)
            return {'q': q, 'dq': dq, 'T': T}
        else:
            return None
    
    def readCameras(self):
        data =  {key:pipe.read() for key,pipe in self.camera_pipes.items()}
        return data

    def sendCommands(self, cmd):
        """
        Send a joint velocity command to the robot. 
        @param cmd: (numpy.ndarray, shape=(9,)) The joint command to send
        """
        cmd_lcm  = fr3_cmd()
        cmd_lcm.cmd =  cmd.tolist()
        self.lcm_bridge.sendCommands(cmd_lcm)

    def close(self):
        """
        Stop the LCM thread, unsubscribe from the LCM topic,
        and effectively shut down the interface.
        """
        self.lcm_bridge.close()

    def reset(self):
        time.sleep(0.3)
        for i in range(100):
            time.sleep(0.01)
            self.sendCommands(np.zeros(9))
            state=self.getStates()