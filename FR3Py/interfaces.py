import select
import threading
import time
from time import sleep
import numpy as np
import pinocchio as pin
from FR3Py.lcm_msgs.fr3_states import fr3_state
from FR3Py.lcm_msgs.fr3_commands import fr3_cmd
import lcm


class FR3Real:
    def __init__(
        self,
        robot_name="franka",
        interface_type="joint_velocity",
    ):
        self.state = None
        self.trigger_timestamp = 0
        self.robot_name = robot_name
        self.state_topic_name = f"{robot_name}_state"
        self.command_topic_name = f"{robot_name}_command"
        self.states_msg = fr3_state()
        self.command_msg = fr3_cmd()
        self.user_callback = None
        # Threading Interface for handleing LCM
        self.lc = lcm.LCM()
        self.subscription = self.lc.subscribe(self.state_topic_name, self.update)
        self.subscription.set_queue_capacity(1)
        self.running = True
        self.lcm_thread = threading.Thread(target=self.LCMThreadFunc)
        self.lcm_thread.start()
        sleep(0.2)
        print("Interface Running...")

    def LCMThreadFunc(self):
        while self.running:
            rfds, wfds, efds = select.select([self.lc.fileno()], [], [], 0.5)
            if rfds:  # Handle only if there are data in the interface file
                self.lc.handle()

    def update(self, channel, data):
        msg = fr3_state.decode(data)
        self.trigger_timestamp = np.array(msg.timestamp) / 1000000
        q = np.hstack([msg.q, np.zeros((2))])
        dq = np.hstack([msg.dq, np.zeros((2))])
        T = np.hstack([msg.T, np.zeros((2))])
        self.joint_state = {"q": q, "dq": dq, "T": T}
        # Call an arbitrary user callback
        if self.user_callback is not None:
            self.user_callback(self.joint_state)

    def get_state(self):
        if time.time() - self.trigger_timestamp > 0.2:
            self.state = None
            return None
        else:
            return self.joint_state

    def send_joint_command(self, cmd):
        self.command_msg.timestamp = int(self.trigger_timestamp * 1000000)
        self.command_msg.cmd = cmd.tolist()
        self.lc.publish(self.command_topic_name, self.command_msg.encode())
        self.cmd_log = cmd

    def close(self):
        self.running = False
        self.lcm_thread.join()
        self.lc.unsubscribe(self.subscription)
        del self.lc
        print("Interface Closed.")
