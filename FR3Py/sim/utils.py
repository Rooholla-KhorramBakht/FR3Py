import numpy as np
import select
import lcm
from FR3Py.lcm_msgs.fr3_commands import fr3_cmd
from FR3Py.lcm_msgs.fr3_states import fr3_state
import yaml

def load_config(file_path):
    with open(file_path, "r") as file:
        config = yaml.safe_load(file)
    return config

class LCMBridgeServer:
    def __init__(
        self,
        robot_name="robot1",
    ):
        self.state = None
        self.trigger_timestamp = 0
        self.robot_name = robot_name
        self.state_topic_name = f"{robot_name}_state"
        self.command_topic_name = f"{robot_name}_command"
        self.commands = fr3_cmd()
        # Threading Interface for handling LCM
        self.lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
        self.subscription = self.lc.subscribe(
            self.command_topic_name, self.command_callback
        )
        self.subscription.set_queue_capacity(1)

    def getCommands(self, timeout=0.1):
        """
        Function that runs on a separate thread and handles LCM communication.
        `lc.handle()` function is called when there are data available to be processed.
        """
        rfds, wfds, efds = select.select([self.lc.fileno()], [], [], timeout)
        if rfds:  # Handle only if there are data in the interface file
            self.lc.handle()
            return self.commands
        else:
            return None

    def sendStates(self, state):
        """
        Send a joint command to the robot.

        @param cmd: (numpy.ndarray, shape=(n,)) The joint command to send
        """
        # self.command_msg.timestamp = int(self.trigger_timestamp * 1000000)
        # self.command_msg.cmd = cmd.tolist()
        self.lc.publish(self.state_topic_name, state.encode())

    def command_callback(self, channel, data):
        """
        Callback function that executes whenever a new robot state
        is received from the C++ driver.

        @param channel: (str) The name of the LCM channel
        @param data: (bytes) The LCM message data
        """
        self.commands = fr3_cmd.decode(data)

    def close(self):
        """
        Stop the LCM thread, unsubscribe from the LCM topic,
        and effectively shut down the interface.
        """
        self.running = False
        self.lc.unsubscribe(self.subscription)
        del self.lc
        print("Interface Closed.")


class LCMBridgeClient:
    def __init__(
        self,
        robot_name="robot1",
        user_callback=None,
    ):
        self.state = None
        self.trigger_timestamp = 0
        self.robot_name = robot_name
        self.state_topic_name = f"{robot_name}_state"
        self.command_topic_name = f"{robot_name}_command"
        self.states = fr3_state()
        self.user_callback = user_callback  # Callback function to be executed when a new state is received
        # Threading Interface for handling LCM
        self.lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
        self.subscription = self.lc.subscribe(
            self.state_topic_name, self.state_callback
        )
        self.subscription.set_queue_capacity(1)

    def sendCommands(self, cmd):
        """
        Function that runs on a separate thread and handles LCM communication.
        `lc.handle()` function is called when there are data available to be processed.
        """
        self.lc.publish(self.command_topic_name, cmd.encode())

    def getStates(self, timeout):
        """
        Send a joint command to the robot.

        @param cmd: (numpy.ndarray, shape=(n,)) The joint command to send
        """
        # self.command_msg.timestamp = int(self.trigger_timestamp * 1000000)
        # self.command_msg.cmd = cmd.tolist()
        rfds, wfds, efds = select.select([self.lc.fileno()], [], [], timeout)
        if rfds:  # Handle only if there are data in the interface file
            self.lc.handle()
            return self.states
        else:
            return None

    def state_callback(self, channel, data):
        """
        Callback function that executes whenever a new robot state
        is received from the C++ driver.

        @param channel: (str) The name of the LCM channel
        @param data: (bytes) The LCM message data
        """
        self.states = fr3_state.decode(data)
        if self.user_callback is not None:
            self.user_callback(self.states)

    def close(self):
        """
        Stop the LCM thread, unsubscribe from the LCM topic,
        and effectively shut down the interface.
        """
        self.running = False
        self.lc.unsubscribe(self.subscription)
        del self.lc
        print("Interface Closed.")

class NumpyMemMapDataPipe:
    def __init__(self, channel_name, force=False, dtype="uint8", shape=(640, 480, 3)):
        self.channel_name = channel_name
        self.force = force
        self.dtype = dtype
        self.shape = shape
        if force:
            self.shm = np.memmap(
                "/dev/shm/" + channel_name, dtype=self.dtype, mode="w+", shape=shape
            )
        else:
            self.shm = np.memmap(
                "/dev/shm/" + channel_name, dtype=self.dtype, mode="r+", shape=shape
            )

    def write(self, data, match_length=True):
        if match_length:
            self.shm[: data.shape[0], ...] = data
        else:
            assert (
                data.shape == self.shape
            ), "The data and the shape of the shared memory must match"
            self.shm[:] = data

    def read(self):
        return self.shm.copy()
    
class simulationManager:
    def __init__(
        self,
        robot,
        lcm_server,
        default_cmd,
        physics_dt,
        lcm_timeout=0.01,
        mode="position_control",
    ):
        assert mode in [
            "position_control",
            "velocity_control",
        ], "mode must be either position_control or velocity_control"
        self.mode = mode
        self.robot = robot
        self.lcm_server = lcm_server
        self.missed_ticks = 0
        self.default_cmd = default_cmd
        self.robot.initialize()
        self.reset_required = True
        self.lcm_timeout = lcm_timeout
        self.physics_dt = physics_dt
        self.robot.setCommands(self.default_cmd)

    def applyCommand(self, cmd):
        if self.mode == "velocity_control":
            self.robot.setCommands(cmd)
        elif self.mode == "position_control":
            self.robot.setState(
                np.array([0, 0, 0]).reshape(-1),
                np.array([0, 0, 0, 1]).reshape(-1),
                np.array(cmd.cmd),
            )
        return None

    def step(self, timestamp):
        # Read the robot's state and send it to the LCM client
        state = self.robot.readStates()
        # state.timestamp = timestamp
        self.lcm_server.sendStates(state)
        # Wait for a response from the LCM client timeout=0.1 second
        lcm_cmd = self.lcm_server.getCommands(timeout=self.lcm_timeout)
        if lcm_cmd is not None:
            self.missed_ticks = 0
            # Reset the robot if the communication has been off for too long
            if self.reset_required:
                self.robot.initialize()
                self.applyCommand(self.default_cmd)
                self.reset_required = False
            self.applyCommand(lcm_cmd)
        else:
            self.missed_ticks += 1
        if self.missed_ticks > 0.2 / (
            self.lcm_timeout + self.physics_dt
        ):  # stopped for more than a second?
            self.reset_required = True
            self.applyCommand(self.default_cmd)
