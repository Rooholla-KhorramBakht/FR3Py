import select

import lcm

from B1Py.lcm_types.unitree_lowlevel import UnitreeLowCommand, UnitreeLowState


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
        self.commands = UnitreeLowCommand()
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
        self.commands = UnitreeLowCommand.decode(data)

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
        self.states = UnitreeLowState()
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
        self.states = UnitreeLowState.decode(data)
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
