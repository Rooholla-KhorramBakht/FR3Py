import numpy as np

class simulationManager:
    def __init__(self, robot, lcm_server, default_cmd, physics_dt, lcm_timeout=0.01, mode = "position_control"):
        assert mode in ["position_control", "velocity_control"], \
        "mode must be either position_control or velocity_control"
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
            self.robot.setState(np.array([0,0,0]).reshape(-1), np.array([0,0,0,1]).reshape(-1),
                                np.array(cmd.cmd))
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
