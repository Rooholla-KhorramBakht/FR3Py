class simulationManager:
    def __init__(self, robot, lcm_server, default_cmd, physics_dt, lcm_timeout=0.01):
        self.robot = robot
        self.lcm_server = lcm_server
        self.missed_ticks = 0
        self.default_cmd = default_cmd
        self.robot.initialize()
        self.reset_required = True
        self.lcm_timeout = lcm_timeout
        self.physics_dt = physics_dt
        self.robot.setCommands(self.default_cmd)

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
                self.robot.setCommands(self.default_cmd)
                self.reset_required = False
            self.robot.setCommands(lcm_cmd)
        else:
            self.missed_ticks += 1
        if self.missed_ticks > 0.2 / (
            self.lcm_timeout + self.physics_dt
        ):  # stopped for more than a second?
            self.reset_required = True
            self.robot.setCommands(self.default_cmd)
