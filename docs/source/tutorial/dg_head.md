# Python API (Dynamic Graph Head)

Once we have started the dynamic graph mananger (DGM), we can access the shared memory and write our controll in Python.

The main control loop evolves over reading the sensor data from the shared memory and writing control commands to the shared memory. From [`FrankaDynamicGraphHead`](https://github.com/BolunDai0216/dgh_franka) we can perform these two operations by calling `read_states` and `write_command`. If we take a look at the content of `read_states`

```python
def read_states(self):
    # Get the sensor values from the shared memory
    
    self.head.read()
    T  = self.head.get_sensor("joint_torques").copy()
    q  = self.head.get_sensor("joint_positions").copy()
    dq = self.head.get_sensor("joint_velocities").copy() 
    return [q, dq, T]
```

we can see that it is reading the joint torques, positions, and velocities from the shared memory. Then, for `write_command`, in the kinematic interface (velocity control), we have

```python
def write_command(self, cmd):
    # Write the sensor values to the shared memory

    assert max(cmd.shape) == 7, "The control command should be a vector of 7 numbers!"
    self.head.set_control("ctrl_joint_velocities", cmd.reshape(7,1))
    self.head.set_control("ctrl_stamp", np.array(self.trigger_timestamp).reshape(1,1)/1000000)
    self.head.write()
    self.cmd_log = cmd
```

it is writing the commanded joint velocities and the timestamp of the last trigger signal to the shared memory. This control command will be read by the robot, which is on the other side of the DGM architecture.

The `for` loop that controls the robot would then look like

```python
for i in range(100000):
    # control at 100 Hz
    time.sleep(0.01)

    # read states
    state = franka_dgh.read_states()
    q, dq, T = state

    # user defined controller
    vel = controller(time.time(), q, dq)  
    
    # send control command
    franka_dgh.cmd = vel
    franka_dgh.write_command(vel)
```

