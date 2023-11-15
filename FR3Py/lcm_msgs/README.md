# Franka LCM Message Definitions
We use [LCM library](http://lcm-proj.github.io/lcm/index.html) in order to interract with the robot through UDP multicast. Here, we define the custom message types that we use for various interfaces of the robot. 

## Joint Interface

Interfaces for commanding the robot in the joint space through torque and velocity commands. The message definition for this interface is franka_joint_interface.lcm and messages definitions for various programming languages can be generated as follows:

```bash
lcm-gen --python fr3_states.lcm #Python state type
lcm-gen --cpp fr3_states.lcm    #C++ state type

lcm-gen --python fr3_commands.lcm #Python state type
lcm-gen --cpp fr3_commands.lcm    #C++ state type
```