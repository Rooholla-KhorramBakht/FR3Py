# FR3Py
FR3Py is unified Python simulation and hardware communication environment for Franka FR3 robots.

## Installation
### Prerequisite 
We assume libfranka has been installed systemwide. If not, follow through the **Building libfranka** steps [here](https://frankaemika.github.io/docs/installation_linux.html) and at the end, do `sudo make install to install the library system-wide. 

### FR3Py
Simply install with pip in the root directory of the repository:
```bash
python3 pip3 install .
```

### C++ Bridge
Compile and install the bridge as follows:

```bash
cd fr3_bridge
mkdir build && cd build
cmake ..
make -j $(( $(nproc) - 1 ))
make 
sudo make install
```

## Gripper Python Bindings
To control the gripper with python install the `fr3_gripper` as instructed [here](fr3_gripper/README.md).

## How to Start
### Step1 
Run the C++ node to communicate with the robot: 

```bash
fr3_joint_interface <robot_ip> <robot_name> <interface_type>
```
where `robot_name` is a unique name used to determine the name of the LCM topics used for the robot with IP `robot_ip` and `interface_type` determines how we want to command the robot. Currently, `torque` and `velocity` are supported.

### Step2 (Optional)
In case you want to access the interface through a python environment hosted by another computer on the same network, you need to enable multicast support for the particular interface used for connecting to the local network. To do so, first get the name of the interfcace by running:
```bash
ifconfig
```
and use it to run the `unicast_config.py` file in the `tools` directory of this repository. This script much be executed with sudo privileges:
```bash
sudo python3 tools/unicast_config.py <interface_name> # e.g. eth0
```

