# FR3Py
FR3Py is unified Python simulation and hardware communication environment for Franka FR3 robots.

## Installation
There are two packages that need to be installed:
### Python Interface
Simply install with pip in the root directory of the repository:
```bash
pip3 install .
```

### C++ Bridge
The C++ bridge communicates with the robot through the libfranka library. First install the libfranka according to the instructions provided [here](https://frankaemika.github.io/docs/installation_linux.html). Then follow these steps to compile and install the bridge from the root directory of this repository:

```
mkdir build && cd build
cmake ..
make -j $(( $(nproc) - 1 ))
make 
sudo make install
```

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

