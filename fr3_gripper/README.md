# FR3 Gripper Python Interface

## Prerequisite 
We assume libfranka has been installed systemwide. If not, follow through the **Building libfranka** steps [here](https://frankaemika.github.io/docs/installation_linux.html) and at the end, do `sudo make install' to install the library system-wide. 

## Installation
Change to `FR3Py/fr3_gripper` and simply run the following to install the binding for the Python interpreter of interest:
```bash
python3 -m pip install .
```
## Testing

Now you can run the `demp.py` script. This script connects to the gripper, initializes it, and commands it to grab a 1cm-wide object. 
