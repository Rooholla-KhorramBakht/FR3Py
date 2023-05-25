# `dgm_franka` & `dgh_franka` installation

After following the installation of the prerequisites in the readme, we can now install `dgm_franka` and `dgh_franka`. 

## Install `dgm_franka`

First, clone the repo

```console
git clone https://github.com/BolunDai0216/dgm_franka

```

Install `lcm` and the corresponding Python package

```
git clone https://github.com/lcm-proj/lcm.git 
cd lcm && mkdir build && cd build 
cmake .. && cmake --build .
sudo cmake --install . 
```

This will install the python package at `/usr/local/lib/python3.<x>/site-packages`. Add this to you `PYTHONPATH` environment variable by running

```console
export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python3.<x>/site-packages
```

You can test this installation by seeing if `lcm` can be imported in Python. Then, go back to the parent directory and build the package. But before that, we need to source the `dgm-ws` we created before

```console
cd dgm_franka
source ~/dgm-ws/install/setup.bash
mkdir build && cd build
cmake ..
cmake --build .
```

Now that the package is built, we can install it anywhere we want

```console
cmake --install . --prefix "<install-dir>/install"
```

Finally, add the path to `dynamic_graph_manager_cpp_bindings` to `PYTHONPATH`, it should be located at `~/dgm-ws/install/dynamic_graph_manager/lib/python3.8/site-packages`.

## Install `dgh_franka`

Now that `dgm_franka` is installed, we also need to install the dynamic graph head (DGH) to use it from Python. First, we also need to clone the repo

```console
git clone https://github.com/BolunDai0216/dgh_franka.git
cd dgh_franka
```

Since this is a python package, we just need to install it

```console
python3 -m pip install -e .
```

The next step is to run the executable in the `install` folder of `dgm_franka` and then control the robot from the Python API.

