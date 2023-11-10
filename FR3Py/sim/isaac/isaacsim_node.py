# launch Isaac Sim before any other imports
# default first two lines in any standalone application
import sys

import numpy as np
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})  # we can also run as headless.

# import cv2
from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.prims import define_prim, get_prim_at_path

from FR3Py.sim.isaac.fr3 import FR3
from FR3Py.sim.isaac.utils import AnnotatorManager

import time

from FR3Py.sim.lcm_bridges import LCMBridgeServer
from FR3Py.lcm_msgs.fr3_commands import fr3_cmd
from FR3Py.sim.utils import simulationManager
from FR3Py.utils import NumpyMemMapDataPipe

import cv2
PHYSICS_DT = 1 / 100
RENDERING_DT = 1 / 100

world = World(physics_dt=PHYSICS_DT, rendering_dt=RENDERING_DT)

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    print("Could not find Isaac Sim assets folder")

# spawn warehouse scene
prim = get_prim_at_path("/World/Warehouse")
if not prim.IsValid():
    prim = define_prim("/World/Warehouse", "Xform")
    asset_path = (
        assets_root_path + "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd"
    )
    prim.GetReferences().AddReference(asset_path)

# spawn a FR3 robot
fr3 = world.scene.add(
    FR3(
        prim_path="/World/FR3",
        name="fr3",
        position=np.array([0, 0, 0.0]),
        physics_dt=PHYSICS_DT,
    )
)

world.reset()
fr3.initialize()

# Add cameras
ann = AnnotatorManager(world)
ann.registerCamera(
    "/World/FR3/fr3_hand",
    "rgb",
    (0.485, 0.0258, 0.028),
    (-90.0, 180.0, 90.0),
    resolution=(640, 480),
)

ann.registerAnnotator("rgb", "rgb")
ann.registerAnnotator("distance_to_camera", "rgb")
ann.setClippingRange("rgb", 0.2, 1000000.0)
ann.setFocalLength("rgb", 28)

lcm_server = LCMBridgeServer(robot_name="fr3")
cmd_stamp = time.time()
cmd_stamp_old = cmd_stamp

command = fr3_cmd()

command.cmd = np.array([
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ])

sim_manager = simulationManager(
    robot=fr3,
    lcm_server=lcm_server,
    default_cmd=command,
    physics_dt=PHYSICS_DT,
    lcm_timeout=1e-4,
)

rgb_data_pipe = NumpyMemMapDataPipe(
    "rgb", force=True, dtype="uint8", shape=(480, 640, 4)
)

depth_data_pipe = NumpyMemMapDataPipe(
    "depth", force=True, dtype="float32", shape=(480, 640)
)
counter = 0

while simulation_app.is_running():
    
    sim_manager.step(counter*PHYSICS_DT)
    # Step the world with rendering 50 times per second
    sim_manager.step(counter * PHYSICS_DT)
    if counter % 2 == 0:
        world.step(render=True)
        img = ann.getData("rgb:rgb")
        depth = ann.getData("rgb:distance_to_camera")
        if img.shape[0]:
            # cv2.imshow(f"rgb", img)
            # cv2.imshow(f"depth", depth)
            rgb_data_pipe.write(img, match_length=False)
            depth_data_pipe.write(depth, match_length=False)
        # cv2.waitKey(1)
    else:
        world.step(render=False)

    counter += 1
simulation_app.close()  # close Isaac Sim
