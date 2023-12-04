# launch Isaac Sim before any other imports
# default first two lines in any standalone application
import sys

import numpy as np
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})  # we can also run as headless.

import pickle
import time

# import cv2
from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.prims import define_prim, get_prim_at_path
from pxr import UsdGeom

from FR3Py.lcm_msgs.fr3_commands import fr3_cmd
from FR3Py.sim.isaac.fr3 import FR3
from FR3Py.sim.isaac.utils import AnnotatorManager, load_config
from FR3Py.sim.utils import LCMBridgeServer, simulationManager, NumpyMemMapDataPipe
import FR3Py

# load the simulation configs
cfg = load_config(
    FR3Py.FR3_ISAACSIM_CFG_PATH
)
robots = cfg["robots"]
cameras = cfg["cameras"]
env_cfg = cfg["environment"]

# create the world
world = World(
    physics_dt=cfg["environment"]["simulation_dt"],
    rendering_dt=cfg["environment"]["rendering_dt"],
)

# load the scenerendering_dt
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    print("Could not find Isaac Sim assets folder")

prim = get_prim_at_path(env_cfg["prim_path"])
if not prim.IsValid():
    prim = define_prim(env_cfg["prim_path"], "Xform")
    asset_path = (
        assets_root_path + env_cfg["usd_path"]
        if env_cfg["buildin"] is True
        else env_cfg["usd_path"]
    )
    prim.GetReferences().AddReference(asset_path)

# spawn table
if "tables" in cfg.keys():
    tables = cfg["tables"]
    prim = get_prim_at_path(tables[0]["prim_path"])
    if not prim.IsValid():
        prim = define_prim(tables[0]["prim_path"], "Xform")
        asset_path = (
            assets_root_path + tables[0]["usd_path"]
            if tables[0]["buildin"] is True
            else tables[0]["usd_path"]
        )
        prim.GetReferences().AddReference(asset_path)
        UsdGeom.Xformable(prim).AddTranslateOp().Set(
            (
                tables[0]["position"][0],
                tables[0]["position"][1],
                tables[0]["position"][2],
            )
        )

        UsdGeom.Xformable(prim).AddRotateXYZOp().Set(
            (
                tables[0]["orientation"][0],
                tables[0]["orientation"][1],
                tables[0]["orientation"][2],
            )
        )

# spawn a FR3 robot
fr3 = world.scene.add(
    FR3(
        prim_path=robots[0]["prim_path"],
        usd_path=(robots[0]["usd_path"] if robots[0]["usd_path"] !='' else None),
        name=robots[0]["name"],
        position=np.array(robots[0]["position"]),
        physics_dt=cfg["environment"]["simulation_dt"],
    )
)
world.reset()
fr3.initialize()

# Add cameras
print("Adding cameras")
ann = AnnotatorManager(world)

for camera in cameras:
    ann.registerCamera(
        camera["prim_path"],
        camera["name"],
        (0.485, 0.0258, 0.028),
        (-90.0, 180.0, 90.0),
        resolution=camera["resolution"],
    )

for camera in cameras:
    ann.registerAnnotator(camera["type"], camera["name"])
    ann.setClippingRange(camera["name"], 0.2, 1000000.0)
    ann.setFocalLength(camera["name"], 28)

# Add the shared memory data channels for image and LiDAR data
print("Creating shared memory data pipes")
camera_pipes = {}
for camera in cameras:
    if camera["type"] == "pointcloud":
        pipe = NumpyMemMapDataPipe(
            camera["name"] + "_" + camera["type"],
            force=True,
            dtype="float32",
            shape=(camera["resolution"][1] * camera["resolution"][0], 3),
        )
        camera_pipes[camera["name"] + "_" + camera["type"]] = pipe
    elif camera["type"] == "rgb":
        pipe = NumpyMemMapDataPipe(
            camera["name"] + "_" + camera["type"],
            force=True,
            dtype="uint8",
            shape=(camera["resolution"][1], camera["resolution"][0], 4),
        )
        camera_pipes[camera["name"] + "_" + camera["type"]] = pipe
    elif camera["type"] == "distance_to_camera":
        pipe = NumpyMemMapDataPipe(
            camera["name"] + "_" + camera["type"],
            force=True,
            dtype="uint8",
            shape=(camera["resolution"][1], camera["resolution"][0]),
        )
        camera_pipes[camera["name"] + "_" + camera["type"]] = pipe

# Store simulation hyperparamters in shared memory
print("Storing simulation hyperparamters in shared memory")

meta_data = {
    "camera_names": [camera["name"] for camera in cameras],
    "camera_types": [camera["type"] for camera in cameras],
    "camera_resolutions": [camera["resolution"] for camera in cameras],
    "camera_intrinsics": [
        ann.getCameraIntrinsics(camera["name"]) for camera in cameras
    ],
    "camera_extrinsics": [
        ann.getCameraExtrinsics(camera["name"]) for camera in cameras
    ],
}
with open("/dev/shm/fr3_sim_meta_data.pkl", "wb") as f:
    pickle.dump(meta_data, f)
# Start the LCM bridge server
print("Starting LCM bridge server")
lcm_server = LCMBridgeServer(robot_name="fr3")
cmd_stamp = time.time()
cmd_stamp_old = cmd_stamp

command = fr3_cmd()

command.cmd = np.array(
    [
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ]
)
# simulation runner
print("Starting simulation manager")
sim_manager = simulationManager(
    robot=fr3,
    lcm_server=lcm_server,
    default_cmd=command,
    physics_dt=cfg["environment"]["simulation_dt"],
    lcm_timeout=(0.1 if cfg["environment"]["synchronous_mode"] else 1e-4),
    mode=env_cfg["mode"],
)

# Start the simulation loop
print("Starting simulation loop...")
counter = 0
while simulation_app.is_running():
    sim_manager.step(counter * cfg["environment"]["simulation_dt"])

    if (
        counter
        % (cfg["environment"]["rendering_dt"] // cfg["environment"]["simulation_dt"])
        == 0
    ):
        world.step(render=True)
        # Push the sensor data to the shared memory pipes
        for camera in cameras:
            data = ann.getData(f"{camera['name']}:{camera['type']}")
            if camera["type"] == "pointcloud":
                payload = data["data"]
                if payload.shape[0]:
                    camera_pipes[camera["name"] + "_" + camera["type"]].write(
                        np.zeros(
                            (camera["resolution"][1] * camera["resolution"][0], 3)
                        ),
                        match_length=True,
                    )
                    camera_pipes[camera["name"] + "_" + camera["type"]].write(
                        payload, match_length=True
                    )
            else:
                if data.shape[0]:
                    camera_pipes[camera["name"] + "_" + camera["type"]].write(
                        data, match_length=False
                    )

    else:
        world.step(render=False)

    counter += 1
simulation_app.close()  # close Isaac Sim
