import omni.replicator.core as rep
from pxr import UsdGeom
import yaml
from omni.isaac.core.utils.prims import define_prim, get_prim_at_path


def load_config(file_path):
    with open(file_path, 'r') as file:
        config = yaml.safe_load(file)
    return config

class AnnotatorManager:
    def __init__(self, world):
        self.annotators = {}
        self.render_products = {}
        self.cameras = {}
        self.annotator_types = [
            "rgb",
            "distance_to_camera",
            "pointcloud",
            "semantic_segmentation",
        ]
        self.camera_info = {}
        self.world = world

    def registerCamera(
        self, parent_prim, name, translation, orientation, resolution=(1024, 600)
    ):
        prim = get_prim_at_path(parent_prim + "/" + name)
        if (name not in self.camera_info.keys()) and (not prim.IsValid()):
            self.camera_info[name] = {
                "translation": translation,
                "orientation": orientation,
                "resolution": resolution,
            }
            self.cameras[name] = self.world.stage.DefinePrim(
                parent_prim + "/" + name, "Camera"
            )
            
            UsdGeom.Xformable(self.cameras[name]).AddTranslateOp().Set(translation)
            UsdGeom.Xformable(self.cameras[name]).AddRotateXYZOp().Set(orientation)
            self.render_products[name] = rep.create.render_product(
                str(self.cameras[name].GetPrimPath()), resolution
            )

    def registerAnnotator(self, type, camera_name):
        assert (
            type in self.annotator_types
        ), "Requested replicator type not available. Choose from: " + str(
            self.annotator_types
        )
        assert (
            camera_name in self.cameras.keys()
        ), "The requested camera is not registered. Please register the camera first."
        self.annotators[camera_name + ":" + type] = rep.AnnotatorRegistry.get_annotator(
            type
        )
        self.annotators[camera_name + ":" + type].attach(
            [self.render_products[camera_name]]
        )

    def getData(self, annotator_name):
        assert (
            annotator_name in self.annotators.keys()
        ), "The requested annotator is not registered. Please register the annotator first."
        return self.annotators[annotator_name].get_data()

    def getStreams(self):
        return list(self.annotators.keys())

    def setFocalLength(self, name, value):
        assert (
            name in self.cameras.keys()
        ), "The requested camera is not registered. Please register the camera first."
        self.cameras[name].GetAttribute("focalLength").Set(value)

    def setClippingRange(self, name, min=0.2, max=1000000.0):
        assert (
            name in self.cameras.keys()
        ), "The requested camera is not registered. Please register the camera first."
        self.cameras[name].GetAttribute("clippingRange").Set((min, max))
    # TODO: implement these functions
    def getCameraIntrinsics(self, name):
        return None
    
    def getCameraExtrinsics(self, name):
        return None
