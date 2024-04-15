import carb
import numpy as np

import omni.usd
from omni.isaac.core import PhysicsContext
from omni.isaac.core.utils.stage import add_reference_to_stage
import omni.isaac.core.utils.prims as prim_utils
from scipy.spatial.transform import Rotation

from environment_base import Environment
from asteroid import Asteroid

class SpaceEnvironment(Environment):
    def __init__(self, world):
        super().__init__(world)

    def setup_scene(self):
        usd_path = "omniverse://localhost/Library/Asteroid.usdc"
        
        if usd_path is None:
            carb.log_error("Failed to load asteroid USD model")
        
        # Load USD to rigid body primitive
        asteroid_prim_path = "/World/asteroid"
        add_reference_to_stage(usd_path=usd_path, prim_path=asteroid_prim_path)
        self._asteroid = Asteroid(asteroid_prim_path)

        # Disable gravity
        self._physics_prim = PhysicsContext(prim_path="/World/physicsContext")
        self._physics_prim.set_gravity(0)

        # Setup lighting
        self._sun_prim = prim_utils.create_prim(
            "/World/sun",
            "DistantLight",
            position=np.array([0.0, 0.0, 0.0]),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            attributes={
                "inputs:intensity": 5e3,
                "inputs:color": (1.0, 1.0, 1.0)
            }
        )

        # Add elements to scene
        self._world.scene.add(self._asteroid.get_prim())

    def setup_post_load(self):
        self._asteroid.setup()

    def setup_pre_reset(self):
        return

    def setup_post_reset(self):
        return

    def world_cleanup(self):
        return
    
    def randomize(self):
        random_quaternion = Rotation.random().as_quat()
        self._asteroid.get_prim().set_world_pose(orientation=random_quaternion)
        
    def get_data_payload(self):
        return self._asteroid.get_prim().get_world_pose()
