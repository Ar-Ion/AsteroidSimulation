from environment_base import Environment

import omni
import omni.usd
from omni.isaac.core import World, PhysicsContext
from omni.isaac.core.prims import RigidPrim
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
import omni.isaac.core.utils.prims as prim_utils

from pxr import Gf, Sdf, UsdPhysics
import carb
import numpy as np


class SpaceEnvironment(Environment):
    def __init__(self, world):
        super().__init__(world)

    def setup_scene(self):
        usd_path = "omniverse://localhost/Library/Asteroid.usdc"
        
        if usd_path is None:
            carb.log_error("Failed to load asteroid USD model")
        
        # Load USD to rigid body primitive
        add_reference_to_stage(usd_path=usd_path, prim_path="/World/Asteroid")
        
        self._asteroid_prim = RigidPrim(
            prim_path="/World/Asteroid", 
            name="asteroid",
            position=np.array([0.0, 0.0, 0.0]),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            scale=[1.0, 1.0, 1.0]
        )

        # Disable motion damping for the asteroid
        stage = omni.usd.get_context().get_stage()
        prim_ref = stage.GetPrimAtPath("/World/Asteroid")

        omni.kit.commands.execute("AddPhysicsComponentCommand", usd_prim=prim_ref, component="PhysicsRigidBodyAPI")
        omni.kit.commands.execute("AddPhysicsComponentCommand", usd_prim=prim_ref, component="PhysicsMassAPI")

        damping_attr = prim_ref.GetAttribute("physxRigidBody:angularDamping")
        damping_attr.Set(0.0)

        self._asteroid_center_attr = prim_ref.GetAttribute("physics:centerOfMass")
        self._asteroid_inertia_attr = prim_ref.GetAttribute("physics:diagonalInertia")
        self._asteroid_axes_attr = prim_ref.GetAttribute("physics:principalAxes")


        # Disable gravity
        self._physics_prim = PhysicsContext(prim_path="/World/physicsContext")
        self._physics_prim.set_gravity(0)

        # Setup lighting
        self._sun_prim = prim_utils.create_prim(
            "/World/Sun",
            "DistantLight",
            position=np.array([0.0, 0.0, 0.0]),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            attributes={
                "inputs:intensity": 5e3,
                "inputs:color": (1.0, 1.0, 1.0)
            }
        )

        # Add elements to scene
        self._world.scene.add(self._asteroid_prim)


    def setup_post_load(self):
        velocities = np.full((1, 3), fill_value=0.2)
        self._asteroid_prim.set_angular_velocity(velocities)

    def setup_pre_reset(self):
        return

    def setup_post_reset(self):
        return

    def world_cleanup(self):
        return
