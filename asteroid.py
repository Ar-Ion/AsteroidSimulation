import numpy as np

import omni
from omni.isaac.core.prims import RigidPrim

from dynamics import DynamicObject

class Asteroid(DynamicObject):
    def __init__(self, prim_path):

        DynamicObject.__init__(self, prim_path)

        self._prim = RigidPrim(
            prim_path=prim_path, 
            name="asteroid",
            position=np.array([0.0, 0.0, 0.0]),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            scale=[1.0, 1.0, 1.0]
        )

        # Disable motion damping for the asteroid
        stage = omni.usd.get_context().get_stage()
        prim_ref = stage.GetPrimAtPath(prim_path)

        omni.kit.commands.execute("AddPhysicsComponentCommand", usd_prim=prim_ref, component="PhysicsRigidBodyAPI")
        omni.kit.commands.execute("AddPhysicsComponentCommand", usd_prim=prim_ref, component="PhysicsMassAPI")

        damping_attr = prim_ref.GetAttribute("physxRigidBody:angularDamping")
        damping_attr.Set(0.0)

        self._center_attr = prim_ref.GetAttribute("physics:centerOfMass")
        self._inertia_attr = prim_ref.GetAttribute("physics:diagonalInertia")
        self._axes_attr = prim_ref.GetAttribute("physics:principalAxes")

    def get_prim(self):
        return self._prim
    
    def setup(self):
        velocities = np.array((5, 0, 0))
        self._prim.set_angular_velocity(velocities)
        self.start_tf_publisher()