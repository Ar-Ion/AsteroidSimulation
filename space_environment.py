import carb
import numpy as np

import omni.usd
from omni.isaac.core import PhysicsContext
from omni.isaac.core.utils.stage import add_reference_to_stage, clear_stage
import omni.isaac.core.utils.prims as prim_utils
from scipy.spatial.transform import Rotation
from pxr import Gf, Usd, UsdGeom
from omni.isaac.core.utils.transformations import pose_from_tf_matrix

from environment_base import Environment
from asteroid import Asteroid
from astronet_msgs import ImageData, Pose

class SpaceEnvironment(Environment):
    def __init__(self, world):
        super().__init__(world)
        self._R_global = np.array((1, 0, 0, 0))
        self._R_local = np.array((1, 0, 0, 0))

    def setup_scene(self):
        usd_path = "/home/arion/AsteroidModels/0.usdc"
        
        if usd_path is None:
            carb.log_error("Failed to load asteroid USD model")
        
        # Load USD to rigid body primitive
        asteroid_prim_path = "/World/asteroid"
        add_reference_to_stage(usd_path=usd_path, prim_path=asteroid_prim_path)
        self._asteroid = Asteroid(asteroid_prim_path)
        
        stage = omni.usd.get_context().get_stage()
        self._prim_ref = stage.GetPrimAtPath(asteroid_prim_path)

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
        
    def set_asteroid_usd(self, usd_path):
        refs = self._prim_ref.GetReferences()
        refs.ClearReferences()
        refs.AddReference(usd_path)

    def setup_post_load(self):
        self._asteroid.setup()

    def setup_pre_reset(self):
        return

    def setup_post_reset(self):
        return

    def world_cleanup(self):
        return

    def set_velocity(self, vel):
        self._asteroid.get_prim().set_angular_velocity(vel)
    
    # def randomize(self):
    #     velocities = np.random.uniform(low=-10.0, high=10.0, size=(3))
    #     self.set_velocity(velocities)
        
    def get_data_payload(self):
        (trans, rot) = self.get_world_pose()
        pose = Pose(trans, rot)
        return ImageData.EnvironmentData(pose)

    def get_world_pose(self):
        """Get a pose defined in the world frame from a pose defined relative to the frame of the prim at prim_path"""

        # Row-major transformation matrix from the prim's coordinate system to the world coordinate system
        #prim_transform_matrix = UsdGeom.Xformable(self._prim_ref).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

        # Convert transformation matrix to column-major
        #prim_to_world = np.transpose(prim_transform_matrix)

        # Translation and quaternion with respect to the world frame of the relatively defined pose
        # world_position, world_orientation = pose_from_tf_matrix(prim_to_world)
        world_position, world_orientation = self._asteroid.get_prim().get_world_pose()

        return world_position, world_orientation

    def rotate(self, angles):               
        diff_rotation = Rotation.from_euler('xyz', angles)
        prev_rotation = Rotation.from_quat(np.roll(self._R_local, -1))
        
        next_rotation = prev_rotation * diff_rotation
        
        self._R_local = np.roll(next_rotation.as_quat(), +1)
        
        self.update_orientation()

    def random_walk(self, max_angle):        
        euler_angles = np.random.uniform(low=-max_angle, high=max_angle, size=(3))
       
        diff_rotation = Rotation.from_euler('xyz', euler_angles)
        prev_rotation = Rotation.from_quat(np.roll(self._R_local, -1))
        
        next_rotation = prev_rotation * diff_rotation
        
        self._R_local = np.roll(next_rotation.as_quat(), +1)
        
        self.update_orientation()

    def randomize(self):
        euler_angles = np.random.uniform(low=-6.28, high=6.28, size=(3))
        rotation = Rotation.from_euler('xyz', euler_angles)
        self._R_global = np.roll(rotation.as_quat(), +1)
        self.update_orientation()

    def update_orientation(self):
        global_rotation = Rotation.from_quat(np.roll(self._R_global, -1))
        local_rotation = Rotation.from_quat(np.roll(self._R_local, -1))
        
        total_rotation = local_rotation * global_rotation
        
        quaternion = np.roll(total_rotation.as_quat(), +1)
        
        self._asteroid.get_prim().set_world_pose(orientation=quaternion)
        