#launch Isaac Sim before any other imports
#default first two lines in any standalone application


from environment_base import Environment

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid

import numpy as np


class SpaceEnvironment(Environment):
    def __init__(self, world):
        super().__init__(world)

    def setup_scene(self):

        self._world.scene.add_default_ground_plane()

        fancy_cube = self._world.scene.add(
            DynamicCuboid(
                prim_path="/World/random_cube", # The prim path of the cube in the USD stage
                name="fancy_cube", # The unique name used to retrieve the object from the scene later on
                position=np.array([0, 0, 1.0]), # Using the current stage units which is in meters by default.
                scale=np.array([0.5015, 0.5015, 0.5015]), # most arguments accept mainly numpy arrays.
                color=np.array([0, 0, 1.0]), # RGB channels, going from 0-1
            )
        )

    def setup_post_load(self):
        self._world = self.get_world()
        self._cube = self._world.scene.get_object("fancy_cube")
        self._world.add_physics_callback("sim_step", callback_fn=self.print_cube_info)

    def print_cube_info(self, step_size):
        position, orientation = self._cube.get_world_pose()
        linear_velocity = self._cube.get_linear_velocity()
        # will be shown on terminal
        #print("Cube position is : " + str(position))
        #print("Cube's orientation is : " + str(orientation))
        #print("Cube's linear velocity is : " + str(linear_velocity))

    def setup_pre_reset(self):
        return

    def setup_post_reset(self):
        return

    def world_cleanup(self):
        return
