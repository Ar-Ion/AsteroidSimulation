import gc
from abc import abstractmethod

from omni.isaac.core import World
from omni.isaac.core.scenes.scene import Scene

from robot_base import Robot

class Environment(object):
    def __init__(self, world):
        self._world = world
        self._robots = []

    def get_world(self):
        return self._world

    def add_robot(self, robot):
        self._robots.append(robot)

    def load(self):
        self.setup_scene()
        
        for robot in self._robots:
            robot.setup_robot()

        self._world.reset()

        self.setup_post_load()

    def start(self):
        for robot in self._robots:
            robot.start_robot()

        self._world.initialize_physics()
        self._world.play()

    def reset(self):
        self.setup_pre_reset()
        self._world.reset()
        self.setup_post_reset()

    def clear(self):
        if self._world is not None:
            self._world_cleanup()
            self._world.clear_instance()
            self._world = None
            gc.collect()
        self.setup_post_clear()

    def tick(self):
        for robot in self._robots:
            robot.tick_robot()
            
    def get_data_payload(self):
        pass

    def _world_cleanup(self):
        self._world.stop()
        self._world.clear_all_callbacks()
        self.world_cleanup()

    @abstractmethod
    def setup_scene(self) -> None:
        """used to setup anything in the world, adding tasks happen here for instance.
        """
        return

    @abstractmethod
    def setup_post_load(self):
        """called after first reset of the world when pressing load,
        intializing provate variables happen here.
        """
        return

    @abstractmethod
    def setup_pre_reset(self):
        """called in reset button before resetting the world
        to remove a physics callback for instance or a controller reset
        """
        return

    @abstractmethod
    def setup_post_reset(self):
        """called in reset button after resetting the world which includes one step with rendering"""
        return

    @abstractmethod
    def setup_post_clear(self):
        """called after clicking clear button
        or after creating a new stage and clearing the instance of the world with its callbacks
        """
        return

    @abstractmethod
    def world_cleanup(self):
        """Function called when extension shutdowns and starts again, (hot reloading feature)"""
        return