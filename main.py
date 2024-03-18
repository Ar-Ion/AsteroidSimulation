import asyncio
from threading import Event

import carb
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({
    "renderer": "RayTracedLighting", 
    "headless": True
})

from omni.isaac.core import World
from omni.isaac.core.utils import extensions, nucleus, stage

from space_environment import SpaceEnvironment
from spacecraft import Spacecraft

print("Loading ROS2 extension...")
extensions.enable_extension("omni.isaac.ros2_bridge")
simulation_app.update()

settings = { 
    "physics_dt": 1.0 / 60,
    "rendering_dt": 1.0 / 60,
    "stage_units_in_meters": 1.0 
}

print("Loading world...")
world = World(**settings)

print("Loading robot...")
robot = Spacecraft()

print("Loading environment...")
env = SpaceEnvironment(world)
env.add_robot(robot)
env.load()

print("Updating app...")
simulation_app.update()

print("Starting simulation...")
env.start()

while simulation_app.is_running():
    world.step()
    env.tick()

simulation_app.close()