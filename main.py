from datetime import datetime

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({
    "renderer": "RayTracedLighting", 
    "headless": True
})

from omni.isaac.core import World
from omni.isaac.core.utils import extensions
import omni.isaac.core.utils.extensions as extensions_utils

from space_environment import SpaceEnvironment
from spacecraft import Spacecraft
from astronet_frontends import AsyncFrontend, factory
from astronet_msgs import ImageData
from matplotlib import pyplot as plt
import numpy as np
import time

print("Loading ROS2 extension...")
extensions.enable_extension("omni.isaac.ros2_bridge")
#extensions_utils.disable_extension(extension_name="omni.physx.flatcache")
simulation_app.update()

settings = { 
    "physics_dt": 1.0 / 60,
    "rendering_dt": 1.0 / 30,
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

frontend = {
    "type": "DriveServerFrontend",
    "path": "/home/arion/AsteroidImageDataset"
}

input_usd_dir = "/home/arion/AsteroidModelDataset/train"

dataset_size = 65536
mode = "train"

print("Loading frontend...")
#frontend_wrapped = TCPFrontend("127.0.0.1", 42666)
frontend_wrapped = factory.instance(frontend, mode, dataset_size)
frontend = AsyncFrontend(frontend_wrapped)
frontend.start()

print("Starting simulation...")
env.start()

world.reset() 

env_data = env.get_data_payload()

for i in range(dataset_size):
    # start_time = datetime.now()
    # end_time = datetime.now()
    # print("Delay: " + str((end_time - start_time).total_seconds()*1000))
    
    if i % 256 == 255:
        env.set_asteroid_usd(input_usd_dir + "/" + str(int(i/256)) + ".usdc")
        world.reset()
    
    if i % 2 == 1:
        env.randomize()
    else:
        env.random_walk(0.3)
        #env.rotate([0.2, 0, 0])
    
    env.tick()
        
    world.step(render=True)

    robot_data = robot.get_data_payload()
    data = ImageData(env_data, robot_data) # The actual rotation is always one frame behind for some reasons (bug?)
    env_data = env.get_data_payload()

    frontend.transmit(data)
            
    if i % 100 == 0:
        print("Generated " + f"{i/dataset_size:.0%}" + " of synthetic image data")    

frontend.stop()
simulation_app.close()