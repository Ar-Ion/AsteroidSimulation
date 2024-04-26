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

print("Loading ROS2 extension...")
extensions.enable_extension("omni.isaac.ros2_bridge")
extensions_utils.disable_extension(extension_name="omni.physx.flatcache")
simulation_app.update()

settings = { 
    "physics_dt": 1.0 / 30,
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
    "source": "/home/arion/AsteroidImageDataset"
}

dataset_size = 65536
mode = "train"

print("Loading frontend...")
#frontend_wrapped = TCPFrontend("127.0.0.1", 42666)
frontend_wrapped = factory.instance(frontend, mode, dataset_size)
frontend = AsyncFrontend(frontend_wrapped, AsyncFrontend.Modes.WAIT)
frontend.start()

print("Starting simulation...")
env.start()

for i in range(dataset_size):
    # start_time = datetime.now()
    # end_time = datetime.now()
    # print("Delay: " + str((end_time - start_time).total_seconds()*1000))
    
    env.randomize()
    env.tick()
    
    world.step()
    world.render()
    
    env_data = env.get_data_payload()
    robot_data = robot.get_data_payload()

    data = ImageData(env_data, robot_data)

    frontend.transmit(data)
    
    if i % 100 == 0:
        print("Generated " + f"{i/dataset_size:.0%}" + " of synthetic image data")    
        
simulation_app.close()