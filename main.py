from datetime import datetime

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({
    "renderer": "RayTracedLighting", 
    "headless": True
})

from omni.isaac.core import World
from omni.isaac.core.utils import extensions

from space_environment import SpaceEnvironment
from spacecraft import Spacecraft
from frontend import AsyncFrontend, TCPFrontend, DatasetFrontend

print("Loading ROS2 extension...")
extensions.enable_extension("omni.isaac.ros2_bridge")
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

print("Loading frontend...")
#frontend_wrapped = TCPFrontend("127.0.0.1", 42666)
frontend_wrapped = DatasetFrontend("/home/arion/AsteroidDataset/train", 70000)
frontend = AsyncFrontend(frontend_wrapped)
frontend.start()

print("Starting simulation...")
env.start()

while simulation_app.is_running():
    env.randomize()
    env.tick()

    # start_time = datetime.now()
    world.step(render=True)
    # end_time = datetime.now()
    # print("Delay: " + str((end_time - start_time).total_seconds()*1000))

    robot_data = robot.get_data_payload()
    env_data = env.get_data_payload()
    
    data = (env_data, robot_data)

    frontend.feed_data(data)
    
simulation_app.close()