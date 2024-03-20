from robot_base import Robot
from Sensors.camera import ROSCamera

class Spacecraft(Robot):
    def setup_robot(self):
        self.add_sensor(ROSCamera(
            name="camera",
            resolution=(1024, 1024),
            fps=60
        ))