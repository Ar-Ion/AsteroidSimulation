from robot_base import Robot
from sensors.camera import ROSCamera, StandardCamera

class Spacecraft(Robot):
    def setup_robot(self):
        self.add_sensor(StandardCamera(
            name="camera",
            resolution=(1024, 1024),
            fps=30
        ))