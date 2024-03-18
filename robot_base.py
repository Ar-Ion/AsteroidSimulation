import gc
from abc import abstractmethod

class Robot(object):
    def __init__(self):
        self._sensors = []

    def add_sensor(self, sensor):
        self._sensors.append(sensor)

    @abstractmethod
    def setup_robot(self):
        pass

    def start_robot(self):
        for sensor in self._sensors:
            sensor.start_sensor()

    def tick_robot(self):
        for sensor in self._sensors:
            sensor.tick_sensor()