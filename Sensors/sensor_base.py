from abc import abstractmethod

class Sensor(object):

    @abstractmethod
    def start_sensor(self):
        return

    @abstractmethod
    def tick_sensor(self):
        return