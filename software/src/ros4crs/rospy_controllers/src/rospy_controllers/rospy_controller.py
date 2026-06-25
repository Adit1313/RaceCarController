from abc import ABC, abstractmethod
from crs_msgs.msg import car_state_cart, car_input


class RosPyController(ABC):
    @abstractmethod
    def get_input(self, state: car_state_cart) -> car_input:
        pass

    @abstractmethod
    def publish_visualization(self):
        pass
