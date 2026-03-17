from abc import ABC, abstractmethod

from rclpy.node import Node
from can_msgs.msg import Frame


class CanbusInterface(ABC):
    def __init__(self, device_id: int, node: Node) -> None:
        super().__init__()
        self.device_id = device_id
        self._node = node

        send_topic = node.get_parameter('send_topic').value \
            if node.has_parameter('send_topic') else '/to_can_bus'
        receive_topic = node.get_parameter('receive_topic').value \
            if node.has_parameter('receive_topic') else '/from_can_bus'

        self.send_publisher = node.create_publisher(Frame, send_topic, 10)
        self.receive_subscriber = node.create_subscription(
            Frame, receive_topic, self._receive_raw_frame, 10)

    def send_frame(self, command_id: int, data: list, frame: Frame = None):
        if frame is None:
            frame = Frame()
            frame.id = (self.device_id << 5) | command_id
            frame.data = [data[i] if i < len(data) else 0 for i in range(8)]
            frame.dlc = len(data)
        self.send_publisher.publish(frame)

    def _receive_raw_frame(self, frame: Frame):
        device_id = frame.id >> 5
        if self.device_id == device_id:
            command_id = frame.id & 0b11111
            self.receive_frame(command_id, frame.data, frame)

    @abstractmethod
    def receive_frame(self, command_id: int, data, frame: Frame):
        pass