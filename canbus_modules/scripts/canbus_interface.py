import rospy

from abc import ABC, abstractmethod

from can_msgs.msg import Frame


class CanbusInterface(ABC):
    def __init__(self, device_id) -> None:
        super().__init__()

        self.device_id = device_id

        self.send_topic = rospy.get_param("~send_topic", "/sent_canbus_messages")
        self.receive_topic = rospy.get_param(
            "~recieve_topic", "/received_canbus_messages"
        )

        self.send_publisher = rospy.Publisher(self.send_topic, Frame, queue_size=10)
        self.receive_subscriber = rospy.Subscriber(
            self.receive_topic, Frame, self.receive_raw_frame
        )

    def send_frame(self, command_id, data, frame: Frame = None):
        if frame is None:
            frame = Frame()
            frame.id = (self.device_id << 5) | command_id
            frame.data = [data[i] if i < len(data) else 0 for i in range(8)]
            frame.dlc = len(data)

        self.send_publisher.publish(frame)

    def receive_raw_frame(self, frame: Frame):
        device_id = frame.id >> 5
        if self.device_id == device_id:
            command_id = frame.id & 0b11111
            self.receive_frame(command_id, frame.data, frame)

    @abstractmethod
    def receive_frame(self, command_id, data, frame: Frame):
        pass
