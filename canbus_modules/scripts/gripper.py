#!/usr/bin/python3
import rospy

from canbus_interface import CanbusInterface

from can_msgs.msg import Frame
from std_msgs.msg import Float32, Empty

FORCE_COMMAND_TOPIC = "~set_force"
OPEN_TRIGGER_TOPIC = "~open_trigger"


class GripperCanbus(CanbusInterface):

    def __init__(self) -> None:
        super().__init__(rospy.get_param("~device_id", 0x31))
        rospy.init_node("gripper")
        self.force_command_subscriber = rospy.Subscriber(
            FORCE_COMMAND_TOPIC, Float32, self.receive_force_command)
        self.open_trigger_subscriber = rospy.Subscriber(
            OPEN_TRIGGER_TOPIC, Empty, self.receive_open_trigger)

    def run(self) -> None:
        rospy.spin()

    def receive_force_command(self, msg: Float32):
        pwm = int(min(1023, max(0, msg.data * 1023)))
        data = [pwm >> 8, pwm & 0xFF]
        self.send_frame(0x1, data)

    def receive_open_trigger(self, msg: Empty):
        self.send_frame(0x0, [0])

    def receive_frame(self, command_id, data, frame: Frame):
        pass


if __name__ == "__main__":
    try:
        GripperCanbus().run()
    except rospy.ROSInterruptException:
        pass
