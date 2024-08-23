#!/usr/bin/python3
import rospy

from canbus_interface import CanbusInterface

from can_msgs.msg import Frame
from std_msgs.msg import String
from joystick_control.msg import Topic

JOY_MULTIPLEXER_TOPIC = "/joy_multiplexer/selected_output"
RELAXING_MIDDLEWARE_TOPIC = "/relaxing_middleware/state"
KLAKSON_TOPIC = "/klakson/cmd"
OVERRIDE_TOPIC = "/lamps/override_color"


class LampsCanbus(CanbusInterface):

    def __init__(self) -> None:
        super().__init__(rospy.get_param("~device_id", 0x32))
        rospy.init_node("lamps_canbus")
        self.lamp_state = {
            "blue": 0,
            "green": 0,
            "yellow": 0,
            "red": 1,
            "buzzer": 0,
        }
        self.is_color_overriden = False

        self.joy_multiplexer_state = rospy.Subscriber(
            JOY_MULTIPLEXER_TOPIC,
            Topic,
            self.receive_joy_multiplexer_state,
            queue_size=10)

        self.relaxing_middleware_state = rospy.Subscriber(
            RELAXING_MIDDLEWARE_TOPIC,
            String,
            self.receive_relaxing_middleware_state,
            queue_size=10)

        self.klakson_subscriber = rospy.Subscriber(
            KLAKSON_TOPIC, String, self.receive_klakson_command, queue_size=10)

        self.lamp_override_subscriber = rospy.Subscriber(
            "/lamps/color_override",
            String,
            self.receive_lamp_override,
            queue_size=10)

        self.rate = rospy.Rate(0.7)

    def run(self) -> None:
        while not rospy.is_shutdown():
            self.send_lamp_command()
            self.rate.sleep()

    def send_lamp_command(self):
        data = [0, 0, 0, 0, 0]
        data[0] = self.lamp_state["blue"]
        data[1] = self.lamp_state["green"]
        data[2] = self.lamp_state["yellow"]
        data[3] = self.lamp_state["red"]
        data[4] = self.lamp_state["buzzer"]
        self.send_frame(0x0, data)

    def receive_joy_multiplexer_state(self, msg):
        if self.is_color_overriden:
            return
        if msg.name == "__none":
            self.lamp_state["green"] = 0
            self.lamp_state["yellow"] = 0
        elif msg.name == "joy_diff_drive" or msg.name == "joy_5dof_manipulator":
            self.lamp_state["green"] = 0
            self.lamp_state["yellow"] = 1
        elif msg.name == "autonomy":
            self.lamp_state["green"] = 1
            self.lamp_state["yellow"] = 0
        self.send_lamp_command()

    def receive_relaxing_middleware_state(self, msg):
        if self.is_color_overriden:
            return
        if msg.data == "Idle":
            self.lamp_state["blue"] = 0
        else:
            self.lamp_state["blue"] = 1
        self.send_lamp_command()

    def receive_lamp_override(self, msg):
        self.is_color_overriden = True
        self.lamp_state["blue"] = 0
        self.lamp_state["green"] = 0
        self.lamp_state["red"] = 0
        self.lamp_state["yellow"] = 0
        self.lamp_state[msg.data] = 1
        self.send_lamp_command()

    def receive_klakson_command(self, msg):
        if msg.data == "off":
            self.lamp_state["buzzer"] = 0
        else:
            self.lamp_state["buzzer"] = 1
        self.send_lamp_command()

    def receive_frame(self, command_id, data, frame: Frame):
        pass


if __name__ == "__main__":
    try:
        LampsCanbus().run()
    except rospy.ROSInterruptException:
        pass
