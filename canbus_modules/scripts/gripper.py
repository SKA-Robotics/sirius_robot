#!/usr/bin/python3
import rospy

from canbus_interface import CanbusInterface

from can_msgs.msg import Frame
from std_msgs.msg import Float32, UInt8


class GripperCanbus(CanbusInterface):
    def __init__(self) -> None:
        super().__init__(rospy.get_param("~device_id", 0x31))

        rospy.init_node("gripper_canbus")

        self.cmd_topic = rospy.get_param("~status_topic", "/gripper_canbus/cmd")
        self.cmd_subscriber = rospy.Subscriber(
            self.cmd_topic, Float32, self.receive_cmd
        )
        self.measurement_topic = rospy.get_param(
            "~voltage_measurement_topic", "/voltage_measurement"
        )
        self.measurement_publisher = rospy.Publisher(
            self.measurement_topic, Float32, queue_size=10
        )

        self.open_pwm = rospy.get_param("~open_pwm", 1952)
        self.open_angle = rospy.get_param("~open_angle", 90)
        self.close_pwm = rospy.get_param("~close_pwm", 1024)
        self.close_angle = rospy.get_param("~close_angle", 0)
        self.max_angle = rospy.get_param("~max_angle", 90)
        self.min_angle = rospy.get_param("~mix_angle", -10)

    def run(self) -> None:
        rospy.spin()

    def receive_cmd(self, angle_msg: Float32):
        angle = angle_msg.data
        angle = min(angle, self.max_angle)
        angle = max(angle, self.min_angle)

        pwm = self.translate_angle(angle)
        data = [pwm >> 8, pwm & 0xFF]
        self.send_frame(0x0, data)

    def translate_angle(self, angle):
        return int(
            (
                (angle - self.open_angle)
                * (self.open_pwm - self.close_pwm)
                / (self.open_angle - self.close_angle)
            )
            + self.open_pwm
        )

    def receive_frame(self, command_id, data, frame: Frame):
        if command_id == 1:
            reading = (data[0] << 8) + data[1]
            voltage = 0.9225 * reading - 25.75
            self.measurement_publisher.publish(Float32(voltage))


if __name__ == "__main__":
    try:
        GripperCanbus().run()
    except rospy.ROSInterruptException:
        pass
