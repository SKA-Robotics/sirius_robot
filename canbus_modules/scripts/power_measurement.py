#!/usr/bin/python3
import rospy

from canbus_interface import CanbusInterface

from can_msgs.msg import Frame
from canbus_modules.msg import PowerStatus


class PowerMeasurementCanbus(CanbusInterface):
    def __init__(self) -> None:
        super().__init__(rospy.get_param("~device_id", 0x30))

        rospy.init_node("power_measurement_canbus")

        self.status_topic = rospy.get_param("~status_topic", "/power_status")

        self.resolution = rospy.get_param("~resolution", 18)
        self.voltage_multiplier = 2 * 2.048 / (1 << self.resolution)

        self.status_publisher = rospy.Publisher(
            self.status_topic, PowerStatus, queue_size=10
        )

        self.readings = [0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF]

    def run(self) -> None:
        rospy.spin()

    def receive_frame(self, command_id, data, frame: Frame):
        if command_id == 0:
            self.readings = [
                (data[index] << 8)
                | data[index + 1]
                for index in range(0, 8, 2)
            ]

        battery_voltage = self.readings[0] * 0.0280248834 + 17.9942457203
        battery_percentage = battery_voltage * 17.8571428571 - 400.0

        motor_current = self.readings[1] * 0.2075 - 63.08
        peripheral_current = self.readings[2] * 0.0638461538 - 12.8330769138
        computer_current = self.readings[3] * 0.0638461538 - 12.8330769138

        total_current = motor_current + computer_current + peripheral_current
        total_power = battery_voltage * total_current

        self.status_publisher.publish(
            PowerStatus(
                battery_voltage=battery_voltage,
                battery_percentage=battery_percentage,
                total_current=total_current,
                motor_current=motor_current,
                computer_current=computer_current,
                peripheral_current=peripheral_current,
                total_power=total_power,
            )
        )


if __name__ == "__main__":
    try:
        PowerMeasurementCanbus().run()
    except rospy.ROSInterruptException:
        pass
