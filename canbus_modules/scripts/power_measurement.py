#!/usr/bin/python3
import rospy
import struct

from canbus_interface import CanbusInterface

from can_msgs.msg import Frame
from canbus_modules.msg import PowerStatus


class PowerMeasurementCanbus(CanbusInterface):

    def __init__(self) -> None:
        rospy.init_node("power_measurement_canbus")

        self.status_topic = rospy.get_param("~status_topic", "/power_status")

        self.status_publisher = rospy.Publisher(self.status_topic,
                                                PowerStatus,
                                                queue_size=10)

        self.msg = PowerStatus()

        self.battery_capacity = 230  # Wh
        self.battery_max_voltage = 29  # V
        self.battery_min_voltage = 21  # V
        self.battery_disconnected_voltage = 15  # V

        self.battery1_energy_offset = 0
        self.battery1_connected = False
        self.battery2_energy_offset = 0
        self.battery2_connected = False

        self.received_messages = [False] * 4

        super().__init__(rospy.get_param("~device_id", 0x28))

    def run(self) -> None:
        rospy.spin()

    def estimate_battery_energy_from_voltage(self, voltage):
        voltage = min(self.battery_max_voltage,
                      max(self.battery_min_voltage, voltage))
        x = voltage
        energy = 0.02609842005534410000 * pow(x, 6) - 3.66634249899684 * pow(
            x, 5) + 213.8803382459820000 * pow(x, 4) - 6631.91686689481 * pow(
                x,
                3) + 115288.0636755470000 * pow(x, 2) - 1.06541212837601 * pow(
                    10, 6) * x + 4.089490766904380000 * pow(10, 6)
        energy = min(self.battery_capacity, max(0, energy))
        return energy

    def process_batteries(self):
        if self.msg.battery1_voltage < self.battery_disconnected_voltage:
            self.battery1_energy_offset = self.msg.battery1_energy
            self.battery1_connected = False
        elif self.battery1_connected is False and self.msg.battery1_energy - self.battery1_energy_offset > 0.01:
            self.battery1_energy_offset = self.msg.battery1_energy
            self.battery1_energy_offset -= self.battery_capacity - self.estimate_battery_energy_from_voltage(
                self.msg.battery1_voltage)
            self.battery1_connected = True

        if self.msg.battery2_voltage < self.battery_disconnected_voltage:
            self.battery1_energy_offset = self.msg.battery1_energy
            self.battery2_connected = False
        elif self.battery2_connected is False and self.msg.battery2_energy - self.battery2_energy_offset > 0.01:
            self.battery2_energy_offset = self.msg.battery2_energy
            self.battery2_energy_offset -= self.battery_capacity - self.estimate_battery_energy_from_voltage(
                self.msg.battery2_voltage)
            self.battery2_connected = True

        if self.battery1_connected:
            self.msg.battery1_percentage = 100 * (
                self.battery_capacity -
                (self.msg.battery1_energy -
                 self.battery1_energy_offset)) / self.battery_capacity
        else:
            self.msg.battery1_percentage = -float("inf")
        if self.battery2_connected:
            self.msg.battery2_percentage = 100 * (
                self.battery_capacity -
                (self.msg.battery2_energy -
                 self.battery2_energy_offset)) / self.battery_capacity
        else:
            self.msg.battery1_percentage = -float("inf")

    def bits_to_float(self, value):
        value_bits = struct.pack("I", value)
        return struct.unpack("f", value_bits)[0]

    def receive_frame(self, command_id, data, frame: Frame):
        self.received_messages[command_id] = True

        data = int.from_bytes(data, "big")
        if command_id == 0:
            self.msg.battery1_voltage = ((data >> 48) & 0xFFFF) / 1000
            self.msg.battery1_current = ((data >> 32) & 0xFFFF) / 1000
            #self.msg.battery1_energy = self.bits_to_float((data) & 0xFFFFFFFF) / 1000
            self.msg.battery1_energy = ((data) & 0xFFFFFFFF) / 1000
        if command_id == 1:
            self.msg.battery2_voltage = ((data >> 48) & 0xFFFF) / 1000
            self.msg.battery2_current = ((data >> 32) & 0xFFFF) / 1000
            #self.msg.battery2_energy = self.bits_to_float((data) & 0xFFFFFFFF) / 1000
            self.msg.battery2_energy = ((data) & 0xFFFFFFFF) / 1000
        if command_id == 2:
            self.msg.current3 = ((data >> 48) & 0xFFFF) / 1000
            self.msg.current2 = ((data >> 32) & 0xFFFF) / 1000
            self.msg.current1 = ((data >> 16) & 0xFFFF) / 1000
            self.msg.current0 = ((data) & 0xFFFF) / 1000
        if command_id == 3:
            self.msg.canbus_current = ((data >> 48) & 0xFFFF) / 1000
            self.msg.current6 = ((data >> 32) & 0xFFFF) / 1000
            self.msg.current5 = ((data >> 16) & 0xFFFF) / 1000
            self.msg.current4 = ((data) & 0xFFFF) / 1000

        if all(self.received_messages):
            self.process_batteries()

            self.status_publisher.publish(self.msg)
            self.msg = PowerStatus()
            self.received_messages = [False] * 4


if __name__ == "__main__":
    try:
        PowerMeasurementCanbus().run()
    except rospy.ROSInterruptException:
        pass
