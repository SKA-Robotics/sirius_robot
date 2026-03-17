#!/usr/bin/env python3
import os
import sys
sys.path.insert(0, os.path.dirname(__file__))

import struct

import rclpy
from rclpy.node import Node

from can_msgs.msg import Frame
from canbus_modules.msg import PowerStatus

from canbus_interface import CanbusInterface


class PowerMeasurementNode(Node):
    def __init__(self):
        super().__init__('power_measurement_canbus')
        self.declare_parameter('device_id', 0x28)
        self.declare_parameter('status_topic', '/power_status')
        self.declare_parameter('send_topic', '/to_can_bus')
        self.declare_parameter('receive_topic', '/from_can_bus')

        status_topic = self.get_parameter('status_topic').value
        device_id = self.get_parameter('device_id').value

        self.status_publisher = self.create_publisher(
            PowerStatus, status_topic, 10)

        self._canbus = _PowerMeasurementCanbus(device_id, self,
                                               self.status_publisher)


class _PowerMeasurementCanbus(CanbusInterface):
    def __init__(self, device_id: int, node: Node, status_publisher):
        super().__init__(device_id, node)
        self._status_publisher = status_publisher
        self._node = node

        self.msg = PowerStatus()
        self.battery_capacity = 230       # Wh
        self.battery_max_voltage = 29     # V
        self.battery_min_voltage = 21     # V
        self.battery_disconnected_voltage = 15  # V

        self.battery1_energy_offset = 0.0
        self.battery1_connected = False
        self.battery2_energy_offset = 0.0
        self.battery2_connected = False

        self.received_messages = [False] * 4

    def _estimate_battery_energy_from_voltage(self, voltage: float) -> float:
        voltage = min(self.battery_max_voltage,
                      max(self.battery_min_voltage, voltage))
        x = voltage
        energy = (0.02609842005534410000 * x**6
                  - 3.66634249899684 * x**5
                  + 213.8803382459820000 * x**4
                  - 6631.91686689481 * x**3
                  + 115288.0636755470000 * x**2
                  - 1.06541212837601e6 * x
                  + 4.089490766904380000e6)
        return min(self.battery_capacity, max(0.0, energy))

    def _process_batteries(self):
        msg = self.msg
        if msg.battery1_voltage < self.battery_disconnected_voltage:
            self.battery1_energy_offset = msg.battery1_energy
            self.battery1_connected = False
        elif not self.battery1_connected and \
                msg.battery1_energy - self.battery1_energy_offset > 0.01:
            self.battery1_energy_offset = msg.battery1_energy
            self.battery1_energy_offset -= (
                self.battery_capacity -
                self._estimate_battery_energy_from_voltage(
                    msg.battery1_voltage))
            self.battery1_connected = True

        if msg.battery2_voltage < self.battery_disconnected_voltage:
            self.battery2_energy_offset = msg.battery2_energy
            self.battery2_connected = False
        elif not self.battery2_connected and \
                msg.battery2_energy - self.battery2_energy_offset > 0.01:
            self.battery2_energy_offset = msg.battery2_energy
            self.battery2_energy_offset -= (
                self.battery_capacity -
                self._estimate_battery_energy_from_voltage(
                    msg.battery2_voltage))
            self.battery2_connected = True

        if self.battery1_connected:
            msg.battery1_percentage = 100.0 * (
                self.battery_capacity -
                (msg.battery1_energy - self.battery1_energy_offset)
            ) / self.battery_capacity
        else:
            msg.battery1_percentage = float('-inf')

        if self.battery2_connected:
            msg.battery2_percentage = 100.0 * (
                self.battery_capacity -
                (msg.battery2_energy - self.battery2_energy_offset)
            ) / self.battery_capacity
        else:
            msg.battery2_percentage = float('-inf')

    @staticmethod
    def _bits_to_float(value: int) -> float:
        return struct.unpack('f', struct.pack('I', value))[0]

    def receive_frame(self, command_id: int, data, frame: Frame):
        self.received_messages[command_id] = True
        raw = int.from_bytes(bytes(data), 'big')

        if command_id == 0:
            self.msg.battery1_voltage = ((raw >> 48) & 0xFFFF) / 1000
            self.msg.battery1_current = ((raw >> 32) & 0xFFFF) / 1000
            self.msg.battery1_energy = (raw & 0xFFFFFFFF) / 1000
        elif command_id == 1:
            self.msg.battery2_voltage = ((raw >> 48) & 0xFFFF) / 1000
            self.msg.battery2_current = ((raw >> 32) & 0xFFFF) / 1000
            self.msg.battery2_energy = (raw & 0xFFFFFFFF) / 1000
        elif command_id == 2:
            self.msg.current3 = ((raw >> 48) & 0xFFFF) / 1000
            self.msg.current2 = ((raw >> 32) & 0xFFFF) / 1000
            self.msg.current1 = ((raw >> 16) & 0xFFFF) / 1000
            self.msg.current0 = (raw & 0xFFFF) / 1000
        elif command_id == 3:
            self.msg.canbus_current = ((raw >> 48) & 0xFFFF) / 1000
            self.msg.current6 = ((raw >> 32) & 0xFFFF) / 1000
            self.msg.current5 = ((raw >> 16) & 0xFFFF) / 1000
            self.msg.current4 = (raw & 0xFFFF) / 1000

        if all(self.received_messages):
            self._process_batteries()
            self._status_publisher.publish(self.msg)
            self.msg = PowerStatus()
            self.received_messages = [False] * 4


def main(args=None):
    rclpy.init(args=args)
    node = PowerMeasurementNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()