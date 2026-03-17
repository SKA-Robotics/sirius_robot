#!/usr/bin/env python3
import os
import sys
sys.path.insert(0, os.path.dirname(__file__))

import rclpy
from rclpy.node import Node

from can_msgs.msg import Frame
from std_msgs.msg import Float32, Empty

from canbus_interface import CanbusInterface


class GripperNode(Node):
    def __init__(self):
        super().__init__('gripper')
        self.declare_parameter('device_id', 0x31)
        self.declare_parameter('send_topic', '/to_can_bus')
        self.declare_parameter('receive_topic', '/from_can_bus')

        device_id = self.get_parameter('device_id').value
        self._canbus = _GripperCanbus(device_id, self)

        self.create_subscription(Float32, '~/set_force',
                                 self._receive_force_command, 10)
        self.create_subscription(Empty, '~/open_trigger',
                                 self._receive_open_trigger, 10)

        self.setpoint = None
        self.create_timer(0.2, self._timer_callback)

    def _timer_callback(self):
        if self.setpoint is not None:
            pwm = self.setpoint
            data = [pwm >> 8, pwm & 0xFF]
            self._canbus.send_frame(0x1, data)

    def _receive_force_command(self, msg: Float32):
        self.setpoint = int(min(1023, max(0, msg.data * 1023)))

    def _receive_open_trigger(self, msg: Empty):
        self._canbus.send_frame(0x0, [0])
        self.setpoint = None


class _GripperCanbus(CanbusInterface):
    def receive_frame(self, command_id, data, frame: Frame):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()