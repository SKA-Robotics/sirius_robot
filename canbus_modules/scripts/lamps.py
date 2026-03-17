#!/usr/bin/env python3
import os
import sys
sys.path.insert(0, os.path.dirname(__file__))

import rclpy
from rclpy.node import Node

from can_msgs.msg import Frame
from std_msgs.msg import String

from joystick_control.msg import Topic
from canbus_interface import CanbusInterface

JOY_MULTIPLEXER_TOPIC = '/joy_multiplexer/selected_output'
RELAXING_MIDDLEWARE_TOPIC = '/relaxing_middleware/state'
KLAKSON_TOPIC = '/klakson/cmd'


class LampsNode(Node):
    def __init__(self):
        super().__init__('lamps_canbus')
        self.declare_parameter('device_id', 0x32)
        self.declare_parameter('send_topic', '/to_can_bus')
        self.declare_parameter('receive_topic', '/from_can_bus')

        device_id = self.get_parameter('device_id').value
        self._canbus = _LampsCanbus(device_id, self)

        self.lamp_state = {
            'blue': 0,
            'green': 0,
            'yellow': 0,
            'red': 1,
            'buzzer': 0,
        }
        self.is_color_overriden = False

        self.create_subscription(Topic, JOY_MULTIPLEXER_TOPIC,
                                 self._receive_joy_multiplexer_state, 10)
        self.create_subscription(String, RELAXING_MIDDLEWARE_TOPIC,
                                 self._receive_relaxing_middleware_state, 10)
        self.create_subscription(String, KLAKSON_TOPIC,
                                 self._receive_klakson_command, 10)
        self.create_subscription(String, '/lamps/color_override',
                                 self._receive_lamp_override, 10)

        # ~0.7 Hz
        self.create_timer(1.43, self._timer_callback)

    def _timer_callback(self):
        self._send_lamp_command()

    def _send_lamp_command(self):
        data = [
            self.lamp_state['blue'],
            self.lamp_state['green'],
            self.lamp_state['yellow'],
            self.lamp_state['red'],
            self.lamp_state['buzzer'],
        ]
        self._canbus.send_frame(0x0, data)

    def _receive_joy_multiplexer_state(self, msg: Topic):
        if self.is_color_overriden:
            return
        if msg.name == '__none':
            self.lamp_state['green'] = 0
            self.lamp_state['yellow'] = 0
        elif msg.name in ('joy_diff_drive', 'joy_5dof_manipulator'):
            self.lamp_state['green'] = 0
            self.lamp_state['yellow'] = 1
        elif msg.name == 'autonomy':
            self.lamp_state['green'] = 1
            self.lamp_state['yellow'] = 0
        self._send_lamp_command()

    def _receive_relaxing_middleware_state(self, msg: String):
        if self.is_color_overriden:
            return
        self.lamp_state['blue'] = 0 if msg.data == 'Idle' else 1
        self._send_lamp_command()

    def _receive_lamp_override(self, msg: String):
        self.is_color_overriden = True
        for key in ('blue', 'green', 'red', 'yellow'):
            self.lamp_state[key] = 0
        self.lamp_state[msg.data] = 1
        self._send_lamp_command()

    def _receive_klakson_command(self, msg: String):
        self.lamp_state['buzzer'] = 0 if msg.data == 'off' else 1
        self._send_lamp_command()


class _LampsCanbus(CanbusInterface):
    def receive_frame(self, command_id, data, frame: Frame):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = LampsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()