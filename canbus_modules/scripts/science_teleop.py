#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from joystick_control.msg import Gamepad

JOYSTICK_DATA = {
    'axes': {
        'left_stick_horizontal': 0,
        'left_stick_vertical': 1,
        'right_stick_horizontal': 2,
        'right_stick_vertical': 3,
    },
    'buttons': {
        'A_button': 0,
        'B_button': 1,
        'X_button': 2,
        'Y_button': 3,
        'left_bumper': 4,
        'right_bumper': 5,
        'left_trigger': 6,
        'right_trigger': 7,
        'back_button': 8,
        'start_button': 9,
        'left_stick_button': 10,
        'right_stick_button': 11,
        'up_cross': 12,
        'down_cross': 13,
        'left_cross': 14,
        'right_cross': 15,
        'power_button': 16,
    }
}


class JoystickTranslator:
    def __init__(self):
        self.AXES_ID: dict = JOYSTICK_DATA['axes']
        self.BUTTONS_ID: dict = JOYSTICK_DATA['buttons']

    def translate(self, data: Gamepad) -> dict:
        inputs = {name: data.buttons[id]
                  for name, id in self.BUTTONS_ID.items()}
        inputs.update({name: data.axes[id]
                       for name, id in self.AXES_ID.items()})
        return inputs


class ScienceTeleopNode(Node):
    def __init__(self):
        super().__init__('science_teleop')

        self.joint_state = JointState()
        self.joint_state.name = ['module_lift', 'drill_lift', 'drill_spin']
        self.drill_cmd = 0.0
        self.gain = {
            'module_lift': 0.5,
            'drill_lift': 0.7,
        }

        self.joystick_translator = JoystickTranslator()

        self.cmd_publisher = self.create_publisher(
            JointState, '/science/command', 10)
        self.create_subscription(
            Gamepad, '/joy_5dof_manipulator',
            self._joy_callback, 10)

    def _joy_callback(self, msg: Gamepad):
        inp = self.joystick_translator.translate(msg)

        module_lift = -inp['left_stick_vertical'] * self.gain['module_lift']
        drill_lift = -inp['right_stick_vertical'] * self.gain['drill_lift']

        if inp['down_cross']:
            self.drill_cmd = -1.0
        elif inp['left_cross'] or inp['right_cross']:
            self.drill_cmd = 0.0
        elif inp['up_cross']:
            self.drill_cmd = 1.0

        if abs(module_lift) < 0.1:
            module_lift = 0.0
        if abs(drill_lift) < 0.1:
            drill_lift = 0.0

        self.joint_state.effort = [module_lift, drill_lift, self.drill_cmd]
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.cmd_publisher.publish(self.joint_state)


def main(args=None):
    rclpy.init(args=args)
    node = ScienceTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()