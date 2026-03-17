#!/usr/bin/env python3
import sys
import os

import rclpy
from rclpy.node import Node

from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5 import uic

from std_msgs.msg import Int32


class CameraGuiNode(Node):
    def __init__(self):
        super().__init__('camera_gui')
        self._publishers = {}

    def get_publisher(self, topic: str):
        if topic not in self._publishers:
            self._publishers[topic] = self.create_publisher(Int32, topic, 10)
        return self._publishers[topic]


class Ui_Frame(QMainWindow):
    def __init__(self, node: CameraGuiNode):
        super().__init__()
        self._node = node
        self.cameras_positions = {
            'left': 0, 'middle': 0, 'right': 0, 'manip': 0
        }

        # Find UI file relative to share directory
        from ament_index_python.packages import get_package_share_directory
        ui_path = os.path.join(
            get_package_share_directory('canbus_modules'),
            'ui', 'camera_interface.ui')
        uic.loadUi(ui_path, self)

        # left camera
        self.left_camera_left_angle.clicked.connect(
            lambda: self.send_message('left', 20, 'relative'))
        self.left_camera_front_wheels.clicked.connect(
            lambda: self.send_message('left', -45))
        self.left_camera_front.clicked.connect(
            lambda: self.send_message('left', -90))
        self.left_camera_back.clicked.connect(
            lambda: self.send_message('left', 90))
        self.left_camera_back_wheels.clicked.connect(
            lambda: self.send_message('left', 0))
        self.left_camera_right_angle.clicked.connect(
            lambda: self.send_message('left', -20, 'relative'))
        # right camera
        self.right_camera_left_angle.clicked.connect(
            lambda: self.send_message('right', 20, 'relative'))
        self.right_camera_front_wheels.clicked.connect(
            lambda: self.send_message('right', 45))
        self.right_camera_front.clicked.connect(
            lambda: self.send_message('right', 90))
        self.right_camera_back.clicked.connect(
            lambda: self.send_message('right', -90))
        self.right_camera_back_wheels.clicked.connect(
            lambda: self.send_message('right', 0))
        self.right_camera_right_angle.clicked.connect(
            lambda: self.send_message('right', -20, 'relative'))
        # middle camera
        self.middle_camera_left_angle.clicked.connect(
            lambda: self.send_message('middle', 20, 'relative'))
        self.middle_camera_left.clicked.connect(
            lambda: self.send_message('middle', 90))
        self.middle_camera_front.clicked.connect(
            lambda: self.send_message('middle', 0))
        self.middle_camera_back.clicked.connect(
            lambda: self.send_message('middle', 180))
        self.middle_camera_right.clicked.connect(
            lambda: self.send_message('middle', -90))
        self.middle_camera_right_angle.clicked.connect(
            lambda: self.send_message('middle', -20, 'relative'))
        # manip camera
        self.manip_camera_left_angle.clicked.connect(
            lambda: self.send_message('manip', 20, 'relative'))
        self.manip_camera_left.clicked.connect(
            lambda: self.send_message('manip', 90))
        self.manip_camera_front.clicked.connect(
            lambda: self.send_message('manip', 0))
        self.manip_camera_back.clicked.connect(
            lambda: self.send_message('manip', 180))
        self.manip_camera_right.clicked.connect(
            lambda: self.send_message('manip', -90))
        self.manip_camera_right_angle.clicked.connect(
            lambda: self.send_message('manip', -20, 'relative'))

    def send_message(self, camera_id: str, angle: int, mode: str = 'absolute'):
        if mode == 'relative':
            angle = self.cameras_positions[camera_id] + angle
        self.cameras_positions[camera_id] = angle
        pub = self._node.get_publisher(f'/camera_rotator/{camera_id}')
        msg = Int32()
        msg.data = angle
        pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraGuiNode()
    app = QApplication(sys.argv)
    window = Ui_Frame(node)
    window.show()
    import threading
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    exit_code = app.exec_()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()