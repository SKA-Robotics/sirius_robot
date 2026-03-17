#!/usr/bin/env python3
import subprocess
import time
from functools import partial

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32


class CameraRotatorNode(Node):
    def __init__(self):
        super().__init__('camera_rotator')
        self.camera_names = {
            '1_2.1.2': 'right',
            '1_2.1.4': 'left',
            '1_2.2.1': 'middle',
            '1_2.2.2': 'manip',
        }
        self.cameras_positions = {}
        self.cameras_dict = {}
        self.subscribers = {}

        self._detect_cameras()
        # 1 Hz timer for camera re-detection
        self.create_timer(1.0, self._timer_callback)

    def _timer_callback(self):
        self._detect_cameras()

    def _detect_cameras(self):
        result = subprocess.run(
            'adb devices -l', shell=True, capture_output=True)
        lines = result.stdout.decode('utf-8').splitlines()[1:-1]

        cameras_dict = {}
        for line in lines:
            parts = line.split()
            if len(parts) >= 3 and parts[-3] == 'device':
                camera_usb = parts[-2]
                camera_id = parts[-1].split(':')[-1]
                cameras_dict[camera_usb] = camera_id

        for usb, id_ in cameras_dict.items():
            if usb not in self.cameras_dict:
                self._add_camera(usb, id_)
                self.cameras_dict[usb] = id_

        for usb in list(self.cameras_dict.keys()):
            if usb not in cameras_dict:
                self._remove_camera(usb)
                del self.cameras_dict[usb]

    def _listener_callback(self, msg: Int32, camera_id: str = ''):
        angle = max(-180, min(180, msg.data))
        final_position = int(angle * (4096 / 360))
        steps = final_position - self.cameras_positions[camera_id]
        subprocess.run(
            f'adb -t {camera_id} shell motor_rotate {steps}', shell=True)
        self.cameras_positions[camera_id] += steps
        time.sleep(abs(steps) / 500 * 1 + 0.2)

    def _add_camera(self, camera_usb: str, camera_id: str):
        usb_nr = camera_usb.split(':')[-1].replace('-', '_')
        camera_name = self.camera_names.get(usb_nr)
        if camera_name is None:
            self.get_logger().warn(f'Unknown camera USB port: {usb_nr}')
            return
        topic_name = f'/camera_rotator/{camera_name}'
        self.get_logger().info(f'{usb_nr} added')
        self.subscribers[camera_usb] = self.create_subscription(
            Int32, topic_name,
            partial(self._listener_callback, camera_id=camera_id),
            10)
        subprocess.run(
            f'adb -t {camera_id} shell motor_init', shell=True)
        self.cameras_positions[camera_id] = 0
        time.sleep(5)

    def _remove_camera(self, camera_usb: str):
        if camera_usb in self.subscribers:
            self.destroy_subscription(self.subscribers[camera_usb])
            del self.subscribers[camera_usb]
        self.get_logger().info(f'{camera_usb} removed')


def main(args=None):
    rclpy.init(args=args)
    node = CameraRotatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()