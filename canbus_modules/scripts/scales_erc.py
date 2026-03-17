#!/usr/bin/env python3
import struct

import rclpy
from rclpy.node import Node

from can_msgs.msg import Frame
from std_msgs.msg import Float32


class ScalesErcNode(Node):
    def __init__(self):
        super().__init__('weight_listener')
        self.pub = self.create_publisher(Float32, '/sample_weight', 10)
        self.create_subscription(
            Frame, '/from_can_bus', self._callback, 10)
        self.get_logger().info(
            'Weight listener started, waiting for CAN frames with ID 0x70')

    def _callback(self, frame: Frame):
        if frame.id == 0x70 and len(frame.data) >= 4:
            weight = struct.unpack('<f', bytes(frame.data[:4]))[0]
            msg = Float32()
            msg.data = weight
            self.pub.publish(msg)
            self.get_logger().info(f'Weight: {weight:.3f} g')


def main(args=None):
    rclpy.init(args=args)
    node = ScalesErcNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()