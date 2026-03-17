#!/usr/bin/env python3
import struct

import rclpy
from rclpy.node import Node

from can_msgs.msg import Frame
from std_msgs.msg import Float32


def bits_to_float(value: int) -> float:
    return struct.unpack('f', struct.pack('I', value))[0]


class ScienceSensorNode(Node):
    def __init__(self):
        super().__init__('listener_from_id_29')

        self.pub_a0 = self.create_publisher(Float32, 'science_a0', 10)
        self.pub_a1 = self.create_publisher(Float32, 'science_a1', 10)
        self.pub_temp = self.create_publisher(Float32, 'science_temp', 10)

        self.create_subscription(
            Frame, '/from_can_bus', self._callback, 10)

    def _callback(self, frame: Frame):
        node_id = (frame.id >> 5) & 0b111111
        if node_id != 0x29:
            return

        data = bytes(frame.data)
        self.get_logger().info(
            f'Sub: from node with id: {node_id} received data: {list(data)}')

        wartosc_A0 = (data[0] << 8) + data[1]
        wartosc_A1 = (data[2] << 8) + data[3]
        temp_sum = struct.unpack('>I', data[4:8])[0]
        temperature = bits_to_float(temp_sum)

        self.get_logger().info(
            f'Parsed values: A0={wartosc_A0}, A1={wartosc_A1}, '
            f'Temp={temperature}')

        try:
            self.pub_a0.publish(Float32(data=float(wartosc_A0)))
            self.pub_a1.publish(Float32(data=float(wartosc_A1)))
            self.pub_temp.publish(Float32(data=temperature))
            self.get_logger().info(
                f'Published data: {wartosc_A0}, {wartosc_A1}, {temperature}')
        except Exception as e:
            self.get_logger().error(f'Failed to publish: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ScienceSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()