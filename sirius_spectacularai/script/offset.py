#!/usr/bin/env python3
"""Adds a fixed XYZ offset to SLAM global odometry and optionally republishes as TF."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros


class OdometryChildFrameTransformer(Node):

    def __init__(self):
        super().__init__('odom_offset')

        self.declare_parameter('publish_tf', False)
        self.declare_parameter('pose.x', 2.732003282)
        self.declare_parameter('pose.y', -2.338536955)
        self.declare_parameter('pose.z', 0.0)

        self._publish_tf = self.get_parameter('publish_tf').value
        self._translation = {
            'x': self.get_parameter('pose.x').value,
            'y': self.get_parameter('pose.y').value,
            'z': self.get_parameter('pose.z').value,
        }

        self._sub = self.create_subscription(
            Odometry, '/slam/global_odometry_orig', self._odom_callback, 10)
        self._pub = self.create_publisher(Odometry, '/slam/global_odometry', 10)

        if self._publish_tf:
            self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def _odom_callback(self, msg: Odometry):
        out = self._offset_odometry(msg)
        self._pub.publish(out)
        if self._publish_tf:
            self._publish_odom_to_tf(out)

    def _offset_odometry(self, msg: Odometry) -> Odometry:
        pose = PoseWithCovariance()
        pose.pose = msg.pose.pose
        pose.pose.position.x += self._translation['x']
        pose.pose.position.y += self._translation['y']
        pose.pose.position.z += self._translation['z']
        pose.covariance = msg.pose.covariance

        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose = pose
        out.twist = msg.twist
        return out

    def _publish_odom_to_tf(self, odom: Odometry):
        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = odom.child_frame_id
        t.transform.translation.x = odom.pose.pose.position.x
        t.transform.translation.y = odom.pose.pose.position.y
        t.transform.translation.z = odom.pose.pose.position.z
        t.transform.rotation = odom.pose.pose.orientation
        self._tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryChildFrameTransformer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
