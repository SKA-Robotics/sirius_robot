#!/usr/bin/env python3
"""Spectacular AI SLAM node for the Sirius rover."""

import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

import numpy as np
import PyKDL
import depthai
import spectacularAI

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField, CameraInfo, Image, NavSatFix
from geometry_msgs.msg import TransformStamped
import tf2_ros
from cv_bridge import CvBridge

from message_constructors import to_camera_info_message, to_odometry_message
from transforms import transform_odometry_child_frame


class SlamNode(Node):

    def __init__(self):
        super().__init__('slam_node')

        self.declare_parameter('camera_id', '19443010114A722700')
        self.declare_parameter('pointcloud_frame', 'slam')
        self.declare_parameter('odom_frame', 'base_link')
        self.declare_parameter('topic_prefix', '/slam')

        camera_id = self.get_parameter('camera_id').value
        self.pointcloud_frame = self.get_parameter('pointcloud_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        topic_prefix = self.get_parameter('topic_prefix').value

        self.odometry_pub = self.create_publisher(
            Odometry, f'{topic_prefix}/odometry', 10)
        self.global_odometry_pub = self.create_publisher(
            Odometry, f'{topic_prefix}/global_odometry', 10)
        self.rgb_pub = self.create_publisher(
            Image, f'{topic_prefix}/rgb', 10)
        self.point_pub = self.create_publisher(
            PointCloud2, f'{topic_prefix}/pointcloud', 10)
        self.depth_pub = self.create_publisher(
            Image, f'{topic_prefix}/depth', 10)
        self.camera_info_pub = self.create_publisher(
            CameraInfo, f'{topic_prefix}/camera_info', 10)

        self.create_subscription(NavSatFix, '/gps/fix',
                                 self._gps_callback, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.bridge = CvBridge()
        self.keyframes = {}
        self.session = None
        self.device = None

        self.camera_id = camera_id
        self.get_logger().info(f'SLAM node initialized, camera_id={camera_id}')

    def _gps_callback(self, msg: NavSatFix):
        if self.session is None:
            return
        cov = [[msg.position_covariance[i * 3 + j] for j in range(3)]
               for i in range(3)]
        coords = spectacularAI.WgsCoordinates()
        coords.altitude = msg.altitude
        coords.latitude = msg.latitude
        coords.longitude = msg.longitude
        self.session.addGnss(self._device_timestamp(), coords, cov)

    def _device_timestamp(self) -> float:
        imu_queue = self.device.getOutputQueue(
            name='spectacularAI_imu', maxSize=1, blocking=True)
        imu_data = imu_queue.get()
        return imu_data.packets[0].acceleroMeter.getTimestampDevice().total_seconds() - 0.5

    def new_odometry_frame(self, vio_output):
        stamp = self.get_clock().now().to_msg()
        msg = to_odometry_message(vio_output, is_global=False, stamp=stamp)
        try:
            msg = transform_odometry_child_frame(msg, 'base_link',
                                                 self.tf_buffer)
        except Exception:
            pass
        msg.header.frame_id = 'map'
        self.odometry_pub.publish(msg)

        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = self.odom_frame
        t.transform.rotation = msg.pose.pose.orientation
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        self.tf_broadcaster.sendTransform(t)

        global_msg = to_odometry_message(vio_output, is_global=True, stamp=stamp)
        if global_msg is not None:
            try:
                global_msg = transform_odometry_child_frame(
                    global_msg, 'base_link', self.tf_buffer)
            except Exception:
                pass
            global_msg.header.frame_id = 'map'
            self.global_odometry_pub.publish(global_msg)

    def new_key_frame(self, frame_id, keyframe):
        if frame_id in self.keyframes:
            return
        self.keyframes[frame_id] = True
        stamp = self.get_clock().now().to_msg()
        seq = int(frame_id)

        rgb_frame = keyframe.frameSet.getUndistortedFrame(
            keyframe.frameSet.rgbFrame).image.toArray()
        rgb_msg = self.bridge.cv2_to_imgmsg(rgb_frame, encoding='rgb8')
        rgb_msg.header.stamp = stamp
        rgb_msg.header.frame_id = 'front_rgb_camera_optical'
        self.rgb_pub.publish(rgb_msg)

        camera = keyframe.frameSet.rgbFrame.cameraPose.camera
        self.camera_info_pub.publish(
            to_camera_info_message(camera, rgb_frame, stamp))

        self._publish_point_cloud(keyframe, stamp)

        depth_frame = keyframe.frameSet.getAlignedDepthFrame(
            keyframe.frameSet.rgbFrame)
        depth = depth_frame.image.toArray()
        depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding='mono16')
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = 'front_rgb_camera_optical'
        self.depth_pub.publish(depth_msg)

    def _publish_point_cloud(self, keyframe, stamp):
        cam_to_world = (keyframe.frameSet.rgbFrame.cameraPose
                        .getCameraToWorldMatrix())
        positions = keyframe.pointCloud.getPositionData()
        pc = np.zeros((positions.shape[0], 6), dtype=np.float32)
        p_C = np.vstack((positions.T, np.ones((1, positions.shape[0])))).T
        pc[:, :3] = (cam_to_world @ p_C[:, :, None])[:, :3, 0]
        if keyframe.pointCloud.hasColors():
            pc[:, 3:] = keyframe.pointCloud.getRGB24Data() * (1.0 / 255.0)

        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = self.pointcloud_frame
        msg.height = 1
        msg.width = pc.shape[0]
        msg.point_step = 4 * 6
        msg.row_step = msg.point_step * pc.shape[0]
        msg.data = pc.tobytes()
        msg.is_bigendian = False
        msg.is_dense = False
        itemsize = np.dtype(np.float32).itemsize
        msg.fields = [
            PointField(name=n, offset=i * itemsize,
                       datatype=PointField.FLOAT32, count=1)
            for i, n in enumerate('xyzrgb')
        ]
        self.point_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SlamNode()

    config_internal = {
        'computeStereoPointCloud': 'true',
        'pointCloudNormalsEnabled': 'true',
        'computeDenseStereoDepth': 'true',
        'useRectification': 'true',
    }

    pipeline = depthai.Pipeline()
    config = spectacularAI.depthai.Configuration()
    config.internalParameters = config_internal
    config.useSlam = True
    config.imuToGnss = spectacularAI.Vector3d(0, -0.93, -0.71)

    def on_mapping_output(output):
        for frame_id in output.updatedKeyFrames:
            keyframe = output.map.keyFrames.get(frame_id)
            if not keyframe or not keyframe.pointCloud:
                continue
            node.new_key_frame(frame_id, keyframe)

    vio_pipeline = spectacularAI.depthai.Pipeline(pipeline, config,
                                                   on_mapping_output)

    try:
        with depthai.Device(
                pipeline,
                deviceInfo=depthai.DeviceInfo(node.camera_id),
        ) as device, vio_pipeline.startSession(device) as vio_session:
            node.device = device
            node.session = vio_session
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0)
                vio_output = vio_session.waitForOutput()
                node.new_odometry_frame(vio_output)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
