#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from autonomy.msg import Marker, MarkerArray
import cv2
import numpy as np


class ArucoPoseEstimator(Node):

    def __init__(self):
        super().__init__('aruco_detector')

        self.declare_parameter(
            'camera_address',
            'rtsp://root:skar@192.168.1.13/stream=0')
        self.declare_parameter('framerate_decimation', 7)

        self.camera_address = self.get_parameter(
            'camera_address').value
        self.framerate_decimation = self.get_parameter(
            'framerate_decimation').value

        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict,
                                                 self.parameters)

        self.image_pub = self.create_publisher(Image, '~/aruco_debug_image', 10)
        self.marker_array_pub = self.create_publisher(MarkerArray,
                                                      '~/aruco_markers', 10)

        self.cap = cv2.VideoCapture(self.camera_address)
        if not self.cap.isOpened():
            self.get_logger().error(
                f'Failed to open camera: {self.camera_address}')
            raise RuntimeError('Camera not accessible.')

        # Drive detection from a timer instead of a blocking loop
        self.create_timer(0.033, self._timer_callback)  # ~30 Hz
        self._frame_count = 0

    def _timer_callback(self):
        self._frame_count += 1
        self.cap.grab()
        if self._frame_count % self.framerate_decimation != 0:
            return

        ret, image = self.cap.retrieve()
        if not ret:
            self.get_logger().warn('Failed to retrieve frame from camera.')
            return

        debug_image, markers = self._marker_detection(image.copy())
        debug_image = cv2.resize(debug_image, (847, 598),
                                 interpolation=cv2.INTER_LINEAR)

        try:
            self.image_pub.publish(
                self.bridge.cv2_to_imgmsg(debug_image, 'bgr8'))
        except Exception as e:
            self.get_logger().error(f'CvBridge Error: {e}')

        self.marker_array_pub.publish(markers)

    def _marker_detection(self, frame):
        _, img_width, _ = frame.shape
        corners, ids, _ = self.detector.detectMarkers(frame)

        marker_array_msg = MarkerArray()

        if ids is not None and len(corners) > 0:
            cv2.aruco.drawDetectedMarkers(frame, corners)
            for i in range(len(ids)):
                marker = Marker()
                marker.id = int(ids[i][0])
                marker.position = self._calculate_position(corners[i][0],
                                                           img_width)
                marker.size = self._calculate_size(corners[i][0], img_width)
                marker_array_msg.markers.append(marker)

        return frame, marker_array_msg

    def _calculate_position(self, corners, img_width):
        avg_x = np.sum([c[0] for c in corners]) / 4
        return float((2 * avg_x - img_width) / img_width)

    def _calculate_size(self, corners, img_width):
        edge_sizes = np.array([
            np.linalg.norm(corners[0] - corners[1]),
            np.linalg.norm(corners[1] - corners[2]),
            np.linalg.norm(corners[2] - corners[3]),
            np.linalg.norm(corners[3] - corners[0]),
        ]) / img_width
        return float(np.max(edge_sizes))

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoPoseEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
