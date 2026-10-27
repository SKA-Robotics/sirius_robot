#!/usr/bin/python3
"""
QR code detector node.

Run:
  rosrun autonomy qr_detector_node.py
  rosrun autonomy qr_detector_node.py _camera_address:="rtsp://..."

Topics (private, under /qr_detector/):
  ~qr_debug_image  - annotated camera image (sensor_msgs/Image)
  ~qr_data          - decoded QR text (std_msgs/String)

View image:
  rosrun image_view image_view image:=/qr_detector/qr_debug_image
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


class QRCodeDetectorNode:
    def __init__(self):
        rospy.init_node('qr_detector', anonymous=True)

        self.camera_address = rospy.get_param('~camera_address', "rtsp://root:skar@192.168.1.13/stream=0")
        self.framerate_decimation = rospy.get_param('~framerate_decimation', 7)

        self.bridge = CvBridge()
        self.qr_detector = cv2.QRCodeDetector()

        self.image_pub = rospy.Publisher('~qr_debug_image', Image, queue_size=10)
        self.data_pub = rospy.Publisher('~qr_data', String, queue_size=10)

        self.cap = cv2.VideoCapture(self.camera_address)
        if not self.cap.isOpened():
            rospy.logerr("Failed to open camera: %s", self.camera_address)
            rospy.signal_shutdown("Camera not accessible.")
            return

    def _normalize_corners(self, pts):
        # OpenCV returns either (4, 2) or (1, 4, 2) depending on detect path.
        corners = np.asarray(pts, dtype=np.int32)
        if corners.ndim == 3:
            corners = corners[0]
        return corners.reshape(4, 2)

    def detect_qr_codes(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        data_msg = String()

        decoded_texts = []
        decoded_points = []

        ok, texts, points, _ = self.qr_detector.detectAndDecodeMulti(gray)
        if ok and texts is not None:
            for text, pts in zip(texts, points):
                if text:
                    decoded_texts.append(text)
                    decoded_points.append(self._normalize_corners(pts))

        if not decoded_texts:
            text, points, _ = self.qr_detector.detectAndDecode(gray)
            if text:
                decoded_texts.append(text)
                if points is not None:
                    decoded_points.append(self._normalize_corners(points))

        if decoded_texts:
            data_msg.data = decoded_texts[0]

            # Optional: print decoded QR data to terminal (comment out to disable)
            for text in decoded_texts:
                rospy.loginfo("QR detected: %s", text)

            for text, pts in zip(decoded_texts, decoded_points):
                x_min, y_min = int(pts[:, 0].min()), int(pts[:, 1].min())
                x_max, y_max = int(pts[:, 0].max()), int(pts[:, 1].max())
                box_w, box_h = x_max - x_min, y_max - y_min

                cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 180, 0), -1)
                cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 4)

                font = cv2.FONT_HERSHEY_DUPLEX
                font_scale = max(1.5, min(box_w / max(len(text), 1) / 8, box_h / 25, 5.0))
                thickness = max(3, int(font_scale * 2))
                (text_w, text_h), _ = cv2.getTextSize(text, font, font_scale, thickness)
                text_x = x_min + (box_w - text_w) // 2
                text_y = y_min + (box_h + text_h) // 2

                cv2.putText(
                    frame,
                    text,
                    (text_x, text_y),
                    font,
                    font_scale,
                    (0, 0, 0),
                    thickness + 4,
                )
                cv2.putText(
                    frame,
                    text,
                    (text_x, text_y),
                    font,
                    font_scale,
                    (0, 255, 255),
                    thickness,
                )

        return frame, data_msg

    def run(self):
        while not rospy.is_shutdown():
            for _ in range(self.framerate_decimation):
                self.cap.grab()
            ret, image = self.cap.retrieve()

            if not ret:
                rospy.logwarn("Failed to retrieve frame from camera.")
                continue

            debug_image, qr_data = self.detect_qr_codes(image.copy())
            debug_image = cv2.resize(debug_image, (847, 598), interpolation=cv2.INTER_LINEAR)

            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(debug_image, "bgr8"))
            except Exception as e:
                rospy.logerr("CvBridge Error: %s", e)

            self.data_pub.publish(qr_data)

        self.cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        node = QRCodeDetectorNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
