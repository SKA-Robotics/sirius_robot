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

import threading
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


class ThreadedFrameGrabber:
    """Continuously reads frames from a cv2.VideoCapture in a background
    thread and keeps only the latest one.

    Why: on the FFmpeg/RTSP backend, cap.grab() typically decodes the frame
    internally (it's not a cheap "skip"). The previous approach of calling
    grab() N times in the main loop to "discard" frames therefore paid the
    full decode cost N times per processed frame, while doing nothing else
    -- that was measured to be >90% of total loop time. Reading in a free-
    running background thread means the decode cost is paid continuously in
    parallel with processing, and the main loop simply grabs whatever the
    latest available frame is, with no artificial waiting/discarding.
    """

    def __init__(self, camera_address):
        self.cap = cv2.VideoCapture(camera_address)
        self._lock = threading.Lock()
        self._latest_frame = None
        self._new_frame_available = False
        self._running = False
        self._thread = None

    def is_opened(self):
        return self.cap.isOpened()

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._update_loop, daemon=True)
        self._thread.start()
        return self

    def _update_loop(self):
        while self._running:
            ret, frame = self.cap.read()
            if not ret:
                continue
            with self._lock:
                self._latest_frame = frame
                self._new_frame_available = True

    def read_latest(self, block_until_new=True, poll_interval=0.005):
        """Return the most recent frame. If block_until_new is True, waits
        until a frame newer than the last one returned is available (still
        far cheaper than decoding N discarded frames synchronously)."""
        while block_until_new:
            with self._lock:
                if self._new_frame_available:
                    break
            rospy.sleep(poll_interval)

        with self._lock:
            frame = self._latest_frame
            self._new_frame_available = False
        return frame is not None, frame

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self.cap.release()


class QRCodeDetectorNode:
    def __init__(self):
        rospy.init_node('qr_detector', anonymous=True)

        self.camera_address = rospy.get_param('~camera_address', "rtsp://root:skar@192.168.1.15/stream=0")

        # Frame counter / FPS reporting
        self.frame_count = 0
        self.fps_log_interval = rospy.get_param('~fps_log_interval', 30)  # log every N frames
        self._fps_window_start_time = None
        self._fps_window_start_count = 0

        # Output/working resolution. Detection now runs directly on this
        # (already-downscaled) frame instead of the full-res camera frame,
        # so we do ONE resize total instead of "detect full-res, then resize".
        self.output_width = rospy.get_param('~output_width', 847)
        self.output_height = rospy.get_param('~output_height', 598)

        self.bridge = CvBridge()
        # Detection engine: WeChatQRCode -- CNN-based (Caffe) detector +
        # super-resolution model. Most robust of the engines we compared
        # for small/blurred/damaged codes, at the cost of being the
        # slowest and needing 4 model files + opencv-contrib-python.
        #
        # Requires:
        #   pip3 install opencv-contrib-python-headless --break-system-packages
        #   (headless avoids pulling in a GUI/Qt build that can clash with
        #   ROS's own OpenCV; -headless has no cv2.imshow etc., which this
        #   node doesn't use anyway)
        #
        # Download the 4 model files (~ a few MB total) into model_dir:
        #   https://github.com/WeChatCV/opencv_3rdparty/tree/wechat_qrcode
        #   (detect.prototxt, detect.caffemodel, sr.prototxt, sr.caffemodel)
        model_dir = rospy.get_param('~wechat_model_dir', "/home/rover/wechat_qrcode_models")
        try:
            self.qr_detector = cv2.wechat_qrcode_WeChatQRCode(
                model_dir + "/detect.prototxt",
                model_dir + "/detect.caffemodel",
                model_dir + "/sr.prototxt",
                model_dir + "/sr.caffemodel",
            )
        except Exception as e:
            rospy.logerr(
                "Failed to load WeChatQRCode models from '%s': %s. "
                "Download detect.prototxt/detect.caffemodel/sr.prototxt/sr.caffemodel "
                "from https://github.com/WeChatCV/opencv_3rdparty/tree/wechat_qrcode "
                "and point ~wechat_model_dir at that folder.",
                model_dir, e,
            )
            rospy.signal_shutdown("WeChatQRCode models not accessible.")
            return

        self.image_pub = rospy.Publisher('~qr_debug_image', Image, queue_size=10)
        self.data_pub = rospy.Publisher('~qr_data', String, queue_size=10)

        self.grabber = ThreadedFrameGrabber(self.camera_address)
        if not self.grabber.is_opened():
            rospy.logerr("Failed to open camera: %s", self.camera_address)
            rospy.signal_shutdown("Camera not accessible.")
            return

        # Keep only the newest frame in the RTSP buffer. Not all backends
        # honor this, but where supported it further reduces latency.
        self.grabber.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.grabber.start()

    def detect_qr_codes(self, frame):
        """Detect + draw directly on `frame` (expected to already be at the
        small output resolution -- see run()). Uses WeChatQRCode
        (detectAndDecode returns (list_of_strings, points_array); points is
        one (4, 2) quadrilateral per decoded string, same order as the text
        list).
        """
        data_msg = String()

        decoded_texts = []
        decoded_points = []

        texts, points = self.qr_detector.detectAndDecode(frame)
        if texts:
            points = np.asarray(points).reshape(-1, 4, 2)
            for text, pts in zip(texts, points):
                if not text:
                    continue
                decoded_texts.append(text)
                decoded_points.append(pts.astype(np.int32))

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

    def _update_fps_counter(self):
        """Increment the frame counter and periodically log measured FPS."""
        self.frame_count += 1

        now = rospy.Time.now()
        if self._fps_window_start_time is None:
            self._fps_window_start_time = now
            self._fps_window_start_count = self.frame_count
            return

        if self.frame_count - self._fps_window_start_count >= self.fps_log_interval:
            elapsed = (now - self._fps_window_start_time).to_sec()
            frames_in_window = self.frame_count - self._fps_window_start_count
            fps = frames_in_window / elapsed if elapsed > 0 else 0.0
            rospy.loginfo("Frame #%d | %.2f FPS (last %d frames)", self.frame_count, fps, frames_in_window)
            self._fps_window_start_time = now
            self._fps_window_start_count = self.frame_count

    def run(self):
        # Per-stage timing accumulators (reset every fps_log_interval frames)
        stage_totals = {"wait_frame": 0.0, "resize": 0.0, "detect": 0.0, "publish": 0.0}

        while not rospy.is_shutdown():
            t0 = rospy.Time.now()
            ret, image = self.grabber.read_latest(block_until_new=True)
            t1 = rospy.Time.now()

            if not ret:
                rospy.logwarn("Failed to retrieve frame from camera.")
                continue

            self._update_fps_counter()

            # Downscale ONCE, up front. Detection then runs on this smaller
            # image instead of the full camera resolution, which is the
            # single biggest cost reduction (detector cost grows with pixel
            # count). Previously the resize happened *after* full-res
            # detection, so we paid the full-res cost for nothing.
            small_image = cv2.resize(
                image,
                (self.output_width, self.output_height),
                interpolation=cv2.INTER_AREA,
            )
            t2 = rospy.Time.now()

            debug_image, qr_data = self.detect_qr_codes(small_image)
            t3 = rospy.Time.now()

            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(debug_image, "bgr8"))
            except Exception as e:
                rospy.logerr("CvBridge Error: %s", e)

            self.data_pub.publish(qr_data)
            t4 = rospy.Time.now()

            stage_totals["wait_frame"] += (t1 - t0).to_sec()
            stage_totals["resize"] += (t2 - t1).to_sec()
            stage_totals["detect"] += (t3 - t2).to_sec()
            stage_totals["publish"] += (t4 - t3).to_sec()

            if self.frame_count % self.fps_log_interval == 0:
                total = sum(stage_totals.values()) or 1e-9
                breakdown = " | ".join(
                    "%s: %.1f%% (%.3fs/frame)" % (
                        name,
                        100.0 * value / total,
                        value / self.fps_log_interval,
                    )
                    for name, value in stage_totals.items()
                )
                rospy.loginfo("Stage timing (avg over %d frames): %s", self.fps_log_interval, breakdown)
                stage_totals = {k: 0.0 for k in stage_totals}

        self.grabber.stop()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        node = QRCodeDetectorNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
