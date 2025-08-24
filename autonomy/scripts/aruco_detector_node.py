#!/usr/bin/python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from autonomy.msg import Marker, MarkerArray

class ArucoPoseEstimator:
    def __init__(self):
        rospy.init_node('aruco_detector', anonymous=True)

        # Load ROS parameters
        self.camera_address = rospy.get_param('~camera_address', "rtsp://root:skar@192.168.1.13/stream=0")
        self.framerate_decimation = rospy.get_param('~framerate_decimation', 7)
        
        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)

        self.image_pub = rospy.Publisher('~aruco_debug_image', Image, queue_size=10)
        self.marker_array_pub = rospy.Publisher('~aruco_markers', MarkerArray, queue_size=10)

        self.cap = cv2.VideoCapture(self.camera_address)
        if not self.cap.isOpened():
            rospy.logerr("Failed to open camera: %s", self.camera_address)
            rospy.signal_shutdown("Camera not accessible.")
            return


    def marker_detection(self, frame):
        _, img_width, _ = frame.shape
        corners, ids, _ = self.detector.detectMarkers(frame)
        
        marker_array_msg = MarkerArray()
        
        if ids is not None and len(corners) > 0:
            cv2.aruco.drawDetectedMarkers(frame, corners) 

            for i in range(0, len(ids)):
                marker = Marker()
                marker.id = ids[i][0]
                marker.position = self.calculate_position(corners[i][0], img_width)
                marker.size = self.calculate_size(corners[i][0], img_width)
                
                marker_array_msg.markers.append(marker)

        return frame, marker_array_msg
    
    def calculate_position(self, corners, img_width):
        avg_x = np.sum([corner[0] for corner in corners])/4
        x = (2*avg_x - img_width)/img_width
        return x
    
    def calculate_size(self, corners, img_width):
        edge_sizes = np.array([
            self.calculate_distance(corners[0], corners[1]),
            self.calculate_distance(corners[1], corners[2]),
            self.calculate_distance(corners[2], corners[3]),
            self.calculate_distance(corners[3], corners[0]),
        ]) / img_width
        return np.max(edge_sizes)
    
    def calculate_distance(self, point1, point2):
        return np.linalg.norm(point1 - point2)

    def run(self):
        while not rospy.is_shutdown():
            # Grabbing multiple frames to ensure the latest frame is processed
            # This is specific to your original code's behavior for RTSP streams
            for _ in range(self.framerate_decimation): 
                self.cap.grab()
            ret, image = self.cap.retrieve()

            if not ret:
                rospy.logwarn("Failed to retrieve frame from camera.")
                continue

            debug_image, markers = self.marker_detection(image.copy())

            # Resize image for display
            debug_image = cv2.resize(debug_image, (847, 598), interpolation=cv2.INTER_LINEAR)
            
            # Publish debug image
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(debug_image, "bgr8"))
            except Exception as e:
                rospy.logerr("CvBridge Error: %s", e)

            # Publish MarkerArray
            self.marker_array_pub.publish(markers)

        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        node = ArucoPoseEstimator()
        node.run()
    except rospy.ROSInterruptException:
        pass