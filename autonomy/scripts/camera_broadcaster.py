#!/usr/bin/python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ArucoPoseEstimator:
    def __init__(self):
        rospy.init_node('aruco_detector', anonymous=True)

        # Load ROS parameters
        self.camera_address = rospy.get_param('~camera_address', "rtsp://root:skar@192.168.1.16/stream=0")
        self.framerate_decimation = rospy.get_param('~framerate_decimation', 7)
        
        self.bridge = CvBridge()
        self.parameters = cv2.aruco.DetectorParameters()

        self.image_pub = rospy.Publisher('~aruco_debug_image', Image, queue_size=10)

        self.cap = cv2.VideoCapture(self.camera_address)
        if not self.cap.isOpened():
            rospy.logerr("Failed to open camera: %s", self.camera_address)
            rospy.signal_shutdown("Camera not accessible.")
            return

    def run(self):
        while not rospy.is_shutdown():
#            for _ in range(self.framerate_decimation): 
 #               self.cap.grab()
            ret, image = self.cap.read()

            if not ret:
                rospy.logwarn("Failed to retrieve frame from camera.")
                continue

            cv2.imshow("window", image)
            cv2.waitKey(25)

#            try:
 #               self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
  #          except Exception as e:
   #             rospy.logerr("CvBridge Error: %s", e)

        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        node = ArucoPoseEstimator()
        node.run()
    except rospy.ROSInterruptException:
        pass
