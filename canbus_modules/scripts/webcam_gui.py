#!/usr/bin/python3
# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'designercgiUNS.ui'
##
## Created by: Qt User Interface Compiler version 5.15.3
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5 import uic
import sys
import os
import rospy
from std_msgs.msg import Int32


class Ui_Frame(QMainWindow):

    def __init__(self):
        super().__init__()
        rospy.init_node('talker', anonymous=True)
        path = "/home/rover/software/ros1/src/sirius_robot/canbus_modules/scripts/camera_interface.ui"
        uic.loadUi(path, self)

        self.cameras_positions = {
            'left': 0,
            'middle': 0,
            'right': 0,
            'manip': 0
        }

        #left camera
        self.left_camera_left_angle.clicked.connect(
            lambda: self.send_message('left', 20, 'relative'))
        self.left_camera_front_wheels.clicked.connect(
            lambda: self.send_message('left', -45))
        self.left_camera_front.clicked.connect(
            lambda: self.send_message('left', -90))
        self.left_camera_back.clicked.connect(
            lambda: self.send_message('left', 90))
        self.left_camera_back_wheels.clicked.connect(
            lambda: self.send_message('left', 0))
        self.left_camera_right_angle.clicked.connect(
            lambda: self.send_message('left', -20, 'relative'))

        #right camera
        self.right_camera_left_angle.clicked.connect(
            lambda: self.send_message('right', 20, 'relative'))
        self.right_camera_front_wheels.clicked.connect(
            lambda: self.send_message('right', 45))
        self.right_camera_front.clicked.connect(
            lambda: self.send_message('right', 90))
        self.right_camera_back.clicked.connect(
            lambda: self.send_message('right', -90))
        self.right_camera_back_wheels.clicked.connect(
            lambda: self.send_message('right', 0))
        self.right_camera_right_angle.clicked.connect(
            lambda: self.send_message('right', -20, 'relative'))

        #middle camera
        self.middle_camera_left_angle.clicked.connect(
            lambda: self.send_message('middle', 20, 'relative'))
        self.middle_camera_left.clicked.connect(
            lambda: self.send_message('middle', 90))
        self.middle_camera_front.clicked.connect(
            lambda: self.send_message('middle', 0))
        self.middle_camera_back.clicked.connect(
            lambda: self.send_message('middle', 180))
        self.middle_camera_right.clicked.connect(
            lambda: self.send_message('middle', -90))
        self.middle_camera_right_angle.clicked.connect(
            lambda: self.send_message('middle', -20, 'relative'))

        #manip camera
        self.manip_camera_left_angle.clicked.connect(
            lambda: self.send_message('manip', 20, 'relative'))
        self.manip_camera_left.clicked.connect(
            lambda: self.send_message('manip', 90))
        self.manip_camera_front.clicked.connect(
            lambda: self.send_message('manip', 0))
        self.manip_camera_back.clicked.connect(
            lambda: self.send_message('manip', 180))
        self.manip_camera_right.clicked.connect(
            lambda: self.send_message('manip', -90))
        self.manip_camera_right_angle.clicked.connect(
            lambda: self.send_message('manip', -20, 'relative'))

    def send_message(self, camera_id, angle, type='absolute'):
        print("Arm button clicked")

        if type == 'absolute':
            pub = rospy.Publisher(f'/camera_rotator/{camera_id}',
                                  Int32,
                                  queue_size=10)
            pub.publish(angle)
            self.cameras_positions[camera_id] = angle
        else:
            pub = rospy.Publisher(f'/camera_rotator/{camera_id}',
                                  Int32,
                                  queue_size=10)
            pub.publish(self.cameras_positions[camera_id] + angle)
            self.cameras_positions[camera_id] += angle


if __name__ == '__main__':
    app = QApplication(sys.argv)
    main = Ui_Frame()
    main.show()
    sys.exit(app.exec_())
