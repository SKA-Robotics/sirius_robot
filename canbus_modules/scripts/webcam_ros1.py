#!/usr/bin/python3

import rospy
import subprocess
from std_msgs.msg import Int32
from functools import partial
import time


class CameraRotator:

    def __init__(self):
        rospy.init_node('camera_rotator')

        self.camera_names = {
            '1_2.1.2': 'right',
            '1_2.1.4': 'left',
            '1_2.2.1': 'middle',
            '1_2.2.2': 'manip',
        }
        self.cameras_positions = {}
        self.cameras_dict = {}
        self.subscribers = {}

        self.rate = rospy.Rate(1.0)

        self.detect_cameras()

    def run(self) -> None:
        while not rospy.is_shutdown():
            self.timer_callback()
            self.rate.sleep()

    def detect_cameras(self):
        cameras_detection_command = 'adb devices -l'
        cameras_result = subprocess.run(cameras_detection_command,
                                        shell=True,
                                        capture_output=True)

        cameras_output_bytes = cameras_result.stdout
        devices_output_string = cameras_output_bytes.decode('utf-8')

        cameras_output_list = devices_output_string.splitlines()
        cameras_output_list = cameras_output_list[1:-1]

        cameras_dict = {}

        for camera_output in cameras_output_list:
            camera_output = camera_output.split()
            if camera_output[-3] == 'device':

                camera_usb = camera_output[-2]
                #wybranie id po :
                camera_id = (camera_output[-1].split(":"))[-1]
                cameras_dict[camera_usb] = camera_id

        for usb, id in cameras_dict.items():
            if usb not in self.cameras_dict:
                self.add_camera(usb, id)
                self.cameras_dict[usb] = id

        cameras_dict_copy = self.cameras_dict.copy()

        for usb, id in cameras_dict_copy.items():
            if usb not in cameras_dict:
                self.remove_camera(usb)
                del self.cameras_dict[usb]

    def listener_callback(self, msg, camera_id=""):

        if msg.data > 180:
            msg.data = 180
        elif msg.data < -180:
            msg.data = -180

        final_position = int(msg.data * (4096 / 360))

        steps = final_position - self.cameras_positions[camera_id]

        rotate_command = f"adb -t {camera_id} shell motor_rotate {steps}"
        #print(rotate_command)
        #time.sleep(4)
        self.cameras_positions[camera_id] += steps
        print(self.cameras_positions[camera_id])
        subprocess.run(rotate_command, shell=True)
        time.sleep(1)

    def add_camera(self, camera_usb, camera_id):
        usb_nr = (camera_usb.split(":"))[-1]
        usb_nr = usb_nr.replace("-", "_")

        topic_name = f"/camera_rotator/{self.camera_names[usb_nr]}"
        print(usb_nr, "added")

        self.subscribers[camera_usb] = rospy.Subscriber(
            topic_name, Int32,
            partial(self.listener_callback, camera_id=camera_id))

        self.subscribers[camera_usb]  #prevent unused variable warning

        init_command = f"adb -t {camera_id} shell motor_init"
        subprocess.run(init_command, shell=True)
        self.cameras_positions[camera_id] = 0
        time.sleep(5)

    def remove_camera(self, camera_usb):

        self.subscribers[camera_usb].unregister()
        print(camera_usb, "removed")
        del self.subscribers[camera_usb]

    def timer_callback(self):
        self.detect_cameras()


if __name__ == '__main__':

    try:
        CameraRotator().run()
    except rospy.ROSInterruptException:
        pass
