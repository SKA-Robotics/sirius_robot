#!/usr/bin/env python3
import rospy
import struct
from can_msgs.msg import Frame
from std_msgs.msg import Float32

def callback(frame):

    if frame.id == 0x77:

        ph = struct.unpack('<f', bytes(frame.data[:4]))[0]

        pub.publish(ph)

def listener():
    rospy.init_node("ph_meter_listener", anonymous=True)
    rospy.Subscriber("/received_canbus_messages", Frame, callback)
    rospy.spin()

if __name__ == "__main__":
    try:
        pub = rospy.Publisher("/sample_ph_meter", Float32, queue_size=10)
        listener()
    except rospy.ROSInterruptException:
        pass
