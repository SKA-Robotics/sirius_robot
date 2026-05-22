#!/usr/bin/env python3
import rospy
import struct
from can_msgs.msg import Frame
from std_msgs.msg import Float32

def callback(frame):

    if frame.id == 0x70:

        weight = struct.unpack('<f', bytes(frame.data[:4]))[0]

        pub.publish(weight)

def listener():
    rospy.init_node("weight_listener", anonymous=True)
    rospy.Subscriber("/received_canbus_messages", Frame, callback)
    rospy.spin()

if __name__ == "__main__":
    try:
        pub = rospy.Publisher("/sample_weight", Float32, queue_size=10)
        listener()
    except rospy.ROSInterruptException:
        pass
