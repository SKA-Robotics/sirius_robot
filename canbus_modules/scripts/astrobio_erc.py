#!/usr/bin/env python3
import rospy
import struct
from can_msgs.msg import Frame
from std_msgs.msg import Float32

def callback(frame):

    pH = struct.unpack('<f', bytes(frame.data[:4]))[0]
    msg = Float32(data=float(pH))

    if frame.id == 0x80:
        pub1.publish(msg)
        rospy.loginfo(f"pH: {pH}")

def listener():
    rospy.init_node("pH_listener", anonymous=True)
    rospy.Subscriber("/received_canbus_messages", Frame, callback)
    rospy.loginfo(
        "pH listener started, waiting for CAN frame 0x80")
    rospy.spin()


if __name__ == "__main__":
    try:
        pub1 = rospy.Publisher("/pH", Float32, queue_size=10)
        listener()
    except rospy.ROSInterruptException:
        pass
