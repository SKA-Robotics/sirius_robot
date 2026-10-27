#!/usr/bin/env python3
import rospy
import struct
from can_msgs.msg import Frame
from std_msgs.msg import Int32

avg_w1 = 0
avg_w2 = 0
avg_w3 = 0

last_w1 = []
last_w2 = []
last_w3 = []

def callback(frame):
    # if len(frame.data)  4:
    #     rospy.loginfo(f"data 1: {frame.data} g")

    #     return

    weight = (struct.unpack('<f', bytes(frame.data[:4]))[0])
    msg = Int32(data=int(weight))


    if frame.id == 0x70:
        pub1.publish(msg)
        rospy.loginfo(f"Weight 1: {weight} g")
        last_w1.append(weight)
        if len(last_w1) > 10:
            last_w1.pop(0)
        avg_w1 = sum(last_w1) / len(last_w1)
        rospy.loginfo(f"Avg W 1: {avg_w1}\n")

    elif frame.id == 0x71:
        pub2.publish(msg)
        rospy.loginfo(f"Weight 2: {weight} g")
        last_w2.append(weight)
        if len(last_w2) > 10:
            last_w2.pop(0)
        avg_w2 = sum(last_w2) / len(last_w2)
        rospy.loginfo(f"Avg W 2: {avg_w2}\n")

    elif frame.id == 0x72:
        pub3.publish(msg)
        rospy.loginfo(f"Weight 3: {weight} g")
        last_w3.append(weight)
        if len(last_w3) > 10:
            last_w3.pop(0)
        avg_w3 = sum(last_w3) / len(last_w3)
        rospy.loginfo(f"Avg W 3: {avg_w3}\n") 
    


def listener():
    rospy.init_node("weight_listener", anonymous=True)
    rospy.Subscriber("/received_canbus_messages", Frame, callback)
    rospy.loginfo(
        "Weight listener started, waiting for CAN frames 0x70, 0x71, 0x72")
    rospy.spin()


if __name__ == "__main__":
    try:
        pub1 = rospy.Publisher("/scales/weight1", Int32, queue_size=10)
        pub2 = rospy.Publisher("/scales/weight2", Int32, queue_size=10)
        pub3 = rospy.Publisher("/scales/weight3", Int32, queue_size=10)
        listener()
    except rospy.ROSInterruptException:
        pass
