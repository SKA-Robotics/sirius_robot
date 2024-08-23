#!/usr/bin/env python3

import rospy
from can_msgs.msg import Frame
from std_msgs.msg import Float32
import struct


def bits_to_float(value):
    value_bits = struct.pack("I", value)
    return struct.unpack("f", value_bits)[0]


def callback(frame):
    node_id = (frame.id >> 5) & 0b111111
    if node_id == 0x29:
        rospy.loginfo(f"Sub: from node with id: {node_id} received data: {frame.data}")

        wartosc_A0 = (frame.data[0] << 8) + frame.data[1] # WILGOTONOSC 1
        wartosc_A1 = (frame.data[2] << 8) + frame.data[3] # WILGOTNOSC 2

        temp_sum = struct.unpack('>I', frame.data[4:8])[0]
        temperature_reading_representation = bits_to_float(temp_sum)

        current_time = rospy.get_time()

        rospy.loginfo(f"Parsed values: A0={wartosc_A0}, A1={wartosc_A1}, Temp={temperature_reading_representation}, Time={current_time}")

        try:
            
            pub_a0 = rospy.Publisher('science_a0', Float32, queue_size=10)
            pub_a0.publish(wartosc_A0)

            pub_a1 = rospy.Publisher('science_a1', Float32, queue_size=10)
            pub_a1.publish(wartosc_A1)

            pub_temp = rospy.Publisher('science_temp', Float32, queue_size=10)
            pub_temp.publish(temperature_reading_representation)
            
            rospy.loginfo(f"Published data: {wartosc_A0}, {wartosc_A1}, {temperature_reading_representation}")
        except IOError as e:
            rospy.logerr(f"Failed to publish to topic: {e}")


def listener():
    rospy.init_node('listener_from_id_29', anonymous=True)
    rospy.Subscriber("/received_canbus_messages", Frame, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()