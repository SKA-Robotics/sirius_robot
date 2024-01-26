#!/usr/bin/env python3

# ROS imports
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

# Python imports
from math import pi
import odrive
from odrive.enums import (AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_IDLE,
                          CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_VEL_RAMP)


class Motor:

    def __init__(self, namespace, axis) -> None:
        self.namespace = namespace
        self.axis = axis

        self.direction = -1 if rospy.get_param(f"{namespace}reverse_motor",
                                               False) else 1
        self.transmission = rospy.get_param(f"{namespace}transmission", 1)

        self.axis.controller.config.input_mode = INPUT_MODE_VEL_RAMP
        self.axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL

        self.torque_constant = self.axis.motor.config.torque_constant

        self.publisher = rospy.Publisher(f"{namespace}joint_state",
                                         JointState,
                                         queue_size=10)
        rospy.Subscriber(f"{namespace}set_joint_state",
                         JointState,
                         self.set_joint_state,
                         queue_size=1)

    def step(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.name = ['motor']

        msg.position = [
            self.axis.encoder.pos_estimate * self.transmission * self.direction
        ]
        msg.velocity = [
            self.axis.encoder.vel_estimate * self.transmission * self.direction
        ]
        msg.effort = [
            self.axis.motor.current_control.Iq_setpoint * self.torque_constant
        ]

        self.publisher.publish(msg)

    def set_joint_state(self, msg):
        if msg.velocity[0] == 0:
            if self.axis.current_state != AXIS_STATE_IDLE:
                self.axis.requested_state = AXIS_STATE_IDLE
        elif self.axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
            self.axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.axis.controller.input_vel = msg.velocity[0] * \
            self.direction * self.transmission / (2 * pi)

    def disable(self):
        self.axis.requested_state = AXIS_STATE_IDLE


class Driver:

    def __init__(self, namespace) -> None:
        self.namespace = namespace
        self.serial_number = rospy.get_param(f"{namespace}serial_number")
        self.odrv = odrive.find_any(serial_number=self.serial_number,
                                    timeout=3)
        self.odrv.clear_errors()

        self.motor0 = Motor(
            f"{namespace}motor0/",
            self.odrv.axis0) if rospy.has_param(f"{namespace}motor0") else None
        self.motor1 = Motor(
            f"{namespace}motor1/",
            self.odrv.axis1) if rospy.has_param(f"{namespace}motor1") else None

    def step(self):
        if self.motor0 is not None:
            self.motor0.step()
        if self.motor1 is not None:
            self.motor1.step()

    def disable(self):
        if self.motor0 is not None:
            self.motor0.disable()
        if self.motor1 is not None:
            self.motor1.disable()


class Node:

    def __init__(self, name) -> None:
        rospy.init_node(name, anonymous=True)

        self.rate = rospy.Rate(20)
        self.drivers = []

        rospy.on_shutdown(self.disable)
        driver_list = rospy.get_param("~drivers")
        for driver in driver_list:
            self.drivers.append(Driver(f"~{driver}/"))

    def run(self):
        while not rospy.is_shutdown():
            self.step()
            self.rate.sleep()

    def step(self):
        for driver in self.drivers:
            driver.step()

    def disable(self):
        for driver in self.drivers:
            driver.disable()


if __name__ == '__main__':
    Node("odrive_driver").run()
