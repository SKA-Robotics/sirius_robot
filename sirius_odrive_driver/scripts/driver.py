#!/usr/bin/env python3

from odrive.enums import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_IDLE,
    CONTROL_MODE_VELOCITY_CONTROL,
    CONTROL_MODE_TORQUE_CONTROL,
    INPUT_MODE_VEL_RAMP,
    INPUT_MODE_PASSTHROUGH,
)
import rospy
import odrive
from odrive.utils import dump_errors

from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from math import pi
from abc import ABC, abstractmethod

import time


class MotorState(ABC):

    @property
    def context(self):
        return self._context

    @context.setter
    def context(self, context):
        self._context = context

    @abstractmethod
    def set_joint_state(self, msg):
        pass

    def change_state(self, msg):
        if len(msg.velocity) > 0:
            self._context.set_state(MotorVelocity())
        elif len(msg.effort) > 0:
            self._context.set_state(MotorEffort())
        else:
            self._context.set_state(MotorIdle())

        self._context.state.set_joint_state(msg)


class MotorIdle(MotorState):

    def setup(self):
        self._context.axis.requested_state = AXIS_STATE_IDLE

    def set_joint_state(self, msg):
        if len(msg.velocity) > 0 or len(msg.effort) > 0:
            self.change_state(msg)


class MotorVelocity(MotorState):

    def setup(self):
        self._context.axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self._context.axis.controller.config.input_mode = INPUT_MODE_VEL_RAMP
        self._context.axis.controller.config.control_mode = (
            CONTROL_MODE_VELOCITY_CONTROL)
        self._context.axis.controller.input_vel = 0.0

    def set_joint_state(self, msg):
        if len(msg.velocity) == 0:
            self.change_state(msg)
            return

        self._context.axis.controller.input_vel = (msg.velocity[0] *
                                                   self._context.direction *
                                                   self._context.transmission /
                                                   (2 * pi))


class MotorEffort(MotorState):

    def setup(self):
        self._context.axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self._context.axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        self._context.axis.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
        self._context.axis.controller.input_torque = 0.0

    def set_joint_state(self, msg):
        if len(msg.effort) == 0:
            self.change_state(msg)
            return

        self._context.axis.controller.input_torque = (msg.effort[0] *
                                                      self._context.direction)


class Motor:

    def __init__(self, namespace, axis, serial_number, odrv, id) -> None:
        self.namespace = namespace
        self.axis = axis
        self.serial_number = serial_number
        self.odrv = odrv
        self.id = id

        self.direction = (-1 if rospy.get_param(f"~{namespace}reverse_motor",
                                                False) else 1)
        self.effort_direction = self.direction * (-1 if rospy.get_param(
            f"~{namespace}reverse_effort", False) else 1)
        self.transmission = rospy.get_param(f"~{namespace}transmission", 1)

        self.torque_constant = self.axis.motor.config.torque_constant

        self.publisher = rospy.Publisher(f"driver_raw/{namespace}joint_state",
                                         JointState,
                                         queue_size=10)
        rospy.Subscriber(
            f"driver_raw/{namespace}set_joint_state",
            JointState,
            self.set_joint_state,
            queue_size=1,
        )

        self.set_state(MotorIdle())

    def set_state(self, new_state):
        self.state = new_state
        self.state.context = self
        self.state.setup()

    def step(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.name = ["motor"]

        msg.position = [
            self.axis.encoder.pos_estimate / self.transmission *
            self.direction * 2 * pi
        ]
        msg.velocity = [
            self.axis.encoder.vel_estimate / self.transmission *
            self.direction * 2 * pi
        ]
        msg.effort = [
            self.axis.motor.current_control.Iq_setpoint *
            self.torque_constant * self.effort_direction
        ]

        self.publisher.publish(msg)

    def set_joint_state(self, msg):
        self.state.set_joint_state(msg)

    def disable(self):
        self.axis.requested_state = AXIS_STATE_IDLE


class Driver:

    def __init__(self, namespace) -> None:
        self.namespace = namespace
        self.serial_number = rospy.get_param(f"~{namespace}serial_number")
        self.odrv = odrive.find_any(serial_number=self.serial_number,
                                    timeout=3)
        self.odrv.clear_errors()

        self.motor0 = (Motor(f"{namespace}motor0/", self.odrv.axis0,
                             self.serial_number, self.odrv, 0)
                       if rospy.has_param(f"{namespace}motor0") else None)
        self.motor1 = (Motor(f"{namespace}motor1/", self.odrv.axis1,
                             self.serial_number, self.odrv, 1)
                       if rospy.has_param(f"{namespace}motor1") else None)

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

        self.rate = rospy.Rate(100)
        self.drivers = []

        rospy.on_shutdown(self.disable)
        driver_list = rospy.get_param(f"~drivers")
        for driver in driver_list:
            self.drivers.append(Driver(f"{driver}/"))

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


if __name__ == "__main__":
    node = Node("odrive_driver")
    node.run()
