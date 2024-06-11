#!/usr/bin/env python3

import rospy
from math import pi
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, String
from abc import ABC, abstractmethod


class State(ABC):

    @property
    def context(self):
        return self._context

    @context.setter
    def context(self, context):
        self._context = context

    @abstractmethod
    def step(self):
        pass

    def make_msg(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.name = ["motor"]

        return msg


class StateIdle(State):

    def setup(self):
        self._context.state_publisher.publish("Idle")

    def step(self):
        if any(joint.velocity_setpoint != 0 for joint in self._context.joints):
            self._context.set_state(StateRunning())
            return

        for joint in self._context.joints:
            msg = self.make_msg()
            joint.publisher.publish(msg)


class StateRunning(State):

    def setup(self):
        self._context.state_publisher.publish("Running")

    def step(self):
        if all(joint.velocity_setpoint == 0 for joint in self._context.joints):
            self._context.set_state(StateBreaking())
            return

        for joint in self._context.joints:
            msg = self.make_msg()
            msg.velocity = [joint.velocity_setpoint]
            joint.publisher.publish(msg)


class StateBreaking(State):

    def setup(self):
        self._context.state_publisher.publish("Breaking")

    def step(self):
        if any(joint.velocity_setpoint != 0 for joint in self._context.joints):
            self._context.set_state(StateRunning())
            return

        if sum(abs(joint.velocity) for joint in self._context.joints) < 0.1:
            self._context.set_state(StateRelaxing())
            return

        for joint in self._context.joints:
            msg = self.make_msg()
            msg.velocity = [0.0]
            joint.publisher.publish(msg)


class StateRelaxing(State):

    def setup(self):
        self._context.state_publisher.publish("Relaxing")
        self.start_efforts = [joint.effort for joint in self._context.joints]
        self.starting_position = sum(joint.position
                                     for joint in self._context.joints)
        self.progress = 0.0

    def step(self):
        if any(joint.velocity_setpoint != 0 for joint in self._context.joints):
            self._context.set_state(StateRunning())
            return

        if self.progress >= 1.0:
            self._context.set_state(StateIdle())
            return

        if (abs(
                sum(joint.position for joint in self._context.joints) -
                self.starting_position)
                > 2 * pi * self._context.lock_threshold):
            self._context.set_state(StateLocked())
            return

        torque_fraction = (self.progress - 1)**4
        for joint, start_effort in zip(self._context.joints,
                                       self.start_efforts):
            msg = self.make_msg()

            msg.effort = [start_effort * torque_fraction]
            joint.publisher.publish(msg)

        self.progress += (self._context.time_since_last_callback /
                          self._context.relax_time)


class StateLocked(State):

    def setup(self):
        self._context.state_publisher.publish("Locked")

    def step(self):
        if any(joint.velocity_setpoint != 0 for joint in self._context.joints):
            self._context.set_state(StateRunning())
            return

        for joint in self._context.joints:
            msg = self.make_msg()
            msg.velocity = [0.0]
            joint.publisher.publish(msg)


class Joint:

    def __init__(self, name):
        self.name = name

        self.publisher = rospy.Publisher(f"driver_raw{name}/set_joint_state",
                                         JointState,
                                         queue_size=10)
        self.command_subscriber = rospy.Subscriber(
            f"driver{name}/set_joint_state",
            JointState,
            self.set_joint_state,
            queue_size=1,
        )
        self.state_subscriber = rospy.Subscriber(
            f"{name}/joint_state",
            JointState,
            self.joint_state,
            queue_size=1,
        )

        self.velocity_setpoint = 0.0
        self.position = 0.0
        self.velocity = 0.0
        self.effort = 0.0

    def set_joint_state(self, msg):
        self.velocity_setpoint = msg.velocity[0]

    def joint_state(self, msg):
        self.position = msg.position[0]
        self.velocity = msg.velocity[0]
        self.effort = msg.effort[0]


class Node:

    def __init__(self, name) -> None:
        rospy.init_node(name, anonymous=True)

        self.rate = rospy.Rate(100)

        rospy.on_shutdown(self.disable)
        joint_list = rospy.get_param(f"~joints")
        self.relax_time = rospy.get_param(f"~relax_time")
        self.lock_threshold = rospy.get_param(f"~lock_threshold")

        self.joints = []
        for joint in joint_list:
            self.joints.append(Joint(joint))

        self.last_callback_time = rospy.get_time()

        self.state_publisher = rospy.Publisher(f"~state",
                                               String,
                                               queue_size=10)
        self.set_state(StateIdle())

    def set_state(self, new_state):
        self.state = new_state
        self.state.context = self
        self.state.setup()
        self.state.step()

    def run(self):
        while not rospy.is_shutdown():
            callback_time = rospy.get_time()
            self.time_since_last_callback = rospy.get_time(
            ) - self.last_callback_time
            self.last_callback_time = callback_time

            self.step()
            self.rate.sleep()

    def step(self):
        self.state.step()

    def disable(self):
        self.set_state(StateIdle())


if __name__ == "__main__":
    node = Node("relaxing_middleware")
    node.run()
