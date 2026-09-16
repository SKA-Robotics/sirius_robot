#!/usr/bin/env python3

from math import pi
from abc import ABC, abstractmethod

import rclpy
from rclpy.node import Node as RclpyNode
from sensor_msgs.msg import JointState
from std_msgs.msg import String


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

    def _make_msg(self, node: RclpyNode) -> JointState:
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name = ['motor']
        return msg


class StateIdle(State):

    def setup(self):
        self._context.state_publisher.publish(String(data='Idle'))

    def step(self):
        if any(j.velocity_setpoint != 0 for j in self._context.joints):
            self._context.set_state(StateRunning())
            return
        for joint in self._context.joints:
            msg = self._make_msg(self._context)
            joint.publisher.publish(msg)


class StateRunning(State):

    def setup(self):
        self._context.state_publisher.publish(String(data='Running'))

    def step(self):
        if all(j.velocity_setpoint == 0 for j in self._context.joints):
            self._context.set_state(StateBreaking())
            return
        for joint in self._context.joints:
            msg = self._make_msg(self._context)
            msg.velocity = [joint.velocity_setpoint]
            joint.publisher.publish(msg)


class StateBreaking(State):

    def setup(self):
        self._context.state_publisher.publish(String(data='Breaking'))

    def step(self):
        if any(j.velocity_setpoint != 0 for j in self._context.joints):
            self._context.set_state(StateRunning())
            return
        if sum(abs(j.velocity) for j in self._context.joints) < 0.1:
            self._context.set_state(StateRelaxing())
            return
        for joint in self._context.joints:
            msg = self._make_msg(self._context)
            msg.velocity = [0.0]
            joint.publisher.publish(msg)


class StateRelaxing(State):

    def setup(self):
        self._context.state_publisher.publish(String(data='Relaxing'))
        self.start_efforts = [j.effort for j in self._context.joints]
        self.starting_position = sum(j.position for j in self._context.joints)
        self.progress = 0.0

    def step(self):
        if any(j.velocity_setpoint != 0 for j in self._context.joints):
            self._context.set_state(StateRunning())
            return
        if self.progress >= 1.0:
            self._context.set_state(StateIdle())
            return
        if (abs(sum(j.position for j in self._context.joints) -
                self.starting_position) >
                2 * pi * self._context.lock_threshold):
            self._context.set_state(StateLocked())
            return
        torque_fraction = (self.progress - 1) ** 4
        for joint, start_effort in zip(self._context.joints,
                                       self.start_efforts):
            msg = self._make_msg(self._context)
            msg.effort = [start_effort * torque_fraction]
            joint.publisher.publish(msg)
        self.progress += (self._context.time_since_last_step /
                          self._context.relax_time)


class StateLocked(State):

    def setup(self):
        self._context.state_publisher.publish(String(data='Locked'))

    def step(self):
        if any(j.velocity_setpoint != 0 for j in self._context.joints):
            self._context.set_state(StateRunning())
            return
        for joint in self._context.joints:
            msg = self._make_msg(self._context)
            msg.velocity = [0.0]
            joint.publisher.publish(msg)


class JointProxy:

    def __init__(self, name: str, ros_node: RclpyNode) -> None:
        self.name = name
        self.velocity_setpoint = 0.0
        self.position = 0.0
        self.velocity = 0.0
        self.effort = 0.0

        self.publisher = ros_node.create_publisher(
            JointState, f'driver_raw{name}/set_joint_state', 10)
        ros_node.create_subscription(
            JointState, f'driver{name}/set_joint_state',
            self._set_joint_state, 1)
        ros_node.create_subscription(
            JointState, f'{name}/joint_state', self._joint_state, 1)

    def _set_joint_state(self, msg: JointState):
        if msg.velocity:
            self.velocity_setpoint = msg.velocity[0]
        else:
            self.velocity_setpoint = 0.0

    def _joint_state(self, msg: JointState):
        if msg.position:
            self.position = msg.position[0]
        if msg.velocity:
            self.velocity = msg.velocity[0]
        if msg.effort:
            self.effort = msg.effort[0]


class RelaxingMiddlewareNode(RclpyNode):

    def __init__(self) -> None:
        super().__init__('relaxing_middleware')

        self.declare_parameter('joints', rclpy.Parameter.Type.STRING)
        self.declare_parameter('relax_time', 1.0)
        self.declare_parameter('lock_threshold', 0.1)

        import yaml
        raw = self.get_parameter('joints').value
        if isinstance(raw, str):
            joint_list = yaml.safe_load(raw) or []
        else:
            joint_list = raw or []

        self.relax_time = self.get_parameter('relax_time').value
        self.lock_threshold = self.get_parameter('lock_threshold').value

        self.joints = [JointProxy(name, self) for name in joint_list]

        self.state_publisher = self.create_publisher(String, '~/state', 10)
        self._last_step_time = self.get_clock().now()
        self.time_since_last_step = 0.0

        self.set_state(StateIdle())

        # 100 Hz
        self.create_timer(0.01, self._timer_callback)

    def set_state(self, new_state: State):
        self.state = new_state
        self.state.context = self
        self.state.setup()

    def _timer_callback(self):
        now = self.get_clock().now()
        self.time_since_last_step = (
            now - self._last_step_time).nanoseconds / 1e9
        self._last_step_time = now
        self.state.step()


def main(args=None):
    rclpy.init(args=args)
    node = RelaxingMiddlewareNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
