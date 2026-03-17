#!/usr/bin/env python3
import os
import sys
sys.path.insert(0, os.path.dirname(__file__))

import random
from math import pi

import rclpy
from rclpy.node import Node as RclpyNode

from can_msgs.msg import Frame
from sensor_msgs.msg import JointState
from sirius_roboszpon_driver.msg import RoboszponStatus

import roboszpon_interface


class RosRoboszponInterface(roboszpon_interface.RoboszponInterface):
    def __init__(self, node_id, ros_node: RclpyNode):
        super().__init__(node_id)
        self._ros_node = ros_node
        self.frame_publisher = ros_node.create_publisher(
            Frame, '/sent_canbus_messages', 10)

    def send_can_frame(self, frame_id, data):
        frame = Frame()
        frame.data = list(data.to_bytes(8, byteorder='big'))
        frame.dlc = 8
        frame.id = frame_id
        frame.is_extended = False
        self.frame_publisher.publish(frame)


class Joint:
    def __init__(self, name: str, ros_node: RclpyNode) -> None:
        self.name = name
        self.node_id = ros_node.get_parameter(
            f'joints.{name}.node_id').value
        self.interface = RosRoboszponInterface(self.node_id, ros_node)
        self._ros_node = ros_node
        self.mode = 'INIT'
        self.last_update_time = ros_node.get_clock().now()
        self.reset_readings()

    def reset_readings(self):
        self.temperature = None
        self.flags = []
        self.position = None
        self.velocity = None
        self.current = None
        self.duty = 0

    def are_readings_valid(self):
        if self.mode != 'RUNNING':
            return False
        if None in [self.temperature, self.position,
                    self.velocity, self.current, self.duty]:
            return False
        return True

    def process_frame(self, frame: Frame):
        msg = self.interface.decode_message(
            frame.id, bytes(frame.data))
        if msg['message_id'] == roboszpon_interface.MSG_STATUS_REPORT:
            self.last_update_time = self._ros_node.get_clock().now()
            self.mode = roboszpon_interface.ROBOSZPON_MODES[msg['mode']]
            self.temperature = msg['temperature']
            self.flags = msg['flags']
        if msg['message_id'] == roboszpon_interface.MSG_MOTOR_REPORT:
            self.duty = msg['duty']
            self.current = msg['current']
        if msg['message_id'] == roboszpon_interface.MSG_AXIS_REPORT:
            self.position = msg['position']
            self.velocity = msg['velocity']

    def step(self):
        if self.mode == 'STOPPED':
            self.interface.arm()
        elapsed = (self._ros_node.get_clock().now() -
                   self.last_update_time).nanoseconds / 1e9
        if elapsed > 0.5:
            self.mode = 'TIMEOUT'
            self.reset_readings()

    def set_position(self, position):
        self.interface.send_position_command(position)

    def set_velocity(self, velocity):
        self.interface.send_velocity_command(velocity)

    def set_effort(self, effort):
        self.interface.send_duty_command(effort)

    def disable(self):
        self.interface.disarm()


class RoboszponDriverNode(RclpyNode):
    def __init__(self) -> None:
        super().__init__('roboszpon_driver')

        self.declare_parameter('joints', rclpy.Parameter.Type.STRING)

        import yaml
        raw = self.get_parameter('joints').value
        if isinstance(raw, str):
            joints_config = yaml.safe_load(raw)
        else:
            joints_config = raw or {}

        # declare per-joint parameters
        for joint_name in joints_config.keys():
            self.declare_parameter(f'joints.{joint_name}.node_id', 0)

        self.joints = {
            name: Joint(name, self)
            for name in joints_config.keys()
        }

        self.frame_subscriber = self.create_subscription(
            Frame,
            '/received_canbus_messages',
            self.receive_raw_frame,
            10,
        )
        self.command_subscriber = self.create_subscription(
            JointState,
            '/set_joint_states',
            self.receive_command,
            10,
        )
        self.joint_state_publisher = self.create_publisher(
            JointState, '/joint_states', 10)
        self.status_publisher = self.create_publisher(
            RoboszponStatus, '/roboszpon_status', 10)

        # 5 Hz timer replaces the ROS1 rate loop
        self.create_timer(0.2, self.step)

    def receive_raw_frame(self, frame: Frame):
        node_id = (frame.id >> 5) & 0b111111
        for joint in self.joints.values():
            if node_id == joint.node_id:
                joint.process_frame(frame)

    def receive_command(self, msg: JointState):
        for i, joint_name in sorted(enumerate(msg.name),
                                    key=lambda _: random.random()):
            if joint_name not in self.joints:
                continue
            joint = self.joints[joint_name]
            if joint.mode != 'RUNNING':
                continue
            if len(msg.position) == len(msg.name):
                joint.set_position(msg.position[i] / (2 * pi))
            if len(msg.velocity) == len(msg.name):
                joint.set_velocity(msg.velocity[i] / (2 * pi))
            if len(msg.effort) == len(msg.name):
                joint.set_effort(msg.effort[i])

    def step(self):
        for joint in self.joints.values():
            joint.step()

        now = self.get_clock().now().to_msg()

        js_msg = JointState()
        js_msg.header.stamp = now
        for joint in self.joints.values():
            if joint.are_readings_valid():
                js_msg.name.append(joint.name)
                js_msg.position.append(joint.position * 2 * pi)
                js_msg.velocity.append(joint.velocity * 2 * pi)
                js_msg.effort.append(joint.current)
        self.joint_state_publisher.publish(js_msg)

        status_msg = RoboszponStatus()
        status_msg.header.stamp = now
        for joint in self.joints.values():
            if joint.are_readings_valid():
                status_msg.name.append(joint.name)
                status_msg.mode.append(joint.mode)
                status_msg.flags.append(str(joint.flags))
                status_msg.temperature.append(joint.temperature)
                status_msg.duty.append(joint.duty)
        self.status_publisher.publish(status_msg)

    def disable_all(self):
        for joint in self.joints.values():
            joint.disable()


def main(args=None):
    rclpy.init(args=args)
    node = RoboszponDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.disable_all()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()