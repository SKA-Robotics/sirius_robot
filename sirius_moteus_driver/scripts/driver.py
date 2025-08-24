#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from math import pi
import math
import moteus
import asyncio


class Joint:

    def __init__(self, name) -> None:
        self.name = name
        self.node_id = rospy.get_param(f"~joints/{name}/node_id")

        self.controller = moteus.Controller(self.node_id)

        self.cmd_position = float("nan")
        self.cmd_velocity = float("nan")
        self.feedforward_torque = 0

        self.position = 0
        self.velocity = 0
        self.torque = 0

    async def start(self):
        # clear errors and ensure motor is stopped
        await self.controller.set_stop()

        self.controller.query()

    async def step(self):
        if not math.isnan(self.cmd_position) or not math.isnan(
                self.cmd_velocity):
            self.state = await self.controller.set_position(
                position=self.cmd_position,
                velocity=self.cmd_velocity,
                feedforward_torque=self.feedforward_torque,
                query=True)
        else:
            self.state = await self.controller.set_stop(query=True)

        self.position = self.state.values[moteus.Register.POSITION]
        self.velocity = self.state.values[moteus.Register.VELOCITY]
        self.torque = self.state.values[moteus.Register.TORQUE]

    async def disable(self):
        await self.controller.set_stop()


class Node:

    def __init__(self, name) -> None:
        rospy.init_node(name, anonymous=True)

        self.rate = rospy.Rate(20)
        self.joints = {}

        joint_list = rospy.get_param(f"~joints").keys()
        for joint in joint_list:
            self.joints[joint] = Joint(f"{joint}")

        self.command_subscriber = rospy.Subscriber("/set_joint_states",
                                                   JointState,
                                                   self.receive_command)
        self.joint_state_publisher = rospy.Publisher("/joint_states",
                                                     JointState,
                                                     queue_size=10)

    def receive_command(self, msg: JointState):
        for joint in self.joints.keys():
            self.joints[joint].cmd_position = float("nan")
            self.joints[joint].cmd_velocity = float("nan")
            self.joints[joint].feedforward_torque = 0

        for i, joint in enumerate(msg.name):
            if joint not in self.joints.keys():
                continue

            if len(msg.position) == len(msg.name):
                self.joints[joint].cmd_position = msg.position[i] / (2 * pi)
            if len(msg.velocity) == len(msg.name):
                self.joints[joint].cmd_velocity = msg.velocity[i] / (2 * pi)
            if len(msg.effort) == len(msg.name):
                self.joints[joint].feedforward_torque = msg.effort[i]

    async def run(self):
        while not rospy.is_shutdown():
            await self.step()
            self.rate.sleep()

        await self.disable()

    async def step(self):
        try:
            await asyncio.wait_for(
                asyncio.gather(
                    *[joint.step() for joint in self.joints.values()]), 0.05)
        except asyncio.TimeoutError:
            pass
        except RuntimeError as e:
            print(e)
            return

        msg = JointState()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        for joint in self.joints.values():
            msg.name.append(joint.name)
            msg.position.append(joint.position * 2 * pi)
            msg.velocity.append(joint.velocity * 2 * pi)
            msg.effort.append(joint.torque)

        self.joint_state_publisher.publish(msg)

    async def disable(self):
        try:
            await asyncio.wait_for(
                asyncio.gather(
                    *[joint.disable() for joint in self.joints.values()]),
                0.05)
        except asyncio.TimeoutError:
            pass


if __name__ == "__main__":
    node = Node("moteus_driver")
    asyncio.run(node.run())
