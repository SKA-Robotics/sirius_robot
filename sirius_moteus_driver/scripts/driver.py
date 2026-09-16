#!/usr/bin/env python3

import asyncio
from math import pi

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class Joint:
    def __init__(self, name: str, node: Node) -> None:
        self.name = name
        self.node_id = node.get_parameter(f'joints.{name}.node_id').value

        import moteus
        self._moteus = moteus
        self.controller = moteus.Controller(self.node_id)

        self.cmd_position = float('nan')
        self.cmd_velocity = float('nan')
        self.feedforward_torque = 0.0
        self.position = 0.0
        self.velocity = 0.0
        self.torque = 0.0

    async def start(self):
        await self.controller.set_stop()

    async def step(self):
        import math
        if not math.isnan(self.cmd_position) or not math.isnan(self.cmd_velocity):
            state = await self.controller.set_position(
                position=self.cmd_position,
                velocity=self.cmd_velocity,
                feedforward_torque=self.feedforward_torque,
                query=True)
        else:
            state = await self.controller.set_stop(query=True)

        self.position = state.values[self._moteus.Register.POSITION]
        self.velocity = state.values[self._moteus.Register.VELOCITY]
        self.torque = state.values[self._moteus.Register.TORQUE]

    async def disable(self):
        await self.controller.set_stop()


class MoteusDriverNode(Node):
    def __init__(self) -> None:
        super().__init__('moteus_driver')

        self.declare_parameter('joints', rclpy.Parameter.Type.STRING)

        import yaml
        raw = self.get_parameter('joints').value
        if isinstance(raw, str):
            joints_config = yaml.safe_load(raw) or {}
        else:
            joints_config = raw or {}

        for joint_name in joints_config.keys():
            self.declare_parameter(f'joints.{joint_name}.node_id', 0)

        self.joints = {name: Joint(name, self) for name in joints_config.keys()}

        self.command_subscriber = self.create_subscription(
            JointState, '/set_joint_states', self._receive_command, 10)
        self.joint_state_publisher = self.create_publisher(
            JointState, '/joint_states', 10)

    def _receive_command(self, msg: JointState):
        for joint in self.joints.values():
            joint.cmd_position = float('nan')
            joint.cmd_velocity = float('nan')
            joint.feedforward_torque = 0.0

        for i, name in enumerate(msg.name):
            if name not in self.joints:
                continue
            joint = self.joints[name]
            if len(msg.position) == len(msg.name):
                joint.cmd_position = msg.position[i] / (2 * pi)
            if len(msg.velocity) == len(msg.name):
                joint.cmd_velocity = msg.velocity[i] / (2 * pi)
            if len(msg.effort) == len(msg.name):
                joint.feedforward_torque = msg.effort[i]

    async def step(self):
        try:
            await asyncio.wait_for(
                asyncio.gather(*[j.step() for j in self.joints.values()]),
                timeout=0.05)
        except asyncio.TimeoutError:
            return
        except RuntimeError as e:
            self.get_logger().warn(str(e))
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for joint in self.joints.values():
            msg.name.append(joint.name)
            msg.position.append(joint.position * 2 * pi)
            msg.velocity.append(joint.velocity * 2 * pi)
            msg.effort.append(joint.torque)
        self.joint_state_publisher.publish(msg)

    async def disable(self):
        try:
            await asyncio.wait_for(
                asyncio.gather(*[j.disable() for j in self.joints.values()]),
                timeout=0.05)
        except asyncio.TimeoutError:
            pass


async def _main_async():
    rclpy.init()
    node = MoteusDriverNode()
    period = 1.0 / 20  # 20 Hz
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)
            await node.step()
            await asyncio.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        await node.disable()
        node.destroy_node()
        rclpy.shutdown()


def main():
    asyncio.run(_main_async())


if __name__ == '__main__':
    main()
