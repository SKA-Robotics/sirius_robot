#!/usr/bin/env python3

from math import pi
from abc import ABC, abstractmethod

import rclpy
from rclpy.node import Node as RclpyNode
from sensor_msgs.msg import JointState


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
        from odrive.enums import AXIS_STATE_IDLE
        self._context.axis.requested_state = AXIS_STATE_IDLE

    def set_joint_state(self, msg):
        if len(msg.velocity) > 0 or len(msg.effort) > 0:
            self.change_state(msg)


class MotorVelocity(MotorState):

    def setup(self):
        from odrive.enums import (AXIS_STATE_CLOSED_LOOP_CONTROL,
                                  CONTROL_MODE_VELOCITY_CONTROL,
                                  INPUT_MODE_VEL_RAMP)
        self._context.axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self._context.axis.controller.config.input_mode = INPUT_MODE_VEL_RAMP
        self._context.axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        self._context.axis.controller.input_vel = 0.0

    def set_joint_state(self, msg):
        if len(msg.velocity) == 0:
            self.change_state(msg)
            return
        self._context.axis.controller.input_vel = (
            msg.velocity[0] * self._context.direction *
            self._context.transmission / (2 * pi))


class MotorEffort(MotorState):

    def setup(self):
        from odrive.enums import (AXIS_STATE_CLOSED_LOOP_CONTROL,
                                  CONTROL_MODE_TORQUE_CONTROL,
                                  INPUT_MODE_PASSTHROUGH)
        self._context.axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self._context.axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        self._context.axis.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
        self._context.axis.controller.input_torque = 0.0

    def set_joint_state(self, msg):
        if len(msg.effort) == 0:
            self.change_state(msg)
            return
        self._context.axis.controller.input_torque = (
            msg.effort[0] * self._context.direction)


class Motor:

    def __init__(self, namespace, axis, odrv, ros_node: RclpyNode) -> None:
        self.namespace = namespace
        self.axis = axis
        self.odrv = odrv
        self._node = ros_node

        self.direction = (-1 if ros_node.get_parameter(
            f'{namespace}reverse_motor').value else 1)
        self.effort_direction = self.direction * (-1 if ros_node.get_parameter(
            f'{namespace}reverse_effort').value else 1)
        self.transmission = ros_node.get_parameter(
            f'{namespace}transmission').value

        self.torque_constant = axis.motor.config.torque_constant

        self.publisher = ros_node.create_publisher(
            JointState, f'driver_raw/{namespace}joint_state', 10)
        ros_node.create_subscription(
            JointState,
            f'driver_raw/{namespace}set_joint_state',
            self.set_joint_state,
            1,
        )

        self.set_state(MotorIdle())

    def set_state(self, new_state):
        self.state = new_state
        self.state.context = self
        self.state.setup()

    def step(self):
        msg = JointState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.name = ['motor']
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

    def set_joint_state(self, msg: JointState):
        self.state.set_joint_state(msg)

    def disable(self):
        from odrive.enums import AXIS_STATE_IDLE
        self.axis.requested_state = AXIS_STATE_IDLE


class Driver:

    def __init__(self, namespace, ros_node: RclpyNode) -> None:
        import odrive
        self.namespace = namespace
        serial_number = ros_node.get_parameter(
            f'{namespace}serial_number').value
        self.odrv = odrive.find_any(serial_number=serial_number, timeout=3)
        self.odrv.clear_errors()

        has_motor0 = ros_node.has_parameter(f'{namespace}motor0.serial_number')
        has_motor1 = ros_node.has_parameter(f'{namespace}motor1.serial_number')

        self.motor0 = (Motor(f'{namespace}motor0/', self.odrv.axis0,
                             self.odrv, ros_node) if has_motor0 else None)
        self.motor1 = (Motor(f'{namespace}motor1/', self.odrv.axis1,
                             self.odrv, ros_node) if has_motor1 else None)

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


class ODriveDriverNode(RclpyNode):

    def __init__(self) -> None:
        super().__init__('odrive_driver')

        self.declare_parameter('drivers', rclpy.Parameter.Type.STRING)

        import yaml
        raw = self.get_parameter('drivers').value
        if isinstance(raw, str):
            driver_list = yaml.safe_load(raw) or []
        else:
            driver_list = raw or []

        for driver_name in driver_list:
            ns = f'{driver_name}/'
            self.declare_parameter(f'{ns}serial_number', '')
            for motor in ('motor0', 'motor1'):
                self.declare_parameter(f'{ns}{motor}/reverse_motor', False)
                self.declare_parameter(f'{ns}{motor}/reverse_effort', False)
                self.declare_parameter(f'{ns}{motor}/transmission', 1.0)

        self.drivers = [Driver(f'{d}/', self) for d in driver_list]

        # 100 Hz step timer
        self.create_timer(0.01, self._step)

    def _step(self):
        for driver in self.drivers:
            driver.step()

    def disable_all(self):
        for driver in self.drivers:
            driver.disable()


def main(args=None):
    rclpy.init(args=args)
    node = ODriveDriverNode()
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
