#!/usr/bin/env python3
"""RoboClaw motor controller driver for Sirius rover (ROS2 Jazzy port)."""

import serial
import time
from copy import deepcopy
from collections import OrderedDict
from math import pi, copysign

import rclpy
from rclpy.node import Node
from sirius_msgs.msg import JointState


def _load_param(node: Node, name: str, default):
    """Declare (if needed) and get a parameter, applying the default."""
    if not node.has_parameter(name):
        node.declare_parameter(name, default)
    return node.get_parameter(name).value


class RoboclawSingle:
    """Thin wrapper around a Roboclaw instance with a fixed address."""

    def __init__(self, roboclaw, address: int = 128):
        self.rc = roboclaw
        self.address = address

    def Open(self, *args, **kwargs):
        if not self.IsOpen():
            self.rc.Open(*args, **kwargs)

    def IsOpen(self):
        return getattr(self.rc, '_port', None) is not None and self.rc._port.is_open

    # Delegate remaining methods to the underlying roboclaw object
    def __getattr__(self, name):
        return getattr(self.rc, name)

    def safe_call(self, method, *args, message: str = '', **kwargs):
        try:
            return method(*args, **kwargs)
        except serial.SerialException as e:
            try:
                return method(*args, **kwargs)
            except serial.SerialException:
                if hasattr(self.rc, '_port'):
                    self.rc._port.close()
                return (0,)

    def Duty(self, val, *, motor: int):
        val = int(val * 32767 / 100)
        method = self.rc.DutyM1 if motor == 1 else self.rc.DutyM2
        return self.safe_call(method, self.address, val)

    def Speed(self, val, *, motor: int):
        method = self.rc.SpeedM1 if motor == 1 else self.rc.SpeedM2
        return self.safe_call(method, self.address, val)

    def ReadEnc(self, *, motor: int):
        method = self.rc.ReadEncM1 if motor == 1 else self.rc.ReadEncM2
        return self.safe_call(method, self.address)

    def ReadSpeed(self, *, motor: int):
        method = self.rc.ReadSpeedM1 if motor == 1 else self.rc.ReadSpeedM2
        return self.safe_call(method, self.address)

    def ReadCurrents(self):
        return self.safe_call(self.rc.ReadCurrents, self.address)

    def ReadPWMs(self):
        pwms = self.safe_call(self.rc.ReadPWMs, self.address)
        if pwms[0]:
            return (pwms[0], pwms[1] * 100 / 32767, pwms[2] * 100 / 32767)
        return (0, 0.0, 0.0)

    def SetMaxCurrent(self, val, *, motor: int):
        method = self.rc.SetM1MaxCurrent if motor == 1 else self.rc.SetM2MaxCurrent
        return self.safe_call(method, self.address, val)


class Motor:
    MAX_SPEED = (2 ** 16) // 2 - 1
    MAX_ACCEL = 2 ** 16 * 10

    def __init__(self, rc: RoboclawSingle, index: int, node: Node,
                 ns: str = '~'):
        self.rc = rc
        self.index = index
        self._node = node
        self.ns = f'{ns}motor{index}/'

        self.ticks_per_revolution = _load_param(
            node, f'{self.ns}ticks_per_revolution', 360.0)
        self.torque_constant = _load_param(
            node, f'{self.ns}torque_constant', 1.0)
        self.command_timeout = _load_param(
            node, f'{self.ns}command_timeout', 500)
        self.max_current = _load_param(node, f'{self.ns}max_current', 10.0)

        self.encoder_offset = 0
        self.joint_state = JointState()
        self.joint_state.position = [0.0]
        self.joint_state.velocity = [0.0]
        self.joint_state.acceleration = [0.0]
        self.joint_state.effort = [0.0]
        self.joint_state.duty = [0.0]
        self.prev_joint_state = deepcopy(self.joint_state)
        self.target_velocity = 0.0
        self.last_cmd_time = node.get_clock().now()

        self.publisher = node.create_publisher(
            JointState, f'{self.ns}joint_state', 1)
        node.create_subscription(
            JointState, f'{self.ns}set_joint_state',
            self._set_joint_state, 1)

    def rad2ticks(self, value: float) -> int:
        return int(value * self.ticks_per_revolution / (2 * pi))

    def ticks2rad(self, value: float) -> float:
        return value * 2 * pi / self.ticks_per_revolution

    def _set_joint_state(self, msg: JointState):
        self.last_cmd_time = self._node.get_clock().now()
        if msg.velocity:
            self.target_velocity = msg.velocity[0]
        elif msg.duty:
            self._apply_duty(msg.duty[0])
        elif msg.position:
            self._apply_position(msg.position[0])

    def _apply_duty(self, duty: float):
        self.rc.Duty(duty, motor=self.index)

    def _apply_position(self, pos_rad: float, vel_rad: float = None,
                        accel_rad: float = None):
        speed = self.rad2ticks(abs(vel_rad)) if vel_rad is not None else self.MAX_SPEED
        accel = self.rad2ticks(abs(accel_rad)) if accel_rad is not None else self.MAX_ACCEL
        pos = self.rad2ticks(pos_rad)
        method = (self.rc.rc.SpeedAccelDeccelPositionM1 if self.index == 1
                  else self.rc.rc.SpeedAccelDeccelPositionM2)
        self.rc.safe_call(method, self.rc.address, accel, speed, accel, pos, 1)

    def step(self, rate_hz: float):
        now = self._node.get_clock().now()
        dt_ms = (now - self.last_cmd_time).nanoseconds / 1e6

        if dt_ms > self.command_timeout:
            self.target_velocity = 0.0

        self.rc.SetMaxCurrent(int(100 * self.max_current), motor=self.index)

        if self.target_velocity != 0.0:
            self.rc.Speed(self.rad2ticks(self.target_velocity), motor=self.index)
        else:
            self.rc.Duty(0.0, motor=self.index)

        self.prev_joint_state = deepcopy(self.joint_state)
        self.joint_state.header.stamp = now.to_msg()

        enc = self.rc.ReadEnc(motor=self.index)
        if enc[0]:
            self.joint_state.position = [self.ticks2rad(enc[1])]

        spd = self.rc.ReadSpeed(motor=self.index)
        if spd[0]:
            self.joint_state.velocity = [self.ticks2rad(spd[1])]

        currents = self.rc.ReadCurrents()
        if currents[0]:
            self.joint_state.effort = [
                currents[self.index] * self.torque_constant / 100]

        pwm = self.rc.ReadPWMs()
        self.joint_state.duty = [pwm[self.index]]

        # Numerical differentiation for acceleration
        if self.prev_joint_state.velocity:
            dv = (self.prev_joint_state.velocity[0] -
                  self.joint_state.velocity[0])
            dt = 1.0 / rate_hz
            if dt > 0:
                alpha = 1.0 / rate_hz
                prev_accel = (self.joint_state.acceleration[0]
                              if self.joint_state.acceleration else 0.0)
                self.joint_state.acceleration = [
                    prev_accel + (1 - alpha) * (dv / dt - prev_accel)]

        self.publisher.publish(self.joint_state)


class RoboclawDriverNode(Node):

    def __init__(self):
        super().__init__('roboclaw_driver')

        self.declare_parameter('device', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('loop_rate', 10)
        self.declare_parameter('address', 128)

        device = self.get_parameter('device').value
        baudrate = self.get_parameter('baudrate').value
        self.rate_hz = self.get_parameter('loop_rate').value
        address = self.get_parameter('address').value

        import roboclaw.roboclaw as roboclaw_lib
        rc_raw = roboclaw_lib.Roboclaw(device, baudrate)
        self.rc = RoboclawSingle(rc_raw, address)

        self.get_logger().info('Connecting to RoboClaw...')
        self.rc.Open()
        if not self.rc.IsOpen():
            self.get_logger().fatal('Cannot open RoboClaw device')
            raise RuntimeError('Cannot open RoboClaw')

        self.motors = [
            Motor(self.rc, 1, self, '~/'),
            Motor(self.rc, 2, self, '~/'),
        ]

        period = 1.0 / self.rate_hz
        self.create_timer(period, self._step)

    def _step(self):
        self.rc.Open()
        if self.rc.IsOpen():
            for motor in self.motors:
                motor.step(self.rate_hz)

    def shutdown(self):
        for motor in self.motors:
            motor.target_velocity = 0.0
        if self.rc.IsOpen() and hasattr(self.rc.rc, '_port'):
            self.rc.rc._port.close()


def main(args=None):
    rclpy.init(args=args)
    node = RoboclawDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
