#!/usr/bin/env python3
import rospy
from sirius_msgs.msg import JointState

import roboclaw.roboclaw as roboclaw
import serial

from math import pi, copysign
from copy import deepcopy
from collections import OrderedDict

import serial.tools.list_ports


def loadParameter(parameter, default):
    # It would be a good idea to put it in some utility package
    # if not rospy.is_shutdown():
    #     return default
    if isinstance(default, dict):
        new_parameter = {}
        for key, value in default.items():
            namespace = f'{parameter}/{key}'
            new_parameter[key] = loadParameter(namespace, value)
    else:
        if not rospy.has_param(parameter):
            rospy.set_param(parameter, default)
        new_parameter = rospy.get_param_cached(parameter, default)

        # Warn if param is of different type than default value
        if type(new_parameter) != type(default):

            def check2(type_new, type_old):
                return isinstance(new_parameter, type_new) and isinstance(default, type_old)

            # Dont warn If conversion is safe
            if not (check2(int, float) or check2((int, float), str)):
                rospy.logwarn(
                    f'Parameter {parameter} should be of type {type(default).__name__} but is of type {type(new_parameter).__name__}'
                )
    return new_parameter


def autodetect():
    addresses = {128, 129, 130, 131, 132, 133, 134, 135}
    drivers = []

    ports = serial.tools.list_ports.comports()
    for port, desc, hwid in sorted(ports):
        if desc.lower().find("roboclaw") >= 0:
            rc = roboclaw.Roboclaw(port, 115200)
            try:
                rc.Open()
                if hasattr(rc, "_port") and rc._port.is_open:
                    for address in addresses:
                        try:
                            if rc.ReadNVM(address):
                                drivers.append((address, port))
                                addresses.remove(address)
                                break
                            else:
                                continue
                        except serial.SerialException as e:
                            continue
                    rc._port.close()
            except serial.SerialException as e:
                continue
    return drivers


class RoboclawSingle:
    CONFIG = {
        "PACKET_SERIAL_MODE": 3,
        "BATTERY_MODE": {
            "0": 0,  # Auto mode
            "6": 4,  # Off mode
            "9": 8,
            "12": 12,
            "15": 16,
            "18": 20,
            "21": 24,
            "24": 28
        },
        "BAUDRATE": {
            "2400": 0,
            "9600": 32,
            "19200": 64,
            "38400": 96,
            "57600": 128,
            "115200": 160,
            "230400": 192,
            "460800": 224
        },
        "PACKET_ADDRESS": {
            "128": 0,
            "129": 256,
            "130": 512,
            "131": 768,
            "132": 1024,
            "133": 1280,
            "134": 1536,
            "135": 1792
        },
        "RELAY_MODE": 4096,
        "SWAP_ENCODERS": 8192,
        "SWAP_BUTTONS": 16384,
        "MULTIUNIT_MODE": 32768
    }
    ENCODER_MODE = {
        "ABSOLUTE_ENCODER": 1,
        "NA1": 2,
        "NA2": 4,
        "NA3": 8,
        "NA4": 16,
        "REVERSE_MOTOR": 32,
        "REVERSE_ENCODER": 64,
        "RC_ANALOG": 128
    }

    # Doesn't inherit from roboclaw, so you can use multiple drivers on same port (but with different address)
    def __init__(self, roboclaw, address=128):
        self.rc = roboclaw
        self.address = address

        self.config_value = [0, 0]
        self.config = {key: False for key in self.CONFIG.keys()}
        self.config["BATTERY_MODE"] = "0"
        self.config["BAUDRATE"] = "2400"
        self.config["PACKET_ADDRESS"] = "128"

        self.encoder_mode_value = [0, 0, 0]
        self.encoder_mode = [{key: False for key in self.ENCODER_MODE.keys()}]
        self.encoder_mode.append(deepcopy(self.encoder_mode[0]))

    def Open(self, *args, **kwargs):
        if not self.IsOpen():
            self.rc.Open(*args, **kwargs)

    def IsOpen(self,):
        if hasattr(self.rc, "_port"):
            return self.rc._port.is_open
        else:
            return False

    def SetSerialTimeout(self, value):

        def SetSerialTimeout(timeout):
            SETSERTIALTIMEOUT = 14
            return self.rc._write1(self.address, SETSERTIALTIMEOUT, int(timeout * 10))

        return self.safe_call(SetSerialTimeout, value, message=f'Could not set communication timeout')

    def ReadSerialTimeout(self):

        def ReadSerialTimeout(address):
            GETSERTIALTIMEOUT = 15
            value = self.rc._read1(self.address, GETSERTIALTIMEOUT)
            return (value[0], value[1] / 10)

        return self.safe_call(ReadSerialTimeout, self.address, message=f'Could not read communication timeout')

    def GetConfig(self):
        config_value = self.safe_call(self.rc.GetConfig, self.address, message=f'Could not read configs')

        if not config_value[0]:
            config_value = self.config_value
        self.config_value[0] = config_value[0]
        if config_value[1] != self.config_value[1]:
            self.config_value[1] = config_value[1]
            config_value = config_value[1]

            def check(key):
                return bool(config_value & RoboclawSingle.CONFIG[key])

            def get(key):
                items = RoboclawSingle.CONFIG[key].items()
                items = [(int(key), val) for key, val in items if not val or config_value & val]
                return max(items, key=lambda item: item[1])[0]

            self.config = {
                "PACKET_SERIAL_MODE": check("PACKET_SERIAL_MODE"),
                "BATTERY_MODE": get("BATTERY_MODE"),
                "BAUDRATE": get("BAUDRATE"),
                "PACKET_ADDRESS": get("PACKET_ADDRESS"),
                "RELAY_MODE": check("RELAY_MODE"),
                "SWAP_ENCODERS": check("SWAP_ENCODERS"),
                "SWAP_BUTTONS": check("SWAP_BUTTONS"),
                "MULTIUNIT_MODE": check("MULTIUNIT_MODE")
            }
        return self.config

    def SetConfig(self, **kwargs):
        if not self.config_value[0]:
            self.GetConfig()
        if self.config_value[0]:
            config = deepcopy(self.config)
            config_value = 0
            for key, value in config.items():
                if key in kwargs:
                    value = kwargs[key]
                if isinstance(RoboclawSingle.CONFIG[key], dict):
                    value = RoboclawSingle.CONFIG[key][str(value)]
                else:
                    value = RoboclawSingle.CONFIG[key] if value else 0
                config_value += value
            if self.config_value[1] == config_value:
                return True
            if self.safe_call(self.rc.SetConfig, self.address, config_value, message=f'Could not set configs'):
                self.config_value[1] = config_value
                return True
        return False

    def Duty(self, val, *args, motor, **kwargs):
        val = int(val * 32767 / 100)
        return self.motor_call(motor,
                               self.rc.DutyM1,
                               self.rc.DutyM2,
                               val,
                               *args,
                               message=f'Could not change motor {motor} duty',
                               **kwargs)

    def Forward(self, *args, motor, **kwargs):
        return self.motor_call(motor,
                               self.rc.ForwardM1,
                               self.rc.ForwardM2,
                               *args,
                               message=f'Could not control motor {motor}!',
                               **kwargs)

    def ReadCurrents(self, *args, **kwargs):
        return self.safe_call(self.rc.ReadCurrents, self.address, message=f'Could not read motors currents')

    def ReadPWMs(self, *args, **kwargs):
        pwms = self.safe_call(self.rc.ReadPWMs, self.address, message=f'Could not read motors duty')
        if pwms[0]:
            (success, m1, m2) = pwms
            return (success, m1 * 100 / 32767, m2 * 100 / 32767)
        else:
            return (0, 0, 0)

    def ReadEnc(self, *args, motor, **kwargs):
        return self.motor_call(motor,
                               self.rc.ReadEncM1,
                               self.rc.ReadEncM2,
                               *args,
                               message=f'Could not read motor {motor} distance',
                               **kwargs)

    def ReadEncoderModes(self, motor=None):
        encoder_mode_value = self.safe_call(self.rc.ReadEncoderModes,
                                            self.address,
                                            message=f'Could not read encoders modes')

        if not encoder_mode_value[0]:
            encoder_mode_value = self.encoder_mode_value
        self.encoder_mode_value[0] = encoder_mode_value[0]
        for encoder in (1, 2):
            value = encoder_mode_value[encoder]
            if value != self.encoder_mode_value[encoder]:
                self.encoder_mode_value[encoder] = value

                def check(mask):
                    return bool(value & mask)

                enc_masks = self.ENCODER_MODE.items()
                enc_mode = {key: check(mask) for key, mask in enc_masks}
                self.encoder_mode[encoder - 1] = enc_mode
        if motor is None:
            return self.encoder_mode
        else:
            return self.encoder_mode[motor - 1]

    def SetEncoderMode(self, *args, motor, **kwargs):
        if not self.encoder_mode_value[0]:
            self.ReadEncoderModes(motor=motor)
        if self.encoder_mode_value[0]:
            encoder_mode = deepcopy(self.encoder_mode[motor - 1])
            encoder_mode_value = 0
            for key, value in encoder_mode.items():
                if key in kwargs:
                    mask = RoboclawSingle.ENCODER_MODE[key]
                    value = mask if kwargs[key] else 0
                encoder_mode_value += value
            if self.encoder_mode_value[motor] == encoder_mode_value:
                return True
            if self.motor_call(motor,
                               self.rc.SetM1EncoderMode,
                               self.rc.SetM2EncoderMode,
                               encoder_mode_value,
                               message=f'Could not set motor {motor} encoder mode'):
                self.encoder_mode_value[motor] = encoder_mode_value
                return True
        return False

    def ReadMaxCurrent(self, *args, motor, **kwargs):
        return self.motor_call(motor,
                               self.rc.ReadM1MaxCurrent,
                               self.rc.ReadM2MaxCurrent,
                               *args,
                               message=f'Could not read motor {motor}default current limit value',
                               **kwargs)

    def ReadPositionPID(self, *args, motor, **kwargs):
        return self.motor_call(motor,
                               self.rc.ReadM1PositionPID,
                               self.rc.ReadM2PositionPID,
                               *args,
                               message=f'Could not read motor {motor} default position PID settings',
                               **kwargs)

    def ReadSpeed(self, *args, motor, **kwargs):
        return self.motor_call(motor,
                               self.rc.ReadSpeedM1,
                               self.rc.ReadSpeedM2,
                               *args,
                               message=f'Could not read motor {motor} velocity',
                               **kwargs)

    def ReadVelocityPID(self, *args, motor, **kwargs):
        return self.motor_call(motor,
                               self.rc.ReadM1VelocityPID,
                               self.rc.ReadM2VelocityPID,
                               *args,
                               message=f'Could not read motor {motor} default velocity PID settings',
                               **kwargs)

    def ResetEncoders(self, *args, **kwargs):
        return self.safe_call(self.rc.ResetEncoders, self.address, message=f'Could not reset encoders')

    def SetEnc(self, *args, motor, **kwargs):
        return self.motor_call(motor,
                               self.rc.SetEncM1,
                               self.rc.SetEncM2,
                               *args,
                               message=f'Could not change motor {motor} encoder ticks count',
                               **kwargs)

    def SetMaxCurrent(self, *args, motor, **kwargs):
        return self.motor_call(motor,
                               self.rc.SetM1MaxCurrent,
                               self.rc.SetM2MaxCurrent,
                               *args,
                               message=f'Could not set motor {motor} max current',
                               **kwargs)

    def SetPositionPID(self, *args, motor, **kwargs):

        def SetPositionPID(address, kp, ki, kd, kimax, deadzone, min, max, motor):
            if motor == 1:
                CMD = self.rc.Cmd.SETM1POSPID
            else:
                CMD = self.rc.Cmd.SETM2POSPID
            return self._write4444444(address, CMD, int(kd * 1024), int(kp * 1024), int(ki * 1024), kimax, deadzone,
                                      min, max)

        return self.safe_call(SetPositionPID,
                              self.address,
                              *args,
                              **kwargs,
                              motor=motor,
                              message=f'Could not set motor {motor} position PID settings')

    def SetVelocityPID(self, *args, motor, **kwargs):

        def SetVelocityPID(address, p, i, d, qpps, motor):
            if motor == 1:
                CMD = self.rc.Cmd.SETM1PID
            else:
                CMD = self.rc.Cmd.SETM2PID
            return self._write4444(address, self.Cmd.SETM1PID, int(d * 65536), int(p * 65536), int(i * 65536), qpps)

        return self.safe_call(SetVelocityPID,
                              self.address,
                              *args,
                              **kwargs,
                              motor=motor,
                              message=f'Could not set motor {motor} velocity PID settings')

    def Speed(self, *args, motor, **kwargs):
        return self.motor_call(motor,
                               self.rc.SpeedM1,
                               self.rc.SpeedM2,
                               *args,
                               message=f'Could not change motor {motor} velocity',
                               **kwargs)

    def SpeedAccel(self, *args, motor, **kwargs):
        return self.motor_call(motor,
                               self.rc.SpeedAccelM1,
                               self.rc.SpeedAccelM2,
                               *args,
                               message=f'Could not change motor {motor} velocity',
                               **kwargs)

    def SpeedAccelDeccelPosition(self, *args, motor, **kwargs):
        return self.motor_call(motor,
                               self.rc.SpeedAccelDeccelPositionM1,
                               self.rc.SpeedAccelDeccelPositionM2,
                               *args,
                               message=f'Could not change motor {motor} position',
                               **kwargs)

    def ReadNVM(self, *args, **kwargs):
        return self.safe_call(self.rc.ReadNVM, self.address, message=f'Could not load roboclaw non-volatile memory')

    def safe_call(self, method, *args, message, **kwargs):
        try:
            return method(*args, **kwargs)
        except serial.SerialException as e:
            rospy.logerr(message + "\n Retrying...")
            rospy.logdebug(e)
            try:
                return method(*args, **kwargs)
            except serial.SerialException as e:
                rospy.logerr("Closing connection and trying to reconnect")
                self.rc._port.close()
                rospy.logerr(message)
                rospy.logdebug(e)
                return (0,)

    def motor_call(self, motor, method1, method2, *args, message, **kwargs):
        method = method1 if motor == 1 else method2
        return self.safe_call(method, self.address, *args, message=message, **kwargs)

    def __getattr__(self, name):
        return self.rc.__getattribute__(name)


class Node:

    def __init__(self, name):
        rospy.init_node(name, log_level=rospy.DEBUG)
        baudrate = loadParameter('~baudrate', 115200)
        self.rate = loadParameter('~loop_rate', 60)
        devices = autodetect()
        self.rc = []
        self.driver = []
        for address, port in devices:
            self.rc.append(roboclaw.Roboclaw(port, baudrate))
            self.driver.append(Driver(self.rc[-1], f"~{address}/", self.rate, address))

    def run(self):
        rate = rospy.Rate(self.rate)  # Every 100ms - 10Hz
        try:
            while not rospy.is_shutdown():
                for driver in self.driver:
                    driver.once()
                # rate.sleep()
        except rospy.ROSInterruptException:
            rospy.logdebug('Received ROSInterruptException')
        rospy.logdebug('Closing roboclaw node')
        self.shutdown()
        rospy.logdebug('Closed roboclaw')

    def shutdown(self):
        for driver in self.driver:
            driver.shutdown()
        for rc in self.rc:
            if hasattr(rc, "_port"):
                rc._port.close()


class Motor:
    MAX_SPEED = (2**16 / 2 - 1)
    MAX_ACCEL = 2**16 * 10

    def __init__(self, roboclaw, index=1, namespace="~", rate=10):
        self.encoder_offset = 0
        self.rc = roboclaw
        self.index = index
        self.rate = rate
        self.namespace = f'{namespace}motor{index}/'

        self.updateParameters()

        # Initialize messages variables
        self.joint_state = JointState(velocity=[0], position=[0], acceleration=[0], effort=[0], duty=[0])
        self.joint_state.header.stamp = rospy.get_rostime()

        self.previous_joint_state = deepcopy(self.joint_state)

        self.target_joint_state = JointState(velocity=[0])
        self.target_joint_state.header.stamp = rospy.get_rostime()

        # Subscribe to topics
        rospy.Subscriber(f'{self.namespace}set_joint_state', JointState, self.setJointState)
        rospy.Subscriber(f'{self.namespace}add_encoder_offset', JointState, self.addEncoderOffset)

        # Define topics publishers
        self.publisher = rospy.Publisher(f'{self.namespace}joint_state', JointState, queue_size=1)

        if not self.absolute_encoder:
            self.rc.SetEnc(0, motor=self.index)

    def once(self):
        # self.updateParameters()

        time = rospy.get_rostime()
        dt = (time - self.target_joint_state.header.stamp).to_sec() * 1000
        # Check for command timeout
        if dt > self.command_timeout:
            rospy.logwarn(f'Motor {self.index} command timeout')
            self.target_joint_state = JointState(velocity=[0])
            self.target_joint_state.header.stamp = rospy.get_rostime()

        # Define temporary variables
        def first_or_none(x):
            return x[0] if len(x) else None

        position = first_or_none(self.target_joint_state.position)
        velocity = first_or_none(self.target_joint_state.velocity)
        acceleration = first_or_none(self.target_joint_state.acceleration)
        effort = first_or_none(self.target_joint_state.effort)
        duty = first_or_none(self.target_joint_state.duty)

        # Limit motor currents
        self.limitEffort(effort)
        # Drive motor
        if duty is not None:
            self.setDuty(duty)
        elif position is not None:
            self.setPosition(position, velocity, acceleration)
        elif velocity is not None:
            self.setVelocity(velocity, acceleration)
        elif effort is not None:
            # Current is already limited, so just max out duty and let the driver handle it
            self.setDuty(copysign(100, effort))
        # Shift joint state memory
        self.previous_joint_state = deepcopy(self.joint_state)
        # Stamp and update current joint state
        self.joint_state.header.stamp = time
        self.updateDistance()
        self.updateVelocity()
        # Acceleration *HAS* to be updated after velocity
        self.updateAcceleration()
        self.updateEffort()
        self.updateDuty()
        # Publish motors joint state
        self.publisher.publish(self.joint_state)
        #  If motor is not moving, then apply encoder offfset
        if not self.absolute_encoder and self.joint_state.velocity == 0 and self.encoder_offset:
            distance = self.rad2ticks(self.joint_state.position[0])
            self.rc.SetEnc(distance + self.encoder_offset, motor=self.index)
            self.encoder_offset = 0

    def shutdown(self):
        rospy.logdebug(f'Closing roboclaw motor{self.index}')
        self.stop()

    def rad2ticks(self, value):
        # Converts value in radians to enconder ticks count
        return int(value * self.ticks_per_revolution / (2 * pi))

    def ticks2rad(self, value):
        # Converts enconder ticks count to value in radians
        return value * 2 * pi / self.ticks_per_revolution

    def setJointState(self, joint_state):
        self.target_joint_state = joint_state

    def addEncoderOffset(self, joint_state):
        self.encoder_offset += self.rad2ticks(joint_state.position[0])

    def setPosition(self, position, speed=None, acceleration=None):
        if speed is None:
            speed = Motor.MAX_SPEED
        if acceleration is None:
            acceleration = Motor.MAX_ACCEL
        self.rc.SpeedAccelDeccelPosition(self.rad2ticks(abs(acceleration)),
                                         self.rad2ticks(abs(speed)),
                                         self.rad2ticks(abs(acceleration)),
                                         self.rad2ticks(position),
                                         1,
                                         motor=self.index)

    def setVelocity(self, velocity, acceleration=None):
        if acceleration is None:
            if velocity == 0:
                self.stop()
            else:
                self.rc.Speed(self.rad2ticks(velocity), motor=self.index)
        else:
            self.rc.SpeedAccel(abs(acceleration), self.rad2ticksM1(velocity), motor=self.index)

    def limitEffort(self, effort=None):
        # Don't limit effort If it's not specified
        current = self.max_current if effort is None else min(self.max_current, abs(effort) / self.torque_constant)
        # Roboclaw reads current in 10mA. Hence, multiply by 100 to convert from Ampers.
        self.rc.SetMaxCurrent(int(100 * current), motor=self.index)

    def setDuty(self, value):
        self.rc.Duty(value, motor=self.index)

    def updateDistance(self):
        encoder_distance = self.rc.ReadEnc(motor=self.index)
        if encoder_distance[0]:
            self.joint_state.position = [self.ticks2rad(encoder_distance[1] + self.encoder_offset)]

    def updateVelocity(self):
        encoder_velocity = self.rc.ReadSpeed(motor=self.index)
        if encoder_velocity[0]:
            self.joint_state.velocity = [self.ticks2rad(encoder_velocity[1])]

    def updateAcceleration(self):
        dv = self.previous_joint_state.velocity[0] - self.joint_state.velocity[0]
        dt = (self.previous_joint_state.header.stamp - self.joint_state.header.stamp).to_sec()
        if dt:
            current_accel = dv / dt
            previous_accel = self.joint_state.acceleration[0]
            alpha = 1 / self.rate
            self.joint_state.acceleration = [previous_accel + (1 - alpha) * (current_accel - previous_accel)]

    def updateEffort(self):
        encoder_current = self.rc.ReadCurrents()
        if encoder_current[0]:
            # Roboclaw return current in 10mA. Hence, divide by 100 to get current in Ampers.
            self.joint_state.effort = [encoder_current[self.index] * self.torque_constant / 100]

    def getPositionPID(self):
        (P, I, D, MaxI, Deadzone, MinPos, MaxPos) = self.rc.ReadPositionPID(motor=self.index)[1:]
        Deadzone = self.ticks2rad(Deadzone)
        MinPos = self.ticks2rad(MinPos)
        MaxPos = self.ticks2rad(MaxPos)
        return {"P": P, "I": I, "D": D, "MaxI": MaxI, "Deadzone": Deadzone, "MinPos": MinPos, "MaxPos": MaxPos}

    def setPositionPID(self, P, I, D, MaxI, Deadzone, MinPos, MaxPos):
        Deadzone = self.rad2ticks(Deadzone)
        MinPos = self.rad2ticks(MinPos)
        MaxPos = self.rad2ticks(MaxPos)
        self.rc.SetPositionPID(P, I, D, MaxI, Deadzone, MinPos, MaxPos, motor=self.index)

    def getVelocityPID(self):
        (P, I, D, QPPS) = self.rc.ReadVelocityPID(motor=self.index)[1:]
        QPPS = self.ticks2rad(QPPS)
        return {"P": P, "I": I, "D": D, "QPPS": QPPS}

    def setVelocityPID(self, P, I, D, QPPS):
        QPPS = self.rad2ticks(QPPS)
        self.rc.SetVelocityPID(P, I, D, QPPS, motor=self.index)

    def updateDuty(self):
        self.joint_state.effort = [self.rc.ReadPWMs()[self.index]]

    def updateParameters(self):

        def load(param):
            old_param = self.__getattribute__(param)
            new_param = loadParameter(self.namespace + param, old_param)
            if str(new_param) == str(old_param):
                return False
            self.__dict__[param] = new_param
            return True

        if not hasattr(self, 'absolute_encoder'):
            encoder_mode = self.rc.ReadEncoderModes(motor=self.index)
            self.absolute_encoder = encoder_mode["ABSOLUTE_ENCODER"]
            self.reverse_encoder = encoder_mode["REVERSE_ENCODER"]
            self.reverse_motor = encoder_mode["REVERSE_MOTOR"]
        if load('absolute_encoder') or load('reverse_encoder') or load('reverse_motor'):
            self.rc.SetEncoderMode(ABSOLUTE_ENCODER=self.absolute_encoder,
                                   REVERSE_MOTOR=self.reverse_motor,
                                   REVERSE_ENCODER=self.reverse_encoder,
                                   motor=self.index)

        if not hasattr(self, 'ticks_per_revolution'):
            self.ticks_per_revolution = 360.0
        load('ticks_per_revolution')

        if not hasattr(self, 'torque_constant'):
            self.torque_constant = 1.0
        load('torque_constant')

        if not hasattr(self, 'command_timeout'):
            self.command_timeout = 500
        load('command_timeout')

        if not hasattr(self, 'max_current'):
            self.max_current = self.rc.ReadMaxCurrent(motor=self.index)[1]
        load('max_current')

        if not hasattr(self, 'velocity_pid'):
            self.velocity_pid = self.getVelocityPID()
        if load('velocity_pid'):
            self.setVelocityPID(**self.velocity_pid)

        if not hasattr(self, 'position_pid'):
            self.position_pid = self.getPositionPID()
        if load('position_pid'):
            self.setPositionPID(**self.position_pid)

    def stop(self):
        self.rc.Forward(0, motor=self.index)


class Driver:

    def __init__(self, roboclaw, namespace="~", rate=10, adress=128):
        self.rc = RoboclawSingle(roboclaw)
        self.namespace = namespace
        self.rate = rate

        rospy.loginfo(f'Connecting to roboclaw driver')  #: {self.device}')
        text = f'Could not open device'  # {self.device}'
        try:
            self.rc.Open()
            if not self.rc.IsOpen():
                rospy.logfatal(text)
                rospy.signal_shutdown(text)
                return
        except serial.SerialException as e:
            rospy.logfatal(text)
            rospy.logdebug(e)
            rospy.signal_shutdown(text)
            return
        # Load address
        self.address = adress
        self.rc.address = self.address
        # Load Roboclaw NVM
        if not self.rc.ReadNVM():
            text = 'Failed to load setting from roboclaw. Check If address is correct'
            rospy.logfatal(text)
            rospy.signal_shutdown(text)
            return
        # Load parameters
        self.updateParameters()

        rospy.loginfo(f'Driver ready')

    def once(self):
        # Open port If it's closed
        self.rc.Open()
        if self.rc.IsOpen():
            # Reload parameters
            # self.updateParameters()
            # Run motors routines
            for motor in self.motor.values():
                motor.once()

    def updateParameters(self):

        def load(param):
            old_param = self.__getattribute__(param)
            new_param = loadParameter(self.namespace + param, old_param)
            if str(new_param) == str(old_param):
                return False
            self.__dict__[param] = new_param
            return True

        if load('address'):
            self.rc.address = self.address

        if not hasattr(self, 'motors'):
            self.motors = [1, 2]
            self.motor = {}
            for motor in self.motors:
                self.motor[str(motor)] = Motor(self.rc, motor, self.namespace, rate=self.rate)
        if load('motors'):
            self.motor = {}
            for motor in self.motors:
                self.motor[str(motor)] = Motor(self.rc, motor, self.namespace, rate=self.rate)

        if not hasattr(self, 'communication_timeout'):
            self.communication_timeout = self.rc.ReadSerialTimeout()[1]
        if load('communication_timeout'):
            self.rc.SetSerialTimeout(self.communication_timeout)

        if not hasattr(self, 'multiunit_mode'):
            configs = self.rc.GetConfig()
            self.multiunit_mode = configs["MULTIUNIT_MODE"]
            self.relay_mode = configs["RELAY_MODE"]
            self.swap_buttons = configs["SWAP_BUTTONS"]
            self.swap_encoders = configs["SWAP_ENCODERS"]
        if load('multiunit_mode') or load('relay_mode') or load('swap_buttons') or load('swap_encoders'):
            self.rc.SetConfig(RELAY_MODE=self.relay_mode,
                              SWAP_ENCODERS=self.swap_encoders,
                              SWAP_BUTTONS=self.swap_buttons,
                              MULTIUNIT_MODE=self.multiunit_mode)

    def shutdown(self):
        rospy.logdebug('Closing roboclaw driver')
        if hasattr(self, 'motor'):
            for motor in self.motor.values():
                motor.shutdown()


if __name__ == "__main__":
    node = Node("roboclaw_driver")
    node.run()
    rospy.loginfo("Exiting")
