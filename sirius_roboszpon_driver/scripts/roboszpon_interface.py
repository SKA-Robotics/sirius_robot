from abc import ABC, abstractmethod
import struct

MSG_MOTOR_COMMAND = 0x01
MSG_ACTION_REQUEST = 0x02
MSG_STATUS_REPORT = 0x03
MSG_AXIS_REPORT = 0x04
MSG_MOTOR_REPORT = 0x05
MSG_PARAMETER_WRITE = 0x06
MSG_PARAMETER_READ = 0x07
MSG_PARAMETER_RESPONSE = 0x08

ACTION_ARM = 0x00
ACTION_DISARM = 0x01
ACTION_COMMIT_CONFIG = 0x02
ACTION_RESTORE_CONFIG = 0x03
ACTION_SET_FACTORY_CONFIG = 0x04
ACTION_SOFTWARE_RESET = 0x05

ROBOSZPON_MODE_STOPPED = 0x00
ROBOSZPON_MODE_RUNNING = 0x01
ROBOSZPON_MODE_ERROR = 0x02
ROBOSZPON_MODES = {0: "STOPPED", 1: "RUNNING", 2: "ERROR"}
ROBOSZPON_PARAMETERS = {
    "COMMAND_TIMEOUT": 0x00,
    "ENCODER_ZERO": 0x01,
    "AXIS_OFFSET": 0x02,
    "REPORT_RATE": 0x03,
    "PPID_Kp": 0x04,
    "PPID_Ki": 0x05,
    "PPID_Kd": 0x06,
    "PPID_deadzone": 0x08,
    "PPID_dUmax": 0x0A,
    "VPID_Kp": 0x0C,
    "VPID_Ki": 0x0D,
    "VPID_Kd": 0x0E,
    "VPID_deadzone": 0x10,
    "VPID_dUmax": 0x12,
    "CPID_Kp": 0x14,
    "CPID_Ki": 0x15,
    "CPID_Kd": 0x16,
    "CPID_deadzone": 0x18,
    "CPID_dUmax": 0x1A,
    "CURRENT_FeedForward": 0x1B,
    "IIR_VALUE_CURMEAS": 0x1C,
    "IIR_VALUE_VELMEAS": 0x1D,
    "IIR_VALUE_PPIDU": 0x1E,
    "IIR_VALUE_VPIDU": 0x1F,
    "IIR_VALUE_CPIDU": 0x20,
    "DUTY_DEADZONE": 0x21,
    "MIN_POSITION": 0x23,
    "MAX_POSITION": 0x24,
    "MIN_VELOCITY": 0x25,
    "MAX_VELOCITY": 0x26,
    "MIN_CURRENT": 0x27,
    "MAX_CURRENT": 0x28,
    "MIN_DUTY": 0x29,
    "MAX_DUTY": 0x2F,
    "OVERHEAT_TEMPERATURE": 0x50,
    "NO_OVERHEAT_TEMPERATURE": 0x51,
    "INVERT_AXIS": 0x52,
    "INVERT_ENCODER": 0x53,
    "ENCODER_FILTER_WINDOW": 0x54,
    "DISABLE_ENCODER_ERRORS": 0x55,
}
ROBOSZPON_FLAGS = {
    "OVERHEAT": (1 << 4),
    "EncDISCONNECT": (1 << 5),
    "EncMGL": (1 << 6),
    "EncMGH": (1 << 7),
    "DrvOLD": (1 << 8),
    "DrvTSD": (1 << 9),
    "DrvOCP": (1 << 10),
    "DrvCPUV": (1 << 11),
    "DrvUVLO": (1 << 12),
    "DrvOTW": (1 << 13),
    "DrvFault": (1 << 14),
    "CMDTIMEOUT": (1 << 15),
}


class RoboszponInterface(ABC):

    def __init__(self, node_id):
        self.node_id = node_id

    @abstractmethod
    def send_can_frame(self, frame_id, data):
        pass

    def build_frame_id(self, message_id):
        return ((self.node_id & 0b111111) << 5) + (message_id & 0b11111)

    def float_to_bits(self, value):
        value_bits = struct.pack("f", value)
        return struct.unpack("I", value_bits)[0]

    def bits_to_float(self, value):
        value_bits = struct.pack("I", value)
        return struct.unpack("f", value_bits)[0]

    def decode_message(self, frame_id, data):
        node_id = (frame_id >> 5) & 0b111111
        message_id = frame_id & 0b11111
        data = int.from_bytes(data, "big")

        if message_id == MSG_STATUS_REPORT:
            mode = (data >> 62) & 0b11
            flags_raw = data & 0xFFFF
            flags = []
            for flag, mask in ROBOSZPON_FLAGS.items():
                if flags_raw & mask != 0:
                    flags.append(flag)

            temperature = (data >> 24) & 0xFFFFFFFF
            temperature *= 0.1
            return {
                "node_id": node_id,
                "message_id": message_id,
                "mode": mode,
                "temperature": temperature,
                "flags": flags,
            }
        if message_id == MSG_AXIS_REPORT:
            position = (data >> 32) & 0xFFFFFFFF
            velocity = data & 0xFFFFFFFF
            return {
                "node_id": node_id,
                "message_id": message_id,
                "position": self.bits_to_float(position),
                "velocity": self.bits_to_float(velocity),
            }
        if message_id == MSG_MOTOR_REPORT:
            current = (data >> 32) & 0xFFFFFFFF
            duty = data & 0xFFFFFFFF
            return {
                "node_id": node_id,
                "message_id": message_id,
                "current": self.bits_to_float(current),
                "duty": self.bits_to_float(duty),
            }
        if message_id == MSG_PARAMETER_RESPONSE:
            parameter_id = (data >> 56) & 0xFF
            value = (data >> 24) & 0xFFFFFFFF
            return {
                "node_id": node_id,
                "message_id": message_id,
                "parameter_id": parameter_id,
                "value": self.bits_to_float(value),
            }
        return {"node_id": node_id, "message_id": message_id, "data": data}

    def send_action_request(self, action_id):
        return self.send_can_frame(self.build_frame_id(MSG_ACTION_REQUEST),
                                   action_id)

    def arm(self):
        return self.send_action_request(ACTION_ARM)

    def disarm(self):
        return self.send_action_request(ACTION_DISARM)

    def send_motor_command(self, motor_command_type, command):
        data = ((motor_command_type & 0xFF) << 56) + (
            (self.float_to_bits(command) & 0xFFFFFFFF) << 24)
        return self.send_can_frame(self.build_frame_id(MSG_MOTOR_COMMAND),
                                   data)

    def send_duty_command(self, command):
        return self.send_motor_command(0, command)

    def send_velocity_command(self, command):
        return self.send_motor_command(1, command)

    def send_position_command(self, command):
        return self.send_motor_command(2, command)

    def emergency_stop(self):
        return self.send_can_frame(0x001, 0)

    def send_parameter_write(self, parameter_id, value):
        data = ((parameter_id & 0xFF) << 56) + (
            (self.float_to_bits(value) & 0xFFFFFFFF) << 24)
        return self.send_can_frame(self.build_frame_id(MSG_PARAMETER_WRITE),
                                   data)

    def send_parameter_read(self, parameter_id):
        return self.send_can_frame(self.build_frame_id(MSG_PARAMETER_READ),
                                   parameter_id)
