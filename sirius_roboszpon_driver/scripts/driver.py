#!/usr/bin/env python3

import rospy
from can_msgs.msg import Frame
import roboszpon_interface
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from math import pi
from sirius_roboszpon_driver.msg import RoboszponStatus


class RosRoboszponInterface(roboszpon_interface.RoboszponInterface):

    def __init__(self, node_id):
        super().__init__(node_id)

        self.frame_publisher = rospy.Publisher("/sent_canbus_messages",
                                               Frame,
                                               queue_size=10)

    def send_can_frame(self, frame_id, data):
        frame = Frame()
        frame.data = data.to_bytes(8, byteorder="big")
        frame.dlc = 8
        frame.id = frame_id
        frame.is_extended = False

        self.frame_publisher.publish(frame)


class Joint:

    def __init__(self, name) -> None:
        self.name = name
        self.node_id = rospy.get_param(f"~joints/{name}/node_id")
        self.interface = RosRoboszponInterface(self.node_id)

        self.mode = "INIT"
        self.last_update_time = rospy.get_time()
        self.reset_readings()

    def reset_readings(self):
        self.temperature = None
        self.flags = []
        self.position = None
        self.velocity = None
        self.current = None
        self.duty = 0
        """
        self.temperature = 0
        self.flags = []
        self.position = 0
        self.velocity = 0
        self.current = 0
        self.duty = 0
        """

    def are_readings_valid(self):
        #return True
        if self.mode != "RUNNING":
            return False

        if None in [
                self.temperature,
                self.position,
                self.velocity,
                self.current,
                self.duty,
        ]:
            return False

        return True

    def process_frame(self, frame: Frame):
        msg = self.interface.decode_message(frame.id, frame.data)

        if msg["message_id"] == roboszpon_interface.MSG_STATUS_REPORT:
            self.last_update_time = rospy.get_time()
            self.mode = roboszpon_interface.ROBOSZPON_MODES[msg["mode"]]
            self.temperature = msg["temperature"]
            self.flags = msg["flags"]

        if msg["message_id"] == roboszpon_interface.MSG_MOTOR_REPORT:
            self.duty = msg["duty"]
            self.current = msg["current"]

        if msg["message_id"] == roboszpon_interface.MSG_AXIS_REPORT:
            self.position = msg["position"]
            self.velocity = msg["velocity"]

    def step(self):
        if self.mode == "STOPPED":
            self.interface.arm()

        if rospy.get_time() - self.last_update_time > 0.5:
            self.mode = "TIMEOUT"
            self.reset_readings()
        """
        print({
            "name": self.name,
            "id": self.node_id,
            "mode": self.mode,
            "flags": self.flags,
            "temperature": self.temperature,
            "position": self.position,
            "velocity": self.velocity,
            "current": self.current,
            "duty": self.duty,
        })
        """

    def set_position(self, position):
        self.interface.send_position_command(position)

    def set_velocity(self, velocity):
        self.interface.send_velocity_command(velocity)

    def set_effort(self, effort):
        self.interface.send_duty_command(effort)

    def disable(self):
        self.interface.disarm()


class Node:

    def __init__(self, name) -> None:
        rospy.init_node(name, anonymous=True)

        self.rate = rospy.Rate(5)
        self.joints = {}

        rospy.on_shutdown(self.disable)
        joint_list = rospy.get_param(f"~joints").keys()
        for joint in joint_list:
            self.joints[joint] = Joint(f"{joint}")

        self.frame_subscriber = rospy.Subscriber("/received_canbus_messages",
                                                 Frame, self.receive_raw_frame)
        self.command_subscriber = rospy.Subscriber("/set_joint_states",
                                                   JointState,
                                                   self.receive_command)
        self.joint_state_publisher = rospy.Publisher("/joint_states",
                                                     JointState,
                                                     queue_size=10)

        self.status_publisher = rospy.Publisher("/roboszpon_status",
                                                RoboszponStatus,
                                                queue_size=10)

    def receive_raw_frame(self, frame: Frame):
        node_id = (frame.id >> 5) & 0b111111
        for joint in self.joints.values():
            if node_id == joint.node_id:
                joint.process_frame(frame)

    def receive_command(self, msg: JointState):
        for i, joint in enumerate(msg.name):
            if joint not in self.joints.keys():
                rospy.logwarn(
                    f"Recieved joint_state command for unsupported joint: {joint}"
                )
                continue

            if self.joints[joint].mode != "RUNNING":
                continue

            if len(msg.position) == len(msg.name):
                self.joints[joint].set_position(msg.position[i] / (2 * pi))
            if len(msg.velocity) == len(msg.name):
                self.joints[joint].set_velocity(msg.velocity[i] / (2 * pi))
            if len(msg.effort) == len(msg.name):
                self.joints[joint].set_effort(msg.effort[i])

    def run(self):
        while not rospy.is_shutdown():
            self.step()
            self.rate.sleep()

    def step(self):
        for joint in self.joints.values():
            joint.step()

        msg = JointState()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        for joint in self.joints.values():
            if joint.are_readings_valid():
                msg.name.append(joint.name)
                msg.position.append(joint.position * 2 * pi)
                msg.velocity.append(joint.velocity * 2 * pi)
                msg.effort.append(joint.current)

        self.joint_state_publisher.publish(msg)

        msg = RoboszponStatus()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        for joint in self.joints.values():
            msg.name.append(joint.name)
            msg.mode.append(joint.mode)
            msg.flags.append(str(joint.flags))
            msg.temperature.append(joint.temperature)
            msg.duty.append(joint.duty)

        self.status_publisher.publish(msg)

    def disable(self):
        for joint in self.joints.values():
            joint.disable()


if __name__ == "__main__":
    node = Node("roboszpon_driver")
    node.run()
