#! /usr/bin/python3
import subprocess
import rospy
from sensor_msgs.msg import JointState
from pynput.keyboard import Key, Listener
from can_msgs.msg import Frame


class ScienceController:

    def __init__(self) -> None:
        rospy.init_node("science_controller")
        self.state_subscriber = rospy.Subscriber("/science/state", JointState,
                                                 self._joint_state_callback)
        self.state_publisher = rospy.Publisher("/science/command",
                                               JointState,
                                               queue_size=10)

        self.can_publisher = rospy.Publisher('/sent_canbus_messages',
                                             Frame,
                                             queue_size=10)

        self.state = {}
        self.rate = rospy.Rate(10)
        self.stage = 0

        Listener(on_press=self.keyboard_callback).start()
        self.key_pressed = False

        self.container_open = False
        self.container_pushed_in = False

        self.drill_current = 0
        self.drill_lift_position = 0
        self.drill_lift_current = 0
        self.module_position = 0
        self.module_current = 0

    def _joint_state_callback(self, msg):
        for i, joint in enumerate(msg.name):
            if joint == 'module_lift':
                self.module_position = msg.position[i]
                self.module_current = msg.effort[i]
            if joint == 'drill_lift':
                self.drill_lift_position = msg.position[i]
                self.drill_lift_current = msg.effort[i]
            if joint == 'drill_spin':
                self.drill_current = msg.effort[i]

    def run(self):
        while not rospy.is_shutdown():
            self.step()
            self.key_pressed = False
            self.command_servos()
            print(
                f"Module current:     {self.module_current:8.3f}, Module position:      {self.module_position:8.3f}"
            )
            print(
                f"Drill lift current: {self.drill_lift_current:8.3f}, Drill lift position:  {self.drill_lift_position:8.3f}"
            )
            print(f"Drill current: {self.drill_current:8.3f}")

            self.rate.sleep()

    def command_servos(self):
        container_open_cmd = 0x200
        container_push_cmd = 0x0200
        if self.container_open:
            container_open_cmd = 0x0
        if self.container_pushed_in:
            container_push_cmd = 0x00d0

        msg = Frame()
        msg.dlc = 4
        msg.id = 0x27 << 5
        msg.data = [
            container_open_cmd >> 8, container_open_cmd & 0b11111111,
            container_push_cmd >> 8, container_push_cmd & 0b11111111, 0, 0, 0,
            0
        ]
        self.can_publisher.publish(msg)

    def move_joint(self, joint, effort):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = [joint]
        msg.effort = [effort]
        self.state_publisher.publish(msg)

    def keyboard_callback(self, key):
        if key == Key.enter:
            self.key_pressed = True
        if key == Key.delete:
            return False

    def step(self):
        if self.stage == 0:
            rospy.loginfo("Ready. Press 'enter' to start")
            if self.key_pressed:
                self.stage = 1
            return
        if self.stage == 1:
            self.move_joint("module_lift", -0.4)
            rospy.loginfo("Lowering drill module... Press 'enter' when done")
            if self.key_pressed:
                self.stage = 2
                self.move_joint("module_lift", 0.0)
            return
        if self.stage == 2:
            rospy.loginfo("Drilling... Press 'enter' when done")
            self.move_joint("drill_lift", -1.0)
            self.move_joint("drill_spin", 1.0)
            if self.key_pressed:
                self.move_joint("drill_lift", 0.0)
                self.stage = 3
            return
        if self.stage == 3:
            rospy.loginfo("Lifting drill... Press 'enter' when done")
            self.move_joint("drill_lift", 1.0)
            self.move_joint("drill_spin", 1.0)
            if self.key_pressed or self.drill_lift_position > -5:
                self.move_joint("drill_lift", 0.0)
                self.stage = 4
            return
        if self.stage == 4:
            rospy.loginfo("Lifting module... Press 'enter' when done")
            self.move_joint("module_lift", 1.0)
            self.move_joint("drill_spin", 1.0)
            if self.key_pressed or self.module_position > -15:
                self.move_joint("module_lift", 0.0)
                self.stage = 5
            return
        if self.stage == 5:
            rospy.loginfo("Pushing container... Press 'enter' when done")
            self.container_open = True
            self.container_pushed_in = True
            if self.key_pressed:
                self.stage = 6
            return
        if self.stage == 6:
            rospy.loginfo("Emptying drill... Press 'enter' when done")
            self.move_joint("drill_spin", -1.0)
            if self.key_pressed:
                self.move_joint("drill_spin", 0.0)
                self.stage = 7
            return
        if self.stage == 7:
            rospy.loginfo("Pushing container away... Press 'enter' when done")
            self.container_pushed_in = False
            if self.key_pressed:
                self.stage = 8
            return
        if self.stage == 8:
            rospy.loginfo("Closing container... Press 'enter' when done")
            self.container_open = False
            if self.key_pressed:
                self.stage = 9
            return

        rospy.loginfo("Done.")


if __name__ == "__main__":
    try:
        ScienceController().run()
    except rospy.ROSInterruptException:
        pass
