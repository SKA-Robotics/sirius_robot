#!/usr/bin/python3

import rospy
from sensor_msgs.msg import JointState
from joystick_control.msg import Gamepad


JOYSTICK_DATA = {
    "axes": {
        "left_stick_horizontal": 0,
        "left_stick_vertical": 1,
        "right_stick_horizontal": 2,
        "right_stick_vertical": 3,
    },
    "buttons": {
        "A_button": 0,
        "B_button": 1,
        "X_button": 2,
        "Y_button": 3,
        "left_bumper": 4,
        "right_bumper": 5,
        "left_trigger": 6,
        "right_trigger": 7,
        "back_button": 8,
        "start_button": 9,
        "left_stick_button": 10,
        "right_stick_button": 11,
        "up_cross": 12,
        "down_cross": 13,
        "left_cross": 14,
        "right_cross": 15,
        "power_button": 16,
    }
}


class JoystickTranslator:

    def __init__(self):
        self.AXES_ID: dict = JOYSTICK_DATA["axes"]
        self.BUTTONS_ID: dict = JOYSTICK_DATA["buttons"]

    def translate(self, data):
        inputs = dict((name, data["buttons"][id])
                      for name, id in self.BUTTONS_ID.items())
        inputs.update(
            dict(
                (name, data["axes"][id]) for name, id in self.AXES_ID.items()))

        return inputs

class ScienceTeleop:
    def __init__(self):
        rospy.init_node("science_teleop")
        self.joint_state = JointState()
        self.joint_state.name = ["module_lift", "drill_lift", "drill_spin"]
        self.drill_cmd = 0
        self.gain = {
            "module_lift": 0.5,
            "drill_lift": 0.7,
        }
        self.joystick_translator = JoystickTranslator()
        self.cmd_publisher = rospy.Publisher("/science/command", JointState, queue_size=10)
        self.joy_subscriber = rospy.Subscriber("/joy_5dof_manipulator", Gamepad, self.joy_callback, queue_size=10)
    
    def joy_callback(self, msg: Gamepad):
        data = {
            "axes": msg.axes,
            "buttons": msg.buttons,
        }
        input = self.joystick_translator.translate(data)
        module_lift = -input["left_stick_vertical"] * self.gain["module_lift"]
        drill_lift = -input["right_stick_vertical"] * self.gain["drill_lift"]
        if input["down_cross"]:
            self.drill_cmd = -1.0
        elif input["left_cross"] or input["right_cross"]:
            self.drill_cmd = 0.0
        elif input["up_cross"]:
            self.drill_cmd = 1.0

        if abs(module_lift) < 0.1:
            module_lift = 0
        if abs(drill_lift) < 0.1:
            drill_lift = 0
        
        self.joint_state.effort = [module_lift, drill_lift, self.drill_cmd]
        self.joint_state.header.stamp = rospy.Time.now()
        print(self.joint_state.effort)

        self.cmd_publisher.publish(self.joint_state)
    
    def run(self):
        print("Teleop node running")
        while not rospy.is_shutdown():
            rospy.spin()
        

if __name__=="__main__":
    ScienceTeleop().run()