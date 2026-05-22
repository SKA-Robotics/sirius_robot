#! /usr/bin/python3
import subprocess
import rospy
from sensor_msgs.msg import JointState
from can_msgs.msg import Frame
from abc import ABC, abstractmethod
import curses


class ScienceController:

    def __init__(self, stdscr) -> None:
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

        self.stdscr = stdscr
        stdscr.timeout(300)
        self.key_pressed = False

        #self.deep_container_open = False
        self.deep_container_pushed_in = False

        self.drill_current = 0
        self.drill_lift_position = 0
        self.drill_lift_current = 0
        self.module_position = 0
        self.module_current = 0

        self.deep_key_pressed = False

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

    def set_state(self, new_state):
        self.state = new_state
        self.state._context = self
        self.state.setup()

    def run(self):
        print_counter = 0
        self.set_state(StateIdle())

        while not rospy.is_shutdown():

            self.command_servos(3)

            # if print_counter % 1 == 0:
            self.stdscr.clear()

            self.stdscr.addstr(
                f"Module current:     {self.module_current:8.3f}, Module position:      {self.module_position:8.3f}\n"
            )
            self.stdscr.addstr(
                f"Drill lift current: {self.drill_lift_current:8.3f}, Drill lift position:  {self.drill_lift_position:8.3f}\n"
            )
            self.stdscr.addstr(f"Drill current: {self.drill_current:8.3f}\n")

            self.stdscr.addstr(
                "Wprowadź tekst (aby rozpocząć pobieranie głębokie naciśnij d, aby zakończyć naciśnij 'q'):\n"
            )
            self.stdscr.addstr(f"State: {self.state}\n")
            self.stdscr.refresh()

            self.deep_key_pressed = False
            char = self.stdscr.getch()

            if char == ord('q'):
                self.move_joint("module_lift", 0)
                self.move_joint("drill_lift", 0)
                self.move_joint("drill_spin", 0)
                break

            if char == ord('d'):
                self.deep_key_pressed = True

            self.state.step()
            print_counter += 1
            self.rate.sleep()

    def command_servos(self, msg_id):

        if msg_id == 3:
            cmd = 0x0200
            if self.deep_container_pushed_in:
                cmd = 0x00d0

        msg = Frame()
        msg.dlc = 2
        msg.id = (0x30 << 5) | msg_id
        msg.data = [cmd >> 8, cmd & 0b11111111, 0, 0, 0, 0, 0, 0]
        self.can_publisher.publish(msg)

    def move_joint(self, joint, effort):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = [joint]
        msg.effort = [effort]
        self.state_publisher.publish(msg)


class State(ABC):

    @property
    def context(self):
        return self._context

    @context.setter
    def context(self, context):
        self._context = context

    @abstractmethod
    def step(self):
        pass

    @abstractmethod
    def __repr__(self):
        pass


class StateIdle(State):

    def setup(self):
        self._context.stdscr.addstr("Idle")

    def step(self):
        if self._context.deep_key_pressed == True:
            self._context.set_state(LoweringModule())
            return

        self._context.move_joint("module_lift", 0)
        self._context.move_joint("drill_lift", 0)
        self._context.move_joint("drill_spin", 0)
        self._context.deep_container_pushed_in = False

    def __repr__(self):
        return "Idle"


#deep
class LoweringModule(State):

    def setup(self):
        self._context.stdscr.addstr("Running")
        self.start_time = rospy.Time.now()

    def step(self):
        current_limit = -0.2
        module_limit = -200
        time = rospy.Time.now()
        if ((time - self.start_time).to_sec() > 2 and
            (self._context.module_current < current_limit)
                or self._context.module_position < module_limit):
            self._context.set_state(StartDeepDrilling())
            return
        self._context.move_joint("module_lift", -0.4)

    def __repr__(self):
        return "LoweringModule"


class StartDeepDrilling(State):

    def setup(self):
        self._context.stdscr.addstr("Deep Drilling")

    def step(self):
        current_limit = 5
        drill_limit = -1000
        if (self._context.drill_current > current_limit
                or self._context.drill_lift_position < drill_limit):
            self._context.set_state(DeepDrillRetraction())
            return
        self._context.move_joint("module_lift", 0.0)
        self._context.move_joint("drill_lift", -1.0)
        self._context.move_joint("drill_spin", 1.0)

    def __repr__(self):
        return "StartDeepDrilling"


class DeepDrillRetraction(State):

    def setup(self):
        self._context.stdscr.addstr("Deep Drilling Retraction")

    def step(self):
        drill_limit = -5
        if self._context.drill_lift_position > drill_limit:
            self._context.set_state(LiftingModule())
            return
        self._context.move_joint("drill_lift", 1.0)
        self._context.move_joint("drill_spin", 0.0)

    def __repr__(self):
        return "DeepDrillRetraction"


class LiftingModule(State):

    def setup(self):
        self._context.stdscr.addstr("Lifting Module")

    def step(self):
        module_limit = -10
        if self._context.module_position > module_limit:
            self._context.set_state(PushingContainer())
            return
        self._context.move_joint("drill_lift", 0.0)
        self._context.move_joint("module_lift", 0.6)
        self._context.move_joint("drill_spin", 0.0)

    def __repr__(self):
        return "LiftingModule"


class PushingContainer(State):

    def setup(self):
        self._context.stdscr.addstr("Pushing Container")
        self.start_time = rospy.Time.now()

    def step(self):
        time = rospy.Time.now()
        if (time - self.start_time).to_sec() > 3:
            self._context.set_state(EmptyingDrill())
            return
        #self._context.deep_container_open = True
        self._context.deep_container_pushed_in = True

    def __repr__(self):
        return "PushingContainer"


class EmptyingDrill(State):

    def setup(self):
        self._context.stdscr.addstr("Emptying Drill")
        self.start_time = rospy.Time.now()

    def step(self):
        time = rospy.Time.now()
        if (time - self.start_time).to_sec() > 6:
            self._context.set_state(PushingContainerAway())
            #self._context.set_state(ShakingContainer())
            return
        self._context.move_joint("drill_spin", -1.0)

    def __repr__(self):
        return "EmtyingDrill"


class ShakingContainer(State):

    def setup(self):
        self._context.stdscr.addstr("ShakingContainer")
        self.start_time = rospy.Time.now()

    def step(self):
        time = rospy.Time.now()
        if (time - self.start_time).to_sec() > 20:
            self._context.set_state(PushingContainerAway())
            return
        self._context.deep_container_pushed_in = True
        rospy.sleep(0.1)
        self._context.deep_container_pushed_in = False
        rospy.sleep(0.1)

    def __repr__(self):
        return "ShakingContainer"


class PushingContainerAway(State):

    def setup(self):
        self._context.stdscr.addstr("Pushing Container Away")
        self.start_time = rospy.Time.now()

    def step(self):
        time = rospy.Time.now()
        if (time - self.start_time).to_sec() > 2:
            self._context.set_state(StateIdle())
            return
        self._context.deep_container_pushed_in = False
        self._context.move_joint("drill_spin", 0.0)

    def __repr__(self):
        return "PushingContainerAway"


def main(stdscr):
    controller = ScienceController(stdscr)
    controller.run()


if __name__ == "__main__":
    try:
        curses.wrapper(main)
        #ScienceController().run()
    except rospy.ROSInterruptException:
        pass
