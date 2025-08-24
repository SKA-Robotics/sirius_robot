#! /usr/bin/python3
import subprocess
import rospy
from sensor_msgs.msg import JointState
from can_msgs.msg import Frame
from abc import ABC, abstractmethod
import curses


class ScienceController:

    def __init__(self, stdscr) -> None:
        rospy.init_node("Science_controller")
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

        self.drill_current = 0
        self.drill_lift_position = 0
        self.drill_lift_current = 0
        self.module_position = 0
        self.module_current = 0

        self.scoop_track = 0
        self.scoop_position_dict = {
            0: 0x034f,
            1: 0x0265,
            2: 0x01db,
            3: 0x0145,
            4: 0x00af
        }
        self.scoop_track_msg = 0x034f

        self.track_first_command = 0x0000
        self.track_second_command = 0x0000
        self.track_third_command = 0x0000
        self.track_moving = False

        self.surface_key_pressed = False
        self.deep_key_pressed = False
        self.track_key_pressed = False

        self.push_liquids = False
        self.pushing_liquids = False

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
        self.track_dict = {'a': 1, 'b': 2, 'c': 3}
        self.set_state(StateIdle())

        while not rospy.is_shutdown():
            self.command_servos(0)
            rospy.sleep(0.01)
            self.command_servos(1)
            rospy.sleep(0.01)
            self.command_servos(2)
            rospy.sleep(0.01)
            self.command_servos(3)
            rospy.sleep(0.01)
            self.command_servos(4)

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
                "Wprowadź tekst (aby rozpocząć zbieranie powierzchniowe naciśnij s, aby rozpocząć pobieranie głębokie naciśnij d, nastepnie wybierz tor do zsypania próbki (1, 2, 3, 4), aby wsypać piach do toru 1 wybierz a, do toru 2 - b, do toru 3 - c, aby wypuscic chemiczne rzeczy nacisnij h, aby zakończyć naciśnij 'q'):\n"
            )
            self.stdscr.addstr(f"State: {self.state}\n")
            self.stdscr.addstr(
                f"Surface key: {self.surface_key_pressed}\tDeep key: {self.deep_key_pressed}\n"
            )
            self.stdscr.addstr(f"Track {self.scoop_track}\n")
            self.stdscr.addstr(f"Track key {self.track_key_pressed}\n")
            self.stdscr.addstr(f"Track first {self.track_first_command}\n")
            self.stdscr.addstr(f"Track second {self.track_second_command}\n")
            self.stdscr.addstr(f"Track third {self.track_third_command}\n")
            self.stdscr.refresh()

            char = self.stdscr.getch()

            if char == ord('q'):
                self.move_joint("module_lift", 0)
                self.move_joint("drill_lift", 0)
                self.move_joint("drill_spin", 0)
                self.scoop_track_msg = 0x034f
                self.track_first_command = 0x0000
                self.track_second_command = 0x0000
                self.track_third_command = 0x0000
                self.push_liquids = False
                self.command_servos(0)
                rospy.sleep(0.01)
                self.command_servos(1)
                rospy.sleep(0.01)
                self.command_servos(2)
                rospy.sleep(0.01)
                self.command_servos(3)
                rospy.sleep(0.01)
                self.command_servos(4)

                break

            if char == ord('s'):
                self.surface_key_pressed = True

            if char == ord('d'):
                self.deep_key_pressed = True

            if char == ord('0') or char == ord('1') or char == ord(
                    '2') or char == ord('3') or char == ord('4'):
                self.scoop_track = int(chr(char))
                self.scoop_track_msg = self.scoop_position_dict[
                    self.scoop_track]
                if char != ord('0'):
                    self.track_key_pressed = True

            if char == ord('a') or char == ord('b') or char == ord('c'):
                self.scoop_track = self.track_dict[chr(char)]
                self.track_moving = True

            if char == ord('h'):
                self.push_liquids = True

            self.state.step()
            print_counter += 1
            self.rate.sleep()

    def command_servos(self, msg_id):

        if msg_id == 0:
            cmd = self.scoop_track_msg

        elif msg_id == 1:
            cmd = self.track_first_command

        elif msg_id == 2:
            cmd = self.track_second_command

        elif msg_id == 3:
            cmd = self.track_third_command

        elif msg_id == 4:
            cmd = 0x0000
            if self.pushing_liquids:
                cmd = 0x03ff
                self.push_liquids = False

        msg = Frame()
        msg.dlc = 3
        msg.id = (0x27 << 5)
        msg.data = [msg_id, cmd >> 8, cmd & 0b11111111, 0, 0, 0, 0, 0]
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
        self.start_time = rospy.Time.now()

    def step(self):
        time = rospy.Time.now()

        if (self._context.deep_key_pressed
                == True) or (self._context.surface_key_pressed == True):
            self._context.set_state(LoweringModule())
            return

        if (self._context.track_key_pressed == True):
            self._context.set_state(PushingScoop())
            return

        if (self._context.track_moving == True):
            self._context.set_state(TrackMoving())
            return

        if (self._context.push_liquids == True):
            self._context.set_state(PushingLiquids())
            return

        self._context.move_joint("module_lift", 0)
        self._context.move_joint("drill_lift", 0)
        self._context.move_joint("drill_spin", 0)
        self._context.pushing_liquids = False

    def __repr__(self):
        return "Idle"


#deep
class LoweringModule(State):

    def setup(self):
        self._context.stdscr.addstr("Running")
        self.start_time = rospy.Time.now()

    def step(self):
        current_limit = -0.2137
        module_limit = -200
        time = rospy.Time.now()
        if ((time - self.start_time).to_sec() > 2
                and (self._context.module_current < current_limit)):
            #or self._context.module_position < module_limit):

            self._context.set_state(StateIdle())

            if self._context.deep_key_pressed == True:
                self._context.set_state(StartDeepDrilling())
                return

            if self._context.surface_key_pressed == True:

                self._context.set_state(StartSurfaceDrilling())
                return

            #if self._context.track_key_pressed == True:
            #self._context.set_state(PushingScoop())

        self._context.move_joint("module_lift", -0.4)

    def __repr__(self):
        return "LoweringModule"


class StartDeepDrilling(State):

    def setup(self):
        self._context.stdscr.addstr("Deep Drilling")
        self.start_time = rospy.Time.now()

    def step(self):
        current_limit = 5
        drill_limit = -23 * 6.28
        time = rospy.Time.now()
        if (self._context.drill_current > current_limit
                or self._context.drill_lift_position < drill_limit
                or (time - self.start_time).to_sec() > 60):
            self._context.set_state(DrillRetraction())
            return
        self._context.move_joint("module_lift", 0.0)
        self._context.move_joint("drill_lift", -0.6)
        self._context.move_joint("drill_spin", -1.0)
        self._context.deep_key_pressed = False
        rospy.sleep(14)
        self._context.move_joint("drill_lift", 0.5)
        rospy.sleep(0.5)

    def __repr__(self):
        return "StartDeepDrilling"


class StartSurfaceDrilling(State):

    def setup(self):
        self._context.stdscr.addstr("Surface Drilling")
        self.start_time = rospy.Time.now()
        #self.start = True

    def step(self):
        current_limit = 5
        drill_limit = -9 * 6.28
        time = rospy.Time.now()
        if (self._context.drill_current > current_limit
                or self._context.drill_lift_position < drill_limit
                or (time - self.start_time).to_sec() > 30):
            self._context.set_state(DrillRetraction())
            return

        self._context.move_joint("module_lift", 0.0)
        self._context.move_joint("drill_lift", -0.6)
        self._context.move_joint("drill_spin", -1.0)
        self._context.surface_key_pressed = False
        rospy.sleep(7)
        self._context.move_joint("drill_lift", 0.5)
        rospy.sleep(0.5)

    def __repr__(self):
        return "StartSurfaceDrilling"


class DrillRetraction(State):

    def setup(self):
        self._context.stdscr.addstr("Drilling Retraction")

    def step(self):
        drill_limit = -1
        if self._context.drill_lift_position > drill_limit:
            #self._context.set_state(StateIdle())
            self._context.set_state(LiftingModule())
            return
        self._context.move_joint("drill_lift", 0.3)
        self._context.move_joint("drill_spin", 0.0)

    def __repr__(self):
        return "DrillRetraction"


class LiftingModule(State):

    def setup(self):
        self._context.stdscr.addstr("Lifting Module")

    def step(self):
        module_limit = 1
        if self._context.module_position < module_limit:
            self._context.set_state(StateIdle())
            return
        self._context.move_joint("drill_lift", 0.0)
        self._context.move_joint("module_lift", 0.4)
        self._context.move_joint("drill_spin", 0.0)

    def __repr__(self):
        return "LiftingModule"


#wybór toru i spuszczenie próbki


class PushingScoop(State):

    def setup(self):
        self._context.stdscr.addstr("Pushing scoop into right track")
        self.start_time = rospy.Time.now()

    def step(self):
        time = rospy.Time.now()
        if (time - self.start_time).to_sec() > 2.5:
            self._context.track_key_pressed = False
            self._context.set_state(ShakingScoop())
            return

    def __repr__(self):
        return "PushingScoop"


class ShakingScoop(State):

    def setup(self):
        self._context.stdscr.addstr("ShakingScoopSandExt")
        self.counter = 0
        self.start_time = rospy.Time.now()
        self.track = self._context.scoop_track

    def step(self):
        time = rospy.Time.now()

        if (time - self.start_time).to_sec() > 35:
            self._context.scoop_track = self.track
            self._context.set_state(ReturnScoop())
            return

        self._context.move_joint("drill_spin", 1.0)
        position_lower = self._context.scoop_position_dict[self.track] - 0x0008
        position_upper = self._context.scoop_position_dict[self.track] + 0x0008

        if (self.counter % 2 == 0):
            self._context.scoop_track_msg = position_lower
        else:
            self._context.scoop_track_msg = position_upper

        self.counter += 1

    def __repr__(self):
        return "Shaking scoop and sand extraction"


class ReturnScoop(State):

    def setup(self):
        self._context.stdscr.addstr("Scoop returning into initial position")
        self.start_time = rospy.Time.now()

    def step(self):
        time = rospy.Time.now()
        if (time - self.start_time).to_sec() > 4:
            self._context.set_state(LowerDrill())
            #self._context.set_state(StateIdle())
            return

        self._context.scoop_track = 0
        self._context.scoop_track_msg = self._context.scoop_position_dict[
            self._context.scoop_track]

    def __repr__(self):
        return "Scoop Returning"


class LowerDrill(State):

    def setup(self):
        self._context.stdscr.addstr("Lowering drill")

    def step(self):
        current_limit = 5
        drill_limit = -10
        if (self._context.drill_current > current_limit
                or self._context.drill_lift_position < drill_limit):
            self._context.set_state(EmptyingDrill())
            #self._context.set_state(StateIdle())
            return
        self._context.move_joint("module_lift", 0.0)
        self._context.move_joint("drill_lift", -0.3)
        self._context.move_joint("drill_spin", 1.0)

    def __repr__(self):
        return "LoweringDrill"


class EmptyingDrill(State):

    def setup(self):
        self._context.stdscr.addstr("Emptying drill")
        self.start_time = rospy.Time.now()

    def step(self):
        time = rospy.Time.now()
        if (time - self.start_time).to_sec() > 10:
            self._context.set_state(StateIdle())
            return
        self._context.move_joint("drill_spin", 1.0)

    def __repr__(self):
        return "EmptyingDrill"


class TrackMoving(State):

    def setup(self):
        self._context.stdscr.addstr("Moving track")
        self.start_time = rospy.Time.now()

    def step(self):
        now = rospy.Time.now()
        elapsed = (now - self.start_time).to_sec()

        track_commands = {
            1: 'track_first_command',
            2: 'track_second_command',
            3: 'track_third_command'
        }

        attr_name = track_commands.get(self._context.scoop_track)
        if not attr_name:
            return

        if elapsed <= 20:
            command = 0x02bc
        elif elapsed <= 28:
            command = 0x0190
        else:
            command = 0x0000
            setattr(self._context, attr_name, command)
            self._context.track_moving = False
            self._context.set_state(DrillRetraction())
            return

        setattr(self._context, attr_name, command)

    def __repr__(self):
        return "Moving track"


class PushingLiquids(State):

    def setup(self):
        self._context.stdscr.addstr("Pushing Liquids")
        self.start_time = rospy.Time.now()

    def step(self):
        time = rospy.Time.now()
        if (time - self.start_time).to_sec() > 80:
            self._context.set_state(StateIdle())
            return
        self._context.pushing_liquids = True

    def __repr__(self):
        return "Pushing liquids"


def main(stdscr):
    controller = ScienceController(stdscr)
    controller.run()


if __name__ == "__main__":
    try:
        curses.wrapper(main)
        #ScienceController().run()
    except rospy.ROSInterruptException:
        pass
