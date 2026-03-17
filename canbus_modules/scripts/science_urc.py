#!/usr/bin/env python3
# NOTE: This node uses blocking time.sleep() calls inside state machine steps.
# Runs via MultiThreadedExecutor to keep ROS2 callbacks alive.
import curses
import time
from abc import ABC, abstractmethod

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import JointState
from can_msgs.msg import Frame


class ScienceController(Node):
    def __init__(self, stdscr) -> None:
        super().__init__('science_controller')

        self.state_subscriber = self.create_subscription(
            JointState, '/science/state', self._joint_state_callback, 10)
        self.state_publisher = self.create_publisher(
            JointState, '/science/command', 10)
        self.can_publisher = self.create_publisher(
            Frame, '/to_can_bus', 10)

        self.state = {}
        self.stdscr = stdscr
        stdscr.timeout(300)

        self.drill_current = 0.0
        self.drill_lift_position = 0.0
        self.drill_lift_current = 0.0
        self.module_position = 0.0
        self.module_current = 0.0

        self.scoop_track = 0
        self.scoop_position_dict = {
            0: 0x034f, 1: 0x0265, 2: 0x01db, 3: 0x0145, 4: 0x00af
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

    def _joint_state_callback(self, msg: JointState):
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

    def get_time(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def run(self):
        track_dict = {'a': 1, 'b': 2, 'c': 3}
        self.set_state(StateIdle())

        while rclpy.ok():
            for i in range(5):
                self.command_servos(i)
                time.sleep(0.01)

            self.stdscr.clear()
            self.stdscr.addstr(
                f'Module current:     {self.module_current:8.3f}, '
                f'Module position:      {self.module_position:8.3f}\n')
            self.stdscr.addstr(
                f'Drill lift current: {self.drill_lift_current:8.3f}, '
                f'Drill lift position:  {self.drill_lift_position:8.3f}\n')
            self.stdscr.addstr(f'Drill current: {self.drill_current:8.3f}\n')
            self.stdscr.addstr(
                's=powierzchnia, d=głęboka, 1-4=tor, a/b/c=ruch toru, '
                'h=ciecze, q=koniec\n')
            self.stdscr.addstr(f'State: {self.state}\n')
            self.stdscr.addstr(
                f'Surface: {self.surface_key_pressed}\t'
                f'Deep: {self.deep_key_pressed}\n')
            self.stdscr.addstr(f'Track {self.scoop_track}\n')
            self.stdscr.refresh()

            char = self.stdscr.getch()
            if char == ord('q'):
                self.move_joint('module_lift', 0)
                self.move_joint('drill_lift', 0)
                self.move_joint('drill_spin', 0)
                self.scoop_track_msg = 0x034f
                self.track_first_command = 0x0000
                self.track_second_command = 0x0000
                self.track_third_command = 0x0000
                self.push_liquids = False
                for i in range(5):
                    self.command_servos(i)
                    time.sleep(0.01)
                break
            if char == ord('s'):
                self.surface_key_pressed = True
            if char == ord('d'):
                self.deep_key_pressed = True
            if char in [ord(str(i)) for i in range(5)]:
                self.scoop_track = int(chr(char))
                self.scoop_track_msg = self.scoop_position_dict[
                    self.scoop_track]
                if char != ord('0'):
                    self.track_key_pressed = True
            if char in [ord('a'), ord('b'), ord('c')]:
                self.scoop_track = track_dict[chr(char)]
                self.track_moving = True
            if char == ord('h'):
                self.push_liquids = True

            self.state.step()
            time.sleep(0.1)

    def command_servos(self, msg_id: int):
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
        else:
            return

        msg = Frame()
        msg.dlc = 3
        msg.id = (0x27 << 5)
        msg.data = [msg_id, cmd >> 8, cmd & 0b11111111, 0, 0, 0, 0, 0]
        self.can_publisher.publish(msg)

    def move_joint(self, joint: str, effort: float):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
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
        self._context.stdscr.addstr('Idle')
        self.start_time = self._context.get_time()

    def step(self):
        if self._context.deep_key_pressed or self._context.surface_key_pressed:
            self._context.set_state(LoweringModule())
            return
        if self._context.track_key_pressed:
            self._context.set_state(PushingScoop())
            return
        if self._context.track_moving:
            self._context.set_state(TrackMoving())
            return
        if self._context.push_liquids:
            self._context.set_state(PushingLiquids())
            return
        self._context.move_joint('module_lift', 0)
        self._context.move_joint('drill_lift', 0)
        self._context.move_joint('drill_spin', 0)
        self._context.pushing_liquids = False

    def __repr__(self):
        return 'Idle'


class LoweringModule(State):
    def setup(self):
        self._context.stdscr.addstr('Running')
        self.start_time = self._context.get_time()

    def step(self):
        t = self._context.get_time()
        if (t - self.start_time > 2 and
                self._context.module_current < -0.2137):
            self._context.set_state(StateIdle())
            if self._context.deep_key_pressed:
                self._context.set_state(StartDeepDrilling())
                return
            if self._context.surface_key_pressed:
                self._context.set_state(StartSurfaceDrilling())
                return
        self._context.move_joint('module_lift', -0.4)

    def __repr__(self):
        return 'LoweringModule'


class StartDeepDrilling(State):
    def setup(self):
        self._context.stdscr.addstr('Deep Drilling')
        self.start_time = self._context.get_time()

    def step(self):
        t = self._context.get_time()
        if (self._context.drill_current > 5 or
                self._context.drill_lift_position < -23 * 6.28 or
                t - self.start_time > 60):
            self._context.set_state(DrillRetraction())
            return
        self._context.move_joint('module_lift', 0.0)
        self._context.move_joint('drill_lift', -0.6)
        self._context.move_joint('drill_spin', -1.0)
        self._context.deep_key_pressed = False
        time.sleep(14)
        self._context.move_joint('drill_lift', 0.5)
        time.sleep(0.5)

    def __repr__(self):
        return 'StartDeepDrilling'


class StartSurfaceDrilling(State):
    def setup(self):
        self._context.stdscr.addstr('Surface Drilling')
        self.start_time = self._context.get_time()

    def step(self):
        t = self._context.get_time()
        if (self._context.drill_current > 5 or
                self._context.drill_lift_position < -9 * 6.28 or
                t - self.start_time > 30):
            self._context.set_state(DrillRetraction())
            return
        self._context.move_joint('module_lift', 0.0)
        self._context.move_joint('drill_lift', -0.6)
        self._context.move_joint('drill_spin', -1.0)
        self._context.surface_key_pressed = False
        time.sleep(7)
        self._context.move_joint('drill_lift', 0.5)
        time.sleep(0.5)

    def __repr__(self):
        return 'StartSurfaceDrilling'


class DrillRetraction(State):
    def setup(self):
        self._context.stdscr.addstr('Drilling Retraction')

    def step(self):
        if self._context.drill_lift_position > -1:
            self._context.set_state(LiftingModule())
            return
        self._context.move_joint('drill_lift', 0.3)
        self._context.move_joint('drill_spin', 0.0)

    def __repr__(self):
        return 'DrillRetraction'


class LiftingModule(State):
    def setup(self):
        self._context.stdscr.addstr('Lifting Module')

    def step(self):
        if self._context.module_position < 1:
            self._context.set_state(StateIdle())
            return
        self._context.move_joint('drill_lift', 0.0)
        self._context.move_joint('module_lift', 0.4)
        self._context.move_joint('drill_spin', 0.0)

    def __repr__(self):
        return 'LiftingModule'


class PushingScoop(State):
    def setup(self):
        self._context.stdscr.addstr('Pushing scoop into right track')
        self.start_time = self._context.get_time()

    def step(self):
        if self._context.get_time() - self.start_time > 2.5:
            self._context.track_key_pressed = False
            self._context.set_state(ShakingScoop())

    def __repr__(self):
        return 'PushingScoop'


class ShakingScoop(State):
    def setup(self):
        self._context.stdscr.addstr('ShakingScoopSandExt')
        self.counter = 0
        self.start_time = self._context.get_time()
        self.track = self._context.scoop_track

    def step(self):
        if self._context.get_time() - self.start_time > 35:
            self._context.scoop_track = self.track
            self._context.set_state(ReturnScoop())
            return
        self._context.move_joint('drill_spin', 1.0)
        base = self._context.scoop_position_dict[self.track]
        if self.counter % 2 == 0:
            self._context.scoop_track_msg = base - 0x0008
        else:
            self._context.scoop_track_msg = base + 0x0008
        self.counter += 1

    def __repr__(self):
        return 'ShakingScoop'


class ReturnScoop(State):
    def setup(self):
        self._context.stdscr.addstr('Scoop returning into initial position')
        self.start_time = self._context.get_time()

    def step(self):
        if self._context.get_time() - self.start_time > 4:
            self._context.set_state(LowerDrill())
            return
        self._context.scoop_track = 0
        self._context.scoop_track_msg = self._context.scoop_position_dict[0]

    def __repr__(self):
        return 'ReturnScoop'


class LowerDrill(State):
    def setup(self):
        self._context.stdscr.addstr('Lowering drill')

    def step(self):
        if (self._context.drill_current > 5 or
                self._context.drill_lift_position < -10):
            self._context.set_state(EmptyingDrill())
            return
        self._context.move_joint('module_lift', 0.0)
        self._context.move_joint('drill_lift', -0.3)
        self._context.move_joint('drill_spin', 1.0)

    def __repr__(self):
        return 'LoweringDrill'


class EmptyingDrill(State):
    def setup(self):
        self._context.stdscr.addstr('Emptying drill')
        self.start_time = self._context.get_time()

    def step(self):
        if self._context.get_time() - self.start_time > 10:
            self._context.set_state(StateIdle())
            return
        self._context.move_joint('drill_spin', 1.0)

    def __repr__(self):
        return 'EmptyingDrill'


class TrackMoving(State):
    def setup(self):
        self._context.stdscr.addstr('Moving track')
        self.start_time = self._context.get_time()

    def step(self):
        elapsed = self._context.get_time() - self.start_time
        track_attrs = {1: 'track_first_command',
                       2: 'track_second_command',
                       3: 'track_third_command'}
        attr = track_attrs.get(self._context.scoop_track)
        if not attr:
            return
        if elapsed <= 20:
            setattr(self._context, attr, 0x02bc)
        elif elapsed <= 28:
            setattr(self._context, attr, 0x0190)
        else:
            setattr(self._context, attr, 0x0000)
            self._context.track_moving = False
            self._context.set_state(DrillRetraction())

    def __repr__(self):
        return 'TrackMoving'


class PushingLiquids(State):
    def setup(self):
        self._context.stdscr.addstr('Pushing Liquids')
        self.start_time = self._context.get_time()

    def step(self):
        if self._context.get_time() - self.start_time > 80:
            self._context.set_state(StateIdle())
            return
        self._context.pushing_liquids = True

    def __repr__(self):
        return 'PushingLiquids'


def main(args=None):
    rclpy.init(args=args)

    def _run(stdscr):
        node = ScienceController(stdscr)
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        import threading
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        try:
            node.run()
        except KeyboardInterrupt:
            pass
        finally:
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown()

    curses.wrapper(_run)


if __name__ == '__main__':
    main()