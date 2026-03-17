#!/usr/bin/env python3
# NOTE: This node uses blocking time.sleep() calls inside state machine steps
# (originally rospy.sleep). This is intentional for sequential hardware
# control. The node runs in a dedicated thread via a MultiThreadedExecutor.
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

        self.deep_container_pushed_in = False
        self.surface_container_pushed_in = False
        self.shovel_in = False
        self.shovel_emptying = False
        self.drill_current = 0.0
        self.drill_lift_position = 0.0
        self.drill_lift_current = 0.0
        self.module_position = 0.0
        self.module_current = 0.0
        self.surface_key_pressed = False
        self.deep_key_pressed = False

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

    def run(self):
        self.set_state(StateIdle())
        while rclpy.ok():
            self.command_servos(0)
            self.command_servos(1)
            self.command_servos(2)

            self.stdscr.clear()
            self.stdscr.addstr(
                f'Module current:     {self.module_current:8.3f}, '
                f'Module position:      {self.module_position:8.3f}\n')
            self.stdscr.addstr(
                f'Drill lift current: {self.drill_lift_current:8.3f}, '
                f'Drill lift position:  {self.drill_lift_position:8.3f}\n')
            self.stdscr.addstr(f'Drill current: {self.drill_current:8.3f}\n')
            self.stdscr.addstr(
                "Wciśnij s=powierzchnia, d=głęboka, q=koniec\n")
            self.stdscr.addstr(f'State: {self.state}\n')
            self.stdscr.addstr(
                f'Surface key: {self.surface_key_pressed}\t'
                f'Deep key: {self.deep_key_pressed}\n')
            self.stdscr.refresh()

            self.surface_key_pressed = False
            self.deep_key_pressed = False

            char = self.stdscr.getch()
            if char == ord('q'):
                self.move_joint('module_lift', 0)
                self.move_joint('drill_lift', 0)
                self.move_joint('drill_spin', 0)
                break
            if char == ord('s'):
                self.surface_key_pressed = True
            if char == ord('d'):
                self.deep_key_pressed = True

            self.state.step()
            time.sleep(0.1)

    def command_servos(self, msg_id: int):
        if msg_id == 0:
            cmd = 0x00d0
            if self.surface_container_pushed_in:
                cmd = 0x01a0
        elif msg_id == 2:
            cmd = 0x0200
            if self.deep_container_pushed_in:
                cmd = 0x00d0
        elif msg_id == 1:
            cmd = 0x0300
            if self.shovel_in:
                cmd = 0x00d0
            if self.shovel_emptying:
                cmd = 0x0150
        else:
            return

        msg = Frame()
        msg.dlc = 2
        msg.id = (0x30 << 5) | msg_id
        msg.data = [cmd >> 8, cmd & 0b11111111, 0, 0, 0, 0, 0, 0]
        self.can_publisher.publish(msg)

    def move_joint(self, joint: str, effort: float):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [joint]
        msg.effort = [effort]
        self.state_publisher.publish(msg)

    def get_time(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


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

    def step(self):
        if self._context.deep_key_pressed:
            self._context.set_state(LoweringModule())
            return
        if self._context.surface_key_pressed:
            self._context.set_state(LoweringSurfaceModule())
            return
        self._context.move_joint('module_lift', 0)
        self._context.move_joint('drill_lift', 0)
        self._context.move_joint('drill_spin', 0)
        self._context.deep_container_pushed_in = False
        self._context.surface_container_pushed_in = False
        self._context.shovel_emptying = False
        self._context.shovel_in = False

    def __repr__(self):
        return 'Idle'


class LoweringModule(State):
    def setup(self):
        self._context.stdscr.addstr('Running')
        self.start_time = self._context.get_time()

    def step(self):
        current_limit = -0.2
        module_limit = -200
        t = self._context.get_time()
        if ((t - self.start_time > 2 and
             self._context.module_current < current_limit) or
                self._context.module_position < module_limit):
            self._context.set_state(StartDeepDrilling())
            return
        self._context.move_joint('module_lift', -0.4)

    def __repr__(self):
        return 'LoweringModule'


class StartDeepDrilling(State):
    def setup(self):
        self._context.stdscr.addstr('Deep Drilling')

    def step(self):
        if (self._context.drill_current > 5 or
                self._context.drill_lift_position < -1000):
            self._context.set_state(DeepDrillRetraction())
            return
        self._context.move_joint('module_lift', 0.0)
        self._context.move_joint('drill_lift', -1.0)
        self._context.move_joint('drill_spin', 1.0)
        # NOTE: originally rospy.sleep(14) — blocking intentional
        time.sleep(14)
        self._context.move_joint('drill_lift', 0.5)
        time.sleep(0.5)

    def __repr__(self):
        return 'StartDeepDrilling'


class DeepDrillRetraction(State):
    def setup(self):
        self._context.stdscr.addstr('Deep Drilling Retraction')

    def step(self):
        if self._context.drill_lift_position > -5:
            self._context.set_state(LiftingModule())
            return
        self._context.move_joint('drill_lift', 1.0)
        self._context.move_joint('drill_spin', 0.0)

    def __repr__(self):
        return 'DeepDrillRetraction'


class LiftingModule(State):
    def setup(self):
        self._context.stdscr.addstr('Lifting Module')

    def step(self):
        if self._context.module_position > -10:
            self._context.set_state(PushingContainer())
            return
        self._context.move_joint('drill_lift', 0.0)
        self._context.move_joint('module_lift', 0.6)
        self._context.move_joint('drill_spin', 0.0)

    def __repr__(self):
        return 'LiftingModule'


class PushingContainer(State):
    def setup(self):
        self._context.stdscr.addstr('Pushing Container')
        self.start_time = self._context.get_time()

    def step(self):
        if self._context.get_time() - self.start_time > 2:
            self._context.set_state(EmptyingDrill())
            return
        self._context.deep_container_pushed_in = True

    def __repr__(self):
        return 'PushingContainer'


class EmptyingDrill(State):
    def setup(self):
        self._context.stdscr.addstr('Emptying Drill')
        self.start_time = self._context.get_time()

    def step(self):
        if self._context.get_time() - self.start_time > 4:
            self._context.set_state(PushingContainerAway())
            return
        self._context.move_joint('drill_spin', -1.0)

    def __repr__(self):
        return 'EmptyingDrill'


class PushingContainerAway(State):
    def setup(self):
        self._context.stdscr.addstr('Pushing Container Away')
        self.start_time = self._context.get_time()

    def step(self):
        if self._context.get_time() - self.start_time > 2:
            self._context.set_state(StateIdle())
            return
        self._context.deep_container_pushed_in = False
        self._context.move_joint('drill_spin', 0.0)

    def __repr__(self):
        return 'PushingContainerAway'


class LoweringSurfaceModule(State):
    def setup(self):
        self._context.stdscr.addstr('Surface Running')
        self.start_time = self._context.get_time()

    def step(self):
        t = self._context.get_time()
        if ((t - self.start_time > 2 and
             self._context.module_current < -0.2) or
                self._context.module_position < -200):
            self._context.set_state(RegolithCollecting())
            return
        self._context.move_joint('module_lift', -0.4)

    def __repr__(self):
        return 'SurfaceLoweringModule'


class RegolithCollecting(State):
    def setup(self):
        self._context.stdscr.addstr('Regolith Collecting')
        self.start_time = self._context.get_time()

    def step(self):
        if self._context.get_time() - self.start_time > 3:
            self._context.set_state(LiftingSurfaceModule())
            return
        self._context.shovel_in = True

    def __repr__(self):
        return 'RegolithCollecting'


class LiftingSurfaceModule(State):
    def setup(self):
        self._context.stdscr.addstr('Lifting Surface Module')

    def step(self):
        if self._context.module_position > -10:
            self._context.set_state(PushingSurfaceContainer())
            return
        self._context.move_joint('module_lift', 0.6)

    def __repr__(self):
        return 'LiftingSurfaceModule'


class PushingSurfaceContainer(State):
    def setup(self):
        self._context.stdscr.addstr('Pushing Surface Container')
        self.start_time = self._context.get_time()

    def step(self):
        if self._context.get_time() - self.start_time > 3:
            self._context.set_state(EmptyingShovel())
            return
        self._context.surface_container_pushed_in = True

    def __repr__(self):
        return 'PushingSurfaceContainer'


class EmptyingShovel(State):
    def setup(self):
        self._context.stdscr.addstr('Emptying Shovel')
        self.start_time = self._context.get_time()

    def step(self):
        if self._context.get_time() - self.start_time > 3:
            self._context.set_state(PushingSurfaceContainerAway())
            return
        self._context.shovel_emptying = True

    def __repr__(self):
        return 'EmptyingShovel'


class PushingSurfaceContainerAway(State):
    def setup(self):
        self._context.stdscr.addstr('Pushing Surface Container Away')
        self.start_time = self._context.get_time()

    def step(self):
        if self._context.get_time() - self.start_time > 3:
            self._context.set_state(StateIdle())
            return
        self._context.surface_container_pushed_in = False

    def __repr__(self):
        return 'PushingSurfaceContainerAway'


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