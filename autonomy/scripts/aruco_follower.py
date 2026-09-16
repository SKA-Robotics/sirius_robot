#!/usr/bin/env python3
# Drive to an ArUco tag using GPS approach followed by visual servoing.

import time
from enum import Enum
from math import cos, sin, atan2

import numpy as np
from scipy.spatial.transform import Rotation
import pymap3d

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose

from autonomy.msg import MarkerArray

CMD_VEL_TOPIC = '/cmd_vel'
MAP_ORIGIN_COORDS = (38.4200181, -110.7847004, 1380.0)  # MDRS


class GpsController:

    def __init__(self, target_gps, target_id, node: Node):
        self._node = node
        self.target_id = target_id
        self.target_pose = self._enu_from_gps(target_gps)
        self.rover_pose = None
        self.rover_pose_np = None

        node.create_subscription(Odometry, '/slam/global_odometry',
                                 self._set_rover_pose, 10)
        node.create_subscription(NavSatFix, '/gps/fix',
                                 self._set_rover_gps_pose, 10)

        self.target_pose_publisher = node.create_publisher(
            PoseStamped, '/target_gps', 10)
        self.twist_publisher = node.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.max_forward_rate = 0.3
        self.max_turn_rate = 0.23
        self.distance_threshold = 1.0
        self.turn_gain = 0.8

    def step(self, autonomy):
        if self.rover_pose is None or self.rover_pose_np is None:
            return

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self._node.get_clock().now().to_msg()
        pose_msg.pose.position.x = self.target_pose[0]
        pose_msg.pose.position.y = self.target_pose[1]
        pose_msg.pose.orientation.w = 1.0
        self.target_pose_publisher.publish(pose_msg)

        if autonomy.aruco_spotted(self.target_id):
            self._node.get_logger().info('Stopping GPS: ArUco spotted')
            autonomy.state = AutonomyState.FOLLOW_ARUCO
            return

        if self._target_reached():
            self._node.get_logger().info('GPS target reached')
            autonomy.state = AutonomyState.SEEK_ARUCO
            return

        distance = np.linalg.norm(self.target_pose - self.rover_pose_np)
        rover_fwd = self._rover_heading()
        target_fwd = (self.target_pose - self.rover_pose_np) / max(
            distance, 1e-6)
        diff = float(np.cross(rover_fwd, target_fwd))
        turn_rate = diff * self.turn_gain
        forward_rate = min(self.max_forward_rate, distance * 0.2)
        self._send_cmd(forward_rate, turn_rate)

    def _target_reached(self):
        return (np.linalg.norm(self.target_pose - self.rover_pose_np) <
                self.distance_threshold)

    def _set_rover_pose(self, msg: Odometry):
        self.rover_pose = msg.pose.pose

    def _set_rover_gps_pose(self, msg: NavSatFix):
        self.rover_pose_np = self._enu_from_gps(
            (msg.latitude, msg.longitude))

    def _rover_heading(self):
        o = self.rover_pose.orientation
        r = Rotation.from_quat([o.x, o.y, o.z, o.w])
        yaw = r.as_euler('zyx')[0]
        return np.array([cos(yaw), sin(yaw)])

    def _enu_from_gps(self, gps):
        e, n, _ = pymap3d.geodetic2enu(
            gps[0], gps[1], MAP_ORIGIN_COORDS[2],
            MAP_ORIGIN_COORDS[0], MAP_ORIGIN_COORDS[1], MAP_ORIGIN_COORDS[2])
        return np.array([e, n])

    def _send_cmd(self, forward_rate, turn_rate):
        forward_rate = max(-self.max_forward_rate,
                           min(self.max_forward_rate, forward_rate))
        turn_rate = max(-self.max_turn_rate,
                        min(self.max_turn_rate, turn_rate))
        msg = Twist()
        msg.linear.x = forward_rate
        msg.angular.z = turn_rate
        self.twist_publisher.publish(msg)


class ArucoSeeker:

    def __init__(self, target_id, node: Node):
        self.target_id = target_id
        self.forward_rate = 0.2
        self.t = 0.1
        self.twist_publisher = node.create_publisher(Twist, CMD_VEL_TOPIC, 10)

    def step(self, autonomy):
        if self.target_id is None:
            autonomy.state = AutonomyState.DONE
            return
        self.t += 0.000333
        turn_rate = self.forward_rate / (2 * np.pi * self.t)
        if autonomy.aruco_spotted(self.target_id):
            autonomy.state = AutonomyState.FOLLOW_ARUCO
        msg = Twist()
        msg.linear.x = self.forward_rate
        msg.angular.z = turn_rate
        self.twist_publisher.publish(msg)


class ArucoFollower:

    def __init__(self, target_id, node: Node):
        self.target_id = target_id
        self.turn_gain = 0.6
        self.max_turn_rate = 0.2
        self.max_forward_rate = 0.5
        self.aruco_detection_timeout = 5.0
        self.last_detection = 0.0
        self.marker_size_stop_threshold = 0.069
        self.twist_publisher = node.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.turn_rate = 0.0
        self.forward_rate = 0.0

    def step(self, autonomy):
        if self.target_id is None:
            autonomy.state = AutonomyState.DONE
            return

        if self.target_id in autonomy.marker_ids['forward']:
            self.last_detection = time.time()
            idx = autonomy.marker_ids['forward'].index(self.target_id)
            pos = autonomy.marker_positions['forward'][idx]
            self.turn_rate = -pos * self.turn_gain
            self.forward_rate = self.max_forward_rate
        elif self.target_id in autonomy.marker_ids['left']:
            self.last_detection = time.time()
            self.turn_rate = self.max_turn_rate
            self.forward_rate = 0.2 * self.max_forward_rate
        elif self.target_id in autonomy.marker_ids['right']:
            self.last_detection = time.time()
            self.turn_rate = -self.max_turn_rate
            self.forward_rate = 0.2 * self.max_forward_rate
        elif self.target_id in autonomy.marker_ids['backward']:
            self.last_detection = time.time()
            self.turn_rate = 0.333 * self.max_turn_rate
            self.forward_rate = 0.0
        elif time.time() > self.last_detection + self.aruco_detection_timeout:
            self.turn_rate = 0.0
            self.forward_rate = 0.0

        if self._stop_condition(autonomy):
            self.turn_rate = 0.0
            self.forward_rate = 0.0
            autonomy.state = AutonomyState.DONE

        self._send_cmd(self.forward_rate, self.turn_rate)

    def _stop_condition(self, autonomy):
        if self.target_id in autonomy.marker_ids['forward']:
            idx = autonomy.marker_ids['forward'].index(self.target_id)
            return autonomy.marker_sizes['forward'][idx] > self.marker_size_stop_threshold
        return False

    def _send_cmd(self, forward_rate, turn_rate):
        forward_rate = max(-self.max_forward_rate,
                           min(self.max_forward_rate, forward_rate))
        turn_rate = max(-self.max_turn_rate,
                        min(self.max_turn_rate, turn_rate))
        msg = Twist()
        msg.linear.x = forward_rate
        msg.angular.z = turn_rate
        self.twist_publisher.publish(msg)


class AutonomyState(Enum):
    GOTO_GPS = 0
    SEEK_ARUCO = 1
    FOLLOW_ARUCO = 2
    DONE = 3


ARUCO_DIRECTIONS = ('forward', 'backward', 'left', 'right')
ARUCO_TOPICS = {
    'forward': '/aruco_bow/aruco_markers',
    'backward': '/aruco_stern/aruco_markers',
    'left': '/aruco_port/aruco_markers',
    'right': '/aruco_starboard/aruco_markers',
}


class ArucoFollowerNode(Node):

    def __init__(self):
        super().__init__('aruco_follower')

        self.declare_parameter('target_latitude', 38.41928851)
        self.declare_parameter('target_longitude', -110.7831721)
        self.declare_parameter('target_marker_id', -1)

        lat = self.get_parameter('target_latitude').value
        lon = self.get_parameter('target_longitude').value
        marker_id_raw = self.get_parameter('target_marker_id').value
        marker_id = None if marker_id_raw < 0 else marker_id_raw

        self.marker_ids = {d: [] for d in ARUCO_DIRECTIONS}
        self.marker_positions = {d: [] for d in ARUCO_DIRECTIONS}
        self.marker_sizes = {d: [] for d in ARUCO_DIRECTIONS}

        for direction, topic in ARUCO_TOPICS.items():
            self.create_subscription(
                MarkerArray, topic,
                lambda msg, d=direction: self._aruco_callback(msg, d), 10)

        self.lamp_publisher = self.create_publisher(String, 'lamps/color_override', 10)
        self.status_publisher = self.create_publisher(String, 'navigation/status', 10)

        self.state = AutonomyState.GOTO_GPS
        self._finished = False

        self.gps_controller = GpsController((lat, lon), marker_id, self)
        self.aruco_seeker = ArucoSeeker(marker_id, self)
        self.aruco_follower = ArucoFollower(marker_id, self)

        self.create_timer(0.1, self._step)  # 10 Hz

    def _aruco_callback(self, msg: MarkerArray, direction: str):
        self.marker_ids[direction] = [m.id for m in msg.markers]
        self.marker_positions[direction] = [m.position for m in msg.markers]
        self.marker_sizes[direction] = [m.size for m in msg.markers]

    def _step(self):
        if self._finished:
            return
        self.lamp_publisher.publish(String(data='red'))
        self.status_publisher.publish(String(data='running'))

        if self.state == AutonomyState.GOTO_GPS:
            self.gps_controller.step(self)
        elif self.state == AutonomyState.SEEK_ARUCO:
            self.aruco_seeker.step(self)
        elif self.state == AutonomyState.FOLLOW_ARUCO:
            self.aruco_follower.step(self)
        elif self.state == AutonomyState.DONE:
            self.get_logger().info('Target reached')
            self._finished = True
            self.lamp_publisher.publish(String(data='green'))
            self.status_publisher.publish(String(data='done'))

    def aruco_spotted(self, aruco_id):
        if aruco_id is None:
            return False
        return any(aruco_id in self.marker_ids[d] for d in ARUCO_DIRECTIONS)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
