#!/usr/bin/python3
# Drive to an aruco tag detected on a camera
# Crude solution, finds the aruco in camera space, tirns to keep it
# in the center of the forward camera
import numpy as np
import time
import rospy
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import NavSatFix
from math import sin, cos
from geometry_msgs.msg import Twist, PoseStamped
from autonomy.msg import Marker, MarkerArray
from mbf_msgs.msg import MoveBaseAction
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from mbf_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from math import atan2
import pymap3d
from enum import Enum

CMD_VEL_TOPIC = "/cmd_vel"
# MAP_ORIGIN_COORDS = (38.407213, -111.63883, 2139.756) # LOA
MAP_ORIGIN_COORDS = (38.4200181, -110.7847004, 1380.0) # MDRS


class GpsController:
    def __init__(self, target_gps, target_id):
        self.target_id = target_id
        self.target_pose = self.calculate_pose(target_gps)
        self.rover_pose = None
        self.rover_pose_np = None
        self.rover_pose_subscriber = rospy.Subscriber("/slam/global_odometry", Odometry, self.set_rover_pose, queue_size=10)
        self.rover_gps_pose_subscriber = rospy.Subscriber("/gps/fix", NavSatFix, self.set_rover_gps_pose, queue_size=10)
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = self.target_pose[0]
        pose.pose.position.y = self.target_pose[1]
        pose.pose.orientation.w = 1
        self.target_ros_pose = pose
        self.target_pose_publisher = rospy.Publisher("/target_gps", PoseStamped, queue_size=10)
        self.twist_publisher = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=10)
        self.max_forward_rate = 0.3
        self.max_turn_rate = 0.23
        self.distance_threshold = 1.0
        self.turn_gain = 0.8

    def step(self, autonomy):
        
        if self.rover_pose is None or self.rover_pose_np is None:
            return

        self.target_pose_publisher.publish(self.target_ros_pose)

        if autonomy.aruco_spotted(self.target_id):
            print("Stopping gps: aruco spotted")
            autonomy.state = Autonomy.State.FOLLOW_ARUCO
            return
        elif self.target_reached():
            print("Gps target reached")
            autonomy.state = Autonomy.State.SEEK_ARUCO
            return
        
        distance = self.distance_to_target()
        rover_forward_vector = self.calculate_rover_heading()
        target_forward_vector = self.calculate_target_heading()
        print(f"Distance={distance}, rover heading={atan2(rover_forward_vector[0], rover_forward_vector[1])}, target heading={atan2(target_forward_vector[0], target_forward_vector[1])}")
        diff = np.cross(rover_forward_vector, target_forward_vector)

        turn_rate = diff * self.turn_gain
        forward_rate = self.max_forward_rate
        forward_rate = min(forward_rate, distance * 0.2)

        self._send_cmd(forward_rate, turn_rate)
    
    def target_reached(self):
        return (np.linalg.norm(self.target_pose - self.rover_pose_np) < self.distance_threshold)
    
    def set_rover_pose(self, msg):
        self.rover_pose = msg.pose.pose

    def set_rover_gps_pose(self, msg):
        gps = (msg.latitude, msg.longitude)
        pose = self.calculate_pose(gps)
        self.rover_pose_np = pose
    
    def calculate_rover_heading(self):
        quat = np.array([self.rover_pose.orientation.x, self.rover_pose.orientation.y, self.rover_pose.orientation.z, self.rover_pose.orientation.w])
        r = Rotation.from_quat(quat)
        euler = r.as_euler('zyx')
        yaw = euler[0]
        heading_vector = np.array([cos(yaw), sin(yaw)])
        return heading_vector

    def distance_to_target(self):
        return np.linalg.norm(self.target_pose - self.rover_pose_np)
    
    def calculate_target_heading(self):
        target_heading = self.target_pose - self.rover_pose_np
        return target_heading / np.linalg.norm(target_heading)


    def calculate_pose(self, target_gps):
        target = pymap3d.geodetic2enu(
            target_gps[0],
            target_gps[1],
            MAP_ORIGIN_COORDS[2],
            MAP_ORIGIN_COORDS[0],
            MAP_ORIGIN_COORDS[1],
            MAP_ORIGIN_COORDS[2])
        pose = np.array([target[0], target[1]])
        return pose
    
    def _send_cmd(self, forward_rate: float, turn_rate: float):
        forward_rate = min(self.max_forward_rate, max(-self.max_forward_rate, forward_rate))
        turn_rate = min(self.max_turn_rate, max(-self.max_turn_rate, turn_rate))
        print(f"turn_rate={np.round(turn_rate, 2)}, forward_rate={np.round(forward_rate, 2)}")
        msg = Twist()
        msg.linear.x = forward_rate
        msg.angular.z = turn_rate
        self.twist_publisher.publish(msg)



class ArucoSeeker:
    def __init__(self, target_id):
        self.target_id = target_id
        self.forward_rate = 0.2
        # self.min_turn_rate = 0.0
        # self.d_turn_rate = -0.00025
        # self.dd_turn_rate = 0.00000025
        # self.dd_turn_rate = 0.0
        self.t = 0.1
        # self.turn_rate = 0.2
        self.twist_publisher = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=10)

    def step(self, autonomy):
        if self.target_id is None:
            autonomy.state = Autonomy.State.DONE
            return
        # self.d_turn_rate += self.dd_turn_rate
        # self.d_turn_rate = min(self.d_turn_rate, 0)
        # self.turn_rate += self.d_turn_rate
        # self.turn_rate = max(self.min_turn_rate, self.turn_rate)
        self.t += 0.0003333333
        self.turn_rate = self.forward_rate / (2 * np.pi * self.t)
        if autonomy.aruco_spotted(self.target_id):
            print("Aruco spotted")
            autonomy.state = Autonomy.State.FOLLOW_ARUCO
        msg = Twist()
        msg.linear.x = self.forward_rate
        msg.angular.z = self.turn_rate
        self.twist_publisher.publish(msg)
        print(autonomy.marker_ids)
        # print(f"turn_rate={np.round(self.turn_rate, 6)}, d_turn_rate={np.round(self.d_turn_rate, 6)}, forward_rate={np.round(self.forward_rate, 2)}")
    


class ArucoFollower:
    def __init__(self, target_id):
        self.target_id = target_id
        self.turn_gain = 0.6
        self.max_turn_rate = 0.2
        self.max_forward_rate = 0.5
        self.aruco_detection_timeout = 5
        self.last_detection = 0
        self.marker_size_stop_threshold = 0.069
        self.twist_publisher = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=10)
    
    def step(self, autonomy):
        if self.target_id is None:
            autonomy.state = Autonomy.State.DONE
            return
        # Following marker
        if self.target_id in autonomy.marker_ids["forward"]:
            self.last_detection = time.time()
            idx = autonomy.marker_ids["forward"].index(self.target_id)
            marker_position = autonomy.marker_positions["forward"][idx]
            self.turn_rate = -marker_position * self.turn_gain
            self.forward_rate = self.max_forward_rate
        elif self.target_id in autonomy.marker_ids["left"]:
            self.last_detection = time.time()
            self.turn_rate = self.max_turn_rate
            self.forward_rate = 0.2 * self.max_forward_rate
        elif self.target_id in autonomy.marker_ids["right"]:
            self.last_detection = time.time()
            self.turn_rate = -self.max_turn_rate
            self.forward_rate = 0.2 * self.max_forward_rate
        elif self.target_id in autonomy.marker_ids["backward"]:
            self.last_detection = time.time()
            self.turn_rate = 0.333 * self.max_turn_rate
            self.forward_rate = 0.0
        else: # Tag is not detected
            if time.time() > self.last_detection + self.aruco_detection_timeout:
                self.turn_rate = 0.0
                self.forward_rate = 0.0

        print(autonomy.marker_ids)
        print(np.round(autonomy.marker_sizes["forward"], 6))

        if self._stop_condition(autonomy):
            print("Stop condition is true")
            self.turn_rate = 0.0
            self.forward_rate = 0.0
            autonomy.state = Autonomy.State.DONE

        self._send_cmd(self.forward_rate, self.turn_rate)
    
    def _stop_condition(self, autonomy):
        if self.target_id in autonomy.marker_ids["forward"]:
            idx = autonomy.marker_ids["forward"].index(self.target_id)
            marker_size = autonomy.marker_sizes["forward"][idx]
            if marker_size > self.marker_size_stop_threshold:
                print("Marker size threshold exceeded.")
                return True

        return False
        
    def _send_cmd(self, forward_rate: float, turn_rate: float):
        forward_rate = min(self.max_forward_rate, max(-self.max_forward_rate, forward_rate))
        turn_rate = min(self.max_turn_rate, max(-self.max_turn_rate, turn_rate))
        print(f"turn_rate={np.round(turn_rate, 2)}, forward_rate={np.round(forward_rate, 2)}")
        msg = Twist()
        msg.linear.x = forward_rate
        msg.angular.z = turn_rate
        self.twist_publisher.publish(msg)



class Autonomy:

    class State(Enum):
        GOTO_GPS=0
        SEEK_ARUCO=1
        FOLLOW_ARUCO=2
        DONE=3
    
    class Finished(Exception):
        def __init__(self, *args):
            super().__init__(*args)

    def __init__(self, gps_coords, marker_id):
        self.finished = False
        self.aruco_topics = {
            "forward": "/aruco_bow/aruco_markers",
            "backward": "/aruco_stern/aruco_markers",
            "left": "/aruco_port/aruco_markers",
            "right": "/aruco_starboard/aruco_markers",
        }
        self.aruco_subscribers = {
            key: rospy.Subscriber(item, MarkerArray, self.callback, callback_args=key, queue_size=10) for key, item in self.aruco_topics.items()
        }
        self.marker_ids = {
            "forward": [],
            "backward": [],
            "left": [],
            "right": [],
        }
        self.marker_positions = {
            "forward": [],
            "backward": [],
            "left": [],
            "right": [],
        }
        self.marker_sizes = {
            "forward": [],
            "backward": [],
            "left": [],
            "right": [],
        }
        self.lamp_publisher = rospy.Publisher("lamps/color_override", String, queue_size=10)
        self.status_publisher = rospy.Publisher("navigation/status", String, queue_size=10)
        self.rate = 10
        self.state = Autonomy.State.GOTO_GPS
        self.gps_controller = GpsController(gps_coords, marker_id)
        self.aruco_seeker = ArucoSeeker(marker_id)
        self.aruco_follower = ArucoFollower(marker_id)
    
    def run(self):
        self.timer = rospy.Timer(rospy.Duration(1/self.rate), self._step)
        try:
            while not self.finished:
                time.sleep(0.1)
        except (KeyboardInterrupt):
            pass

    def callback(self, message, direction):
        self.marker_ids[direction] = [marker.id for marker in message.markers]
        self.marker_positions[direction] = [marker.position for marker in message.markers]
        self.marker_sizes[direction] = [marker.size for marker in message.markers]
    
    def _step(self, _):
        self.set_lamp("red")
        self.status_publisher.publish(String("running"))
        if self.finished:
            return
        if self.state == Autonomy.State.GOTO_GPS:
            self.gps_controller.step(self)
        elif self.state == Autonomy.State.SEEK_ARUCO:
            self.aruco_seeker.step(self)
        elif self.state == Autonomy.State.FOLLOW_ARUCO:
            self.aruco_follower.step(self)
        elif self.state == Autonomy.State.DONE:
            print("Target reached")
            self.finished = True
            self.set_lamp("green")
            for i in range(50):
                self.status_publisher.publish(String("done"))
                time.sleep(0.2)

    def aruco_spotted(self, aruco_id):
        if aruco_id in self.marker_ids["forward"]:
            return True
        if aruco_id in self.marker_ids["backward"]:
            return True
        if aruco_id in self.marker_ids["left"]:
            return True
        if aruco_id in self.marker_ids["right"]:
            return True
        return False
    
    def set_lamp(self, color):
        self.lamp_publisher.publish(String(color))
    
    def animation(self):
        self.set_lamp("green")
        time.sleep(1)
        self.set_lamp("yellow")
        time.sleep(1)
        self.set_lamp("red")
        time.sleep(1)
        self.set_lamp("green")
        time.sleep(1)

def clear_aruco(seconds):
    time.sleep(5)
    publisher = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=10)
    cmd = Twist()
    cmd.linear.x = -0.4
    cmd.angular.z = 0.2
    dt = 0.1
    print("Clearing aruco tag")
    for i in range(int(seconds/dt)):
        publisher.publish(cmd)
        time.sleep(dt)
    print("Done")
    cmd.linear.x = 0.0
    cmd.angular.z = 0.0
    publisher.publish(cmd)

def drive_forward(seconds):
    publisher = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=10)
    cmd = Twist()
    cmd.linear.x = 0.4
    cmd.angular.z = 0.0
    dt = 0.1
    print("Clearing aruco tag")
    for i in range(int(seconds/dt)):
        publisher.publish(cmd)
        time.sleep(0.1)
    print("Done")
    cmd.linear.x = 0.0
    cmd.angular.z = 0.0
    publisher.publish(cmd)



if __name__=="__main__":
    rospy.init_node("aruco_follower")
    print("GOD BLESS THE AUTONOMY")


    # drive_forward(60)
    Autonomy((38.41928851, -110.7831721), None).run()

    # clear_aruco(15)
    # Autonomy((38.41957183, -110.7833951), None).run()



    # clear_aruco(15)
    # Autonomy((38.41965712, -110.7843735), 2).run()

    # Autonomy((38.41965712, -110.7843735), None).run()
    # clear_aruco(15)


    # clear_aruco(10)
    # Autonomy((38.4072654, -111.6385895), 4).run()
    # Autonomy((38.4072654, -111.6385687), None).run()

    # clear_aruco(10)
    # Autonomy((38.4076558, -111.6385895), 5).run()
    # Autonomy((38.4076558, -111.6385895), None).run()

