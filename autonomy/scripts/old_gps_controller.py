/navigation/status
std_msgs::msg::String
wartość "running" lub "done"
po zakończeniu wysyłana 50 razy przez 10 sekund


class GpsControllerLegacy:
    def __init__(self, gps_target, target_id):
        self.gps_target = gps_target
        self.target_id = target_id
        self.pose_target = self.calculate_pose_target(gps_target)
        self.move_base_action = SimpleActionClient("move_base/move_base", MoveBaseAction)
        self.pose_publisher = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.running = False
        self.done = False

    def calculate_pose_target(self, gps_target):
        target = pymap3d.geodetic2enu(
            gps_target[0],
            gps_target[1],
            MAP_ORIGIN_COORDS[2],
            MAP_ORIGIN_COORDS[0],
            MAP_ORIGIN_COORDS[1],
            MAP_ORIGIN_COORDS[2])
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = target[0]
        pose.pose.position.y = target[1]
        pose.pose.position.z = target[2]
        pose.pose.orientation.w = 1
        return pose
    
    def _navigate_to_pose(self, pose: PoseStamped):
        print("Sent navigation goal")
        self.running = True
        self.pose_publisher.publish(pose)
        # self.move_base_action.send_goal(MoveBaseGoal(target_pose=pose), self.move_base_done)
        
    
    def move_base_done(self, terminal_state, result):
        self.done = True
    
    def is_failed(self):
        failed_status = {
            GoalStatus.ABORTED, GoalStatus.REJECTED,
            GoalStatus.RECALLED, GoalStatus.LOST
        }
        return (self.move_base_action.get_state() in failed_status)

    def stop_navigating(self):
        print("Stop navigating")
        self.move_base_action.cancel_all_goals()

    def step(self, autonomy):
        if self.done:
            print("GNSS coordinates reached.")
            self.stop_navigating()
            autonomy.state = Autonomy.State.SEEK_ARUCO
            return
        if self._aruco_detected(autonomy):
            print("Aruco tag spotted")
            self.stop_navigating()
            autonomy.state = Autonomy.State.FOLLOW_ARUCO
            return

        if (not self.running):
            self._navigate_to_pose(self.pose_target)
        if self.is_failed():
            print("Failed. retrying")
            time.sleep(1)
            self._navigate_to_pose(self.pose_target)
        
    def _aruco_detected(self, autonomy):
        if self.target_id in autonomy.marker_ids["forward"]:
            return True
        if self.target_id in autonomy.marker_ids["backward"]:
            return True
        if self.target_id in autonomy.marker_ids["left"]:
            return True
        if self.target_id in autonomy.marker_ids["right"]:
            return True
        return False