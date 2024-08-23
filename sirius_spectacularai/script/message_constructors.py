import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
import spectacularAI


def to_pose_message(cameraPose):
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "front_rgb_camera_optical"

    msg.pose.position.x = cameraPose.position.x
    msg.pose.position.y = cameraPose.position.y
    msg.pose.position.z = cameraPose.position.z
    msg.pose.orientation.x = cameraPose.orientation.x
    msg.pose.orientation.y = cameraPose.orientation.y
    msg.pose.orientation.z = cameraPose.orientation.z
    msg.pose.orientation.w = cameraPose.orientation.w
    return msg


def to_odometry_message(vioOutput, is_global=False):
    msg = Odometry()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "odom"
    msg.child_frame_id = "front_rgb_camera_optical"

    if vioOutput.globalPose is None and is_global == True:
        #return None
        is_global = False

    if is_global:
        datum = spectacularAI.WgsCoordinates()
        datum.altitude = 907.476
        datum.latitude = 39.9011333333333333
        datum.longitude = 32.77005

        # Środek marsjardu
        # latitude: 39.9013943
        # longitude: 32.7704792
        # altitude: 907.476

        pose = vioOutput.globalPose.getEnuCameraPose(0, datum).pose
        velocity = vioOutput.globalPose.velocity
        angularVelocity = vioOutput.globalPose.angularVelocity
        positionCovariance = vioOutput.globalPose.enuPositionCovariance
        velocityCovariance = vioOutput.globalPose.velocityCovariance
    else:
        pose = vioOutput.pose
        velocity = vioOutput.velocity
        angularVelocity = vioOutput.angularVelocity
        positionCovariance = vioOutput.positionCovariance
        velocityCovariance = vioOutput.velocityCovariance

    msg.pose.pose.position.x = pose.position.x
    msg.pose.pose.position.y = pose.position.y
    msg.pose.pose.position.z = pose.position.z
    msg.pose.pose.orientation.x = pose.orientation.x
    msg.pose.pose.orientation.y = pose.orientation.y
    msg.pose.pose.orientation.z = pose.orientation.z
    msg.pose.pose.orientation.w = pose.orientation.w

    for i, row in enumerate(positionCovariance):
        for j, value in enumerate(row):
            if value > 1.0:
                value = 1

            msg.pose.covariance[6 * i + j] = value

    msg.twist.twist.linear.x = velocity.x
    msg.twist.twist.linear.y = velocity.y
    msg.twist.twist.linear.z = velocity.z
    msg.twist.twist.angular.x = angularVelocity.x
    msg.twist.twist.angular.y = angularVelocity.y
    msg.twist.twist.angular.z = angularVelocity.z

    for i, row in enumerate(velocityCovariance):
        for j, value in enumerate(row):
            msg.twist.covariance[6 * i + j] = value

    return msg


def to_camera_info_message(camera, frame, ts):
    intrinsic = camera.getIntrinsicMatrix()
    msg = CameraInfo()
    msg.header.stamp = ts
    msg.header.frame_id = "front_rgb_camera_optical"
    msg.height = frame.shape[0]
    msg.width = frame.shape[1]
    msg.distortion_model = "plumb_bob"
    msg.D = [0, 0, 0, 0, 0]
    msg.K = intrinsic.ravel().tolist()
    msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.P = msg.K[0:3] + [0] + msg.K[3:6] + [0] + msg.K[6:9] + [0]
    return msg
