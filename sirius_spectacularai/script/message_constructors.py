import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
import spectacularAI
import PyKDL
import math


def to_pose_message(cameraPose, frame_id):
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id

    msg.pose.position.x = cameraPose.position.x
    msg.pose.position.y = cameraPose.position.y
    msg.pose.position.z = cameraPose.position.z
    msg.pose.orientation.x = cameraPose.orientation.x
    msg.pose.orientation.y = cameraPose.orientation.y
    msg.pose.orientation.z = cameraPose.orientation.z
    msg.pose.orientation.w = cameraPose.orientation.w
    return msg


def to_odometry_message(vioOutput, coordinates,
                        config,
                        is_global=False,):
    msg = Odometry()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = config.get('world_frame', 'odom')
    msg.child_frame_id = config.get('frame_id',
                                    'front_rgb_camera_optical')

    if vioOutput.globalPose is None and is_global:
        return None

    if is_global:
        datum = spectacularAI.WgsCoordinates()

        # MDRS
        datum.altitude = coordinates.altitude
        datum.longitude = coordinates.longitude
        datum.latitude = coordinates.latitude

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

    marsyard_angle = config.get('marsyard_angle', 0)

    if marsyard_angle != 0.0 and is_global is True:
        frame = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(pose.orientation.x, pose.orientation.y,
                                      pose.orientation.z, pose.orientation.w),
            PyKDL.Vector(pose.position.x, pose.position.y, pose.position.z))

        rotation = PyKDL.Rotation()
        rotation.DoRotZ(math.pi * marsyard_angle / 180)
        frame = PyKDL.Frame(rotation, PyKDL.Vector()) * frame

        quaternion = frame.M.GetQuaternion()
        msg.pose.pose.position.x = frame.p.x()
        msg.pose.pose.position.y = frame.p.y()
        msg.pose.pose.position.z = frame.p.z()
        msg.pose.pose.orientation.x = quaternion[0]
        msg.pose.pose.orientation.y = quaternion[1]
        msg.pose.pose.orientation.z = quaternion[2]
        msg.pose.pose.orientation.w = quaternion[3]
    else:
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


def to_camera_info_message(camera, frame, frame_id, ts):
    intrinsic = camera.getIntrinsicMatrix()
    msg = CameraInfo()
    msg.header.stamp = ts
    msg.header.frame_id = frame_id
    msg.height = frame.shape[0]
    msg.width = frame.shape[1]
    msg.distortion_model = "plumb_bob"
    msg.D = [0, 0, 0, 0, 0]
    msg.K = intrinsic.ravel().tolist()
    msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.P = msg.K[0:3] + [0] + msg.K[3:6] + [0] + msg.K[6:9] + [0]
    return msg
