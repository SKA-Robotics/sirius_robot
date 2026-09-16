import math
import PyKDL
import spectacularAI
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo

# MDRS datum (default)
_DEFAULT_DATUM_LAT = 38.4200181
_DEFAULT_DATUM_LON = -110.7847004
_DEFAULT_DATUM_ALT = 1380.0


def to_pose_message(camera_pose, stamp: Time):
    msg = PoseStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = 'front_rgb_camera_optical'
    msg.pose.position.x = camera_pose.position.x
    msg.pose.position.y = camera_pose.position.y
    msg.pose.position.z = camera_pose.position.z
    msg.pose.orientation.x = camera_pose.orientation.x
    msg.pose.orientation.y = camera_pose.orientation.y
    msg.pose.orientation.z = camera_pose.orientation.z
    msg.pose.orientation.w = camera_pose.orientation.w
    return msg


def to_odometry_message(vio_output, is_global: bool, stamp: Time,
                        datum_lat=_DEFAULT_DATUM_LAT,
                        datum_lon=_DEFAULT_DATUM_LON,
                        datum_alt=_DEFAULT_DATUM_ALT,
                        marsyard_angle: float = 0.0):
    msg = Odometry()
    msg.header.stamp = stamp
    msg.header.frame_id = 'odom'
    msg.child_frame_id = 'front_rgb_camera_optical'

    if is_global:
        if vio_output.globalPose is None:
            return None
        datum = spectacularAI.WgsCoordinates()
        datum.altitude = datum_alt
        datum.longitude = datum_lon
        datum.latitude = datum_lat
        pose = vio_output.globalPose.getEnuCameraPose(0, datum).pose
        velocity = vio_output.globalPose.velocity
        angular_velocity = vio_output.globalPose.angularVelocity
        pos_cov = vio_output.globalPose.enuPositionCovariance
        vel_cov = vio_output.globalPose.velocityCovariance
    else:
        pose = vio_output.pose
        velocity = vio_output.velocity
        angular_velocity = vio_output.angularVelocity
        pos_cov = vio_output.positionCovariance
        vel_cov = vio_output.velocityCovariance

    if marsyard_angle != 0.0 and is_global:
        frame = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(pose.orientation.x, pose.orientation.y,
                                      pose.orientation.z, pose.orientation.w),
            PyKDL.Vector(pose.position.x, pose.position.y, pose.position.z))
        rotation = PyKDL.Rotation()
        rotation.DoRotZ(math.pi * marsyard_angle / 180)
        frame = PyKDL.Frame(rotation, PyKDL.Vector()) * frame
        q = frame.M.GetQuaternion()
        msg.pose.pose.position.x = frame.p.x()
        msg.pose.pose.position.y = frame.p.y()
        msg.pose.pose.position.z = frame.p.z()
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
    else:
        msg.pose.pose.position.x = pose.position.x
        msg.pose.pose.position.y = pose.position.y
        msg.pose.pose.position.z = pose.position.z
        msg.pose.pose.orientation.x = pose.orientation.x
        msg.pose.pose.orientation.y = pose.orientation.y
        msg.pose.pose.orientation.z = pose.orientation.z
        msg.pose.pose.orientation.w = pose.orientation.w

    for i, row in enumerate(pos_cov):
        for j, value in enumerate(row):
            msg.pose.covariance[6 * i + j] = min(value, 1.0)

    msg.twist.twist.linear.x = velocity.x
    msg.twist.twist.linear.y = velocity.y
    msg.twist.twist.linear.z = velocity.z
    msg.twist.twist.angular.x = angular_velocity.x
    msg.twist.twist.angular.y = angular_velocity.y
    msg.twist.twist.angular.z = angular_velocity.z

    for i, row in enumerate(vel_cov):
        for j, value in enumerate(row):
            msg.twist.covariance[6 * i + j] = value

    return msg


def to_camera_info_message(camera, frame, stamp: Time):
    intrinsic = camera.getIntrinsicMatrix()
    msg = CameraInfo()
    msg.header.stamp = stamp
    msg.header.frame_id = 'front_rgb_camera_optical'
    msg.height = frame.shape[0]
    msg.width = frame.shape[1]
    msg.distortion_model = 'plumb_bob'
    msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    msg.k = intrinsic.ravel().tolist()
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.p = msg.k[0:3] + [0.0] + msg.k[3:6] + [0.0] + msg.k[6:9] + [0.0]
    return msg
