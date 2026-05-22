import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
import spectacularAI
import PyKDL
import math


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
        #datum.altitude = 250
        #datum.latitude = 50.066220965664
        #datum.longitude = 19.91318631828

        #datum.altitude = 245.226
        #datum.latitude = 50.0662169
        #datum.longitude = 19.913204399999998

        # ITC
        #datum.altitude = 148.824
        #datum.longitude = 21.010336799999997
        #datum.latitude = 52.2198959

        # Loa
        # datum.altitude = 2139.756
        # datum.longitude = -111.6388144
        # datum.latitude = 38.407213

        # MDRS
        #datum.altitude = 1380.0
        #datum.longitude = -110.7847004
        #datum.latitude = 38.4200181

        # WAT
        #datum.altitude = 142.673
        #datum.latitude = 52.2527425
        #datum.longitude = 20.9064178

        # AGH akademik
        #datum.latitude = 50.0692336
        #datum.longitude = 19.9046307
        #datum.altitude = 246.15800000000002
        # ERC Marsyard
        datum.latitude = 50.0662241
        datum.longitude = 19.9131708
        #datum.altitude = 245.53
        datum.altitude = 243.6
        """
        PUNKT 0,0
        ==================================
        header: 
        seq: 899
        stamp: 
            secs: 1725524793
            nsecs:    397459
        frame_id: "gnss"
        status: 
        status: 0
        service: 1
        latitude: 50.0662169
        longitude: 19.913204399999998
        altitude: 245.226
        position_covariance: [0.0064, 0.0, 0.0, 0.0, 0.0064, 0.0, 0.0, 0.0, 0.011236]
        position_covariance_type: 2

        PUNKT K2
        ======================================
        header: 
        seq: 1251
        stamp: 
            secs: 1725525145
            nsecs:    307394
        frame_id: "gnss"
        status: 
        status: 0
        service: 1
        latitude: 50.066166499999994
        longitude: 19.913472199999998
        altitude: 244.61100000000002
        position_covariance: [0.007568999999999999, 0.0, 0.0, 0.0, 0.007568999999999999, 0.0, 0.0, 0.0, 0.013225]
        position_covariance_type: 2
        """

        # Środek marsjardu ARC
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

    #marsyard_angle = 106.340833
    marsyard_angle = 0

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
