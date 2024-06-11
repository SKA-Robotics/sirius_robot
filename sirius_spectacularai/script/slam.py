#!/usr/bin/python3
"""
!!! For ROS2 see ros2/ folder for a more complete example !!!

Runs spectacularAI mapping and publishes poses and frames in ROS.

Make sure to have your ROS environment sourced before running this script. Tested with ROS noetic.

The SpectacularAI SDK and other dependencies can for example be installed in a virtual environment.
"""
import spectacularAI
import depthai
import rospy
import numpy as np
import PyKDL
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import Vector3, Twist, Pose, PoseStamped, PoseWithCovariance, TwistWithCovariance
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField


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


def to_odometry_message(vioOutput):
    msg = Odometry()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    msg.child_frame_id = "front_rgb_camera_optical"

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


def transform_odometry_child_frame(msg, target_frame, tf_buffer):
    """Transform the child frame of the odom message to the target frame."""

    target_pose = transform_pose_child_frame(msg.pose.pose, target_frame,
                                             msg.child_frame_id, tf_buffer,
                                             msg.header.stamp)
    target_twist = transform_twist_child_frame(msg.twist.twist, target_frame,
                                               msg.child_frame_id, tf_buffer,
                                               msg.header.stamp)

    return Odometry(header=msg.header,
                    child_frame_id=target_frame,
                    pose=PoseWithCovariance(pose=target_pose),
                    twist=TwistWithCovariance(twist=target_twist))


def transform_pose_child_frame(pose, target_frame, child_frame, tf_buffer,
                               time):
    """Transform the child frame of the odom message to the target frame."""

    # Convert the pose to a PyKDL Frame
    child_to_parent_transform = PyKDL.Frame(
        PyKDL.Rotation.Quaternion(pose.orientation.x, pose.orientation.y,
                                  pose.orientation.z, pose.orientation.w),
        PyKDL.Vector(pose.position.x, pose.position.y, pose.position.z))

    # Lookup the transform from the target frame to the child frame
    # and convert it to a PyKDL Frame
    # This is the same as the target pose in the child frame
    target_to_child_transform = tf2_geometry_msgs.transform_to_kdl(
        tf_buffer.lookup_transform(child_frame, target_frame, time))

    # Combine the two transforms to get the target pose in the parent frame
    target_to_parent_transform = child_to_parent_transform * \
        target_to_child_transform

    # Convert the transform to a geometry_msgs/Pose
    target_pose = Pose()
    target_pose.position.x = target_to_parent_transform[(0, 3)]
    target_pose.position.y = target_to_parent_transform[(1, 3)]
    target_pose.position.z = target_to_parent_transform[(2, 3)]
    (target_pose.orientation.x, target_pose.orientation.y,
     target_pose.orientation.z, target_pose.orientation.w) = \
        target_to_parent_transform.M.GetQuaternion()

    return target_pose


def transform_twist(twist, transform):
    linear_velocity = PyKDL.Vector(twist.linear.x, twist.linear.y,
                                   twist.linear.z)
    angular_velocity = PyKDL.Vector(twist.angular.x, twist.angular.y,
                                    twist.angular.z)

    translation = PyKDL.Vector(
        transform[0, 3],
        transform[1, 3],
        transform[2, 3],
    )
    rotation = PyKDL.Rotation(
        transform[0, 0],
        transform[1, 0],
        transform[2, 0],
        transform[0, 1],
        transform[1, 1],
        transform[2, 1],
        transform[0, 2],
        transform[1, 2],
        transform[2, 2],
    )

    target_linear_velocity = rotation * \
        linear_velocity + angular_velocity * translation
    target_angular_velocity = rotation * angular_velocity

    return Twist(
        Vector3(target_linear_velocity[0], target_linear_velocity[1],
                target_linear_velocity[2]),
        Vector3(target_angular_velocity[0], target_angular_velocity[1],
                target_angular_velocity[2]))


def transform_twist_child_frame(twist, target_frame, child_frame, tf_buffer,
                                time):

    # Lookup the transform from the child frame to the target frame
    # and convert it to a PyKDL Frame
    child_to_target_transform = tf2_geometry_msgs.transform_to_kdl(
        tf_buffer.lookup_transform(target_frame, child_frame, time))

    twist = transform_twist(twist, child_to_target_transform)

    return twist


def to_camera_info_message(camera, frame, ts):
    intrinsic = camera.getIntrinsicMatrix()
    msg = CameraInfo()
    msg.header.stamp = ts
    msg.header.frame_id = "rgb_optical"
    msg.height = frame.shape[0]
    msg.width = frame.shape[1]
    msg.distortion_model = "none"
    msg.D = []
    msg.K = intrinsic.ravel().tolist()
    return msg


class SLAMNode:

    def __init__(self):
        rospy.init_node("slam_node", anonymous=True)
        self.odometry_publisher = rospy.Publisher("/slam/odometry",
                                                  Odometry,
                                                  queue_size=10)
        """
        self.keyframe_publisher = rospy.Publisher("/slam/keyframe",
                                                  PoseStamped,
                                                  queue_size=10)
        self.rgb_publisher = rospy.Publisher("/slam/rgb", Image, queue_size=10)
        self.point_publisher = rospy.Publisher("/slam/pointcloud",
                                               PointCloud2,
                                               queue_size=10)
        self.depth_publisher = rospy.Publisher("/slam/depth",
                                               Image,
                                               queue_size=10)
        self.camera_info_publisher = rospy.Publisher("/slam/camera_info",
                                                     CameraInfo,
                                                     queue_size=10)
        """
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.bridge = CvBridge()
        self.keyframes = {}

    def has_keyframe(self, frame_id):
        return frame_id in self.keyframes

    def newKeyFrame(self, frame_id, keyframe):
        now = rospy.Time.now()
        self.keyframes[frame_id] = True
        camToWorld = keyframe.frameSet.rgbFrame.cameraPose.pose
        sequence_number = int(frame_id)

        msg = to_pose_message(camToWorld)
        msg.header.seq = sequence_number
        msg.header.stamp = now
        self.keyframe_publisher.publish(msg)

        rgb_frame = keyframe.frameSet.getUndistortedFrame(
            keyframe.frameSet.rgbFrame).image
        rgb_frame = rgb_frame.toArray()
        rgb_message = self.bridge.cv2_to_imgmsg(rgb_frame, encoding="rgb8")
        rgb_message.header.stamp = now
        rgb_message.header.frame_id = "front_rgb_camera_optical"
        rgb_message.header.seq = sequence_number
        self.rgb_publisher.publish(rgb_message)

        camera = keyframe.frameSet.rgbFrame.cameraPose.camera
        info_msg = to_camera_info_message(camera, rgb_frame, now)
        self.camera_info_publisher.publish(info_msg)

        self.newPointCloud(keyframe)

        depth_frame = keyframe.frameSet.getAlignedDepthFrame(
            keyframe.frameSet.rgbFrame)
        depth = depth_frame.image.toArray()
        depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding="mono16")
        depth_msg.header.stamp = now
        depth_msg.header.frame_id = "front_rgb_camera_optical"
        depth_msg.header.seq = sequence_number
        self.depth_publisher.publish(depth_msg)

    def newOdometryFrame(self, vioOutput):
        msg = to_odometry_message(vioOutput)
        msg = transform_odometry_child_frame(msg, "base_link", self.tf_buffer)
        self.odometry_publisher.publish(msg)

    def newPointCloud(self, keyframe):
        camToWorld = keyframe.frameSet.rgbFrame.cameraPose.getCameraToWorldMatrix(
        )
        positions = keyframe.pointCloud.getPositionData()
        pc = np.zeros((positions.shape[0], 6), dtype=np.float32)
        p_C = np.vstack((positions.T, np.ones((1, positions.shape[0])))).T
        pc[:, :3] = (camToWorld @ p_C[:, :, None])[:, :3, 0]

        msg = PointCloud2()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        if keyframe.pointCloud.hasColors():
            pc[:, 3:] = keyframe.pointCloud.getRGB24Data() * (1. / 255.)
        msg.point_step = 4 * 6
        msg.height = 1
        msg.width = pc.shape[0]
        msg.row_step = msg.point_step * pc.shape[0]
        msg.data = pc.tobytes()
        msg.is_bigendian = False
        msg.is_dense = False
        ros_dtype = PointField.FLOAT32
        itemsize = np.dtype(np.float32).itemsize
        msg.fields = [
            PointField(name=n,
                       offset=i * itemsize,
                       datatype=ros_dtype,
                       count=1) for i, n in enumerate('xyzrgb')
        ]
        self.point_publisher.publish(msg)


if __name__ == '__main__':
    configInternal = {
        "computeStereoPointCloud": "true",
        "pointCloudNormalsEnabled": "true",
        "computeDenseStereoDepth": "true",
    }
    configInternal["useRectification"] = "true"

    slam_node = SLAMNode()

    def onVioOutput(vioOutput):
        slam_node.newOdometryFrame(vioOutput)

    def onMappingOutput(output):
        for frame_id in output.updatedKeyFrames:
            keyFrame = output.map.keyFrames.get(frame_id)

            # Remove deleted key frames from visualisation
            if not keyFrame:
                continue

            # Check that point cloud exists
            if not keyFrame.pointCloud: continue

            if not slam_node.has_keyframe(frame_id):
                pass
                # slam_node.newKeyFrame(frame_id, keyFrame)

        if output.finalMap:
            print("Final map ready!")

    print("Starting OAK-D device")
    pipeline = depthai.Pipeline()
    config = spectacularAI.depthai.Configuration()
    config.internalParameters = configInternal
    config.useSlam = True
    #config.useColor = True

    vioPipeline = spectacularAI.depthai.Pipeline(pipeline, config,
                                                 onMappingOutput)

    with depthai.Device(pipeline) as device, \
        vioPipeline.startSession(device) as vio_session:
        while not rospy.is_shutdown():
            onVioOutput(vio_session.waitForOutput())
