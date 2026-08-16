#!/usr/bin/python3

import spectacularAI
import depthai
import rospy
import numpy as np
import tf2_ros
import PyKDL
from dataclasses import dataclass
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2, PointField, CameraInfo, Image, NavSatFix
from geometry_msgs.msg import TransformStamped

from message_constructors import to_camera_info_message, to_odometry_message, to_pose_message
from transforms import transform_odometry_child_frame


# print(camera_id)
# print(pointcloud_frame)
# print(odom_frame)
# print(camera_id)
# print(f"Point cloud edge margins (px) - left: {margin_left_px}, right: {margin_right_px}, "
#       f"top: {margin_top_px}, bottom: {margin_bottom_px}")

@dataclass
class Coordinates:
    altitude: float
    longitude: float
    latitude: float


class SLAMNode:
    def __init__(self):
        rospy.init_node("slam_node", anonymous=True)

        self._manipMount = rospy.get_param("~manip_mount", False)

        self._camera_id = rospy.get_param("~camera_id", "19443010114A722700")
        self._pointcloud_frame = rospy.get_param("~pointcloud_frame", "slam")
        self._odom_frame = rospy.get_param("~odom_frame", "base_link")

        topic_prefix = rospy.get_param("~topic_prefix", "/slam")
        self._header_frame_id = rospy.get_param('~header_frame_id',
                                                'front_rgb_camera_optical')

        self._message_config = {
            'frame_id': self._header_frame_id,
            'world_frame': rospy.get_param('~world_frame', 'odom'),
            'marsyard_angle': rospy.get_param('~marsyard_angle', 0)
        }

        self._camera_info_frame_id = rospy.get_param('~frame_id',
                                                     'front_rgb_camera_optical')

        self.odometry_publisher = rospy.Publisher(topic_prefix + "/odometry",
                                                  Odometry,
                                                  queue_size=10)
        self.global_odometry_publisher = rospy.Publisher(topic_prefix +
                                                         "/global_odometry",
                                                         Odometry,
                                                         queue_size=10)
        self.rgb_publisher = rospy.Publisher(topic_prefix + "/rgb",
                                             Image,
                                             queue_size=10)
        self.point_publisher = rospy.Publisher(topic_prefix + "/pointcloud",
                                               PointCloud2,
                                               queue_size=10)
        self.depth_publisher = rospy.Publisher(topic_prefix + "/depth",
                                               Image,
                                               queue_size=10)
        self.camera_info_publisher = rospy.Publisher(topic_prefix +
                                                     "/camera_info",
                                                     CameraInfo,
                                                     queue_size=10)
        self.gps_subscriber = rospy.Subscriber('/gps/fix', NavSatFix,
                                               self.gps_fix_callback)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.bridge = CvBridge()
        self.keyframes = {}

        self.session = None
        self.device = None

        self.start_time = rospy.Time.now()
        self._frame_id = rospy.get_param('~frame_id', 'map')

        alt = rospy.get_param('~altitude', 148.824)
        long = rospy.get_param('~longitude', 21.010336799999997)
        lat = rospy.get_param('~latitude', 52.2198959)
        self._coordinates = Coordinates(alt, long, lat)

        self._manip_offset = rospy.get_param('~manip_offset', [0.037, 0.0849, 0.24])
        self._imu_to_gnss_offset = rospy.get_param('~imu_to_gnss_offset',
                                                   [0, -0.93, -0.71])

        self.margin_left_px = rospy.get_param("~point_cloud_margin_left_px", 10)
        self.margin_right_px = rospy.get_param("~point_cloud_margin_right_px", 10)
        self.margin_top_px = rospy.get_param("~point_cloud_margin_top_px", 10)
        self.margin_bottom_px = rospy.get_param("~point_cloud_margin_bottom_px", 10)

    @property
    def camera_id(self):
        return self._camera_id

    @property
    def imu_to_gnss_offset(self):
        return self._imu_to_gnss_offset

    def computeGPSTimeOffset(self):
        imu_queue = self.device.getOutputQueue(name="spectacularAI_imu",
                                               maxSize=1,
                                               blocking=True)
        imu_data = imu_queue.get()
        acc = imu_data.packets[0].acceleroMeter
        ts_device = acc.getTimestampDevice().total_seconds()

        return ts_device - 0.5

    def gps_fix_callback(self, msg: NavSatFix):
        if self.session is not None:
            position_covariance = [
                [a for a in msg.position_covariance[0:3]],
                [a for a in msg.position_covariance[3:6]],
                [a for a in msg.position_covariance[6:9]],
            ]
            """
            position_covariance = [
                [1, 0, 0],
                [0, 1, 0],
                [0, 0, 5],
            ]
            """

            coordinates = spectacularAI.WgsCoordinates()
            coordinates.altitude = msg.altitude
            coordinates.latitude = msg.latitude
            coordinates.longitude = msg.longitude
            rospy.loginfo(
                f"{self.computeGPSTimeOffset()} {coordinates.latitude} {coordinates.longitude}"
            )
            self.session.addGnss(self.computeGPSTimeOffset(), coordinates,
                                 position_covariance)

    def has_keyframe(self, frame_id):
        return frame_id in self.keyframes

    def newKeyFrame(self, frame_id, keyframe):
        now = rospy.Time.now()
        self.keyframes[frame_id] = True
        sequence_number = int(frame_id)

        rgb_frame = keyframe.frameSet.getUndistortedFrame(
            keyframe.frameSet.rgbFrame).image
        rgb_frame = rgb_frame.toArray()
        rgb_message = self.bridge.cv2_to_imgmsg(rgb_frame, encoding="rgb8")
        rgb_message.header.stamp = now

        rgb_message.header.frame_id = self._header_frame_id
        rgb_message.header.seq = sequence_number
        self.rgb_publisher.publish(rgb_message)

        camera = keyframe.frameSet.rgbFrame.cameraPose.camera
        info_msg = to_camera_info_message(camera, rgb_frame,
                                          self._message_config['frame_id'],
                                          now)
        self.camera_info_publisher.publish(info_msg)

        self.newPointCloud(keyframe)

        depth_frame = keyframe.frameSet.getAlignedDepthFrame(
            keyframe.frameSet.rgbFrame)
        depth = depth_frame.image.toArray()
        depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding='mono16')
        depth_msg.header.stamp = now
        depth_msg.header.frame_id = self._header_frame_id
        depth_msg.header.seq = sequence_number
        self.depth_publisher.publish(depth_msg)

    def newOdometryFrame(self, vioOutput):
        msg = to_odometry_message(vioOutput, self._coordinates,
                                  self._message_config,
                                  is_global=False)
        if not self._manipMount:
            msg = transform_odometry_child_frame(msg, self._odom_frame,
                                                 self.tf_buffer)
        else:
            pose = msg.pose.pose
            frame = PyKDL.Frame(
                PyKDL.Rotation.Quaternion(pose.orientation.x,
                                          pose.orientation.y,
                                          pose.orientation.z,
                                          pose.orientation.w),
                PyKDL.Vector(pose.position.x, pose.position.y,
                             pose.position.z))

            frame *= PyKDL.Frame(PyKDL.Rotation(),
                                 PyKDL.Vector(self._manip_offset[0],
                                              self._manip_offset[1],
                                              self._manip_offset[2]))
            msg.pose.pose.position.x = frame.p.x()
            msg.pose.pose.position.y = frame.p.y()
            msg.pose.pose.position.z = frame.p.z()

        msg.header.frame_id = self._frame_id
        self.odometry_publisher.publish(msg)

        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = self._odom_frame
        t.transform.rotation = msg.pose.pose.orientation
        t.transform.translation = msg.pose.pose.position
        br.sendTransform(t)
        msg = to_odometry_message(vioOutput, self._coordinates,
                                  self._message_config,
                                  is_global=True)
        if msg is not None:
            msg = transform_odometry_child_frame(msg, self._odom_frame,
                                                 self.tf_buffer)
            msg.header.frame_id = self._frame_id
            self.global_odometry_publisher.publish(msg)

    def computeEdgeMask(self, rgb_bitmap, camera, positions):
        width = rgb_bitmap.getWidth()
        height = rgb_bitmap.getHeight()

        z = positions[:, 2]
        in_front = z > 1e-6

        K = camera.getIntrinsicMatrix()
        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]

        u = np.full(positions.shape[0], -1.0)
        v = np.full(positions.shape[0], -1.0)
        u[in_front] = fx * positions[in_front, 0] / z[in_front] + cx
        v[in_front] = fy * positions[in_front, 1] / z[in_front] + cy

        return (
            in_front &
            (u > self.margin_left_px) &
            (u < width - self.margin_right_px) &
            (v > self.margin_top_px) &
            (v < height - self.margin_bottom_px)
        )

    def newPointCloud(self, keyframe):
        camToWorld = keyframe.frameSet.rgbFrame.cameraPose.getCameraToWorldMatrix(
        )
        camera = keyframe.frameSet.rgbFrame.cameraPose.camera
        rgb_bitmap = keyframe.frameSet.getUndistortedFrame(
            keyframe.frameSet.rgbFrame).image

        positions = keyframe.pointCloud.getPositionData()

        mask = self.computeEdgeMask(rgb_bitmap, camera, positions)
        positions = positions[mask]

        pc = np.zeros((positions.shape[0], 6), dtype=np.float32)
        p_C = np.vstack((positions.T, np.ones((1, positions.shape[0])))).T
        pc[:, :3] = (camToWorld @ p_C[:, :, None])[:, :3, 0]

        msg = PointCloud2()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self._pointcloud_frame
        if keyframe.pointCloud.hasColors():
            colors = keyframe.pointCloud.getRGB24Data()
            colors = colors[mask]
            pc[:, 3:] = colors * (1. / 255.)
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
    infos = depthai.DeviceBootloader.getAllAvailableDevices()

    for info in infos:
        state = str(info.state).split('X_LINK_')[1]

        print(
            f"Found device '{info.name}', MxId: '{info.mxid}', State: '{state}'"
        )

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
                slam_node.newKeyFrame(frame_id, keyFrame)

        if output.finalMap:
            print("Final map ready!")

    print("Starting OAK-D device")
    pipeline = depthai.Pipeline()
    config = spectacularAI.depthai.Configuration()
    config.internalParameters = configInternal
    config.useSlam = True
    # config.useColor = True
    config.imuToGnss = spectacularAI.Vector3d(slam_node.imu_to_gnss_offset[0],
                                              slam_node.imu_to_gnss_offset[1],
                                              slam_node.imu_to_gnss_offset[2])
    vioPipeline = spectacularAI.depthai.Pipeline(pipeline, config,
                                                 onMappingOutput)

    with depthai.Device(
            pipeline, deviceInfo=depthai.DeviceInfo(slam_node.camera_id)
    ) as device, vioPipeline.startSession(device) as vio_session:
        """
        vio_session.addAbsolutePose(
            spectacularAI.Pose.fromMatrix(1.0, [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]), 0)
        """
        slam_node.device = device
        slam_node.session = vio_session
        while not rospy.is_shutdown():
            onVioOutput(vio_session.waitForOutput())
