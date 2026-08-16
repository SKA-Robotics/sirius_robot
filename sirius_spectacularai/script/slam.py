#!/usr/bin/python3

import spectacularAI
import depthai
import rospy
import tf2_ros
from script.pointcloud_processor import PointCloudProcessor
from script.odometry_processor import OdometryProcessor
from script.gps_handler import GPSHandler
from dataclasses import dataclass
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2, CameraInfo, Image, NavSatFix

from message_constructors import to_camera_info_message


@dataclass
class Coordinates:
    altitude: float
    longitude: float
    latitude: float


class SLAMNode:
    def __init__(self):
        rospy.init_node("slam_node", anonymous=True)
        self._camera_id = rospy.get_param("~camera_id", "19443010114A722700")

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
        self._tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.bridge = CvBridge()
        self.keyframes = {}

        self.session = None
        self.device = None

        self.start_time = rospy.Time.now()

        alt = rospy.get_param('~altitude', 148.824)
        long = rospy.get_param('~longitude', 21.010336799999997)
        lat = rospy.get_param('~latitude', 52.2198959)
        self._coordinates = Coordinates(alt, long, lat)

        odom_config = {
            'frame_id': rospy.get_param('~frame_id', 'map'),
            'odom_frame': rospy.get_param("~odom_frame", 'base_link'),
            'manip_mount': rospy.get_param('~manip_mount', False),
            'manip_offset': rospy.get_param('~manip_offset',
                                            [0.037, 0.0849, 0.24])
        }
        self._odometry_processor = OdometryProcessor(self._coordinates,
                                                     self._message_config,
                                                     self.tf_buffer,
                                                     odom_config)

        self._imu_to_gnss_offset = rospy.get_param('~imu_to_gnss_offset',
                                                   [0, -0.93, -0.71])
        margins = {
            'margin_left_px': rospy.get_param("~point_cloud_margin_left_px", 10),
            'margin_right_px': rospy.get_param("~point_cloud_margin_right_px", 10),
            'margin_top_px': rospy.get_param("~point_cloud_margin_top_px", 10),
            'margin_bottom_px': rospy.get_param("~point_cloud_margin_bottom_px", 10)
        }
        pointcloud_frame = rospy.get_param('~pointcloud_frame',
                                           'slam')
        self._pc_processor = PointCloudProcessor(pointcloud_frame,
                                                 margins)
        self._gps_handler = GPSHandler()

    @property
    def camera_id(self):
        return self._camera_id

    @property
    def imu_to_gnss_offset(self):
        return self._imu_to_gnss_offset

    def gps_fix_callback(self, msg: NavSatFix):
        self._gps_handler.gps_fix_callback(msg, self.device,
                                           self.session)

    def has_keyframe(self, frame_id):
        return frame_id in self.keyframes

    def new_key_frame(self, frame_id, keyframe):
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

        msg = self._pc_processor.new_point_cloud(keyframe)
        self.point_publisher.publish(msg)

        depth_frame = keyframe.frameSet.getAlignedDepthFrame(
            keyframe.frameSet.rgbFrame)
        depth = depth_frame.image.toArray()
        depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding='mono16')
        depth_msg.header.stamp = now
        depth_msg.header.frame_id = self._header_frame_id
        depth_msg.header.seq = sequence_number
        self.depth_publisher.publish(depth_msg)

    def new_odometry_frame(self, vioOutput):
        msgs = self._odometry_processor.new_odometry_frame(vioOutput)

        t = msgs['tf']
        self._tf_broadcaster.sendTransform(t)

        self.odometry_publisher.publish(msgs['odometry'])

        if msgs['global_odometry'] is not None:
            self.global_odometry_publisher.publish(msgs['global_odometry'])


if __name__ == '__main__':
    infos = depthai.DeviceBootloader.getAllAvailableDevices()

    for info in infos:
        state = str(info.state).split('X_LINK_')[1]

        rospy.loginfo(
            f"Found device '{info.name}', MxId: '{info.mxid}', State: '{state}'"
        )

    configInternal = {
        "computeStereoPointCloud": "true",
        "pointCloudNormalsEnabled": "true",
        "computeDenseStereoDepth": "true",
        "useRectification": "true"
    }

    slam_node = SLAMNode()

    def on_vio_output(vioOutput):
        slam_node.new_odometry_frame(vioOutput)

    def on_mapping_output(output):
        for frame_id in output.updatedKeyFrames:
            keyFrame = output.map.keyFrames.get(frame_id)

            # Remove deleted key frames from visualisation
            if not keyFrame:
                continue

            # Check that point cloud exists
            if not keyFrame.pointCloud: continue

            if not slam_node.has_keyframe(frame_id):
                slam_node.new_key_frame(frame_id, keyFrame)

        if output.finalMap:
            rospy.loginfo("Final map ready!")

    rospy.loginfo("Starting OAK-D device")
    pipeline = depthai.Pipeline()
    config = spectacularAI.depthai.Configuration()
    config.internalParameters = configInternal
    config.useSlam = True
    # config.useColor = True
    config.imuToGnss = spectacularAI.Vector3d(slam_node.imu_to_gnss_offset[0],
                                              slam_node.imu_to_gnss_offset[1],
                                              slam_node.imu_to_gnss_offset[2])
    vioPipeline = spectacularAI.depthai.Pipeline(pipeline, config,
                                                 on_mapping_output)

    with depthai.Device(
            pipeline, deviceInfo=depthai.DeviceInfo(slam_node.camera_id)
    ) as device, vioPipeline.startSession(device) as vio_session:
        slam_node.device = device
        slam_node.session = vio_session
        while not rospy.is_shutdown():
            on_vio_output(vio_session.waitForOutput())
