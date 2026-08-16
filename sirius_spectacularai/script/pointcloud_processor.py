import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField


class PointCloudProcessor:
    def __init__(self, pointcloud_frame, margins):
        self._pointcloud_frame = pointcloud_frame
        self._margins = margins

    def compute_edge_mask(self, rgb_bitmap, camera, positions):
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
            (u > self._margins['margin_left_px']) &
            (u < width - self._margins['margin_right_px']) &
            (v > self._margins['margin_top_px']) &
            (v < height - self._margins['margin_bottom_px'])
        )

    def new_point_cloud(self, keyframe):
        cam_to_world = keyframe.frameSet.rgbFrame.cameraPose.getCameraToWorldMatrix(
        )
        camera = keyframe.frameSet.rgbFrame.cameraPose.camera
        rgb_bitmap = keyframe.frameSet.getUndistortedFrame(
            keyframe.frameSet.rgbFrame).image

        positions = keyframe.pointCloud.getPositionData()

        mask = self.compute_edge_mask(rgb_bitmap, camera, positions)
        positions = positions[mask]

        pc = np.zeros((positions.shape[0], 6), dtype=np.float32)
        p_C = np.vstack((positions.T, np.ones((1, positions.shape[0])))).T
        pc[:, :3] = (cam_to_world @ p_C[:, :, None])[:, :3, 0]

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
        return msg
