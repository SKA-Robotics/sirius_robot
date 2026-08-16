import PyKDL
from message_constructors import to_odometry_message
from transforms import transform_odometry_child_frame
from geometry_msgs.msg import TransformStamped


class OdometryProcessor:
    def __init__(self, coordinates, message_config, tf_buffer, odom_config):
        self._coordinates = coordinates
        self._message_config = message_config
        self.tf_buffer = tf_buffer

        self._frame_id = odom_config['frame_id']
        self._odom_frame = odom_config['odom_frame']
        self._manip_mount = odom_config['manip_mount']
        self._manip_offset = odom_config['manip_offset']

    def new_odometry_frame(self, vioOutput):
        msg = to_odometry_message(vioOutput, self._coordinates,
                                  self._message_config,
                                  is_global=False)
        if not self._manip_mount:
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
        msgs = {
            'odometry': msg
            }

        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = self._odom_frame
        t.transform.rotation = msg.pose.pose.orientation
        t.transform.translation = msg.pose.pose.position
        msgs['tf'] = t

        msg = to_odometry_message(vioOutput, self._coordinates,
                                  self._message_config,
                                  is_global=True)
        if msg is not None:
            msg = transform_odometry_child_frame(msg, self._odom_frame,
                                                 self.tf_buffer)
            msg.header.frame_id = self._frame_id
        msgs['global_odometry'] = msg
        return msgs
