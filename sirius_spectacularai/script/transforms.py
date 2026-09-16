import PyKDL
import tf2_geometry_msgs
from geometry_msgs.msg import Vector3, Twist, Pose, PoseWithCovariance, TwistWithCovariance
from nav_msgs.msg import Odometry


def transform_odometry_child_frame(msg: Odometry, target_frame: str,
                                   tf_buffer) -> Odometry:
    target_pose = _transform_pose_child_frame(
        msg.pose, target_frame, msg.child_frame_id, tf_buffer, msg.header.stamp)
    target_twist = _transform_twist_child_frame(
        msg.twist, target_frame, msg.child_frame_id, tf_buffer, msg.header.stamp)

    out = Odometry()
    out.header = msg.header
    out.child_frame_id = target_frame
    out.pose = target_pose
    out.twist = target_twist
    return out


def _transform_pose_child_frame(pose: PoseWithCovariance, target_frame: str,
                                child_frame: str, tf_buffer,
                                stamp) -> PoseWithCovariance:
    covariance = pose.covariance
    p = pose.pose

    child_to_parent = PyKDL.Frame(
        PyKDL.Rotation.Quaternion(p.orientation.x, p.orientation.y,
                                  p.orientation.z, p.orientation.w),
        PyKDL.Vector(p.position.x, p.position.y, p.position.z))

    target_to_child = tf2_geometry_msgs.transform_to_kdl(
        tf_buffer.lookup_transform(child_frame, target_frame, stamp))

    combined = child_to_parent * target_to_child

    target_pose = Pose()
    target_pose.position.x = combined[(0, 3)]
    target_pose.position.y = combined[(1, 3)]
    target_pose.position.z = combined[(2, 3)]
    q = combined.M.GetQuaternion()
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]

    max_pos = max(covariance[0], covariance[7], covariance[14])
    max_rot = max(covariance[21], covariance[28], covariance[35])
    result = PoseWithCovariance()
    result.pose = target_pose
    result.covariance = [0.0] * 36
    result.covariance[0] = result.covariance[7] = result.covariance[14] = max_pos
    result.covariance[21] = result.covariance[28] = result.covariance[35] = max_rot
    return result


def _transform_twist(twist: TwistWithCovariance,
                     transform) -> TwistWithCovariance:
    covariance = twist.covariance
    t = twist.twist

    lin = PyKDL.Vector(t.linear.x, t.linear.y, t.linear.z)
    ang = PyKDL.Vector(t.angular.x, t.angular.y, t.angular.z)
    translation = PyKDL.Vector(transform[0, 3], transform[1, 3],
                                transform[2, 3])
    rotation = PyKDL.Rotation(
        transform[0, 0], transform[1, 0], transform[2, 0],
        transform[0, 1], transform[1, 1], transform[2, 1],
        transform[0, 2], transform[1, 2], transform[2, 2])

    target_lin = rotation * lin + ang * translation
    target_ang = rotation * ang

    target_twist = Twist(
        Vector3(x=target_lin[0], y=target_lin[1], z=target_lin[2]),
        Vector3(x=target_ang[0], y=target_ang[1], z=target_ang[2]))

    max_lin = max(covariance[0], covariance[7], covariance[14])
    max_ang = max(covariance[21], covariance[28], covariance[35])
    result = TwistWithCovariance()
    result.twist = target_twist
    result.covariance = [0.0] * 36
    result.covariance[0] = result.covariance[7] = result.covariance[14] = max_lin
    result.covariance[21] = result.covariance[28] = result.covariance[35] = max_ang
    return result


def _transform_twist_child_frame(twist: TwistWithCovariance, target_frame: str,
                                 child_frame: str, tf_buffer,
                                 stamp) -> TwistWithCovariance:
    child_to_target = tf2_geometry_msgs.transform_to_kdl(
        tf_buffer.lookup_transform(target_frame, child_frame, stamp))
    return _transform_twist(twist, child_to_target)
