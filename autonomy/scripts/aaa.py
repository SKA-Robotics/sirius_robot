from scipy.spatial.transform import Rotation
from math import sin, cos

r = Rotation.from_quat(self.robot_pose)
euler = r.as_euler('zyx')
yaw = euler[0]

heading_vector = np.array([cos(yaw), sin(yaw)])
