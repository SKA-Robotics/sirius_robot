import spectacularAI
import rospy
from sensor_msgs.msg import NavSatFix


class GPSHandler:
    def __init__(self):
        pass

    def compute_gps_time_offset(self, device):
        imu_queue = device.getOutputQueue(name='spectacularAI_imu',
                                               maxSize=1,
                                               blocking=True)
        imu_data = imu_queue.get()
        acc = imu_data.packets[0].acceleroMeter
        ts_device = acc.getTimestampDevice().total_seconds()

        return ts_device - 0.5

    def gps_fix_callback(self, msg: NavSatFix, device,
                         session):
        if session is not None and device is not None:
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

            time_offset = self.compute_gps_time_offset(device)
            rospy.loginfo(
                f'{time_offset} {coordinates.latitude} {coordinates.longitude}'
            )
            session.addGnss(time_offset, coordinates,
                                 position_covariance)
