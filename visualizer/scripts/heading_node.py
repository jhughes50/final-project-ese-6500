"""
    quick node to view headings 
""" 

import rospy
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Imu, NavSatFix
import numpy as np

class HeadingViz:

    def __init__(self) -> None:
        rospy.Subscriber("/ublox/fix", NavSatFix, self.gps_callback)
        #rospy.Subscriber("/imu/data", Imu, self.imu_callback)

        self.prev_latlon = (0.0, 0.0)
        self.initialized_ = False

    def gps_callback(self, msg : NavSatFix) -> None:
        if not self.initialized_:
            self.prev_latlon = (msg.latitude, msg.longitude)
            self.initialized_ = True
        else:
            lat2 = np.radians(msg.latitude)
            lon2 = np.radians(msg.longitude)
            lat1 = np.radians(self.prev_latlon[0])
            lon1 = np.radians(self.prev_latlon[1])

            lon_diff = lon2 - lon1

            y = np.sin(lon_diff) * np.cos(lat2)
            x = np.cos(lat1) * np.sin(lat2) - np.sin(lat1) * np.cos(lat2) * np.cos(lon_diff)

            heading_rad = np.arctan2(y, x)
            heading_deg = np.degrees(heading_rad)
            heading_nrm = (heading_deg + 360.0)  % 360

            self.prev_latlon_ = (msg.latitude, msg.longitude)
            print("GPS heading: ", heading_nrm)


    def imu_callback(self, msg : Imu) -> None:
        qmsg = msg.orientation
        q = Rotation.from_quat([qmsg.x, qmsg.y, qmsg.x, qmsg.w])
        print("IMU heading : ", q.as_euler('xyz', degrees=True))

if __name__ == "__main__":
    rospy.init_node("heading_viz")

    HeadingViz()

    rospy.spin()
