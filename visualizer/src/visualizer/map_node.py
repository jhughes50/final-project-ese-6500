"""
    Jason Hughes
    March 2025

    Plot things on a map
"""
import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import utm

from scipy.spatial.transform import Rotation
import numpy as np
from visualizer.map_app import MapApp

class MapNode:

    def __init__(self) -> None:

        ip = rospy.get_param("/visualizer/app/ip_address")
        port = rospy.get_param("/visualizer/app/port")
        
        path = rospy.get_param("/visualizer/path")

        self.zone_num_ = rospy.get_param("/visualizer/map/zone_number")
        self.zone_id_ = rospy.get_param("/visualizer/map/zone_id")
        # print(self.zone_num_, self.zone_id_)

        rospy.Subscriber("/gps", NavSatFix, self.gps_callback)
        rospy.Subscriber("/Odometry", Odometry, self.odom_callback)
        rospy.Subscriber("/imu/data", String, self.imu_callback)

        self.app_ = MapApp(path, ip = ip, port = port)
        self.app_.run_in_thread()
        self.imu_orientation = (0, 0, 0, 1)

    def gps_callback(self, msg : NavSatFix) -> None:
        lat = msg.latitude
        lon = msg.longitude
        # print(f"gpslat: {lat}, lon: {lon}")
        self.app_.update_map(lat,lon, source="gps", popup=f"GPS: {lat}, {lon}")

    def odom_callback(self, msg : Odometry) -> None:
        # convert gps to utm
        gps_x, gps_y = 39.9418626, -75.1991958999
        # convert to utm
        utm_x, utm_y, _, _ = utm.from_latlon(gps_x, gps_y)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # print(x, y)
        # quaternion = (
        #     msg.pose.pose.orientation.x,
        #     msg.pose.pose.orientation.y,
        #     msg.pose.pose.orientation.z,
        #     msg.pose.pose.orientation.w
        # )
        quaternion = self.imu_orientation
        rot = Rotation.from_quat(quaternion)
        rotation_matrix = rot.as_matrix()

        # rotate the point
        rotated_point = rotation_matrix[:2, :2] @ np.array([x, y])
        # print("rotated_point", rotated_point)
        # print("utm_x, utm_y", utm_x, utm_y)
        x = utm_x - rotated_point[0]
        y = utm_y - rotated_point[1]
        # print(x, y)

        lat, lon = utm.to_latlon(x, y, self.zone_num_, self.zone_id_)
        # print(f"odmlat: {lat}, lon: {lon}")
        self.app_.update_map(lat, lon, source='odom', popup=f"ODOM: {lat}, {lon}")

    def imu_callback(self, msg : String) -> None:
        self.imu_orientation = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
