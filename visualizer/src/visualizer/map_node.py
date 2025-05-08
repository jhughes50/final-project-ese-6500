"""
    Jason Hughes
    March 2025

    Plot things on a map
"""
import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import utm

from scipy.spatial.transform import Rotation
import numpy as np
from visualizer.map_app import MapApp
from visualizer.heading import differential_heading

class MapNode:

    def __init__(self) -> None:

        ip = rospy.get_param("/visualizer/app/ip_address")
        port = rospy.get_param("/visualizer/app/port")
        
        path = rospy.get_param("/visualizer/path")

        self.zone_num_ = rospy.get_param("/visualizer/map/zone_number")
        self.zone_id_ = rospy.get_param("/visualizer/map/zone_id")
        self.declination_ = rospy.get_param("/visualizer/map/declination")

        self.initial_gps_ = (0.0, 0.0)
        self.prev_gps_ = (0.0, 0.0)
        self.initial_utm_ = (0.0, 0.0)

        self.heading_ = None
        self.headings_ = []
        self.initialized_ = False
        self.heading_initialized_ = False 
        self.heading_counter_ = 0

        rospy.Subscriber("/gps", NavSatFix, self.gps_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/imu/data", Imu, self.imu_callback)

        self.app_ = MapApp(path, ip = ip, port = port)
        self.app_.run_in_thread()
        self.imu_orientation = (0, 0, 0, 1)
        self.last_imu_orientation = (0, 0, 0, 0)
        self.imu_in = False

    def gps_callback(self, msg : NavSatFix) -> None:
        lat = msg.latitude
        lon = msg.longitude
        if not self.initialized_:
            self.initial_gps_ = (lat, lon)
            self.prev_gps_ = (lat, lon)
            self.initial_utm_ = utm.from_latlon(lat, lon)[:2]
            self.initialized_ = True
        elif self.heading_counter_ < 20:
            heading = differential_heading(self.prev_gps_[0], self.prev_gps_[1], lat, lon)
            self.headings_.append(heading)
            self.prev_gps_ = (lat, lon)
            self.heading_counter_ += 1
        elif self.heading_counter_ == 20:
            self.heading_ = sum(self.headings_) / len(self.headings_)
            self.heading_ = ((np.degrees(self.heading_)+360) % 360) - self.declination_
            self.heading_ = np.radians(self.heading_)
            # self.heading_ = np.radians(-192.69) # UNCOMMENT FOR GRASS DATA
            self.heading_ = np.radians(-285) # UNCOMMENT FOR BACK DATA
            self.heading_counter_ += 1
            self.heading_initialized_ = True
            print("Using heading: ", (np.degrees(self.heading_) + 360) % 360)
        self.app_.update_gps(lat, lon, popup=f"GPS: {lat:.2f}, {lon:.2f}")

    def odom_callback(self, msg : Odometry) -> None:
        x_pose = msg.pose.pose.position.x
        y_pose = msg.pose.pose.position.y
        
        if self.heading_initialized_:
            rot = np.array([[np.cos(self.heading_), -np.sin(self.heading_)], [np.sin(self.heading_), np.cos(self.heading_)]])
            rotated = rot @ np.array([x_pose,y_pose])
            print("rotated from heading:", rotated)
            x = self.initial_utm_[0] - rotated[1]
            y = self.initial_utm_[1] + rotated[0]

     
            lat, lon = utm.to_latlon(x, y, self.zone_num_, self.zone_id_)
            self.app_.update_odom(lat, lon, popup=f"ODOM: {lat:.2f}, {lon:.2f}")
        
        
        if self.imu_in:
            q_ned = np.array(self.imu_orientation)
            q_ned = q_ned / np.linalg.norm(q_ned)
            q_enu = np.array([ q_ned[1], q_ned[0], -q_ned[2], q_ned[3] ])  # x↔y, z↔-z
            imu_rot_enu = Rotation.from_quat(q_enu)

            # Extract yaw angle (rotation about Z-axis)
            _, _, yaw = imu_rot_enu.as_euler('zyx')
            # yaw += np.pi/2 # UNCOMMENT FOR GRASS DATA
            yaw -+ np.pi/2
            # Rotate the point in ENU
            # 2x2 rotation matrix from yaw
            rot = np.array([   
                [np.cos(yaw), -np.sin(yaw)],
                [np.sin(yaw),  np.cos(yaw)]
            ])
            rotated = rot @ np.array([x_pose, y_pose])
            
            print("rotated from imu:", rotated)
            x = self.initial_utm_[0] - rotated[0]
            y = self.initial_utm_[1] - rotated[1]

            lat, lon = utm.to_latlon(x, y, self.zone_num_, self.zone_id_)
            self.app_.update_odom_w_imu(lat, lon, popup=f"ODOM2: {lat:.2f}, {lon:.2f}")
            self.imu_in = False

        
    def imu_callback(self, msg : Imu) -> None:
        self.imu_in = True
        self.imu_orientation = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)