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
        odom_offset = rospy.get_param("/visualizer/map/offset")

        self.initial_gps_ = (0.0, 0.0)
        self.prev_gps_ = (0.0, 0.0)
        self.initial_utm_ = (0.0, 0.0)

        self.heading_ = np.radians(-odom_offset)
        self.initialized_ = False
        self.odom_initialized_ = False
        self.app_ = MapApp(path, ip = ip, port = port)
        self.app_.run_in_thread()
        self.imu_orientation = (0, 0, 0, 1)

        self.glat_ = None
        self.glon_ = None
        self.initial_x =0.0
        self.initial_y =0.0
        self.timer_ = rospy.Timer(rospy.Duration(0.2), self.glins_update)
        if rospy.get_param("/visualizer/gps"):
            rospy.Subscriber("/gps", NavSatFix, self.gps_callback)
        if rospy.get_param("/visualizer/odom"):
            rospy.Subscriber("/odom", Odometry, self.odom_callback)
        if rospy.get_param("/visualizer/glins"):
            rospy.Subscriber("/glins", Odometry, self.glins_callback)
        
        rospy.Subscriber("/imu/data", Imu, self.imu_callback)
    


    def glins_callback(self, msg : Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        #print(x, y)
        try:
            self.glat_, self.glon_ = utm.to_latlon(x, y, self.zone_num_, self.zone_id_)
            #self.app_.update_glins(lat, lon)
        except utm.OutOfRangeError:
            print(x,y)

    def glins_update(self, event) -> None:
        if self.glat_ is not None and self.glon_ is not None:
            self.app_.update_glins(self.glat_, self.glon_)

    def gps_callback(self, msg : NavSatFix) -> None:
        lat = msg.latitude
        lon = msg.longitude
        if not self.initialized_:
            self.initial_gps_ = (lat, lon)
            self.prev_gps_ = (lat, lon)
            self.initial_utm_ = utm.from_latlon(lat, lon)[:2]
            self.initialized_ = True
            print("Using heading: ", (np.degrees(self.heading_) + 360) % 360)
        self.app_.update_gps(lat, lon, popup=f"GPS: {lat:.2f}, {lon:.2f}")

    def odom_callback(self, msg : Odometry) -> None:
        if not self.odom_initialized_:
            self.initial_x = msg.pose.pose.position.x
            self.initial_y = msg.pose.pose.position.x
            self.odom_initialized_ = True
            return
        x = msg.pose.pose.position.x + self.initial_x
        y = msg.pose.pose.position.y + self.initial_y
        
        if self.initialized_: 
            rot = np.array([[np.cos(self.heading_), -np.sin(self.heading_)], [np.sin(self.heading_), np.cos(self.heading_)]])
            rotated = rot @ np.array([x,y])
            x = self.initial_utm_[0] - rotated[1]
            y = self.initial_utm_[1] + rotated[0]
     
            lat, lon = utm.to_latlon(x, y, self.zone_num_, self.zone_id_)
            self.app_.update_odom(lat, lon, popup=f"ODOM: {lat:.2f}, {lon:.2f}")
        
    def imu_callback(self, msg : Imu) -> None:
        self.imu_orientation = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
