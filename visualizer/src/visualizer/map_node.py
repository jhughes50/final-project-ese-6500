"""
    Jason Hughes
    March 2025

    Plot things on a map
"""
import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from visualizer.map_app import MapApp

class MapNode:

    def __init__(self) -> None:

        ip = rospy.get_param("/visualizer/app/ip_address")
        port = rospy.get_param("/visualizer/app/port")
        
        path = rospy.get_param("/visualizer/path")

        self.zone_num_ = rospy.get_param("/visualizer/map/zone_number")
        self.zone_id_ = rospy.get_param("/visualizer/map/zone_id")

        rospy.Subscriber("/gps", NavSatFix, self.gps_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        self.app_ = MapApp(path, ip = ip, port = port)
        self.app_.run_in_thread()

    def gps_callback(self, msg : NavSatFix) -> None:
        lat = msg.latitude
        lon = msg.longitude

        self.app_.update_map(lat,lon)

    def odom_callback(self, msg : Odometry) -> None:
        lat, lon = utm.to_latlon(msg.pose.pose.position.x, msg.pose.pose.position.y, self.zone_num_, self.zone_id_)
        self.app_.update_map(lat, lon)


