import rospy
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

class CovarianceTracker:
    def __init__(self):
        rospy.init_node('covariance_tracker', anonymous=True)
        
        # Initialize variables to store largest covariances
        self.max_odom_covariance = np.zeros((6, 6))
        self.max_navsatfix_covariance = np.zeros((9))
        
        # Create subscribers
        rospy.Subscriber('/Odometry', Odometry, self.odom_callback)
        rospy.Subscriber('/ublox/fix', NavSatFix, self.gps_callback)
        
        self.max_odom = 0.0
        self.max_gps = 0.0

    def odom_callback(self, msg):
        cmax = max(msg.pose.covariance)
        if cmax > self.max_odom:
            self.max_odom = cmax
            print("Max odom: ", self.max_odom)

    def gps_callback(self, msg):
        cmax = max(msg.position_covariance)
        if cmax > self.max_gps:
            self.max_gps = cmax
            print("Max gps: ", self.max_gps)

if __name__ == "__main__":
    try:
        tracker = CovarianceTracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
