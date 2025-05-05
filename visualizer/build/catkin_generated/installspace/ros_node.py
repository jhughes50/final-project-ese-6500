import rospy

from visualizer.map_node import MapNode

if __name__ == "__main__":
    rospy.init_node("gps_tracker")

    MapNode()

    rospy.spin()
