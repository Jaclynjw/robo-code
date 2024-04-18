#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math

class NavPath(object):
    def __init__(self):
        self.node_name = "nav_path_visualization"
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        self._path = []
        self.threshold_distance = 0.5  # Distance threshold to add a new point, in meters.
        self.last_point = None

    def callback(self, msg):
        # Extract position data from Odometry message
        position = msg.pose.pose.position
        new_point = Point(position.x, position.y, position.z)

        # Only add the new point if it's sufficiently far from the last point
        if self.last_point is None or self.distance(self.last_point, new_point) > self.threshold_distance:
            rospy.loginfo(f"Adding new point to path: {new_point}")
            self._path.append(new_point)
            self.last_point = new_point
            self.publish_markers()

    def distance(self, point1, point2):
        return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2 + (point1.z - point2.z)**2)

    def publish_markers(self):
        # Create and publish the marker
        marker = Marker()
        marker.header.frame_id = "/odom"
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        marker.points = self._path
        marker.scale.x = 0.05  # Line width
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Alpha
        marker.lifetime = rospy.Duration()  # 0 means infinite
        self.marker_pub.publish(marker)

def main():
    rospy.init_node('nav_path_visualization')
    nav_path = NavPath()
    rospy.Subscriber('odom', Odometry, nav_path.callback)
    rospy.spin()

if __name__ == '__main__':
    main()


