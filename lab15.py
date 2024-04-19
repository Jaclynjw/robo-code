#!/usr/bin/env python

import rospy
import copy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class Base(object):
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('robot_base_controller')
        self._odom_sub = rospy.Subscriber('odom', Odometry, callback=self._odom_callback)
        self._vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.current_odom = None

    def _odom_callback(self, msg):
        """Odometry callback function."""
        self.current_odom = msg

    def go_forward(self, distance, speed=0.1):
        """Moves the robot a certain distance."""
        rospy.loginfo("Starting to move forward")
        # Wait until the base has received at least one message on /odom
        while self.current_odom is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

        # Record start position using deepcopy to avoid referencing the same object
        start = copy.deepcopy(self.current_odom.pose.pose.position)
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            current_position = self.current_odom.pose.pose.position
            dx = current_position.x - start.x
            dy = current_position.y - start.y
            current_distance = math.sqrt(dx**2 + dy**2)  # Euclidean distance

            if current_distance >= abs(distance):
                break

            direction = math.copysign(1, distance)  # Positive for forward, negative for backward
            self.move(direction * speed, 0)
            rate.sleep()

        self.move(0, 0)  # Stop the robot after moving

    def turn(self, angular_distance, speed=0.5):
        """Rotates the robot a certain angle."""
        rospy.loginfo("Starting to turn")
        while self.current_odom is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

        start_yaw = self.get_yaw_from_odom(self.current_odom.pose.pose.orientation)
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            current_yaw = self.get_yaw_from_odom(self.current_odom.pose.pose.orientation)
            yaw_diff = (current_yaw - start_yaw + 2 * math.pi) % (2 * math.pi)

            if abs(yaw_diff) >= abs(angular_distance % (2 * math.pi)):
                break

            direction = math.copysign(1, angular_distance)  # Positive for counterclockwise, negative for clockwise
            self.move(0, direction * speed)
            rate.sleep()

        self.move(0, 0)  # Stop the robot after rotation

    def move(self, linear_speed, angular_speed):
        """Send velocity commands to the robot."""
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        self._vel_pub.publish(twist)

    def get_yaw_from_odom(self, odom_orientation):
        """Extracts the yaw angle from an odometry message's orientation."""
        quaternion = (
            odom_orientation.x,
            odom_orientation.y,
            odom_orientation.z,
            odom_orientation.w
        )
        _, _, yaw = euler_from_quaternion(quaternion)
        return yaw

def euler_from_quaternion(quat):
    """Convert quaternion (x, y, z, w) to euler angles."""
    x, y, z, w = quat
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z

if __name__ == '__main__':
    rospy.spin()
