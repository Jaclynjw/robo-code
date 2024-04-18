#!/usr/bin/env python
import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist

class RobotController:
    def __init__(self):
        rospy.init_node('robot_control_markers')

        # Publisher for moving the robot
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # Create an interactive marker server on the topic namespace simple_marker
        self.server = InteractiveMarkerServer("simple_marker")

        # Create interactive markers
        self.create_interactive_markers()

        # Apply changes to the server
        self.server.applyChanges()

    def create_interactive_markers(self):
        # Forward movement marker
        forward_marker = InteractiveMarker()
        forward_marker.header.frame_id = "base_link"
        forward_marker.name = "move_forward"
        forward_marker.description = "Move Forward"
        forward_marker.pose.position.x = 1.0  # Position ahead of the robot base

        # Marker for moving forward
        box = Marker()
        box.type = Marker.CUBE
        box.scale.x = 0.2
        box.scale.y = 0.2
        box.scale.z = 0.2
        box.color.r = 0.0
        box.color.g = 1.0
        box.color.b = 0.0
        box.color.a = 1.0

        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.markers.append(box)
        box_control.interaction_mode = InteractiveMarkerControl.BUTTON
        forward_marker.controls.append(box_control)

        # Rotation markers
        self.create_rotation_marker("turn_left", 0.5, 0.0, "Rotate Left", True)
        self.create_rotation_marker("turn_right", -0.5, 0.0, "Rotate Right", False)

        # Add the forward movement marker to the server
        self.server.insert(forward_marker, self.process_feedback)

    def create_rotation_marker(self, name, position_y, position_x, description, counter_clockwise):
        rotation_marker = InteractiveMarker()
        rotation_marker.header.frame_id = "base_link"
        rotation_marker.name = name
        rotation_marker.description = description
        rotation_marker.pose.position.y = position_y
        rotation_marker.pose.position.x = position_x

        # Marker for rotation
        cylinder = Marker()
        cylinder.type = Marker.CYLINDER
        cylinder.scale.x = 0.2
        cylinder.scale.y = 0.2
        cylinder.scale.z = 0.2
        cylinder.color.r = 0.0
        cylinder.color.g = 0.0
        cylinder.color.b = 1.0
        cylinder.color.a = 1.0

        cylinder_control = InteractiveMarkerControl()
        cylinder_control.always_visible = True
        cylinder_control.markers.append(cylinder)
        cylinder_control.interaction_mode = InteractiveMarkerControl.BUTTON
        rotation_marker.controls.append(cylinder_control)

        # Add to server
        self.server.insert(rotation_marker, self.process_feedback)

    def process_feedback(self, feedback):
        twist = Twist()
        if feedback.marker_name == "move_forward":
            twist.linear.x = 0.5  # Move forward half a meter
        elif feedback.marker_name == "turn_left":
            twist.angular.z = 0.5236  # Rotate 30 degrees counter-clockwise
        elif feedback.marker_name == "turn_right":
            twist.angular.z = -0.5236  # Rotate 30 degrees clockwise

        self.vel_pub.publish(twist)

def main():
    try:
        RobotController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
