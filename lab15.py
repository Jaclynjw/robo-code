#!/usr/bin/env python
import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from geometry_msgs.msg import Point

from base import Base  # Assuming Base class is defined as previously discussed

def make_button_marker(position, name, description):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.pose.position = position
    int_marker.scale = 0.2

    int_marker.name = name
    int_marker.description = description

    button_control = InteractiveMarkerControl()
    button_control.interaction_mode = InteractiveMarkerControl.BUTTON
    button_control.always_visible = True

    box_marker = Marker()
    box_marker.type = Marker.CUBE
    box_marker.scale.x = 0.15
    box_marker.scale.y = 0.15
    box_marker.scale.z = 0.15
    box_marker.color.r = 0.5
    box_marker.color.g = 0.5
    box_marker.color.b = 0.5
    box_marker.color.a = 1.0

    button_control.markers.append(box_marker)
    int_marker.controls.append(button_control)

    return int_marker

def handle_marker_feedback(feedback):
    command = feedback.marker_name
    print(f"Handling {command}")
    if command == "forward":
        base.go_forward(0.5)  # Move forward by 0.5 meters
    elif command == "turn_left":
        base.turn(math.radians(30))  # Turn left by 30 degrees
    elif command == "turn_right":
        base.turn(math.radians(-30))  # Turn right by 30 degrees

if __name__ == "__main__":
    rospy.init_node("interactive_robot_control")

    base = Base()

    server = InteractiveMarkerServer("base_marker_control")

    forward_position = Point(0.6, 0, 0)  # Forward button in front of the robot
    turn_left_position = Point(0, 0.3, 0)  # Turn left button to the left of the robot
    turn_right_position = Point(0, -0.3, 0)  # Turn right button to the right of the robot

    forward_marker = make_button_marker(forward_position, "forward", "Move Forward")
    turn_left_marker = make_button_marker(turn_left_position, "turn_left", "Turn Left")
    turn_right_marker = make_button_marker(turn_right_position, "turn_right", "Turn Right")

    server.insert(forward_marker, handle_marker_feedback)
    server.insert(turn_left_marker, handle_marker_feedback)
    server.insert(turn_right_marker, handle_marker_feedback)

    server.applyChanges()

    rospy.spin()
