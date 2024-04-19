#!/usr/bin/env python
import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist

class RobotController:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    def move_forward(self, distance, speed=0.1):
        """Sends a simple motion command to move forward a specified distance."""
        move_time = distance / speed
        twist = Twist()
        twist.linear.x = speed
        rate = rospy.Rate(10)  # 10 Hz
        start_time = rospy.Time.now()

        while rospy.Time.now() - start_time < rospy.Duration(move_time):
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        # Stop the robot
        twist.linear.x = 0
        self.cmd_vel_pub.publish(twist)

def handle_viz_input(input, robot_controller):
    if input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        rospy.loginfo(input.marker_name + ' was clicked.')
        robot_controller.move_forward(0.5)  # Move forward 0.5 meters
    else:
        rospy.loginfo('Cannot handle this InteractiveMarker event')

def main():
    rospy.init_node('interactive_marker_demo')

    # Create an Interactive Marker Server on the specified topic
    server = InteractiveMarkerServer("simple_marker")
    robot_controller = RobotController()

    # Create an Interactive Marker
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.name = "my_marker"
    int_marker.description = "Simple Click Control"
    int_marker.pose.position.x = 1
    int_marker.pose.orientation.w = 1

    # Create a teal cube Marker for the InteractiveMarker
    box_marker = Marker()
    box_marker.type = Marker.CUBE
    box_marker.pose.orientation.w = 1
    box_marker.scale.x = 0.45
    box_marker.scale.y = 0.45
    box_marker.scale.z = 0.45
    box_marker.color.r = 0.0
    box_marker.color.g = 0.5
    box_marker.color.b = 0.5
    box_marker.color.a = 1.0

    # Create an InteractiveMarkerControl, add the Marker to it, and add the control to the InteractiveMarker
    button_control = InteractiveMarkerControl()
    button_control.interaction_mode = InteractiveMarkerControl.BUTTON
    button_control.always_visible = True
    button_control.markers.append(box_marker)
    int_marker.controls.append(button_control)

    # Add the InteractiveMarker to the server with the callback information
    server.insert(int_marker, lambda feedback: handle_viz_input(feedback, robot_controller))

    # Apply changes to the server to make everything visible
    server.applyChanges()

    # Keep the node running
    rospy.spin()

if __name__ == "__main__":
    main()
