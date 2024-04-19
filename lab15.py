#!/usr/bin/env python
import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist

class RobotController:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.distance_target = 0.5  # 目标距离
        self.current_distance = 0.0  # 当前已移动距离
        self.moving = False  # 是否正在移动

    def move_forward(self):
        if not self.moving:  # 如果当前不在移动
            self.moving = True
            self.current_distance = 0.0  # 重置已移动距离
            twist = Twist()
            twist.linear.x = 0.2  # 设定速度
            rospy.Timer(rospy.Duration(0.1), self.update_movement, oneshot=False)  # 每0.1秒调用一次update_movement

    def update_movement(self, event):
        if self.current_distance < self.distance_target:
            self.current_distance += 0.02  # 假设每次调用移动0.02米
            twist = Twist()
            twist.linear.x = 0.2
            self.cmd_vel_pub.publish(twist)  # 发送速度命令
        else:
            rospy.loginfo("Reached target distance.")
            twist = Twist()
            twist.linear.x = 0  # 停止移动
            self.cmd_vel_pub.publish(twist)
            self.moving = False  # 更新移动状态为停止

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
