#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Bool
from math import atan2, atan, tan, sqrt
from turtle_interfaces.srv import SetWayPoint

class WaypointController(Node):
    def __init__(self):
        super().__init__('set_way_point_node')

        # Initialize
        self.current_pose = Pose()
        self.waypoint = (7.0, 7.0)  # Default target
        self.Kp_angle = 4.0         # Angular gain
        self.Kp_linear = 1.5        # Linear gain
        self.tolerance = 0.1        # Position tolerance

        # Publishers and Subscribers
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.status_pub = self.create_publisher(Bool, 'is_moving', 10)

        # Timer for control loop
        self.create_timer(0.05, self.control_loop)

        # Service to update the waypoint
        self.create_service(SetWayPoint, 'set_waypoint_service', self.set_waypoint_callback)

    def pose_callback(self, msg):
        self.current_pose = msg

    def control_loop(self):
        x = self.current_pose.x
        y = self.current_pose.y
        theta = self.current_pose.theta
        x_target, y_target = self.waypoint

        # Calculate distance and angle to waypoint
        distance = sqrt((x_target - x) ** 2 + (y_target - y) ** 2)
        angle_desired = atan2(y_target - y, x_target - x)
        angle_error = atan(tan((angle_desired - theta) / 2))

        # Publish is_moving
        moving = Bool()
        moving.data = distance > self.tolerance
        self.status_pub.publish(moving)

        # Build velocity command
        cmd = Twist()
        if distance > self.tolerance:
            cmd.angular.z = self.Kp_angle * angle_error
            cmd.linear.x = self.Kp_linear * distance
        else:
            cmd.angular.z = 0.0
            cmd.linear.x = 0.0

        self.cmd_pub.publish(cmd)

    def set_waypoint_callback(self, request, response):
        self.get_logger().info(f"Received new waypoint: ({request.x}, {request.y})")
        self.waypoint = (request.x, request.y)
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    node = WaypointController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()