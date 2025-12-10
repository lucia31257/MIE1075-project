#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import sys

class GoToXY(Node):
    def __init__(self, target_x, target_y):
        super().__init__('go_to_xy')

        self.target_x = target_x
        self.target_y = target_y

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.timer = self.create_timer(0.1, self.update)

        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        self.state = "rotate"

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # get yaw
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w*q.z + q.x*q.y)
        cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.current_yaw = math.atan2(siny, cosy)

    def update(self):
        if self.current_x is None:
            return

        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        dist = math.sqrt(dx*dx + dy*dy)
        angle_to_goal = math.atan2(dy, dx)
        angle_error = angle_to_goal - self.current_yaw

        cmd = Twist()

        if self.state == "rotate":
            if abs(angle_error) > 0.1:
                cmd.angular.z = 0.5 * angle_error
            else:
                self.state = "forward"

        elif self.state == "forward":
            if dist > 0.05:
                cmd.linear.x = min(0.3, dist)
            else:
                self.get_logger().info("Goal reached.")
                rclpy.shutdown()

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 3:
        print("Usage: ros2 run simple_nav goto_xy <x> <y>")
        return

    x = float(sys.argv[1])
    y = float(sys.argv[2])

    node = GoToXY(x, y)
    rclpy.spin(node)

if __name__ == '__main__':
    main()

