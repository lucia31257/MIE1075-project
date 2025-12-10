#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

class StepTeleop(Node):
    def __init__(self):
        super().__init__('step_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Step teleop started. Use keys: w/s/a/d, q=exit")

    def step(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular

        # 只发布一次
        self.pub.publish(msg)

        # 立即停止
        stop = Twist()
        self.pub.publish(stop)

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def main(args=None):
    rclpy.init(args=args)
    node = StepTeleop()

    while rclpy.ok():
        key = get_key()
        if key == 'w':
            node.step(0.2, 0.0)   # 前进一步
        elif key == 's':
            node.step(-0.2, 0.0)  # 后退一步
        elif key == 'a':
            node.step(0.0, 0.5)   # 左转一点
        elif key == 'd':
            node.step(0.0, -0.5)  # 右转一点
        elif key == 'q':
            break

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
