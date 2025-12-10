#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

STEP_LINEAR = 0.20     # 每次走 20 cm
STEP_ANGULAR = 0.35    # 每次旋转 20° (约 0.35 rad)


def get_key():
    """读取单次按键，不需要按回车"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key


class StepTeleop(Node):
    def __init__(self):
        super().__init__("step_teleop")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.get_logger().info("Step Teleop Ready. Press q to quit.")

    def send_cmd(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.pub.publish(msg)

        # 立即停止
        stop = Twist()
        self.pub.publish(stop)

    def loop(self):
        while rclpy.ok():
            key = get_key()

            if key == 'w':
                self.send_cmd(STEP_LINEAR, 0.0)
                print("Forward step")

            elif key == 's':
                self.send_cmd(-STEP_LINEAR, 0.0)
                print("Backward step")

            elif key == 'a':
                self.send_cmd(0.0, STEP_ANGULAR)
                print("Turn left step")

            elif key == 'd':
                self.send_cmd(0.0, -STEP_ANGULAR)
                print("Turn right step")

            elif key == 'q':
                print("Exit")
                break


def main(args=None):
    rclpy.init(args=args)
    node = StepTeleop()

    try:
        node.loop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
