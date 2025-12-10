#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import termios, tty, sys, select, time


class StepTeleop(Node):
    def __init__(self):
        super().__init__('step_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 可调参数
        self.step_time = 0.2        # 每次运动持续时间（秒）
        self.linear_speed = 0.08    # 前进/后退速度
        self.angular_speed = 0.6    # 左右转速度

        self.get_logger().info(
            "Step Teleop Running:\n"
            "i = forward\n"
            ", = back\n"
            "j = turn left\n"
            "l = turn right\n"
            "k = stop\n"
            "q = quit\n"
        )

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def move_once(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular

        end_time = time.time() + self.step_time
        while time.time() < end_time:
            self.pub.publish(twist)
            time.sleep(0.01)

        # 自动停止
        self.pub.publish(Twist())

    def run(self):
        while rclpy.ok():
            key = self.get_key()
            if key == 'q':
                break
            elif key == 'i':
                self.move_once(self.linear_speed, 0.0)
            elif key == ',':
                self.move_once(-self.linear_speed, 0.0)
            elif key == 'j':
                self.move_once(0.0, self.angular_speed)
            elif key == 'l':
                self.move_once(0.0, -self.angular_speed)
            elif key == 'k':
                self.pub.publish(Twist())


settings = termios.tcgetattr(sys.stdin)

def main():
    rclpy.init()
    node = StepTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

