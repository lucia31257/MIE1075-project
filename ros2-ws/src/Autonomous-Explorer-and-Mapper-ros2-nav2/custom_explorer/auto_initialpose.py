import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class AutoInitialPose(Node):
    """
    简单的自动 initial pose 节点：
    启动后每 1 秒往 /initialpose 发布一次初始化位姿，
    连续发几次（默认 10 次）然后停止。
    """

    def __init__(self):
        super().__init__("auto_initialpose")

        # 可以在 launch 里通过参数覆盖
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("yaw", 0.0)  # 弧度制

        self.x = float(self.get_parameter("x").value)
        self.y = float(self.get_parameter("y").value)
        self.yaw = float(self.get_parameter("yaw").value)

        self.get_logger().info(
            f"AutoInitialPose started with x={self.x:.3f}, y={self.y:.3f}, yaw={self.yaw:.3f} rad"
        )

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )

        # 连续发 N 次，防止 AMCL 没及时收到
        self.max_publish = 10
        self.publish_count = 0
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        if self.publish_count >= self.max_publish:
            self.get_logger().info(
                f"Initial pose published {self.max_publish} times, stop publishing."
            )
            self.timer.cancel()
            return

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"   # 很关键：map 坐标系

        # 位置
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0

        # yaw -> quaternion
        half_yaw = self.yaw / 2.0
        qz = math.sin(half_yaw)
        qw = math.cos(half_yaw)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        # 给一个比较小的协方差（表示我们比较确定这个位置）
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = 0.25      # x 方差
        msg.pose.covariance[7] = 0.25      # y 方差
        msg.pose.covariance[35] = 0.1 ** 2 # yaw 方差

        self.publisher.publish(msg)
        self.publish_count += 1

        if self.publish_count == 1:
            self.get_logger().info("Initial pose published once.")
        else:
            self.get_logger().debug(
                f"Initial pose published {self.publish_count}/{self.max_publish}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = AutoInitialPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("AutoInitialPose interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
