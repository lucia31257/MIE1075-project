import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class TestDetectionPublisher(Node):
    def __init__(self):
        super().__init__('test_detection_publisher')
        self.pub = self.create_publisher(Bool, '/start_detection', 10)
        self.get_logger().info("🟢 Test Node Started. Type 1 or 0 to publish start_detection signal.")

        # Timer to check terminal input
        self.timer = self.create_timer(0.5, self.check_input)

    def check_input(self):
        try:
            user_input = input("Enter 1 = start detection, 0 = stop detection: ").strip()
            if user_input == "1":
                msg = Bool()
                msg.data = True
                self.pub.publish(msg)
                print(">>> Published: start_detection = True")
            elif user_input == "0":
                msg = Bool()
                msg.data = False
                self.pub.publish(msg)
                print(">>> Published: start_detection = False")
        except EOFError:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = TestDetectionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

