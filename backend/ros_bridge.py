import rclpy
from rclpy.node import Node
from my_interfaces.srv import SpeechToText
from std_msgs.msg import String
from threading import Lock
import json

_ros_bridge_instance = None   # 单例实例


class RosBridge(Node):
    def __init__(self):
        super().__init__('fastapi_ros_bridge')
        # stt client
        self.cli_stt = self.create_client(SpeechToText, 'speech_to_text')
        while not self.cli_stt.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /speech_to_text service...")
        # Subscribers
        self.search_results_sub = self.create_subscription(
            String,
            '/search/results',
            self.search_results_callback,
            10
        )
        self.search_status_sub = self.create_subscription(
            String,
            '/search/status',
            self.search_status_callback,
            10
        )
        self.search_query_pub = self.create_publisher(
            String,
            '/search/query',
            10
        )
        self.latest_results = None
        self.latest_status = None
        self.results_lock = Lock()

    def call_speech_to_text(self, path: str) -> str:
        req = SpeechToText.Request()
        req.audio_path = path

        future = self.cli_stt.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            return future.result().text
        else:
            return "error"

    def search_results_callback(self, msg):
        """get result"""
        with self.results_lock:
            try:
                self.latest_results = json.loads(msg.data)
            except Exception as e:
                self.get_logger().error(f"Error parsing results: {e}")

    def search_status_callback(self, msg):
        """get status"""
        with self.results_lock:
            self.latest_status = msg.data

    def publish_search_query(self, query: str):
        """pushlish query to topic"""
        msg = String()
        msg.data = query
        self.search_query_pub.publish(msg)
        self.get_logger().info(f"Published search query: {query}")

    def get_latest_results(self):
        """get the latest result"""
        with self.results_lock:
            return self.latest_results

    def get_latest_status(self):
        """get the latest status"""
        with self.results_lock:
            return self.latest_status

    def clear_results(self):
        """clear result"""
        with self.results_lock:
            self.latest_results = None
            self.latest_status = None


def get_bridge():
    """
    返回共享的 RosBridge 实例。
    FastAPI 会重复调用，但 ROS2 Node 只初始化一次。
    """
    global _ros_bridge_instance

    if _ros_bridge_instance is None:
        rclpy.init()
        _ros_bridge_instance = RosBridge()

    return _ros_bridge_instance
