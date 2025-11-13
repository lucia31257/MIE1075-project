import rclpy
from rclpy.node import Node
from my_interfaces.srv import SpeechToText

_ros_bridge_instance = None   # 单例实例

class RosBridge(Node):
    def __init__(self):
        super().__init__('fastapi_ros_bridge')

        self.cli_stt = self.create_client(SpeechToText, 'speech_to_text')
        while not self.cli_stt.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /speech_to_text service...")

    def call_speech_to_text(self, path: str) -> str:
        req = SpeechToText.Request()
        req.audio_path = path

        future = self.cli_stt.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            return future.result().text
        else:
            return "error"


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
