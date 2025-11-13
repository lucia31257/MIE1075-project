import rclpy
from rclpy.node import Node
from my_interfaces.srv import SpeechToText

# 假装语音识别（你之后换 Whisper）
def fake_speech_recognition(path):
    print("Speech file received:", path)
    return "milk"  

class SpeechToTextServer(Node):

    def __init__(self):
        super().__init__('speech_to_text_server')
        self.srv = self.create_service(
            SpeechToText,
            'speech_to_text',
            self.callback
        )
        self.get_logger().info("SpeechToText service ready.")

    def callback(self, request, response):
        text = fake_speech_recognition(request.audio_path)
        response.text = text
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SpeechToTextServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
