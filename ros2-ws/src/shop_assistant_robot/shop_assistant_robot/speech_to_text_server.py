import rclpy
from rclpy.node import Node
from my_interfaces.srv import SpeechToText
from faster_whisper import WhisperModel
from std_msgs.msg import String
import os
# 假装语音识别（你之后换 Whisper）
#def fake_speech_recognition(path):
 #   print("Speech file received:", path)
  #  return "milk"  

class SpeechToTextServer(Node):

    def __init__(self):
        super().__init__('speech_to_text_server')
        # service
        self.srv = self.create_service(
            SpeechToText,
            'speech_to_text',
            self.callback
        )
        self.model = WhisperModel(
            "base",
            device="cpu",       
            compute_type="int8" 
        )
        self.get_logger().info("SpeechToText service ready.")
        # publisher
        self.query = self.create_publisher(
            String,
            '/search/query',
            10
        )
        

#    def callback(self, request, response):
 #       text = fake_speech_recognition(request.audio_path)
  #      response.text = text
   #     return response
    def callback(self, request, response):
        audio_path = request.audio_path
        self.get_logger().info(f"Processing audio file: {audio_path}")
        
        try:
            # Check if file exists

            if not os.path.exists(audio_path):
                self.get_logger().error(f"Audio file not found: {audio_path}")
                response.text = ""
                return response
            
            # Transcribe with Whisper
            segments, info = self.model.transcribe(
                audio_path,
                beam_size=5,
                language="en"  
            )

            # Join all segments
            text = "".join([seg.text for seg in segments]).strip()

            self.get_logger().info(f"Transcribed text: {text}")
            response.text = text
            try:
                msg = String()
                msg.data = text
                self.query.publish(msg)
                self.get_logger().info("Published transcription to /search/query")
            except Exception as e_pub:
                self.get_logger().error(f"Failed to publish /search/query: {e_pub}")

            
        except Exception as e:
            self.get_logger().error(f"Error during transcription: {str(e)}")
            response.text = ""
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SpeechToTextServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
