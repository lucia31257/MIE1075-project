import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
import onnxruntime as ort

class EmptySlotDetector(Node):
    def __init__(self):
        super().__init__("empty_slot_detector")

        self.bridge = CvBridge()
        self.detect_enabled = False   # Controlled by navigation script

        # ===== Load model =====
        self.model_path = "/home/lwl/best.onnx"
        self.session = ort.InferenceSession(self.model_path, providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name

        # ===== Subscribe to camera =====
        self.image_sub = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.image_callback,
            10
        )

        # ===== Subscribe to /start_detection from navigation program =====
        self.start_sub = self.create_subscription(
            Bool,
            "/start_detection",
            self.start_callback,
            10
        )

        self.get_logger().info("🟢 Detector ready. Waiting for /start_detection = True...")

    def start_callback(self, msg):
        """Triggered by your navigation script"""
        self.detect_enabled = msg.data
        
        if self.detect_enabled:
            self.get_logger().info("Detection ENABLED by navigation program.")
        else:
            self.get_logger().info(" Detection DISABLED.")

    def image_callback(self, msg):
        if not self.detect_enabled:
            return  # Ignore frames until activated

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        img = cv2.resize(frame, (640, 640))
        img = img[:, :, ::-1]
        img = img.transpose(2, 0, 1)
        img = img.astype(np.float32) / 255.0
        img = np.expand_dims(img, axis=0)

        outputs = self.session.run(None, {self.input_name: img})
        detections = outputs[0]

        empty_slot_detected = False

        for det in detections:
            conf = det[4]
            cls = int(det[5])
            if cls == 0 and conf > 0.5:
                empty_slot_detected = True
                break

        if empty_slot_detected:
            self.get_logger().info("EMPTY slot detected.")
        else:
            self.get_logger().info("No empty slot detected.")

def main(args=None):
    rclpy.init(args=args)
    node = EmptySlotDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

