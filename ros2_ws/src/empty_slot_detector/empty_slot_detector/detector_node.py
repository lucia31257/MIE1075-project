import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge

import cv2
import numpy as np
import onnxruntime as ort


# =========================================================
#                 NumPy NMS for YOLOv9
# =========================================================
def nms_numpy(boxes, scores, iou_threshold=0.45):
    if len(boxes) == 0:
        return []

    boxes = boxes.astype(np.float32)
    scores = scores.astype(np.float32)

    x1 = boxes[:, 0] - boxes[:, 2] / 2
    y1 = boxes[:, 1] - boxes[:, 3] / 2
    x2 = boxes[:, 0] + boxes[:, 2] / 2
    y2 = boxes[:, 1] + boxes[:, 3] / 2

    areas = (x2 - x1) * (y2 - y1)
    order = scores.argsort()[::-1]

    keep = []

    while order.size > 0:
        i = order[0]
        keep.append(i)

        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])

        w = np.maximum(0.0, xx2 - xx1)
        h = np.maximum(0.0, yy2 - yy1)
        inter = w * h

        iou = inter / (areas[i] + areas[order[1:]] - inter)
        idxs = np.where(iou <= iou_threshold)[0]

        order = order[idxs + 1]

    return keep


# =========================================================
#                 ROS2 Empty Slot Detector Node
# =========================================================
class EmptySlotDetector(Node):
    def __init__(self):
        super().__init__("empty_slot_detector")

        self.bridge = CvBridge()
        self.detect_enabled = False

        # ===== Load YOLOv9 ONNX =====
        self.model_path = "/home/lwl/best.onnx"
        self.get_logger().info(f"Loading YOLOv9 ONNX: {self.model_path}")
        self.session = ort.InferenceSession(self.model_path, providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name

        # ===== Camera Subscriber =====
        self.image_sub = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.image_callback,
            10
        )

        # ===== Signal from Navigation =====
        self.start_sub = self.create_subscription(
            Bool,
            "/start_detection",
            self.start_callback,
            10
        )

        # ===== Publish detection result =====
        self.result_pub = self.create_publisher(Bool, "/empty_slot_detected", 10)

        # ===== Publish debug image =====
        self.debug_pub = self.create_publisher(Image, "/empty_slot/debug_image", 10)

        self.get_logger().info("🟢 YOLOv9 Detector ready. Waiting for /start_detection = True...")

    def start_callback(self, msg):
        self.detect_enabled = msg.data

        if msg.data:
            self.get_logger().info("🔛 Detection ENABLED by navigation.")
        else:
            self.get_logger().info("⛔ Detection DISABLED.")

    # =========================================================
    #                 Main YOLO detection logic
    # =========================================================
    def image_callback(self, msg):
        if not self.detect_enabled:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        h0, w0 = frame.shape[:2]

        # Preprocess for YOLO (640×640)
        img = cv2.resize(frame, (640, 640))
        img_rgb = img[:, :, ::-1]
        img_input = img_rgb.transpose(2, 0, 1).astype(np.float32) / 255.0
        img_input = img_input[np.newaxis, :, :, :]

        # ===== YOLOv9 inference =====
        pred = self.session.run(None, {self.input_name: img_input})[0]  # (1,5,8400)

        # ===== FIX: reshape (1,5,8400) -> (8400,5) =====
        pred = np.squeeze(pred)       # (5,8400)
        pred = pred.T                # (8400,5)

        # Extract boxes & confidence
        boxes = pred[:, :4]
        scores = pred[:, 4]

        # Filter confidence
        mask = scores > 0.5
        boxes = boxes[mask]
        scores = scores[mask]

        if len(scores) == 0:
            self.result_pub.publish(Bool(data=False))
            self.get_logger().info("✔️ No empty slot detected.")
            self.detect_enabled = False
            return

        # ===== Apply NMS =====
        keep = nms_numpy(boxes, scores)
        boxes = boxes[keep]
        scores = scores[keep]

        empty_slot_detected = len(boxes) > 0
        self.result_pub.publish(Bool(data=empty_slot_detected))

        # ===== Draw debug boxes =====
        debug_img = frame.copy()

        scale_x = w0 / 640
        scale_y = h0 / 640

        for i, b in enumerate(boxes):
            x, y, w, h = b
            x1 = int((x - w/2) * scale_x)
            y1 = int((y - h/2) * scale_y)
            x2 = int((x + w/2) * scale_x)
            y2 = int((y + h/2) * scale_y)

            cv2.rectangle(debug_img, (x1, y1), (x2, y2), (0,0,255), 2)
            cv2.putText(debug_img, f"{scores[i]:.2f}",
                        (x1, y1-5),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0,0,255), 2)

        # Publish debug image
        debug_msg = self.bridge.cv2_to_imgmsg(debug_img, "bgr8")
        self.debug_pub.publish(debug_msg)

        if empty_slot_detected:
            self.get_logger().info("⚠️ EMPTY slot detected ✔️")
        else:
            self.get_logger().info("✔️ No empty slot detected.")

        # Only detect ONCE per navigation request
        self.detect_enabled = False


# =========================================================
#                      ROS2 main()
# =========================================================
def main(args=None):
    rclpy.init(args=args)
    node = EmptySlotDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

