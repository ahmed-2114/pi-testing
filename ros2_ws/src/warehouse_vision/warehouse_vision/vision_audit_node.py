import os
import threading
from datetime import datetime
from pathlib import Path

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from audix_interfaces.srv import ShelfScan
from cv_bridge import CvBridge
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO


SHELF_PRODUCTS = {
    "indomie": "Indomie",
    "beans_can": "beans_can",
    "fruit_rings_cereal": "fruit_rings_cereal",
}


class VisionAuditNode(Node):
    def __init__(self):
        super().__init__("vision_audit_node")

        self.callback_group = ReentrantCallbackGroup()
        self.bridge = CvBridge()
        self.frame_condition = threading.Condition()
        self.latest_frame = None
        self.latest_frame_time_s = 0.0

        default_model_path = str(
            Path(get_package_share_directory("warehouse_vision")) / "models" / "best.pt"
        )
        default_image_dir = os.path.expanduser("~/audix/audit_images")

        self.model_path = str(self.declare_parameter("model_path", default_model_path).value)
        self.image_topic = str(self.declare_parameter("image_topic", "image_raw").value)
        self.scan_service = str(self.declare_parameter("scan_service", "scan_shelf").value)
        self.confidence_threshold = float(self.declare_parameter("confidence_threshold", 0.5).value)
        self.target_count = int(self.declare_parameter("target_count", 2).value)
        self.audit_image_dir = str(self.declare_parameter("audit_image_dir", default_image_dir).value)
        self.scan_wait_for_new_frame_s = float(
            self.declare_parameter("scan_wait_for_new_frame_s", 2.0).value
        )

        if not os.path.exists(self.model_path):
            raise FileNotFoundError(f"YOLO model not found: {self.model_path}")

        self.get_logger().info(f"Loading model: {self.model_path}")
        self.model = YOLO(self.model_path)
        self.get_logger().info("Model loaded")

        self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10,
            callback_group=self.callback_group,
        )

        self.create_service(
            ShelfScan,
            self.scan_service,
            self.scan_callback,
            callback_group=self.callback_group,
        )

        self.get_logger().info(
            f"Vision audit ready service={self.scan_service} image_topic={self.image_topic}"
        )

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        with self.frame_condition:
            self.latest_frame = frame
            self.latest_frame_time_s = self.get_clock().now().nanoseconds / 1e9
            self.frame_condition.notify_all()

    def _fresh_frame_copy(self, trigger_time_s: float):
        deadline_s = trigger_time_s + max(0.0, self.scan_wait_for_new_frame_s)
        with self.frame_condition:
            while self.get_clock().now().nanoseconds / 1e9 <= deadline_s:
                if self.latest_frame is not None and self.latest_frame_time_s > trigger_time_s:
                    return self.latest_frame.copy()
                remaining_s = deadline_s - self.get_clock().now().nanoseconds / 1e9
                self.frame_condition.wait(timeout=max(0.02, min(0.1, remaining_s)))
            return None

    def scan_callback(self, request, response):
        trigger_time_s = self.get_clock().now().nanoseconds / 1e9
        shelf_id = request.shelf_id.strip()
        response.shelf_id = shelf_id

        if shelf_id not in SHELF_PRODUCTS:
            response.success = False
            response.status = "ERROR"
            response.message = f"Unknown shelf_id: {shelf_id}"
            return response

        frame = self._fresh_frame_copy(trigger_time_s)
        if frame is None:
            response.success = False
            response.status = "ERROR"
            response.message = "No fresh camera frame after scan trigger"
            return response

        expected_product = SHELF_PRODUCTS[shelf_id]
        results = self.model(frame, conf=self.confidence_threshold, verbose=False)

        count = 0
        expected_confidences = []
        detected_products = []
        wrong_products = []

        for box in results[0].boxes:
            cls_id = int(box.cls[0])
            class_name = str(self.model.names[cls_id])
            confidence = float(box.conf[0])
            detected_products.append(class_name)

            if class_name == expected_product:
                count += 1
                expected_confidences.append(confidence)
            else:
                wrong_products.append(class_name)

        detected_products = sorted(set(detected_products))
        wrong_products = sorted(set(wrong_products))
        best_confidence = max(expected_confidences) if expected_confidences else 0.0

        if wrong_products:
            status = "MISPLACED_PRODUCT"
            message = f"Wrong product detected: {wrong_products}"
        elif count > self.target_count:
            status = "OVER_STOCKED"
            message = f"Detected {count}, expected {self.target_count}"
        elif count < self.target_count:
            status = "UNDER_STOCKED"
            message = f"Detected {count}, expected {self.target_count}"
        else:
            status = "CORRECT_STOCK"
            message = f"Detected correct stock: {count}"

        image_path = self._save_annotated_image(results[0], shelf_id, status)

        response.success = True
        response.expected_product = expected_product
        response.expected_count = self.target_count
        response.detected_count = count
        response.detected_products = list(detected_products)
        response.wrong_products = list(wrong_products)
        response.confidence = float(best_confidence)
        response.status = status
        response.message = message
        response.image_path = image_path

        self.get_logger().info(
            f"{shelf_id}: {status}: {message} confidence={best_confidence:.3f}"
        )
        if image_path:
            self.get_logger().info(f"Annotated image saved: {image_path}")

        return response

    def _save_annotated_image(self, result, shelf_id: str, status: str) -> str:
        save_dir = os.path.expanduser(self.audit_image_dir)
        os.makedirs(save_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        image_name = f"{timestamp}_{shelf_id}_{status}.jpg"
        image_path = os.path.join(save_dir, image_name)
        cv2.imwrite(image_path, result.plot())
        return image_path


def main(args=None):
    rclpy.init(args=args)
    node = VisionAuditNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
