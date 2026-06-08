import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class WebcamNode(Node):
    def __init__(self):
        super().__init__("webcam_node")

        self.camera_index = int(self.declare_parameter("camera_index", 0).value)
        self.image_topic = str(self.declare_parameter("image_topic", "image_raw").value)
        self.frame_period_s = float(self.declare_parameter("frame_period_s", 0.1).value)
        self.frame_width = int(self.declare_parameter("frame_width", 0).value)
        self.frame_height = int(self.declare_parameter("frame_height", 0).value)
        self.reopen_after_failures = int(self.declare_parameter("reopen_after_failures", 10).value)

        self.publisher = self.create_publisher(Image, self.image_topic, 10)
        self.bridge = CvBridge()
        self.read_failures = 0

        self.cap = None
        self._open_camera()
        self.timer = self.create_timer(max(0.02, self.frame_period_s), self.publish_frame)

    def _open_camera(self):
        if self.cap is not None:
            self.cap.release()
        self.cap = cv2.VideoCapture(self.camera_index)
        if self.frame_width > 0:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        if self.frame_height > 0:
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)

        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open webcam index {self.camera_index}")
        else:
            self.get_logger().info(
                f"Webcam opened index={self.camera_index} topic={self.image_topic}"
            )
        self.read_failures = 0

    def publish_frame(self):
        if self.cap is None or not self.cap.isOpened():
            self._open_camera()
            return

        ret, frame = self.cap.read()
        if not ret:
            self.read_failures += 1
            if self.read_failures >= max(1, self.reopen_after_failures):
                self.get_logger().warn("Could not read frame; reopening camera")
                self._open_camera()
            else:
                self.get_logger().warn("Could not read frame")
            return
        self.read_failures = 0

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher.publish(msg)

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebcamNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
