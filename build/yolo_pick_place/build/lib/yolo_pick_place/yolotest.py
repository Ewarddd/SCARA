import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        # Subscribe to raw camera feed
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.listener_callback,
            10)

        # Publisher for detection info (text)
        self.publisher = self.create_publisher(String, '/yolo/detections', 10)

        # Publisher for annotated image (for visualization)
        self.image_pub = self.create_publisher(Image, '/yolo/annotated', 10)

        self.bridge = CvBridge()

        # Load YOLOv8 model (replace with your own path)
        self.model = YOLO('/home/reach/SCARA/src/yolo_pick_place/yolov8m.pt')

    def listener_callback(self, msg):
        # Convert ROS2 Image to OpenCV frame
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Run YOLOv8 inference
        results = self.model(frame)

        # Draw detections on the frame
        annotated_frame = results[0].plot()

        # Publish annotated image for visualization
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
        self.image_pub.publish(annotated_msg)

        # Publish detection info as text (optional)
        for r in results:
            for box in r.boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                xyxy = box.xyxy[0].tolist()
                detection_str = f"class:{cls}, conf:{conf:.2f}, box:{xyxy}"
                self.publisher.publish(String(data=detection_str))

def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
