import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import torch


class YoloObjectDetectionNode(Node):
    def __init__(self):
        super().__init__("yolo_object_detection_node")

        device = "cuda" if torch.cuda.is_available() else "cpu"
        print(device)  # YOLOv8 모델 로드
        self.model = YOLO("/home/jinju/runs/detect/train6/weights/best.pt").to(
            device
        )  # GPU로 이동  # 모델 경로 수정
        self.bridge = CvBridge()

        # 이미지 구독
        self.subscription = self.create_subscription(
            Image,
            "/camera/image_raw",  # 카메라에서 이미지 구독
            self.image_callback,
            10,
        )

        # 결과 이미지 퍼블리시
        self.publisher = self.create_publisher(Image, "/yolo/objects_detected", 10)

    def image_callback(self, msg):
        # ROS2 이미지를 OpenCV 형식으로 변환
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # YOLOv8 추론 수행
        results = self.model(frame)  # 추론 결과

        # 감지된 객체의 바운딩 박스만 포함된 이미지 만들기
        annotated_image = frame.copy()

        # 바운딩 박스 그리기
        for box in results[0].boxes:
            x1, y1, x2, y2 = map(int, box.xywh[0])  # xywh 형식으로 좌표 변환
            annotated_image = cv2.rectangle(
                annotated_image, (x1, y1), (x2, y2), (0, 255, 0), 2
            )  # 초록색 바운딩 박스

        # 결과 이미지를 ROS2 메시지로 변환
        result_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")

        # 객체가 감지된 이미지를 퍼블리시
        self.publisher.publish(result_msg)


def main(args=None):
    rclpy.init(args=args)

    yolo_node = YoloObjectDetectionNode()
    rclpy.spin(yolo_node)

    yolo_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
