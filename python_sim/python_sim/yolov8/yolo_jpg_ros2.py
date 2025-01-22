import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import torch
import os
import time
from std_msgs.msg import Header


class YoloObjectDetectionNode(Node):
    def __init__(self):
        super().__init__("yolo_object_detection_node")

        device = "cuda" if torch.cuda.is_available() else "cpu"
        print(device)  # YOLOv8 모델 로드
        self.model = YOLO("/home/jinju/runs/detect/train6/weights/best.pt").to(
            device
        )  # GPU로 이동  # 모델 경로 수정
        self.bridge = CvBridge()

        # 이미지 파일 목록 설정
        self.image_paths = [
            "/home/jinju/labelImg/images/y_cone/WIN_20240325_17_42_28_Pro_001200.jpg",
            "/home/jinju/labelImg/images/y_cone/WIN_20240325_17_42_28_Pro_001201.jpg",
            "/home/jinju/labelImg/images/y_cone/WIN_20240325_17_42_28_Pro_001204.jpg",
            "/home/jinju/labelImg/images/y_cone/WIN_20240325_17_42_28_Pro_001214.jpg",
            "/home/jinju/labelImg/images/y_cone/WIN_20240325_17_42_28_Pro_001213.jpg",
        ]

        # 주기적인 이미지 퍼블리시
        self.current_image_idx = 0  # 현재 이미지 인덱스
        self.create_timer(1.0, self.publish_next_image)

        # 결과 이미지 퍼블리시
        self.publisher = self.create_publisher(Image, "/yolo/objects_detected", 10)

    def publish_next_image(self):
        # 이미지 목록에서 현재 이미지 경로 가져오기
        if self.current_image_idx < len(self.image_paths):
            image_path = self.image_paths[self.current_image_idx]
            self.current_image_idx += 1  # 다음 이미지로 이동

            if os.path.exists(image_path):
                # 이미지 파일 읽기
                frame = cv2.imread(image_path)
                h, w, _ = frame.shape  # 이미지의 높이, 너비

                # YOLOv8 추론 수행
                results = self.model(frame)  # 추론 결과

                # 감지된 객체의 바운딩 박스만 포함된 이미지 만들기
                annotated_image = frame.copy()

                # 바운딩 박스 그리기
                for box in results[0].boxes:
                    # 바운딩 박스 좌표가 (center_x, center_y, width, height) 형태로 제공됨
                    x_center, y_center, width, height = box.xywh[0]
                    x_center, y_center, width, height = map(
                        int, (x_center, y_center, width, height)
                    )

                    # 좌표를 (x1, y1, x2, y2)로 변환
                    x1 = int(x_center - width / 2)
                    y1 = int(y_center - height / 2)
                    x2 = int(x_center + width / 2)
                    y2 = int(y_center + height / 2)

                    # 이미지 크기에 맞게 바운딩 박스 좌표를 다시 조정
                    x1, y1 = max(0, x1), max(0, y1)
                    x2, y2 = min(w, x2), min(h, y2)

                    # 초록색 바운딩 박스 그리기
                    annotated_image = cv2.rectangle(
                        annotated_image, (x1, y1), (x2, y2), (0, 255, 0), 2
                    )  # 초록색 바운딩 박스

                # 결과 이미지를 ROS2 메시지로 변환
                result_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")

                # frame_id 설정
                header = Header()
                header.stamp = self.get_clock().now().to_msg()  # 현재 시간
                header.frame_id = "map"  # 카메라 프레임을 "map"으로 설정
                result_msg.header = header

                # 객체가 감지된 이미지를 퍼블리시
                self.publisher.publish(result_msg)

            else:
                self.get_logger().warn(f"Image file not found: {image_path}")
        else:
            # 모든 이미지를 처리했으면 종료
            self.get_logger().info("All images processed.")
            self.current_image_idx = 0
            # self.destroy_node()


def main(args=None):
    rclpy.init(args=args)

    yolo_node = YoloObjectDetectionNode()
    rclpy.spin(yolo_node)

    yolo_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
