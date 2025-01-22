import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import Header
from sensor_msgs import msg
import struct
from cv_bridge import CvBridge
import numpy as np
from sensor_msgs.msg import PointField
import cv2


class ImageToPointCloud(Node):
    def __init__(self):
        super().__init__("image_to_pointcloud")

        # PointCloud2 메시지 퍼블리셔 설정
        self.publisher_ = self.create_publisher(PointCloud2, "/image_pointcloud", 10)

        # image_raw 토픽 구독
        self.image_sub = self.create_subscription(
            Image, "camera1/image_raw", self.image_callback, 10
        )

        # CvBridge 객체 생성
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            # ROS2 이미지 메시지를 OpenCV 형식으로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # 이미지 크기 다운그레이드 (예: 1/2 크기로 축소)
            scale_factor = 0.5  # 축소 비율
            cv_image_resized = cv2.resize(
                cv_image, None, fx=scale_factor, fy=scale_factor
            )

            self.publish_pointcloud(cv_image_resized)
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

    def publish_pointcloud(self, cv_image):
        # 이미지 크기
        height, width, _ = cv_image.shape

        # 포인트 클라우드의 포인트들을 위한 리스트
        points = []
        colors = []

        for v in range(height):
            for u in range(width):
                # 이미지의 각 픽셀에 대해 포인트를 생성
                x = u / 10.0  # 단위: 미터 (가정: 픽셀당 1cm)
                y = v / 10.0  # 단위: 미터 (가정: 픽셀당 1cm)
                z = 0.0  # 깊이는 0으로 설정 (평면에 배치)

                # 색상 정보 (BGR -> PointCloud2 색상으로 변환)
                bgr = cv_image[v, u]
                color = struct.unpack(
                    "I", struct.pack("BBBB", bgr[0], bgr[1], bgr[2], 255)
                )[0]

                points.append([x, y, z])  # 포인트 좌표 추가
                colors.append(color)  # 색상 추가

        # 포인트 클라우드 메시지 생성
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"

        # PointCloud2 메시지 생성
        pc_data = PointCloud2()
        pc_data.header = header
        pc_data.height = height
        pc_data.width = width
        pc_data.is_dense = True
        pc_data.is_bigendian = False

        # 포인트 필드 설정 (x, y, z 좌표)
        pc_data.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
        ]

        # 포인트 클라우드 데이터 크기 설정
        point_step = 16  # (x, y, z, rgb) 각 4바이트씩 -> 16바이트
        row_step = point_step * width  # 한 행의 크기 (point_step * width)

        pc_data.point_step = point_step
        pc_data.row_step = row_step
        pc_data.data = bytearray()

        # 데이터 배열 생성 (포인트와 색상 추가)
        for i in range(len(points)):
            # 포인트 데이터를 바이트로 변환하여 추가
            pc_data.data.extend(
                struct.pack("fff", points[i][0], points[i][1], points[i][2])
            )  # x, y, z
            pc_data.data.extend(struct.pack("I", colors[i]))  # 색상 (rgb)

        # 퍼블리시
        self.publisher_.publish(pc_data)
        self.get_logger().info("Publishing PointCloud2 with image data")


def main(args=None):
    rclpy.init(args=args)
    image_to_pointcloud = ImageToPointCloud()

    rclpy.spin(image_to_pointcloud)

    image_to_pointcloud.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
