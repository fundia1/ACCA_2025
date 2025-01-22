import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import struct
from cv_bridge import CvBridge
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, Image


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
            self.publish_pointcloud(cv_image)
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

    def publish_pointcloud(self, cv_image):
        # 이미지 크기
        height, width, _ = cv_image.shape

        # 왼쪽 아래 1/9 영역만 선택 (높이와 너비 각각 1/3 크기만 사용)
        crop_height = height // 3
        crop_width = width // 3
        cropped_image = cv_image[height - crop_height :, :crop_width]  # 좌측 하단 1/9

        # 선택된 이미지 크기
        cropped_height, cropped_width, _ = cropped_image.shape

        # 포인트 클라우드의 포인트들을 위한 리스트
        points = []
        colors = []

        for v in range(cropped_height):
            for u in range(cropped_width):
                # 이미지의 각 픽셀에 대해 포인트를 생성
                x = u / 100.0  # 단위: 미터 (가정: 픽셀당 1cm)
                y = v / 100.0  # 단위: 미터 (가정: 픽셀당 1cm)
                z = -v / 50.0  # 각 가로줄에 대해 z가 증가하도록 설정 (v값에 비례)

                # 색상 정보 (BGR -> PointCloud2 색상으로 변환)
                bgr = cropped_image[v, u]
                color = struct.unpack(
                    "I", struct.pack("BBBB", bgr[2], bgr[1], bgr[0], 255)
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
        pc_data.height = 1  # height는 1로 설정 (행을 1로 설정)
        pc_data.width = len(points)  # 포인트 개수에 맞게 설정
        pc_data.is_dense = True
        pc_data.is_bigendian = False

        # PointField 설정 (x, y, z, rgb)
        pc_data.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
        ]

        # point_step 설정: x, y, z, rgb로 각 4바이트씩 (16바이트)
        point_step = 16
        row_step = point_step * len(points)  # 한 행의 크기 (point_step * width)

        pc_data.point_step = point_step
        pc_data.row_step = row_step
        pc_data.data = bytearray()

        # 포인트 데이터를 `data` 필드에 추가
        for i in range(len(points)):
            # 포인트 데이터를 바이트로 변환하여 추가
            pc_data.data.extend(
                struct.pack("fff", points[i][0], points[i][1], points[i][2])
            )  # x, y, z
            pc_data.data.extend(struct.pack("I", colors[i]))  # 색상 (rgb)

        # 퍼블리시
        self.publisher_.publish(pc_data)
        self.get_logger().info("Publishing left-bottom 1/9 region as PointCloud2")


def main(args=None):
    rclpy.init(args=args)
    image_to_pointcloud = ImageToPointCloud()

    rclpy.spin(image_to_pointcloud)

    image_to_pointcloud.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
