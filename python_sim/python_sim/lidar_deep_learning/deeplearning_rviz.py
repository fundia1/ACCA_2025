import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
import torch
import numpy as np
import std_msgs.msg


class TrafficConeDetectionNode(Node):
    def __init__(self):
        super().__init__("traffic_cone_detection_node")

        # 모델 로드
        self.model = self.load_model(
            "/home/jinju/ws/src/python_sim/python_sim/lidar_deep_learning/traffic_cone_model.pth"
        )
        self.model.eval()  # 평가 모드로 설정

        # PCD 데이터를 구독
        self.create_subscription(
            PointCloud2, "/points_no_ground", self.pcd_callback, 10
        )

        # 바운딩 박스를 퍼블리시할 퍼블리셔 설정
        self.box_pub = self.create_publisher(MarkerArray, "/bounding_boxes", 10)

    def load_model(self, model_path):
        model = PointCloudModel()
        model.load_state_dict(torch.load(model_path))
        return model

    def pcd_callback(self, msg):
        # PointCloud2 메시지에서 포인트 데이터를 추출
        points = self.convert_pointcloud2_to_numpy(msg)

        # 포인트가 3D 형태로 되어 있는지 확인
        if points.shape[1] != 3:
            self.get_logger().warn("Invalid point cloud format!")
            return

        # nan 및 inf 값 필터링
        valid_points = points[np.isfinite(points).all(axis=1)]

        # 모델에 맞게 입력 데이터 준비
        points_tensor = torch.tensor(valid_points, dtype=torch.float32)

        # 모델 예측
        with torch.no_grad():
            outputs = self.model(points_tensor)

            # 예측된 값이 nan 또는 inf인 경우 체크
            if torch.isnan(outputs).any() or torch.isinf(outputs).any():
                self.get_logger().warn("Model output contains NaN or Inf values!")
                return

            _, predicted = torch.max(outputs, 1)  # 예측된 클래스를 추출

        # 예측된 확신도(확률)가 임계값 이상인 경우에만 바운딩 박스 생성
        threshold = (
            0.7  # 예시: 70% 이상의 확신도가 있어야 traffic_cone으로 예측된 것으로 간주
        )
        predicted_confidence = torch.max(torch.softmax(outputs, dim=1), dim=1).values
        high_confidence_mask = predicted_confidence >= threshold

        # traffic_cone 클래스인 포인트만 필터링
        traffic_cone_points = valid_points[
            high_confidence_mask & (predicted == 1)
        ]  # 1: traffic_cone

        # 바운딩 박스 계산
        if (
            traffic_cone_points.shape[0] > 10
        ):  # 포인트가 10개 이상일 경우에만 바운딩 박스를 생성
            # 여러 개의 바운딩 박스를 계산하여 추가
            marker_array = MarkerArray()

            # 여러 개의 바운딩 박스를 위한 그룹화 (예시: x, y 범위로 그룹화)
            unique_x_bins = np.unique(traffic_cone_points[:, 0])

            for x_bin in unique_x_bins:
                group_points = traffic_cone_points[traffic_cone_points[:, 0] == x_bin]

                if (
                    group_points.shape[0] > 5
                ):  # 최소 5개의 포인트가 있어야 바운딩 박스를 생성
                    min_bound = np.min(group_points, axis=0)
                    max_bound = np.max(group_points, axis=0)

                    # nan 또는 inf 값이 포함된 경우 처리
                    if np.isnan(min_bound).any() or np.isnan(max_bound).any():
                        self.get_logger().warn(
                            f"NaN values detected in bounding box: min={min_bound}, max={max_bound}"
                        )
                        continue
                    if np.isinf(min_bound).any() or np.isinf(max_bound).any():
                        self.get_logger().warn(
                            f"Inf values detected in bounding box: min={min_bound}, max={max_bound}"
                        )
                        continue

                    # 값의 범위 제한 (오버플로우 방지)
                    min_bound = np.clip(min_bound, -1e6, 1e6)
                    max_bound = np.clip(max_bound, -1e6, 1e6)

                    # Marker 객체 생성
                    marker = Marker()
                    marker.header.frame_id = "map"  # 프레임 설정
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.ns = "traffic_cone_bounding_box"
                    marker.id = len(marker_array.markers)  # 고유 id 설정
                    marker.type = Marker.CUBE
                    marker.action = Marker.ADD
                    marker.pose.position.x = float(
                        min_bound[0] + (max_bound[0] - min_bound[0]) / 2
                    )  # 가운데 위치
                    marker.pose.position.y = float(
                        min_bound[1] + (max_bound[1] - min_bound[1]) / 2
                    )
                    marker.pose.position.z = float(
                        min_bound[2] + (max_bound[2] - min_bound[2]) / 2
                    )

                    # 바운딩 박스 크기 계산 (float 형으로 명시적 형변환)
                    marker.scale.x = float(max_bound[0] - min_bound[0])
                    marker.scale.y = float(max_bound[1] - min_bound[1])
                    marker.scale.z = float(max_bound[2] - min_bound[2])

                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                    marker.color.a = 0.5  # 투명도

                    # MarkerArray에 추가
                    marker_array.markers.append(marker)

            # MarkerArray 퍼블리시
            if len(marker_array.markers) > 0:
                self.box_pub.publish(marker_array)
                self.get_logger().info(
                    f"{len(marker_array.markers)} bounding boxes published!"
                )
            else:
                self.get_logger().info("No traffic cone detected!")

    def convert_pointcloud2_to_numpy(self, msg):
        """PointCloud2 메시지를 NumPy 배열로 변환"""
        # PointCloud2 메시지에서 포인트 클라우드 데이터를 추출
        # 메시지의 포인트 데이터는 바이트 배열로 되어 있음
        pc_data = np.frombuffer(msg.data, dtype=np.float32)

        # 각 포인트는 4개의 값(x, y, z, intensity)을 가짐. 각 값은 4바이트.
        pc_data = pc_data.reshape((-1, 4))  # (N, 4) 형태로 재구성
        points = pc_data[:, :3]  # x, y, z 값만 사용
        return points


class PointCloudModel(torch.nn.Module):
    def __init__(self):
        super(PointCloudModel, self).__init__()
        self.fc1 = torch.nn.Linear(3, 64)  # 입력은 3차원 포인트
        self.fc2 = torch.nn.Linear(64, 64)
        self.fc3 = torch.nn.Linear(64, 2)  # 2개의 클래스 (배경, traffic_cone)

    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        x = self.fc3(x)
        return x


def main(args=None):
    rclpy.init(args=args)
    node = TrafficConeDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
