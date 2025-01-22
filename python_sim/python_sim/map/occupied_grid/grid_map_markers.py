import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import Header
import numpy as np
from PIL import Image
import yaml
from visualization_msgs.msg import Marker, MarkerArray
import math
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import os

os.chdir("/home/jinju/ws/src/python_sim/python_sim/map/occupied_grid")


class MapUpdater(Node):
    def __init__(self):
        super().__init__("map_updater_node")

        # YAML 파일 경로
        self.yaml_file = "map.yaml"
        self.map_image_file = None
        self.map_metadata = None

        # 맵 데이터 초기화
        self.map_array = None
        self.resolution = None
        self.origin = None
        self.inflation_radius = 0.7  # 인플레이션 반경 (미터 단위)

        self.robot = [0, 0, 0]
        # 맵 퍼블리셔
        self.map_publisher = self.create_publisher(OccupancyGrid, "/static_map", 10)

        self.create_subscription(
            Odometry, "/localization/kinematic_state", self.odom_callback, 10
        )

        self.create_subscription(MarkerArray, "/markers", self.marker_callback, 10)

        # # 구독자 생성
        self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self.initialpose_callback, 10
        )
        # self.create_subscription(PoseStamped, "/goal_pose", self.goal_pose_callback, 10)

        # 초기 맵 로드
        self.load_map()
        self.publish_map()

    def odom_callback(self, msg):
        # Odometry 메시지에서 위치와 회전 정보를 추출
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        # 로봇의 현재 위치
        self.robot[0] = position.x
        self.robot[1] = position.y

        # Quaternion을 yaw로 변환
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        self.robot[2] = euler_from_quaternion(quaternion)[2]  # yaw (회전각)

    def marker_callback(self, msg):
        # MarkerArray 데이터 처리
        for marker in msg.markers:
            for point in marker.points:
                map_x, map_y = self.velodyne_to_map(point.x, point.y)

                # 맵 업데이트 함수 호출
                self.update_map_with_pose(
                    (map_x, map_y), value=100, apply_inflation=True
                )

        self.publish_map()

    def velodyne_to_map(self, x, y):
        """Velodyne 기준 좌표를 map 기준 좌표로 변환"""
        robot_x, robot_y, robot_yaw = self.robot
        x += 1
        # 벨로다인 좌표를 로봇 기준에서 맵 기준으로 변환
        map_x = robot_x + x * math.cos(robot_yaw) - y * math.sin(robot_yaw)
        map_y = robot_y + x * math.sin(robot_yaw) + y * math.cos(robot_yaw)

        return map_x, map_y

    def load_map(self):
        with open(self.yaml_file, "r") as file:
            self.map_metadata = yaml.safe_load(file)

        self.get_logger().info(f"Map metadata loaded: {self.map_metadata}")

        self.map_image_file = self.map_metadata["image"]
        self.resolution = self.map_metadata["resolution"]
        self.origin = self.map_metadata["origin"]

        # 추가 디버깅 메시지
        self.get_logger().info(f"Map origin: {self.origin}")
        self.get_logger().info(f"Resolution: {self.resolution}")

        image = Image.open(self.map_image_file)
        self.map_array = np.array(image, dtype=np.uint8)

    def publish_map(self):
        # OccupancyGrid 메시지 생성
        occupancy_grid_msg = OccupancyGrid()
        occupancy_grid_msg.header = Header()
        occupancy_grid_msg.header.stamp = self.get_clock().now().to_msg()
        occupancy_grid_msg.header.frame_id = "map"
        occupancy_grid_msg.info.resolution = self.resolution
        occupancy_grid_msg.info.width = self.map_array.shape[1]
        occupancy_grid_msg.info.height = self.map_array.shape[0]
        occupancy_grid_msg.info.origin.position.x = self.origin[0]
        occupancy_grid_msg.info.origin.position.y = self.origin[1]
        occupancy_grid_msg.info.origin.position.z = 0.0
        occupancy_grid_msg.info.origin.orientation.w = 1.0

        # 회전 정보를 쿼터니언 형식으로 설정 (yaw 값을 회전으로 적용)
        yaw = self.origin[2]
        quaternion = self.yaw_to_quaternion(yaw)
        occupancy_grid_msg.info.origin.orientation.x = quaternion[0]
        occupancy_grid_msg.info.origin.orientation.y = quaternion[1]
        occupancy_grid_msg.info.origin.orientation.z = quaternion[2]
        occupancy_grid_msg.info.origin.orientation.w = quaternion[3]

        # 맵 데이터를 OccupancyGrid 메시지로 변환
        occupancy_grid_msg.data = (
            self.convert_image_to_grid(self.map_array).flatten().tolist()
        )
        self.map_publisher.publish(occupancy_grid_msg)
        self.get_logger().info("Updated map published.")

    def yaw_to_quaternion(self, yaw):
        # yaw 값을 쿼터니언으로 변환
        qx = 0.0
        qy = 0.0
        qz = np.sin(yaw / 2.0)
        qw = np.cos(yaw / 2.0)
        return [qx, qy, qz, qw]

    def convert_image_to_grid(self, map_array):
        # OccupancyGrid 데이터로 변환
        grid_data = np.full(
            map_array.shape, -1, dtype=int
        )  # 기본 값은 모르는 지역 (-1)

        occupied_thresh = self.map_metadata["occupied_thresh"] * 255
        free_thresh = self.map_metadata["free_thresh"] * 255

        # 점유 상태 변환
        grid_data[map_array >= free_thresh] = 0  # 비점유 영역은 0
        grid_data[map_array < occupied_thresh] = 100  # 점유 영역은 100으로 설정

        return grid_data

    def update_map_with_pose(self, pose, value=255, apply_inflation=True):
        # 포즈를 Occupancy Grid 좌표로 변환
        x, y = pose
        x -= self.origin[0]
        y -= self.origin[1]

        # 회전 변환 적용
        yaw = self.origin[2]  # 맵의 회전(yaw)
        map_x = int((x * math.cos(yaw) + y * math.sin(yaw)) / self.resolution)
        map_y = int((-x * math.sin(yaw) + y * math.cos(yaw)) / self.resolution)

        if (
            map_x < 0
            or map_x >= self.map_array.shape[1]
            or map_y < 0
            or map_y >= self.map_array.shape[0]
        ):
            return

        self.map_array[map_y, map_x] = value

        if apply_inflation:
            self.apply_inflation(map_x, map_y, max_cost=10)

    def apply_inflation(self, x, y, max_cost):
        inflation_radius_cells = int(self.inflation_radius / self.resolution)
        for i in range(-inflation_radius_cells, inflation_radius_cells + 1):
            for j in range(-inflation_radius_cells, inflation_radius_cells + 1):
                nx, ny = x + i, y + j
                if (
                    nx < 0
                    or nx >= self.map_array.shape[1]
                    or ny < 0
                    or ny >= self.map_array.shape[0]
                ):
                    continue

                distance = np.sqrt(i**2 + j**2) * self.resolution
                if distance <= self.inflation_radius:
                    cost = max_cost * (1 - distance / self.inflation_radius)
                    cost = max(0, int(cost))
                    self.map_array[ny, nx] = min(self.map_array[ny, nx], cost)

    def save_map_to_file(self):
        updated_image = Image.fromarray(self.map_array)
        updated_image.save(self.map_image_file)
        self.get_logger().info(f"Updated map saved to {self.map_image_file}.")

    def initialpose_callback(self, msg):
        self.update_map_with_pose(
            (msg.pose.pose.position.x, msg.pose.pose.position.y),
            value=0,
            apply_inflation=True,
        )
        self.publish_map()
        # self.save_map_to_file()

    # def goal_pose_callback(self, msg):
    #     self.update_map_with_pose((msg.pose.position.x, msg.pose.position.y), value=0)
    #     self.publish_map()
    #     self.save_map_to_file()


def main(args=None):
    rclpy.init(args=args)
    node = MapUpdater()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
