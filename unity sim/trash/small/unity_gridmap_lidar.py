import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import Header
import numpy as np
from PIL import Image
import yaml
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Pose
import math
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import os

os.chdir("/home/jinju/ws/src/Unity_sim/obstacle/small")


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

        self.create_subscription(
            PointCloud2, "/velodyne_points", self.lidar_callback, 10
        )

        # 구독자 생성
        self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self.initialpose_callback, 10
        )
        self.create_subscription(PoseStamped, "/goal_pose", self.goal_pose_callback, 10)

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

    def lidar_callback(self, msg):
        # LIDAR 데이터 처리
        lidar_points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        for point in lidar_points:
            x, y, z = point

            self.update_map_with_pose(
                (x, y),
                value=0,  # goal 위치는 비점유(흰색)으로 설정
            )

        self.publish_map()

    def velodyne_to_map(self, x, y):
        """Velodyne 기준 좌표를 map 기준 좌표로 변환"""
        robot_x, robot_y, robot_yaw = self.robot

        # 회전 행렬 적용
        map_x = x * math.cos(robot_yaw) - y * math.sin(robot_yaw) + robot_x
        map_y = x * math.sin(robot_yaw) + y * math.cos(robot_yaw) + robot_y

        return map_x, map_y

    def load_map(self):
        # YAML 파일 읽기
        with open(self.yaml_file, "r") as file:
            self.map_metadata = yaml.safe_load(file)

        self.map_image_file = self.map_metadata["image"]
        self.resolution = self.map_metadata["resolution"]
        self.origin = self.map_metadata["origin"]

        # 이미지 로드
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
        occupancy_grid_msg.info.origin.position.z = -0.3
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
        # 포즈를 Occupancy Grid 좌표로 변환 (회전 변환 포함)
        if isinstance(pose, tuple):
            x, y = pose
            x -= self.origin[0]
            y -= self.origin[1]
        else:
            x = pose.position.x - self.origin[0]
            y = pose.position.y - self.origin[1]
        # 회전 변환 적용 (맵의 yaw 값을 사용)
        yaw = self.origin[2]  # 맵의 회전(yaw)
        map_x = int((x * math.cos(yaw) + y * math.sin(yaw)) / self.resolution)
        map_y = int((-x * math.sin(yaw) + y * math.cos(yaw)) / self.resolution)

        if (
            map_x < 0
            or map_x >= self.map_array.shape[1]
            or map_y < 0
            or map_y >= self.map_array.shape[0]
        ):
            # self.get_logger().warn(
            #     f"Pose ({pose.position.x}, {pose.position.y}) is out of bounds."
            # )
            return

        # 목표 위치를 비점유로 설정하거나 장애물로 설정
        if value == 255:  # goal 위치 처리
            self.map_array[map_y, map_x] = 255  # 목표는 비점유(흰색)으로 표시
        else:
            self.map_array[map_y, map_x] = value  # 장애물(검정)으로 표시

        # 인플레이션을 적용할지 여부 결정
        if apply_inflation:
            self.apply_inflation(map_x, map_y, max_cost=10)

    def apply_inflation(self, x, y, max_cost):
        # 인플레이션 반경을 셀 단위로 계산
        inflation_radius_cells = int(self.inflation_radius / self.resolution)
        for i in range(-inflation_radius_cells, inflation_radius_cells + 1):
            for j in range(-inflation_radius_cells, inflation_radius_cells + 1):
                nx, ny = x + i, y + j
                # 맵 경계를 넘지 않도록 처리
                if (
                    nx < 0
                    or nx >= self.map_array.shape[1]
                    or ny < 0
                    or ny >= self.map_array.shape[0]
                ):
                    continue

                distance = np.sqrt(i**2 + j**2) * self.resolution
                if distance <= self.inflation_radius:
                    # 거리 기반 코스트 계산
                    cost = max_cost * (1 - distance / self.inflation_radius)
                    cost = max(0, max(0, int(cost)))
                    # 인플레이션 적용 (검정)
                    self.map_array[ny, nx] = min(self.map_array[ny, nx], cost)

    def save_map_to_file(self):
        # 수정된 맵 데이터를 이미지로 저장
        updated_image = Image.fromarray(self.map_array)
        updated_image.save(self.map_image_file)
        self.get_logger().info(f"Updated map saved to {self.map_image_file}.")

    def initialpose_callback(self, msg):

        self.update_map_with_pose(msg.pose.pose, value=255, apply_inflation=False)
        self.publish_map()
        # self.save_map_to_file()

    def goal_pose_callback(self, msg):

        pass
        # 목표 위치를 비점유로 설정하고, 그 근처를 회색으로 설정
        # self.update_map_with_pose(
        #     msg.pose,
        #     value=0,  # goal 위치는 비점유(흰색)으로 설정
        # )
        # self.publish_map()
        # self.save_map_to_file()

    def print_inflated_map(self):
        for y in range(self.map_array.shape[0]):
            for x in range(self.map_array.shape[1]):
                cost = self.map_array[y, x]
                if cost > 0:  # 코스트가 0보다 큰 셀만 출력
                    print(f"Cell ({x}, {y}) - Cost: {cost}")


def main(args=None):
    rclpy.init(args=args)
    node = MapUpdater()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
