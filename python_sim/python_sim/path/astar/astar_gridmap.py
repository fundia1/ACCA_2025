import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from nav_msgs.msg import Path
import math
import numpy as np
import heapq
from tf_transformations import quaternion_from_euler, euler_from_quaternion


class PathPlanner(Node):
    def __init__(self):
        super().__init__("path_planner_node")

        # 초기 위치와 목표 설정
        self.current_position = None
        self.goal_position = None
        self.map_array = None
        self.resolution = 0.1  # 맵 해상도 (m/cell)
        self.origin = (0.0, 0.0)  # 맵 원점 (x, y)

        # /odometry/navsat 서브스크라이브
        self.odom_subscription = self.create_subscription(
            Odometry, "/localization/kinematic_state", self.odom_callback, 10
        )

        # /goal_pose 서브스크라이브
        self.goal_subscription = self.create_subscription(
            PoseStamped, "/goal_pose", self.goal_callback, 10
        )

        # /map 서브스크라이브하여 맵 데이터 받기
        self.map_subscription = self.create_subscription(
            OccupancyGrid, "/static_map", self.map_callback, 10
        )

        # 경로 퍼블리시할 마커 퍼블리셔
        self.path_publisher = self.create_publisher(Path, "/path", 10)

    def map_callback(self, msg):
        # 맵 정보를 받으면 해상도와 원점 정보를 설정
        self.map_array = np.array(msg.data).reshape((msg.info.height, msg.info.width))

        self.resolution = msg.info.resolution

        # 맵의 원점 (x, y, yaw) 설정
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)

        # 쿼터니언을 euler 각도로 변환하여 yaw 값을 얻기
        orientation = msg.info.origin.orientation
        _, _, yaw = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )

        # yaw 값을 origin에 추가
        self.origin = (self.origin[0], self.origin[1], yaw)

    def odom_callback(self, msg):
        # 로봇의 현재 위치를 Odometry에서 가져오기
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

        # if (
        #     self.current_position is not None
        #     and self.goal_position is not None
        #     and self.map_array is not None
        # ):
        #     # 목표와 맵 데이터가 있을 때만 경로 찾기
        #     self.plan_path()

    def goal_callback(self, msg):
        # 목표 위치를 받아서 저장
        self.goal_position = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f"Goal position: {self.goal_position}")

        if self.current_position is not None and self.map_array is not None:
            # 목표와 맵 데이터가 있을 때만 경로 찾기
            self.plan_path()

    def plan_path(self):
        if self.map_array is None:
            self.get_logger().warn("Map data is not available yet.")
            return

        # A* 알고리즘을 사용하여 경로 찾기
        self.costmap = (
            self.map_array
        )  # self.convert_occupancygrid_to_costmap(self.map_array)

        # start와 goal을 map 좌표계로 변환
        start_idx = self.convert_to_map_coords(self.current_position)
        goal_idx = self.convert_to_map_coords(self.goal_position)

        print(start_idx, goal_idx)
        # A* 알고리즘 실행
        a_star = AStar(start_idx, goal_idx, self.costmap)
        path = a_star.run()
        if path:
            # self.get_logger().info(f"Path found: {path}")
            self.publish_path(path)
        else:
            self.get_logger().info("No path found.")

    def convert_occupancygrid_to_costmap(self, map_array):
        # 비어 있는 공간은 0, 점유된 공간은 100으로 설정
        costmap = np.full(map_array.shape, 0)  # 비어 있는 공간은 0
        costmap[map_array == 100] = 100  # 점유된 공간은 100 (장애물)
        return costmap

    def convert_to_map_coords(self, position):
        # (x, y) -> (map_x, map_y) 변환

        # 현재 위치에서 기준점 (self.origin) 이동
        print(self.origin)
        x, y = position
        x -= self.origin[0]
        y -= self.origin[1]

        # 맵의 회전 값 (yaw)
        yaw = self.origin[2]  # self.origin[2]는 맵의 yaw 값

        # 회전 변환 적용
        rotated_x = x * math.cos(yaw) + y * math.sin(yaw)
        rotated_y = -x * math.sin(yaw) + y * math.cos(yaw)

        # 회전된 좌표를 맵의 해상도에 맞게 변환
        map_x = int(rotated_x / self.resolution)
        map_y = int(rotated_y / self.resolution)

        # 맵의 경계를 벗어나지 않도록 인덱스 제한
        map_x = max(0, min(map_x, self.map_array.shape[1] - 1))
        map_y = max(0, min(map_y, self.map_array.shape[0] - 1))

        return (map_x, map_y)

    def publish_path(self, path):
        """
        축소된 좌표 경로를 self.origin과 self.resolution을 기준으로 map 프레임 경로로 변환 및 퍼블리시
        """
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        origin_x, origin_y, origin_yaw = self.origin
        resolution = self.resolution

        for i in range(len(path)):
            x, y = path[i]

            # 축소된 좌표를 map 프레임 좌표로 변환
            map_x = x * resolution
            map_y = y * resolution

            # origin에서 yaw 회전 적용 (월드 좌표 변환)
            rotated_x = math.cos(origin_yaw) * map_x - math.sin(origin_yaw) * map_y
            rotated_y = math.sin(origin_yaw) * map_x + math.cos(origin_yaw) * map_y

            world_x = rotated_x + origin_x
            world_y = rotated_y + origin_y

            # 방향 계산 (yaw)
            if i < len(path) - 1:  # 다음 점이 있는 경우
                next_x, next_y = path[i + 1]
                next_map_x = next_x * resolution
                next_map_y = next_y * resolution

                # 다음 점을 map 프레임으로 변환
                next_rotated_x = (
                    math.cos(origin_yaw) * next_map_x
                    - math.sin(origin_yaw) * next_map_y
                )
                next_rotated_y = (
                    math.sin(origin_yaw) * next_map_x
                    + math.cos(origin_yaw) * next_map_y
                )

                next_world_x = next_rotated_x + origin_x
                next_world_y = next_rotated_y + origin_y

                dx = next_world_x - world_x
                dy = next_world_y - world_y
            elif i > 0:  # 마지막 점인 경우 이전 점 사용
                prev_x, prev_y = path[i - 1]
                prev_map_x = prev_x * resolution
                prev_map_y = prev_y * resolution

                # 이전 점을 map 프레임으로 변환
                prev_rotated_x = (
                    math.cos(origin_yaw) * prev_map_x
                    - math.sin(origin_yaw) * prev_map_y
                )
                prev_rotated_y = (
                    math.sin(origin_yaw) * prev_map_x
                    + math.cos(origin_yaw) * prev_map_y
                )

                prev_world_x = prev_rotated_x + origin_x
                prev_world_y = prev_rotated_y + origin_y

                dx = world_x - prev_world_x
                dy = world_y - prev_world_y
            else:  # 경로에 단일 점만 있는 경우
                dx, dy = 1.0, 0.0  # 기본 yaw 설정

            yaw = math.atan2(dy, dx)

            # 쿼터니언으로 변환
            quaternion = quaternion_from_euler(0, 0, yaw)

            # Pose 생성
            pose = Pose()
            pose.position = Point(x=world_x, y=world_y, z=0.0)
            pose.orientation = Quaternion(
                x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3]
            )

            # PoseStamped 생성 및 path에 추가
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            pose_stamped.pose = pose
            path_msg.poses.append(pose_stamped)

        # Path 퍼블리시
        self.path_publisher.publish(path_msg)


class AStar:
    def __init__(self, start, goal, costmap):
        self.start = start  # 배열 형식
        self.goal = goal
        self.costmap = costmap
        self.rows = len(costmap[0])
        self.cols = len(costmap)
        print(self.rows, self.cols)
        self.open_list = []
        self.closed_list = set()
        self.came_from = {}

        # 우선순위 큐에 시작 노드 추가
        heapq.heappush(self.open_list, (0, start))
        self.g_score = {start: 0}
        self.f_score = {start: self.heuristic(start)}

    def heuristic(self, point):
        # 휴리스틱: 맨해튼 거리 계산
        return abs(point[0] - self.goal[0]) + abs(point[1] - self.goal[1])

    def get_neighbors(self, current):
        """주변 노드 가져오기."""
        neighbors = []
        directions = [
            (-1, 0),
            (1, 0),  # 상, 하
            (0, -1),
            (0, 1),  # 좌, 우
            (-1, -1),
            (-1, 1),  # 대각선 위
            (1, -1),
            (1, 1),  # 대각선 아래
        ]
        for d in directions:
            new_row, new_col = current[0] + d[0], current[1] + d[1]
            if (
                0 <= new_row < self.rows
                and 0 <= new_col < self.cols
                and self.costmap[new_col][new_row] != 100  # 장애물이 아닌 경우
            ):
                neighbors.append((new_row, new_col))
        return neighbors

    def run(self):
        while self.open_list:
            _, current = heapq.heappop(self.open_list)

            # 디버깅: 현재 노드 확인
            # print(f"Current Node: {current}")

            if current == self.goal:
                return self.reconstruct_path(current)

            for neighbor in self.get_neighbors(current):
                tentative_g_score = self.g_score[current] + 1

                # 디버깅: 이웃 확인
                # print(f"Neighbor: {neighbor}, G-Score: {tentative_g_score}")

                if (
                    neighbor not in self.g_score
                    or tentative_g_score < self.g_score[neighbor]
                ):
                    self.came_from[neighbor] = current
                    self.g_score[neighbor] = tentative_g_score
                    self.f_score[neighbor] = tentative_g_score + self.heuristic(
                        neighbor
                    )
                    heapq.heappush(self.open_list, (self.f_score[neighbor], neighbor))
        print("No path found.")  # 경로가 없을 때
        return False

    def reconstruct_path(self, current):
        """경로 복원."""
        path = [current]
        while current in self.came_from:
            current = self.came_from[current]
            path.append(current)
        return path[::-1]


def main():
    rclpy.init()

    # PathPlanner 노드 생성
    path_planner = PathPlanner()

    # 노드를 실행
    rclpy.spin(path_planner)

    # 종료 시 clean up
    path_planner.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
