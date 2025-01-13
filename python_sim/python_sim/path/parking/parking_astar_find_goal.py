import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from nav_msgs.msg import Path
import math
import numpy as np
import heapq
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import yaml
import reeds_shepp_planner as rs
import os

os.chdir("/home/jinju/ws/src/python_sim/python_sim/path/parking")


class PathPlanner(Node):
    def __init__(self):
        super().__init__("path_planner_node")

        self.yaml_file = "goal_point.yaml"

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

        # /map 서브스크라이브하여 맵 데이터 받기
        self.map_subscription = self.create_subscription(
            OccupancyGrid, "/static_map", self.map_callback, 10
        )

        self.parking_pub = self.create_publisher(Path, "/parking_path", 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.goal_1 = True
        self.goal_2 = True
        self.goal_3 = True

        self.find_path = False

        self.goal_num = 0
        self.once = 1

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

    def plan_path(self, num):
        if self.map_array is None:
            self.get_logger().warn("Map data is not available yet.")
            return True

        # A* 알고리즘을 사용하여 경로 찾기
        self.costmap = (
            self.map_array
        )  # self.convert_occupancygrid_to_costmap(self.map_array)

        # start와 goal을 map 좌표계로 변환
        start_idx = self.convert_to_map_coords(self.current_position)
        goal_idx = self.convert_to_map_coords(self.goal_position)

        # print(start_idx, goal_idx)
        # A* 알고리즘 실행
        a_star = AStar(start_idx, goal_idx, self.costmap)
        path = a_star.run()
        if path:
            # self.get_logger().info(f"number {num} found.")
            return True

        else:
            # self.get_logger().info(f"number {num} No path found.")
            return False

    def convert_occupancygrid_to_costmap(self, map_array):
        # 비어 있는 공간은 0, 점유된 공간은 100으로 설정
        costmap = np.full(map_array.shape, 0)  # 비어 있는 공간은 0
        costmap[map_array == 100] = 100  # 점유된 공간은 100 (장애물)
        return costmap

    def convert_to_map_coords(self, position):
        # (x, y) -> (map_x, map_y) 변환

        # 현재 위치에서 기준점 (self.origin) 이동
        # print(self.origin)
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

    def timer_callback(self):
        if self.current_position is not None and not self.find_path:
            if self.goal_1:
                self.load_data(1)
                self.goal_1 = self.plan_path(1)
            if self.goal_2:
                self.load_data(2)
                self.goal_2 = self.plan_path(2)
            if self.goal_3:
                self.load_data(3)
                self.goal_3 = self.plan_path(3)

            if sum((self.goal_1, self.goal_2, self.goal_3)) == 1:
                if self.goal_1:
                    self.goal_num = 1
                    print("goal 1")
                if self.goal_2:
                    self.goal_num = 2
                    print("goal_2")
                if self.goal_3:
                    self.goal_num = 3
                    print("goal_3")
                self.find_path = True
        if self.find_path and self.once == 1:
            self.make_goal()
            self.once += 1

    def load_data(self, num):
        with open(self.yaml_file, "r") as file:
            self.goal_matadatas = yaml.safe_load(file)

        self.goal_position = self.goal_matadatas[f"goal_{num}"]

    def make_goal(self):
        path_x = []
        path_y = []
        path_yaw = []

        min_radius = 2

        step_size = 0.1

        pts = [
            (35.4294, 0.0211545, 3.131),
            (30.602, 0.266457, -2.93609),
            (34.9868, 3.30065, 2.95774),
            (31.1231, 0.59355, 3.131),
            (25.333, 1.67201, -2.93609),
            (29.2895, 4.48971, 2.95774),
        ]
        for i in range(2):

            start_x = pts[i][0]
            start_y = pts[i][1]
            start_yaw = pts[i][2]

            end_x = pts[i + 1][0]
            end_y = pts[i + 1][1]
            end_yaw = pts[i + 1][2]

            px, py, pyaw, mode, clen = rs.reeds_shepp_planner(
                start_x,
                start_y,
                start_yaw,
                end_x,
                end_y,
                end_yaw,
                min_radius,
                step_size,
            )
            path_x.extend(px)
            path_y.extend(py)
            path_yaw.extend(pyaw)
        self.publish_parking_path(path_x, path_y, path_yaw)

    def publish_parking_path(self, px, py, pyaw):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"  # 사용 중인 좌표계 프레임 ID 설정

        # px, py, pyaw를 사용하여 Path 메시지에 포즈 추가
        for x, y, yaw in zip(px, py, pyaw):
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            # yaw를 quaternion으로 변환하여 orientation 설정
            orientation = self.yaw_to_quaternion(yaw)
            pose.pose.orientation = orientation

            path_msg.poses.append(pose)

        # Path 메시지 퍼블리시
        self.parking_pub.publish(path_msg)
        self.get_logger().info("Path published")

    def yaw_to_quaternion(self, yaw):
        """
        Converts a yaw angle (in radians) to a Quaternion message.
        """
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q


class AStar:
    def __init__(self, start, goal, costmap):
        self.start = start  # 배열 형식
        self.goal = goal
        self.costmap = costmap
        self.rows = len(costmap[0])
        self.cols = len(costmap)
        # print(self.rows, self.cols)
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
            # (0, -1),
            (0, 1),  # 좌, 우
            # (-1, -1),
            (-1, 1),  # 대각선 위
            # (1, -1),
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
        # print("No path found.")  # 경로가 없을 때
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
