import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, PoseArray
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
import math
import heapq
import numpy as np
from tf_transformations import (
    quaternion_from_euler,
    euler_from_quaternion,
    rotation_matrix,
)
import yaml
import sys
import pathlib
import os
from std_msgs.msg import Int16

os.chdir("/home/jinju/ws/src/python_sim/python_sim/path/parking")

sys.path.append(str(pathlib.Path(__file__)))
import hybrid_a_star as hy

show_animation = False


class ParkingHybridAStar(Node):
    def __init__(self):
        super().__init__("parking_path_node")

        self.yaml_file = "goal_point.yaml"

        self.current_position = None
        self.goal_position = None
        self.mid_position = None
        self.map_array = None
        self.resolution = 0.1  # 맵 해상도 (m/cell)
        self.origin = (0.0, 0.0)  # 맵 원점 (x, y)

        self.once = 1

        # /odometry/navsat 서브스크라이브
        self.odom_subscription = self.create_subscription(
            Odometry, "/localization/kinematic_state", self.odom_callback, 10
        )

        # /map 서브스크라이브하여 맵 데이터 받기
        self.map_subscription = self.create_subscription(
            OccupancyGrid, "/static_map", self.map_callback, 10
        )

        self.goal_num_sub = self.create_subscription(
            Int16, "/goal_num", self.goal_num_callback, 10
        )

        self.marker_pub = self.create_publisher(
            MarkerArray, "/parking_path_markers", 10
        )

        # PoseArray 퍼블리셔
        self.posearray_pub = self.create_publisher(
            PoseArray, "/parking_path_posearray", 10
        )

        self.parking_pub = self.create_publisher(Path, "/parking_path2", 10)
        # self.timer = self.create_timer(0.1, self.timer_callback)

    def goal_num_callback(self, msg):
        with open(self.yaml_file, "r") as file:
            self.goal_matadatas = yaml.safe_load(file)

        self.goal_position = self.goal_matadatas[f"goal_{msg.data}"]
        self.mid_position = self.goal_matadatas[f"h_goal_{msg.data}"]
        self.once_callback()

    def odom_callback(self, msg):
        # 로봇의 현재 위치를 Odometry에서 가져오기
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        current_ori = msg.pose.pose.orientation
        _, _, self.current_angle = euler_from_quaternion(
            [current_ori.x, current_ori.y, current_ori.z, current_ori.w]
        )

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

    def convert_occupancygrid_to_obstlist(self):
        ox, oy = [], []  # 장애물 좌표를 저장할 리스트 초기화

        # map_array에서 점유된 공간을 찾아 ox, oy에 좌표 추가
        for i in range(self.map_array.shape[0]):
            for j in range(self.map_array.shape[1]):
                if self.map_array[i, j] == 100:  # 점유된 공간
                    ox.append(i)  # 장애물 x 좌표
                    oy.append(j)  # 장애물 y 좌표

        return ox, oy  # 점유된 장애물의 좌표들 반환

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

    def once_callback(self):
        if self.goal_position is not None:
            start_x, start_y = self.convert_to_map_coords(self.current_position)
            goal_x, goal_y = self.convert_to_map_coords(self.goal_position)
            mid_x, mid_y = self.convert_to_map_coords(self.mid_position)
            oy, ox = self.convert_occupancygrid_to_obstlist()

            start = [start_x, start_y, np.deg2rad(90.0)]
            mid = [mid_x, mid_y, np.deg2rad(90.0)]
            goal = [goal_x, goal_y, np.deg2rad(90.0)]

            path = hy.hybrid_a_star_planning(
                start, mid, ox, oy, hy.XY_GRID_RESOLUTION, hy.YAW_GRID_RESOLUTION
            )

            x = path.x_list
            y = path.y_list
            yaw = path.yaw_list

            path = hy.hybrid_a_star_planning(
                mid, goal, ox, oy, hy.XY_GRID_RESOLUTION, hy.YAW_GRID_RESOLUTION
            )

            x.extend(path.x_list)
            y.extend(path.y_list)
            yaw.extend(path.yaw_list)

            self.publish_parking_path(x, y, yaw)

    # def publish_parking_path(self, x_list, y_list, yaw_list):
    #     # Path 메시지 초기화
    #     path_msg = Path()
    #     path_msg.header.frame_id = "map"  # 프레임 ID 설정
    #     path_msg.header.stamp = self.get_clock().now().to_msg()

    #     # 원점 yaw를 기준으로 회전 행렬 생성
    #     origin_yaw = self.origin[2]
    #     rot_mat = rotation_matrix(origin_yaw, [0, 0, 1])[:2, :2]  # 2D 회전 행렬

    #     # 각 (x, y, yaw) 값을 PoseStamped로 변환
    #     for x, y, yaw in zip(x_list, y_list, yaw_list):
    #         pose = PoseStamped()
    #         pose.header.frame_id = "map"
    #         pose.header.stamp = self.get_clock().now().to_msg()

    #         # (x, y)를 0.1배 축소하고, 원점 좌표 기준으로 회전 및 이동
    #         scaled_x = x * 0.1
    #         scaled_y = y * 0.1
    #         transformed_pos = rot_mat @ [scaled_x, scaled_y]  # 원점 yaw 기준 회전
    #         transformed_x = self.origin[0] + transformed_pos[0]
    #         transformed_y = self.origin[1] + transformed_pos[1]

    #         # yaw를 원점 yaw와 합산
    #         transformed_yaw = yaw + origin_yaw

    #         # 위치 설정
    #         pose.pose.position.x = transformed_x
    #         pose.pose.position.y = transformed_y
    #         pose.pose.position.z = 0.0  # z는 2D 경로이므로 0

    #         # Orientation 설정 (yaw를 quaternion으로 변환)
    #         quaternion = quaternion_from_euler(0, 0, transformed_yaw)
    #         pose.pose.orientation.x = quaternion[0]
    #         pose.pose.orientation.y = quaternion[1]
    #         pose.pose.orientation.z = quaternion[2]
    #         pose.pose.orientation.w = quaternion[3]

    #         # Path 메시지에 PoseStamped 추가
    #         path_msg.poses.append(pose)

    #     # Path 퍼블리시
    #     self.parking_pub.publish(path_msg)
    #     self.get_logger().info("Published parking path")

    def publish_parking_path(self, x_list, y_list, yaw_list):
        # Path 메시지 초기화
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        # MarkerArray 초기화
        marker_array = MarkerArray()

        # 원점 yaw를 기준으로 회전 행렬 생성
        origin_yaw = self.origin[2]
        rot_mat = rotation_matrix(origin_yaw, [0, 0, 1])[:2, :2]  # 2D 회전 행렬

        for i in range(len(x_list) - 1):  # 각 점과 그 다음 점을 비교
            # 현재 점과 다음 점의 좌표와 yaw
            x, y, yaw = x_list[i], y_list[i], yaw_list[i]
            next_x, next_y = x_list[i + 1], y_list[i + 1]

            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()

            # (x, y)를 0.1배 축소하고, 원점 좌표 기준으로 회전 및 이동
            scaled_x = x * 0.1
            scaled_y = y * 0.1
            transformed_pos = rot_mat @ [scaled_x, scaled_y]
            transformed_x = self.origin[0] + transformed_pos[0]
            transformed_y = self.origin[1] + transformed_pos[1]

            # yaw를 원점 yaw와 합산
            transformed_yaw = yaw + origin_yaw

            # 위치 설정
            pose.pose.position.x = transformed_x
            pose.pose.position.y = transformed_y
            pose.pose.position.z = 0.0

            # Orientation 설정
            quaternion = quaternion_from_euler(0, 0, transformed_yaw)
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]

            path_msg.poses.append(pose)

            # Marker 생성
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "parking_path"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.pose.position.x = transformed_x
            marker.pose.position.y = transformed_y
            marker.pose.position.z = 0.0
            marker.pose.orientation = pose.pose.orientation

            # 두 점 간 이동 벡터 계산
            move_vec = np.array([next_x - x, next_y - y])
            move_vec = move_vec / np.linalg.norm(move_vec)  # 정규화

            # yaw를 단위 벡터로 변환
            yaw_vec = np.array([np.cos(yaw), np.sin(yaw)])

            # 이동 방향과 yaw 방향 비교 (내적 사용)
            dot_product = np.dot(move_vec, yaw_vec)
            if dot_product > 0:  # 직진
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
            else:  # 후진
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0

            marker_array.markers.append(marker)

        # Path 퍼블리시
        self.parking_pub.publish(path_msg)

        # MarkerArray 퍼블리시
        self.marker_pub.publish(marker_array)

        self.get_logger().info("Published parking path and markers")


def main():
    rclpy.init()

    # PathPlanner 노드 생성
    path_planner = ParkingHybridAStar()

    # 노드를 실행
    rclpy.spin(path_planner)

    # 종료 시 clean up
    path_planner.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

# def main():
#     print("Start Hybrid A* planning")

#     ox, oy = [], []

#     for i in range(60):
#         ox.append(i)
#         oy.append(0.0)
#     for i in range(60):
#         ox.append(60.0)
#         oy.append(i)
#     for i in range(61):
#         ox.append(i)
#         oy.append(60.0)
#     for i in range(61):
#         ox.append(0.0)
#         oy.append(i)
#     for i in range(40):
#         ox.append(20.0)
#         oy.append(i)
#     for i in range(40):
#         ox.append(40.0)
#         oy.append(60.0 - i)

#     # Set Initial parameters
#     start = [10.0, 10.0, np.deg2rad(90.0)]
#     goal = [30.0, 50.0, np.deg2rad(90.0)]

#     print("start : ", start)
#     print("goal : ", goal)

#     if show_animation:
#         plt.plot(ox, oy, ".k")
#         rs.plot_arrow(start[0], start[1], start[2], fc="g")
#         rs.plot_arrow(goal[0], goal[1], goal[2])

#         plt.grid(True)
#         plt.axis("equal")

#     path = hybrid_a_star_planning(
#         start, goal, ox, oy, XY_GRID_RESOLUTION, YAW_GRID_RESOLUTION
#     )

#     x = path.x_list
#     y = path.y_list
#     yaw = path.yaw_list

#     if show_animation:
#         for i_x, i_y, i_yaw in zip(x, y, yaw):
#             plt.cla()
#             plt.plot(ox, oy, ".k")
#             plt.plot(x, y, "-r", label="Hybrid A* path")
#             plt.grid(True)
#             plt.axis("equal")
#             plot_car(i_x, i_y, i_yaw)
#             plt.pause(0.1)

#     print(__file__ + " done!!")


# if __name__ == "__main__":
#     main()
