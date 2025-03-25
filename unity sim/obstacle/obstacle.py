import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
import math
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np

from fuzzy import RobotNavigation


class State:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.v = 0


class Obstacle(Node):
    def __init__(self):
        super().__init__("avoid_path_publisher")

        # 구독
        self.create_subscription(String, "/current_mission", self.mission_callback, 10)
        self.create_subscription(Path, "/global_path", self.global_path_callback, 10)
        self.create_subscription(MarkerArray, "/markers", self.obstacle_callback, 10)
        self.create_subscription(
            Odometry, "/localization/kinematic_state", self.odom_callback, 10
        )
        # 퍼블리셔
        self.local_path_pub = self.create_publisher(Path, "/local_path", 10)

        self.state = ""
        self.global_x = []
        self.global_y = []
        self.global_yaw = []

        self.local_x = []
        self.local_y = []
        self.local_yaw = []

        self.start = []
        self.goal = []
        self.obstacles = []
        self.threshold = 0.5

        self.odom = State()

        self.fuzzy = RobotNavigation()
        self.x_min = None
        self.x_max = None
        self.y_min = None
        self.y_max = None

        self.timer = None  # 타이머 초기화 (아직 실행 안 함)

    def global_path_callback(self, msg):
        """global_path를 받아서 저장"""
        if len(self.global_x) == 0 or self.global_x[0] != msg.poses[0].pose.position.x:
            self.global_x = []
            self.global_y = []
            self.global_yaw = []

            for pose in msg.poses:
                self.global_x.append(pose.pose.position.x)
                self.global_y.append(pose.pose.position.y)

                # orientation을 yaw로 변환
                qx, qy, qz, qw = (
                    pose.pose.orientation.x,
                    pose.pose.orientation.y,
                    pose.pose.orientation.z,
                    pose.pose.orientation.w,
                )
                yaw = self.quaternion_to_yaw(qx, qy, qz, qw)
                self.global_yaw.append(yaw)

            self.start = np.array([self.global_x[0], self.global_y[0]])
            self.goal = np.array([self.global_x[-1], self.global_y[-1]])

            # 방향 벡터 (start -> goal)
            direction = self.goal - self.start
            direction /= np.linalg.norm(direction)  # 정규화

            # 90도 회전한 법선 벡터 (좌측)
            normal = np.array([direction[1], -direction[0]])

            # 2m 떨어진 사각형 좌표
            left_start = self.start + normal * 0.5  # 왼쪽 시작점
            right_start = self.start - normal * 0.5  # 오른쪽 시작점
            left_goal = self.goal + normal * 0.5  # 왼쪽 목표점
            right_goal = self.goal - normal * 0.5  # 오른쪽 목표점

            # 경계값 설정
            self.x_min = min(left_start[0], right_start[0], left_goal[0], right_goal[0])
            self.x_max = max(left_start[0], right_start[0], left_goal[0], right_goal[0])
            self.y_min = min(left_start[1], right_start[1], left_goal[1], right_goal[1])
            self.y_max = max(left_start[1], right_start[1], left_goal[1], right_goal[1])

    def mission_callback(self, msg):
        self.state = msg.data
        if self.state == "obstacle":
            self.run_avoid_path()
            self.start_timer()
        else:
            self.stop_timer()

    def odom_callback(self, msg):
        # Odometry 메시지에서 위치와 회전 정보를 추출
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        # 로봇의 현재 위치
        self.odom.x = position.x
        self.odom.y = position.y

        # Quaternion을 yaw로 변환
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        self.odom.yaw = euler_from_quaternion(quaternion)[2]  # yaw (회전각)
        """미션 상태 업데이트"""

    def obstacle_callback(self, msg):
        # 프레임 맵으로 변환 해야함
        if msg.markers[0].header.frame_id != "map":
            print(msg.markers[0].header.frame_id)
            # MarkerArray 데이터 처리
            for marker in msg.markers:
                for point in marker.points:
                    map_x, map_y = self.velodyne_to_map(point.x, point.y)
                    self.add_obstacle(map_x, map_y)
        else:

            for marker in msg.markers:
                point = marker.pose.position
                self.add_obstacle(point.x, point.y)

    def velodyne_to_map(self, x, y):
        """Velodyne 기준 좌표를 map 기준 좌표로 변환"""
        robot_x, robot_y, robot_yaw = self.robot
        x += 1
        # 벨로다인 좌표를 로봇 기준에서 맵 기준으로 변환
        map_x = robot_x + x * math.cos(robot_yaw) - y * math.sin(robot_yaw)
        map_y = robot_y + x * math.sin(robot_yaw) + y * math.cos(robot_yaw)

        return map_x, map_y

    def add_obstacle(self, x, y, threshold=0.1):
        # 기존 장애물들과 비교하여 가까운 점들은 필터링
        new_obstacle = [x, y]  # 리스트 형태 유지
        if not self.obstacles:
            self.obstacles.append(new_obstacle)
        else:
            for obs in self.obstacles:
                if np.linalg.norm(np.array(new_obstacle) - np.array(obs)) < threshold:
                    return  # 가까운 점이 있는 경우 추가하지 않음
            self.obstacles.append(new_obstacle)

    def run_avoid_path(self):
        if len(self.start) != 0:

            self.fuzzy.initialize(
                self.start,
                self.goal,
                self.obstacles,
                -100,
                100,
                -100,
                100,
            )

            _, path_smooth = self.fuzzy.navigate(step_size=0.7)

            self.local_x = []
            self.local_y = []
            self.local_yaw = []

            self.local_x = [point[0] for point in path_smooth]
            self.local_y = [point[1] for point in path_smooth]
            for i in range(1, len(path_smooth)):
                dx = path_smooth[i][0] - path_smooth[i - 1][0]
                dy = path_smooth[i][1] - path_smooth[i - 1][1]
                yaw = math.atan2(dy, dx)
                self.local_yaw.append(yaw)

            # 첫 점의 yaw는 두 번째 점과 동일하게 설정
            if self.local_yaw:
                self.local_yaw.insert(0, self.local_yaw[0])

            # locl_path_publish
            # print(len(self.local_x))
            self.publish_path()

    def start_timer(self):
        """타이머 시작 (이미 실행 중이면 무시)"""
        if self.timer is None:
            self.timer = self.create_timer(0.001, self.timercallback)

    def stop_timer(self):
        """타이머 정지"""
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None

    def timercallback(self):
        """local_path를 퍼블리시"""
        if self.state != "obstacle":
            return  # 현재 미션이 driving이 아니면 퍼블리시 안 함

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"

        if len(self.local_x) > 0:
            x_list, y_list, yaw_list = self.local_x, self.local_y, self.local_yaw
        elif len(self.global_x) > 0:
            x_list, y_list, yaw_list = self.global_x, self.global_y, self.global_yaw
        else:
            return  # 퍼블리시할 데이터가 없으면 종료

        for x, y, yaw in zip(x_list, y_list, yaw_list):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            qx, qy, qz, qw = self.yaw_to_quaternion(yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            path_msg.poses.append(pose)

        self.local_path_pub.publish(path_msg)

    def publish_path(self):
        if self.state != "obstacle":
            return  # 현재 미션이 driving이 아니면 퍼블리시 안 함

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"

        if len(self.local_x) > 0:
            x_list, y_list, yaw_list = self.local_x, self.local_y, self.local_yaw
        elif len(self.global_x) > 0:
            x_list, y_list, yaw_list = self.global_x, self.global_y, self.global_yaw
        else:
            return  # 퍼블리시할 데이터가 없으면 종료

        for x, y, yaw in zip(x_list, y_list, yaw_list):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            qx, qy, qz, qw = self.yaw_to_quaternion(yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            path_msg.poses.append(pose)

        self.local_path_pub.publish(path_msg)

    def quaternion_to_yaw(self, qx, qy, qz, qw):
        """쿼터니언을 yaw로 변환"""
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)

    def yaw_to_quaternion(self, yaw):
        """yaw를 쿼터니언으로 변환"""
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return (0.0, 0.0, qz, qw)


def main(args=None):
    rclpy.init(args=args)
    node = Obstacle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
