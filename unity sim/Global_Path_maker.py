import rclpy
from rclpy.node import Node
import sqlite3
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math

import os
import yaml
import sys


class Pose:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.yaw = 0


class PathMaker(Node):
    def __init__(self):
        super().__init__("global_path_maker")

        self.create_subscription(
            Odometry, "/localization/kinematic_state", self.odom_callback, 10
        )
        self.create_subscription(PoseStamped, "/goal_pose", self.goal_pose_callback, 10)
        self.create_timer(0.1, self.timer_callback)

        self.missions = self.load_missions_from_yaml("missions.yaml")

        self.odom = Pose()
        self.state = self.missions[0]
        self.current_id = 1  # 초기 ID 설정

        self.prev_x = None  # 이전 x 좌표
        self.prev_y = None  # 이전 y 좌표
        self.move_threshold = 0.05  # 저장 최소 이동 거리 (5cm)

        # 데이터베이스 초기화
        self.init_db()

    def load_missions_from_yaml(self, file_name):
        """YAML 파일에서 미션 목록을 로드하는 함수"""
        script_dir = os.path.dirname(
            os.path.abspath(sys.argv[0])
        )  # 현재 실행되는 스크립트의 디렉토리 경로
        file_path = os.path.join(script_dir, file_name)
        try:
            with open(file_path, "r") as file:
                data = yaml.safe_load(file)
                return data["missions"]  # 'missions' 항목 반환
        except Exception as e:
            self.get_logger().error(f"⚠️ Error loading missions from YAML: {e}")
            return []

    def init_db(self):
        """SQLite 데이터베이스 초기화"""
        db_path = os.path.join(os.path.dirname(__file__), "U-Turn.db")
        self.conn = sqlite3.connect(db_path)
        self.cursor = self.conn.cursor()

        # 로봇 상태 저장 테이블
        self.cursor.execute(
            """
            CREATE TABLE IF NOT EXISTS robot_state (
                id INTEGER ,
                state TEXT,
                x REAL,
                y REAL,
                yaw REAL
            )
        """
        )

        # 가장 큰 id 값을 가져와 current_id 초기화
        self.cursor.execute("SELECT MAX(id) FROM robot_state")
        result = self.cursor.fetchone()[0]
        if result:
            self.current_id = result + 1  # 기존 ID가 있으면 다음 ID부터 시작
            try:
                self.state = self.missions[self.current_id - 1]
            except:
                self.state = "driving"
        self.conn.commit()

    def save_to_db(self):
        """로봇의 현재 상태를 데이터베이스에 저장"""
        self.cursor.execute(
            """
            INSERT INTO robot_state (id, state, x, y, yaw) 
            VALUES (?, ?, ?, ?, ?)
            """,
            (self.current_id, self.state, self.odom.x, self.odom.y, self.odom.yaw),
        )
        self.conn.commit()

    def has_moved_significantly(self, x, y):
        """이전 위치와 비교하여 일정 거리 이상 이동했는지 확인"""
        if self.prev_x is None or self.prev_y is None:
            return True  # 초기 상태에서는 저장

        distance = math.sqrt((x - self.prev_x) ** 2 + (y - self.prev_y) ** 2)
        return distance > self.move_threshold  # 최소 이동 거리 초과 여부 반환

    def odom_callback(self, msg):
        """Odometry 메시지에서 위치와 방향 업데이트"""
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        x, y = position.x, position.y
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        yaw = euler_from_quaternion(quaternion)[2]

        # 일정 거리 이상 이동했을 때만 저장
        if self.has_moved_significantly(x, y):
            self.odom.x = x
            self.odom.y = y
            self.odom.yaw = yaw
            self.save_to_db()

            # 이전 위치 업데이트
            self.prev_x = x
            self.prev_y = y

    def goal_pose_callback(self, msg):
        """목표 지점이 들어오면 id += 1"""
        self.current_id += 1
        try:
            self.state = self.missions[self.current_id - 1]
        except:
            self.state = "Null"

    def timer_callback(self):
        """로깅 출력"""
        self.get_logger().info(
            f"ID: {self.current_id}, State: {self.state}, 위치: ({self.odom.x}, {self.odom.y}), Yaw: {self.odom.yaw}"
        )

    def __del__(self):
        """객체 삭제 시 데이터베이스 연결 종료"""
        self.conn.close()


def main(args=None):
    rclpy.init(args=args)
    node = PathMaker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
