import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import sqlite3
import os
import math


class StateSelecter(Node):
    def __init__(self):
        super().__init__("state_selecter")

        self.global_path_pub = self.create_publisher(Path, "/global_path", 10)

        self.global_x = []
        self.global_y = []
        self.global_yaw = []

        # 데이터베이스 연결
        self.db_path = os.path.join(os.path.dirname(__file__), "BS_final.db")
        self.conn = sqlite3.connect(self.db_path)
        self.cursor = self.conn.cursor()

        # 초기 데이터 로드 및 퍼블리시
        self.load_data()
        self.publish_data()

        # 타이머 시작 (0.1초마다 퍼블리시)
        self.timer = self.create_timer(0.1, self.publish_data)

    def load_data(self):
        """현재 ID에 맞는 모든 데이터 로드"""
        self.cursor.execute("SELECT  x, y, yaw FROM Path ")
        results = self.cursor.fetchall()

        if results:

            self.global_x = [row[0] for row in results]
            self.global_y = [row[1] for row in results]
            self.global_yaw = [row[2] for row in results]
        else:
            self.get_logger().warn(f"No data found for id: {0}")

    def yaw_to_quaternion(self, yaw):
        """yaw 값을 쿼터니언으로 변환"""
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return (0.0, 0.0, qz, qw)

    def publish_data(self):
        """global_path와 current_mission을 퍼블리시"""
        if self.global_x and self.global_y and self.global_yaw:
            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = "map"

            for x, y, yaw in zip(self.global_x, self.global_y, self.global_yaw):
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

            self.global_path_pub.publish(path_msg)

    def __del__(self):
        """객체 삭제 시 데이터베이스 연결 종료"""
        self.conn.close()


def main(args=None):
    rclpy.init(args=args)
    node = StateSelecter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
