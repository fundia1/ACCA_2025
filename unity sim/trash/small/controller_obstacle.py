import rclpy
from rclpy.node import Node

# 현재위치, goal위치
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler, euler_from_quaternion

import sys
import pathlib

sys.path.append(str(pathlib.Path(__file__).parent))


class Pose:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.yaw = 0


class ControllerObstacle(Node):
    def __init__(self):
        super().__init__("controller_obstcle")
        self.create_subscription(PoseStamped, "/goal_pose", self.goal_pose_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 타이머 시작
        self.i = 0

    def goal_pose_callback(self, msg):
        # Odometry 메시지에서 위치와 회전 정보를 추출
        position = msg.pose.position
        orientation = msg.pose.orientation

        # 골 위치
        self.goal.x = position.x
        self.goal.y = position.y

        # Quaternion을 yaw로 변환
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        self.goal.yaw = euler_from_quaternion(quaternion)[2]  # yaw (회전각)
        self.get_logger().info(
            f"Updated goal to {self.goal.x,self.goal.y,self.goal.yaw}."
        )

    def timer_callback(self):
        print(self.i)
        self.i += 1

    def stop_timer(self):
        """타이머를 강제로 멈추는 메서드"""
        if self.timer:
            self.timer.cancel()  # 타이머 취소
            self.get_logger().info("⚠️ Timer stopped.")
            self.destroy_node()  # 노드의 리소스를 정리
            rclpy.shutdown()  # 전체 ROS2 시스템 종료


def main(args=None):
    rclpy.init(args=args)

    # ControllerObstacle 클래스의 인스턴스를 생성합니다.
    controller = ControllerObstacle()

    # 타이머를 시작한 후 이벤트 루프를 실행합니다.
    rclpy.spin(controller)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
