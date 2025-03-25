import rclpy
from rclpy.node import Node

# 현재위치,goal위치
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


class StateMachine(Node):
    def __init__(self):
        super().__init__("controller_obstcle")

        self.create_subscription(
            Odometry, "/localization/kinematic_state", self.odom_callback, 10
        )
        self.create_subscription(PoseStamped, "/goal_pose", self.goal_pose_callback, 10)
        self.create_timer(0.1, self.timer_callback)
        self.odom = Pose()
        self.goal = Pose()

        self.mission = ["drive", "obs", "parking", "delivery"]
        self.state = self.mission[0]  # 초기 상태 드라이브

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
        pass


def main(args=None):
    rclpy.init(args=args)
    node = StateMachine()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
