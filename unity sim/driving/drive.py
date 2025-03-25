import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math


class NomalDrive(Node):
    def __init__(self):
        super().__init__("driving_path_publisher")

        # 구독
        self.create_subscription(String, "/current_mission", self.mission_callback, 10)
        self.create_subscription(Path, "/global_path", self.global_path_callback, 10)

        # 퍼블리셔
        self.local_path_pub = self.create_publisher(Path, "/local_path", 10)

        self.state = ""
        self.global_x = []
        self.global_y = []
        self.global_yaw = []

        self.local_x = []
        self.local_y = []
        self.local_yaw = []

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

    def mission_callback(self, msg):
        """미션 상태 업데이트"""

        self.state = msg.data
        if self.state == "driving":
            self.run_driving_path()
            self.start_timer()
        else:
            self.stop_timer()

    def run_driving_path(self):
        """global path를 local path로 설정"""
        self.local_x = self.global_x[:]
        self.local_y = self.global_y[:]
        self.local_yaw = self.global_yaw[:]
        # print(self.state, len(self.global_x[:]))

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
        if self.state != "driving":
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
    node = NomalDrive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
