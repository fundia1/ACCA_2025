import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import time
import math
from geometry_msgs.msg import Pose, Twist
import model_predictive_speed_and_steer_control as mpc


class FakeOdometryPublisher(Node):
    def __init__(self):
        super().__init__("fake_odometry_publisher")

        # 퍼블리셔 생성
        self.odom_publisher = self.create_publisher(Odometry, "/fake_odom", 10)

        # 초기 상태
        self.initial_state = mpc.State(x=0.0, y=0.0, yaw=0.0, v=0.0)

        # 시뮬레이션에 필요한 경로 정보 (예시 데이터)
        self.cx, self.cy, self.cyaw, self.ck = [], [], [], []
        self.sp = None

        # 시뮬레이션 반복
        self.timer = self.create_timer(0.1, self.timer_callback)  # 0.1초마다 실행
        self.simulation = None

        # Path 메시지 구독
        self.path_subscription = self.create_subscription(
            Path, "/parking_path2", self.path_callback, 10  # 경로 토픽
        )
        self.once = 1

    def path_callback(self, msg):
        # 경로의 각 PoseStamped를 처리하여 x, y, yaw, curvature 리스트 업데이트
        self.cx.clear()
        self.cy.clear()
        self.cyaw.clear()
        self.ck.clear()

        # 경로의 첫 번째 점을 초기 위치로 설정
        if msg.poses:
            first_pose = msg.poses[0].pose
            self.initial_state.x = first_pose.position.x
            self.initial_state.y = first_pose.position.y
            self.initial_state.yaw = self.get_yaw_from_quaternion(
                first_pose.orientation
            )

        prev_pose = None
        for pose_stamped in msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            yaw = self.get_yaw_from_quaternion(pose_stamped.pose.orientation)

            self.cx.append(x)
            self.cy.append(y)
            self.cyaw.append(yaw)

            # Curvature 계산 (단순하게 인접한 두 점을 이용한 방법)
            if prev_pose is not None:
                prev_x, prev_y = prev_pose
                dx = x - prev_x
                dy = y - prev_y
                distance = math.sqrt(dx**2 + dy**2)
                if distance != 0:
                    curvature = 1 / distance
                else:
                    curvature = 0.0
                self.ck.append(curvature)
            prev_pose = (x, y)

        # 속도 프로파일 계산
        self.sp = mpc.calc_speed_profile(self.cx, self.cy, self.cyaw, mpc.TARGET_SPEED)

    def timer_callback(self):
        # 시뮬레이션에서 현재 상태 얻기
        odom_msg = Odometry()
        if self.sp is not None and self.once == 1:

            for (
                state_x,
                state_y,
                state_yaw,
                state_v,
                time,
                steer,
                accel,
            ) in mpc.do_simulation(
                self.cx, self.cy, self.cyaw, self.ck, self.sp, 1.0, self.initial_state
            ):

                x, y, yaw, v, time, d, a = (
                    state_x,
                    state_y,
                    state_yaw,
                    state_v,
                    time,
                    steer,
                    accel,
                )

                # Odometry 메시지 생성

                # 로봇의 위치 설정
                odom_msg.header.stamp = self.get_clock().now().to_msg()
                odom_msg.header.frame_id = "map"

                # 로봇의 위치 (포즈)
                odom_msg.pose.pose.position.x = x
                odom_msg.pose.pose.position.y = y
                odom_msg.pose.pose.position.z = 0.0
                quat = self.euler_to_quaternion(yaw)  # yaw를 쿼터니언으로 변환
                odom_msg.pose.pose.orientation.x = quat[0]
                odom_msg.pose.pose.orientation.y = quat[1]
                odom_msg.pose.pose.orientation.z = quat[2]
                odom_msg.pose.pose.orientation.w = quat[3]

                # 로봇의 속도 (트위스트)
                odom_msg.twist.twist.linear.x = v
                odom_msg.twist.twist.angular.z = a  # 각속도

                # 퍼블리시
                self.odom_publisher.publish(odom_msg)

                self.get_logger().info(
                    f"Published Odometry: x={x}, y={y}, yaw={yaw}, v={v}, time={time}"
                )
            self.once += 1
        else:
            self.get_logger().info("Waiting for simulation to start...")

    def euler_to_quaternion(self, yaw):
        """Euler to Quaternion (yaw only)"""
        qx = 0.0
        qy = 0.0
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return [qx, qy, qz, qw]

    def get_yaw_from_quaternion(self, orientation):
        """Quaternion을 이용하여 yaw를 계산"""
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w
        # Quaternion to Euler (yaw)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw


def main(args=None):
    rclpy.init(args=args)
    fake_odometry_publisher = FakeOdometryPublisher()

    rclpy.spin(fake_odometry_publisher)  # 노드 실행

    fake_odometry_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
