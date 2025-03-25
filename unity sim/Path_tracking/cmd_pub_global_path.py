import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from erp42_msgs.msg import ControlMessage
from math import *
import sys
import pathlib
import os
import numpy as np
import time
from std_msgs.msg import String, Bool

sys.path.append(str(pathlib.Path(__file__)))
from stanley import Stanley


# class Speed_Supporter:
#     def __init__(self):
#         self.M_speed = 3
#         self.m_speed = 2

#     def get_Max_min_speed(self, state):
#         if state == "driving":
#             self.M_speed = 3
#             self.m_speed = 2


class State:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.v = 8


class Pathtracking(Node):
    def __init__(self):
        super().__init__("path_tracker_node")

        # Odometry subscription (for robot localization)
        self.odom_subscription = self.create_subscription(
            Odometry, "/localization/kinematic_state", self.odom_callback, 10
        )

        # Path subscription
        self.parking_sub = self.create_subscription(
            Path, "/global_path", self.path_callback, 10
        )

        self.mission_publisher = self.create_publisher(Bool, "/mission_cleared", 10)

        self.control_publisher = self.create_publisher(ControlMessage, "/erp42_cmd", 10)

        # Timer callback to compute control messages
        self.timer = self.create_timer(0.001, self.timer_callback)

        self.state = State()
        # Initialize variables
        self.local_x = []
        self.local_y = []
        self.local_yaw = []
        self.local_dir = []  # 전진(True) 또는 후진(False) 여부 저장
        self.target_idx = 0
        self.velocity = 0.0
        self.st = Stanley()  # Stanley 컨트롤러 객체 초기화

        self.prev_path = []

    def odom_callback(self, msg):
        # 현재 로봇의 위치 및 자세를 업데이트
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        # Quaternion을 yaw 각도로 변환
        _, _, yaw = self.euler_from_quaternion(
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        self.state.x = position.x
        self.state.y = position.y
        self.state.yaw = yaw

    def path_callback(self, msg):
        # 경로를 수신하고 local 좌표 리스트에 저장
        self.local_x = []
        self.local_y = []
        self.local_yaw = []

        for pose in msg.poses:
            position = pose.pose.position
            orientation = pose.pose.orientation
            self.local_x.append(position.x)
            self.local_y.append(position.y)

            # Quaternion을 yaw로 변환
            _, _, yaw = self.euler_from_quaternion(
                orientation.x, orientation.y, orientation.z, orientation.w
            )
            self.local_yaw.append(yaw)

    def timer_callback(self):
        msg = ControlMessage()
        mission_msg = Bool()

        if len(self.local_x) == 0:
            return

        self.steer, self.target_idx, hdr, ctr = self.st.stanley_control(
            self.state,
            self.local_x,
            self.local_y,
            self.local_yaw,
            h_gain=1.3,
            c_gain=0.3,
            reverse=False,  # 현재 기어 상태에 따라 반대로 설정
        )
        msg.speed = 8

        msg.mora = 0
        msg.estop = 0
        msg.brake = 0
        msg.alive = 0
        msg.gear = 0

        print(self.target_idx)
        # # target_idx가 2 이상일 때 경로 리스트 갱신
        # if self.target_idx >= 1:
        #     self.local_x = self.local_x[self.target_idx - 1 :]
        #     self.local_y = self.local_y[self.target_idx - 1 :]
        #     self.local_yaw = self.local_yaw[self.target_idx - 1 :]
        #     self.local_dir = self.local_dir[self.target_idx - 1 :]
        # print(self.target_idx, len(self.local_x))
        # 기어 변경 확인

        # 속도 제어
        if self.target_idx >= len(self.local_x) - 10:
            if self.prev_path != self.local_x:
                self.prev_path = self.local_x
                self.get_logger().info("Target reached or close to target.")
                msg.estop = 1  # 정지 신호
                msg.speed = 0  # 속도 0으로 설정
                mission_msg.data = True

        else:
            pass
            mission_msg.data = False  # 미션 진행 중
            msg.speed = msg.speed  # 정상 주행 속도 (정수)

        msg.steer = int(degrees((-1) * self.steer))  # 조향각을 degree로 변환
        self.control_publisher.publish(msg)
        self.mission_publisher.publish(mission_msg)  # 미션 상태 퍼블리시

    def euler_from_quaternion(self, x, y, z, w):
        # Convert quaternion to Euler angles (yaw, pitch, roll)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = atan2(siny_cosp, cosy_cosp)
        return 0.0, 0.0, yaw


def main():
    rclpy.init()
    path_planner = Pathtracking()
    rclpy.spin(path_planner)
    path_planner.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
