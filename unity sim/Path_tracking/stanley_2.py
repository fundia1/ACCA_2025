#!/usr/bin/env python3

import rclpy
import numpy as np
import math as m
from erp42_msgs.msg import StanleyError, SerialFeedBack
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32


"""

Export module. Stanley Control Class.
Input: (state(class: State), [cx], [cy], [cyaw], last_target_idx)
Output: steer

"""


class Stanley(Node):
    def __init__(self):
        super().__init__("stanley")
        self.__L = 1.040  # [m] Wheel base of vehicle
        # self.__k = self.declare_parameter("/stanley_controller/c_gain", 0.8).value
        # self.__hdr_ratio = self.declare_parameter("/stanley_controller/hdr_ratio", 0.03).value
        # self.__hdr_ratio = self.declare_parameter("/stanley_controller/hdr_ratio", 0.06).value

        # self.__hdr_ratio = self.declare_parameter("/stanley_controller/hdr_ratio", 0.5).value
        # self.__k = self.declare_parameter("/stanley_controller/c_gain", 0.24).value

        self.__hdr = 0.0  # heading error
        self.__ctr = 0.0  # crosstrack error

        self.k_v = 0.5

    # def stanley_control(self, state, cx, cy, cyaw, last_target_idx, reverse=False):
    def stanley_control(self, state, cx, cy, cyaw, h_gain, c_gain, reverse=False):

        current_target_idx, error_front_axle = self.calc_target_index(
            state, cx, cy, reverse=reverse
        )

        theta_e = (
            self.normalize_angle(
                cyaw[current_target_idx] - (state.yaw + (np.pi if reverse else 0.0))
            )
        ) * h_gain

        theta_d = np.arctan2(c_gain * error_front_axle, (self.k_v + state.v)) * (
            -1.0 if reverse else 1.0
        )

        # Field
        self.__hdr = theta_e
        self.__ctr = theta_d

        # Steering control
        delta = theta_e + theta_d

        delta = np.clip(delta, m.radians((-1) * 28), m.radians(28))

        return delta, current_target_idx, self.__hdr, self.__ctr

    def normalize_angle(self, angle):
        """
        Normalize an angle to [-pi, pi].
        :param angle: (float)
        :return: (float) Angle in radian in [-pi, pi]
        """
        while angle > np.pi:
            angle -= 2.0 * np.pi

        while angle < -np.pi:
            angle += 2.0 * np.pi

        return angle

    #!/usr/bin/env python3


import rclpy
import numpy as np
import math as m
from erp42_msgs.msg import StanleyError, SerialFeedBack
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32


"""

Export module. Stanley Control Class.
Input: (state(class: State), [cx], [cy], [cyaw], last_target_idx)
Output: steer

"""


class Stanley(Node):
    def __init__(self):
        super().__init__("stanley")
        self.__L = 1.040  # [m] Wheel base of vehicle
        # self.__k = self.declare_parameter("/stanley_controller/c_gain", 0.8).value
        # self.__hdr_ratio = self.declare_parameter("/stanley_controller/hdr_ratio", 0.03).value
        # self.__hdr_ratio = self.declare_parameter("/stanley_controller/hdr_ratio", 0.06).value

        # self.__hdr_ratio = self.declare_parameter("/stanley_controller/hdr_ratio", 0.5).value
        # self.__k = self.declare_parameter("/stanley_controller/c_gain", 0.24).value

        self.__hdr = 0.0  # heading error
        self.__ctr = 0.0  # crosstrack error

        self.k_v = 0.5

    # def stanley_control(self, state, cx, cy, cyaw, last_target_idx, reverse=False):
    def stanley_control(self, state, cx, cy, cyaw, h_gain, c_gain, reverse=False):

        current_target_idx, error_front_axle = self.calc_target_index(
            state, cx, cy, reverse=reverse
        )

        theta_e = (
            self.normalize_angle(
                cyaw[current_target_idx] - (state.yaw + (np.pi if reverse else 0.0))
            )
        ) * h_gain

        theta_d = np.arctan2(c_gain * error_front_axle, (self.k_v + state.v)) * (
            -1.0 if reverse else 1.0
        )

        # Field
        self.__hdr = theta_e
        self.__ctr = theta_d

        # Steering control
        delta = theta_e + theta_d

        delta = np.clip(delta, m.radians((-1) * 28), m.radians(28))

        return delta, current_target_idx, self.__hdr, self.__ctr

    def normalize_angle(self, angle):
        """
        Normalize an angle to [-pi, pi].
        :param angle: (float)
        :return: (float) Angle in radian in [-pi, pi]
        """
        while angle > np.pi:
            angle -= 2.0 * np.pi

        while angle < -np.pi:
            angle += 2.0 * np.pi

        return angle

    def calc_target_index(
        self, state, cx, cy, reverse=False, last_target_idx=None, search_radius=5
    ):
        """
        Compute index in the trajectory list of the target.
        :param state: (State object)
        :param cx: [float]
        :param cy: [float]
        :return: (int, float)
        """
        fx = state.x + self.__L * np.cos(state.yaw) / 2.0 * (-1.0 if reverse else 1.0)
        fy = state.y + self.__L * np.sin(state.yaw) / 2.0 * (-1.0 if reverse else 1.0)

        # Search nearest point index within the search radius
        dx = [fx - icx for icx in cx]
        dy = [fy - icy for icy in cy]

        d = np.hypot(dx, dy)

        # 제한된 범위 내에서 가장 가까운 점 찾기
        start_idx = last_target_idx if last_target_idx is not None else 0
        end_idx = min(start_idx + search_radius, len(cx))

        # 검색 범위 내에서 가장 가까운 점을 찾음
        target_idx = start_idx + np.argmin(d[start_idx:end_idx])

        # 인덱스 차이를 제한하여 급격한 변화 방지
        if last_target_idx is not None:
            idx_diff = abs(target_idx - last_target_idx)
            if idx_diff > 5:  # 임계값을 조정하여 급격한 변화 방지
                target_idx = (
                    last_target_idx + np.sign(target_idx - last_target_idx) * 1
                )  # 1만큼만 변경

        # Project RMS error onto front axle vector
        front_axle_vec = [
            -np.cos(state.yaw + np.pi / 2),
            -np.sin(state.yaw + np.pi / 2),
        ]
        error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

        return target_idx, error_front_axle
