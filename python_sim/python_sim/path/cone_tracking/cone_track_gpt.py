import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point, Pose
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray

import numpy as np
from scipy.spatial import Delaunay
import matplotlib.pyplot as plt


class Delaunay_path(Node):
    def __init__(self):
        super().__init__("Delaunay_path")

        self.y_cone_sub = self.create_subscription(
            PoseStamped, "/goal_pose", self.y_callback, 10
        )
        self.b_cone_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self.b_callback, 10
        )

        self.cones_pub = self.create_publisher(MarkerArray, "/cones", 10)

        self.Blue_cones = []
        self.Yellow_cones = []
        self.Blue_Yellow_cones = []  # 이 리스트에 파랑 노랑 번갈아 넣을 예정

    def y_callback(self, msg):
        self.Blue_cones.append(msg.pose.position.x, msg.pose.position.y)
        set(self.Blue_cones)
        pass

    def b_callback(self, msg):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = Delaunay_path()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
