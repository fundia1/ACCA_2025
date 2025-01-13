import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point, Pose
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray

import reeds_shepp_planner as rs
import matplotlib.pyplot as plt
import math


# 차량의 크기와 회전 반경 설정


def main():
    # points to be followed
    pts = [
        (-6.0, -7.0, 0),
        (4.0, 0.0, 0),
    ]
    for i in range(len(pts) - 1):

        start_x = pts[i][0]
        start_y = pts[i][1]
        start_yaw = pts[i][2]

        end_x = pts[i + 1][0]
        end_y = pts[i + 1][1]
        end_yaw = pts[i + 1][2]

        min_radius = 5
        step_size = 0.02
        px, py, pyaw, mode, clen = rs.reeds_shepp_planner(
            start_x, start_y, start_yaw, end_x, end_y, end_yaw, min_radius, step_size
        )

        # if show_animation:
        # plt.cla()
        plt.plot(px, py, label=str(mode))

        rs.plot_arrow(start_x, start_y, start_yaw)
        rs.plot_arrow(end_x, end_y, end_yaw)

        plt.legend()
        plt.grid(True)
        plt.axis("equal")
    plt.show()

    if not px:
        assert False, "No path"


if __name__ == "__main__":
    main()
