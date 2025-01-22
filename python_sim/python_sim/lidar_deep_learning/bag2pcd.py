import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import open3d as o3d
import struct


class BagToPCD(Node):
    def __init__(self):
        super().__init__("bag_to_pcd")
        self.subscription = self.create_subscription(
            PointCloud2, "/points_no_ground", self.callback, 10
        )
        self.output_dir = (
            "/home/jinju/ws/src/python_sim/python_sim/lidar_deep_learning/"
        )
        os.makedirs(self.output_dir, exist_ok=True)
        self.frame_id = 0

    def callback(self, msg):
        self.get_logger().info(f"Processing frame {self.frame_id}")

        # PointCloud2 데이터를 NumPy 배열로 변환
        points = self.pointcloud2_to_array(msg)

        # Open3D PointCloud 객체 생성
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # PCD 파일로 저장
        pcd_file = os.path.join(self.output_dir, f"{self.frame_id:06d}.pcd")
        o3d.io.write_point_cloud(pcd_file, pcd)
        self.get_logger().info(f"Saved: {pcd_file}")

        self.frame_id += 1

    def pointcloud2_to_array(self, msg):
        """PointCloud2 메시지를 NumPy 배열로 변환"""
        # 메시지 필드 정보 추출
        fmt = "fff"  # Assuming fields are x, y, z (float32)
        cloud_data = []

        for i in range(msg.height * msg.width):
            offset = msg.point_step * i
            x, y, z = struct.unpack_from(fmt, msg.data, offset)
            if not np.isnan(x) and not np.isnan(y) and not np.isnan(z):
                cloud_data.append((x, y, z))

        return np.array(cloud_data)


def main(args=None):
    rclpy.init(args=args)
    node = BagToPCD()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
