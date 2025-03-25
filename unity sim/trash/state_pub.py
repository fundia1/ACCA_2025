import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml

import sys
import os


class MissionPublisherNode(Node):
    def __init__(self):
        super().__init__("mission_publisher")

        # YAML 파일에서 미션 목록 로드
        self.missions = self.load_missions_from_yaml("missions.yaml")

        # 초기 미션 인덱스 설정
        self.current_mission_idx = 0

        # 퍼블리셔 설정
        self.publisher = self.create_publisher(String, "/state_command", 10)

        # 토픽 구독 설정: /flag 또는 /emergency 토픽을 받으면 미션 전환
        self.create_subscription(String, "/flag", self.flag_callback, 10)
        self.create_subscription(String, "/emergency", self.emergency_callback, 10)

        self.get_logger().info("✅ Mission Publisher Node Started")

    def load_missions_from_yaml(self, file_name):
        """YAML 파일에서 미션 목록을 로드하는 함수"""
        script_dir = os.path.dirname(
            os.path.abspath(sys.argv[0])
        )  # 현재 실행되는 스크립트의 디렉토리 경로
        file_path = os.path.join(script_dir, file_name)
        try:
            with open(file_path, "r") as file:
                data = yaml.safe_load(file)
                return data["missions"]  # 'missions' 항목 반환
        except Exception as e:
            self.get_logger().error(f"⚠️ Error loading missions from YAML: {e}")
            return []

    def flag_callback(self, msg):
        """/flag 토픽을 받으면 미션을 전환"""
        self.get_logger().info("⚠️ Flag received, advancing to next mission.")
        self.advance_mission()

    def emergency_callback(self, msg):
        """/emergency 토픽을 받으면 미션을 전환"""
        self.get_logger().warn("🛑 Emergency received, advancing to next mission.")

    def advance_mission(self):
        """다음 미션으로 전환하고 /state_command 퍼블리시"""
        if self.current_mission_idx < len(self.missions) - 1:
            self.current_mission_idx += 1
            next_mission = self.missions[self.current_mission_idx]
            self.get_logger().info(f"▶️ Moving to next mission: {next_mission}")

            # /state_command 퍼블리시
            msg = String()
            msg.data = next_mission
            self.publisher.publish(msg)
        else:
            self.get_logger().info("🚨 No more missions left.")
            # 최종 미션일 경우 처리 (예: Idle 상태로 돌아가기)
            msg = String()
            msg.data = "all_complete"
            self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
