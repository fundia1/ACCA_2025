import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from transitions import Machine

import sys
import pathlib

sys.path.append(str(pathlib.Path(__file__).parent))

from obstacle.small import controller_obstacle
from obstacle.big import controller_obstacle2
from driving import controller_driving
from Parking import controller_parking
from Path_tracking import Stanley_controller


class StateMachineNode(Node):
    states = [
        "Idle",
        "Navigate",
        "Obstacle_Avoidance",
        "Pickup",
        "Delivery",
        "Parking",
        "Emergency_Stop",
    ]

    def __init__(self):
        super().__init__("state_machine")
        self.machine = Machine(model=self, states=self.states, initial="Idle")

        # ✅ 상태 전이 정의
        self.machine.add_transition(
            "start", "Idle", "Navigate", after="on_enter_navigate"
        )
        self.machine.add_transition(
            "obstacle",
            "Navigate",
            "Obstacle_Avoidance",
            after="on_enter_obstacle_avoidance",
        )

        self.machine.add_transition(
            "pickup", "Navigate", "Pickup", after="on_enter_pickup"
        )

        self.machine.add_transition(
            "delivery", "Navigate", "Delivery", after="on_enter_delivery"
        )

        self.machine.add_transition(
            "parking", "Navigate", "Parking", after="on_enter_parking"
        )
        self.machine.add_transition(
            "driving", "*", "Navigate", after="on_enter_navigate"
        )

        # self.machine.add_transition(
        #     "mission_complete", "*", "Navigate", after="on_enter_navigate"
        # )

        self.machine.add_transition(
            "all_complete", "Navigate", "Idle", after="on_enter_idle"
        )

        # ✅ ROS2 구독자 설정
        self.subscription = self.create_subscription(
            String, "/state_command", self.state_callback, 10
        )

        self.get_logger().info("✅ State Machine Node Started")

        # ✅ 시스템이 처음 시작될 때 on_enter_idle() 호출
        self.on_enter_idle()

        self.controller = None

    def state_callback(self, msg):
        command = msg.data.lower()

        try:
            if hasattr(self, command):

                self.get_logger().info(f"✅ State changed to: {self.state}")
                getattr(self, command)()  # ✅ 상태 전이 실행
            else:
                self.get_logger().warn(f"⚠️ Invalid command: {command}")
        except Exception as e:
            self.get_logger().warn(f"⚠️ Error while changing state: {e}")

    # ✅ 상태별 실행 함수 (로그 추가)
    def on_enter_idle(self):
        self.get_logger().info("🛑 시스템 대기 상태")

    def on_enter_navigate(self):
        self.get_logger().info("🚗 기본 주행 시작")
        if self.controller is not None:
            self.controller.stop_timer()  # 기존 타이머 멈추기
            self.controller.shutdown()  # 기존 노드 종료

    def on_enter_obstacle_avoidance(self):
        self.get_logger().info("🚧 장애물 회피 모드 시작")
        self.controller = (
            controller_obstacle.ControllerObstacle()
        )  # 새로운 ControllerObstacle 인스턴스 생성
        self.controller.start_timer()  # 타이머 시작

    def on_enter_pickup(self):
        self.get_logger().info("📦 픽업 구역 도착, 작업 수행")

    def on_enter_delivery(self):
        self.get_logger().info("🚚 딜리버리 진행 중")

    def on_enter_parking(self):
        self.get_logger().info("🅿️ 주차 중")


def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
