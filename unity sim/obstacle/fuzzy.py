import numpy as np
import matplotlib.pyplot as plt
import copy


class RobotNavigation:
    def __init__(self):
        """RobotNavigation 클래스 초기화"""
        self.start = None
        self.goal = None
        self.obstacles = None
        self.x_min = None
        self.x_max = None
        self.y_min = None
        self.y_max = None
        self.robot_position = None  # 로봇 현재 위치

        # APF 관련 파라미터
        self.attractive_gain = 1.0
        self.repulsive_gain = 1.0
        self.obstacle_radius = 2.0
        self.obstacle_strength = 10.0

    def initialize(self, start, goal, obstacles, x_min, x_max, y_min, y_max):
        """초기값 설정 및 로봇 위치 초기화"""
        self.start = np.array(start, dtype=np.float64)
        self.goal = np.array(goal, dtype=np.float64)
        self.obstacles = np.array(obstacles, dtype=np.float64)
        self.x_min, self.x_max = x_min, x_max
        self.y_min, self.y_max = y_min, y_max
        self.robot_position = self.start.copy()

    def apf_fuzzy_navigation(self):
        """
        APF 기반의 장애물 회피 경로 생성 (효율적인 방향으로 회피)
        :return: steering_angle, attractive_potential, fuzzy_factor
        """
        delta_pos = self.goal - self.robot_position
        distance_to_goal = np.linalg.norm(delta_pos)
        attractive_potential = self.attractive_gain * distance_to_goal

        repulsive_potential = 0.0
        steering_angle = np.arctan2(delta_pos[1], delta_pos[0])

        for obs in self.obstacles:
            dist_to_obstacle = np.linalg.norm(self.robot_position - obs)

            if dist_to_obstacle < self.obstacle_radius:
                repulsive_potential += self.obstacle_strength * (
                    1.0 / dist_to_obstacle - 1.0 / self.obstacle_radius
                )

                # 장애물의 상대 위치에 따라 회피 방향을 효율적으로 결정
                angle_to_obstacle = np.arctan2(
                    self.robot_position[1] - obs[1], self.robot_position[0] - obs[0]
                )

                # 장애물의 상대적인 위치가 로봇의 왼쪽에 있으면 왼쪽으로 회피
                # 장애물이 로봇의 오른쪽에 있으면 오른쪽으로 회피
                if angle_to_obstacle > 0:  # 장애물이 왼쪽에 있을 때
                    steering_angle += np.pi / 2  # 오른쪽으로 회피
                else:  # 장애물이 오른쪽에 있을 때
                    steering_angle -= np.pi / 2  # 왼쪽으로 회피

        fuzzy_factor = 1.0 / (1.0 + np.exp(-repulsive_potential))  # Fuzzy 로직 적용

        return steering_angle, attractive_potential, fuzzy_factor

    def navigate(self, step_size=0.5):
        """
        로봇이 목표로 가는 경로 탐색 (포인트 간 거리를 step_size로 설정)
        :param step_size: 이동 거리 (한 번 이동할 때의 거리)
        :return: 로봇 경로, 부드러운 경로
        """
        if self.robot_position is None:
            raise ValueError("초기화되지 않았습니다. initialize()를 먼저 호출하세요.")

        robot_trajectory = []
        robot_trajectory.append(self.robot_position)

        while np.linalg.norm(self.robot_position - self.goal) > step_size:
            steering_angle, _, fuzzy_factor = self.apf_fuzzy_navigation()

            move_vector = np.array(
                [np.cos(steering_angle), np.sin(steering_angle)], dtype=np.float64
            )
            self.robot_position += move_vector * step_size * fuzzy_factor

            # 직사각형 구역 제한
            self.robot_position[0] = np.clip(
                self.robot_position[0], self.x_min, self.x_max
            )
            self.robot_position[1] = np.clip(
                self.robot_position[1], self.y_min, self.y_max
            )

            robot_trajectory.append(copy.deepcopy(self.robot_position))
        robot_trajectory = robot_trajectory[1:]
        smoothed_trajectory = self.moving_average_path(robot_trajectory, window_size=40)

        return np.array(robot_trajectory), smoothed_trajectory

    def moving_average_path(self, trajectory, window_size=4):
        """경로를 부드럽게 만드는 함수"""
        smoothed_trajectory = []
        for i in range(len(trajectory)):
            start_idx = max(0, i - window_size // 2)
            end_idx = min(len(trajectory), i + window_size // 2 + 1)
            window = trajectory[start_idx:end_idx]
            smoothed_trajectory.append(np.mean(window, axis=0))
        return np.array(smoothed_trajectory)

    def plot_trajectory(self, robot_trajectory, smoothed_trajectory):
        """경로 시각화"""
        plt.plot(
            robot_trajectory[:, 0],
            robot_trajectory[:, 1],
            label="Robot Path",
            color="b",
        )
        plt.scatter(self.goal[0], self.goal[1], c="red", label="Goal")
        plt.scatter(
            self.obstacles[:, 0], self.obstacles[:, 1], c="blue", label="Obstacles"
        )
        plt.scatter(self.start[0], self.start[1], c="green", label="Start")

        plt.axvline(x=self.x_min, color="black", linestyle="--")
        plt.axvline(x=self.x_max, color="black", linestyle="--")
        plt.axhline(y=self.y_min, color="black", linestyle="--")
        plt.axhline(y=self.y_max, color="black", linestyle="--")

        plt.plot(
            smoothed_trajectory[:, 0],
            smoothed_trajectory[:, 1],
            label="Smoothed Path",
            linestyle="--",
            color="red",
        )

        plt.xlabel("X")
        plt.ylabel("Y")
        plt.legend()
        plt.grid(True)
        plt.show()


def main():
    start = [0.0, 0.0]
    goal = [10.0, 10.0]
    obstacles = [[4, 4], [6, 6], [8, 8]]

    x_min, x_max, y_min, y_max = -5, 15, -5, 15

    robot_nav = RobotNavigation()
    robot_nav.initialize(start, goal, obstacles, x_min, x_max, y_min, y_max)

    # 첫 번째 탐색 (step_size=0.5로 설정)
    robot_trajectory, smoothed_trajectory = robot_nav.navigate(step_size=0.5)
    robot_nav.plot_trajectory(robot_trajectory, smoothed_trajectory)


if __name__ == "__main__":
    main()
