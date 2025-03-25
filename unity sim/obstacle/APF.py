import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import make_interp_spline


class APF:
    def __init__(self):
        self.start = []
        self.goal = []
        self.obstacles = []

        # 포텐셜 함수 매개변수 설정
        self.eta = 1000.0  # 장애물 반발력 상수
        self.alpha = 500.0  # 목표 끌림 상수
        self.rho0 = 1.5 + 1  # 장애물 영향 반경
        self.num_points = 5

        # 경로 제한 영역 (min_x, max_x, min_y, max_y)
        self.x_min = 0
        self.x_max = 15
        self.y_min = 0
        self.y_max = 15

    def make_apf_path(self):
        hint_points = self.generate_logical_hint_points(
            self.start, self.goal, self.obstacles, self.num_points
        )

        goals = list(hint_points) + [self.goal]
        path = self.gradient_descent(
            self.start, goals, self.obstacles, self.eta, self.alpha, self.rho0
        )

        # B-spline을 사용하여 부드러운 경로 생성
        x = path[:, 0]
        y = path[:, 1]
        t = np.linspace(0, 1, len(x))
        spl = make_interp_spline(t, np.vstack((x, y)).T, k=5)
        t_new = np.linspace(0, 1, 100)

        path_smooth = spl(t_new)
        path_smooth = self.moving_average_path(path_smooth)
        return path_smooth

    def moving_average_path(self, trajectory, window_size=10):
        """경로를 부드럽게 만드는 함수"""
        smoothed_trajectory = []
        for i in range(len(trajectory)):
            start_idx = max(0, i - window_size // 2)
            end_idx = min(len(trajectory), i + window_size // 2 + 1)
            window = trajectory[start_idx:end_idx]
            smoothed_trajectory.append(np.mean(window, axis=0))
        return np.array(smoothed_trajectory)

    def generate_logical_hint_points(self, start, goal, obstacles, num_points):
        hint_points = np.linspace(start, goal, num_points + 2)[1:-1]
        for i, point in enumerate(hint_points):
            for obs in obstacles:
                if np.linalg.norm(point - obs) < 2.0:
                    direction = point - obs
                    if np.linalg.norm(direction) > 0:  # 정상적인 방향 계산
                        direction = direction / np.linalg.norm(direction)
                        point = obs + direction * 2.0
                    else:  # 장애물과 점이 겹치는 경우
                        direction = np.array([1, 0])  # 기본 방향 설정
                        point = obs + direction * 2.0
                    hint_points[i] = point
        return hint_points

    def attractive_potential(self, q, goal):
        return 0.5 * self.alpha * np.linalg.norm(q - goal)

    def repulsive_potential(self, q, obstacles, eta, rho0):
        repulsive = 0
        for obstacle in obstacles:
            rho = np.linalg.norm(q - obstacle)
            if rho <= rho0:
                repulsive += 0.5 * eta * ((1 / rho) - (1 / rho0)) ** 2
        return repulsive

    def total_potential(self, q, goal, obstacles, eta, alpha, rho0):
        return self.attractive_potential(q, goal) + self.repulsive_potential(
            q, obstacles, eta, rho0
        )

    def gradient_descent(
        self, start, goals, obstacles, eta, alpha, rho0, epsilon=0.1, max_steps=500
    ):
        path = []
        q = np.array(start, dtype=float)
        for goal in goals:
            path_segment = []
            for _ in range(max_steps):
                grad = np.zeros(2)
                for i in range(2):
                    q_temp = q.copy()
                    q_temp[i] += epsilon
                    grad[i] = (
                        self.total_potential(q_temp, goal, obstacles, eta, alpha, rho0)
                        - self.total_potential(q, goal, obstacles, eta, alpha, rho0)
                    ) / epsilon
                q -= epsilon * grad / np.linalg.norm(grad)

                # 경로 제한 영역을 벗어나지 않도록 제한
                q[0] = np.clip(q[0], self.x_min, self.x_max)  # x 값 제한
                q[1] = np.clip(q[1], self.y_min, self.y_max)  # y 값 제한

                path_segment.append(q.copy())
                if np.linalg.norm(q - goal) < epsilon:
                    break
            path.extend(path_segment)
        return np.array(path)


def main():
    apf = APF()
    # 목표, 장애물, 시작점 설정
    apf.start = np.array([0, 0])
    apf.goal = np.array([10, 10])
    apf.obstacles = [np.array([3, 4]), np.array([6, 5])]
    apf.num_points = 3

    path_smooth = apf.make_apf_path()

    # 경로 시각화
    plt.plot(path_smooth[:, 0], path_smooth[:, 1], "b-", label="Path")
    plt.plot(apf.start[0], apf.start[1], "ro", label="Start")
    plt.plot(apf.goal[0], apf.goal[1], "go", label="Goal")
    for obstacle in apf.obstacles:
        plt.plot(obstacle[0], obstacle[1], "ko", label="Obstacle")
    plt.grid()
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Path Planning using APF with Smooth Hint Points")
    plt.show()


if __name__ == "__main__":
    main()
