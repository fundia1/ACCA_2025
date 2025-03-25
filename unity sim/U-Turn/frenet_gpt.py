import numpy as np
from scipy.interpolate import CubicSpline


class FrenetPathPlanner:
    def __init__(self, global_path):
        self.global_path = np.array(global_path)  # (N,2) 형태, [x, y] 글로벌 경로
        self.s_list = self.compute_s_values(
            self.global_path
        )  # 글로벌 경로의 진행 거리 리스트

    def compute_s_values(self, path):
        """경로의 각 점에 대해 진행 거리(s)를 계산"""
        s_values = np.zeros(len(path))
        for i in range(1, len(path)):
            s_values[i] = s_values[i - 1] + np.linalg.norm(path[i] - path[i - 1])
        return s_values

    def xy_to_frenet(self, x, y):
        """(x, y) 좌표를 Frenet 좌표계 (s, d)로 변환"""
        dists = np.linalg.norm(self.global_path - np.array([x, y]), axis=1)
        nearest_idx = np.argmin(dists)  # 가장 가까운 글로벌 경로점 찾기
        nearest_s = self.s_list[nearest_idx]  # 해당 경로점의 진행 거리 s
        d = dists[nearest_idx]  # 현재 위치에서 글로벌 패스까지의 거리
        return nearest_s, d

    def frenet_to_xy(self, s, d):
        """Frenet 좌표 (s, d)를 (x, y)로 변환"""
        nearest_idx = np.argmin(
            np.abs(self.s_list - s)
        )  # s에 가장 가까운 글로벌 경로점 찾기
        base_x, base_y = self.global_path[nearest_idx]
        tangent_vector = (
            self.global_path[min(nearest_idx + 1, len(self.global_path) - 1)]
            - self.global_path[nearest_idx]
        )
        normal_vector = np.array([-tangent_vector[1], tangent_vector[0]])  # 법선 벡터
        normal_vector /= np.linalg.norm(normal_vector)  # 정규화
        new_x, new_y = base_x + d * normal_vector[0], base_y + d * normal_vector[1]
        return new_x, new_y

    def modify_local_path(self, obstacles):
        """장애물이 있을 경우 Frenet 좌표계를 이용해 로컬 패스를 수정"""
        new_frenet_path = []

        for x, y in self.global_path:
            s, d = self.xy_to_frenet(x, y)

            # 장애물과 너무 가까우면 d를 조정
            for obs_x, obs_y in obstacles:
                obs_s, obs_d = self.xy_to_frenet(obs_x, obs_y)
                if abs(s - obs_s) < 2.0:  # 장애물이 경로 근처에 있을 경우
                    d += 1.0  # 일정 거리만큼 경로를 옆으로 이동 (왼쪽 회피)

            new_x, new_y = self.frenet_to_xy(s, d)
            new_frenet_path.append((new_x, new_y))

        # Cubic Spline을 이용해 부드러운 곡선으로 변환
        new_frenet_path = np.array(new_frenet_path)
        cs_x = CubicSpline(self.s_list, new_frenet_path[:, 0])
        cs_y = CubicSpline(self.s_list, new_frenet_path[:, 1])
        smooth_x = cs_x(self.s_list)
        smooth_y = cs_y(self.s_list)

        return np.column_stack((smooth_x, smooth_y))  # 수정된 로컬 패스 반환


# 예제 실행
global_path = [(i, 0) for i in range(50)]  # 직선 경로 예제
planner = FrenetPathPlanner(global_path)

obstacles = [(15, 0.5), (30, -0.5)]  # 장애물 위치 (x, y)
local_path = planner.modify_local_path(obstacles)

print(local_path)  # 수정된 로컬 패스 출력
