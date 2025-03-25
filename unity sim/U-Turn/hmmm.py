import matplotlib.pyplot as plt
import numpy as np

# 장애물 리스트
a = 8
obstacles = [
    [-87.66267, 36.81775 + a],
    [-86.18295, 40.51861 + a],
    [-83.41397, 43.80231 + a],
    [-79.83937, 46.19911 + a],
    [-75.92238, 47.30313 + a],
    [-71.75175, 46.96906 + a],
    [-68.17937, 45.46938 + a],
    [-65.27208, 43.29974 + a],
    [-62.69083, 40.49252 + a],
    [-61.39494, 36.38269 + a],
    [-74.7186, 31.20725 + a],
    [-74.56528, 26.9605 + a],
    [-74.65209, 22.10976 + a],
    [-74.58686, 18.36543 + a],
    [-74.73897, 14.00041 + a],
    [-74.68031, 9.516319 + a],
    [-74.72476, 5.370415 + a],
    [-74.80842, 1.542259 + a],
    [-74.71, -14.76 + a],
]


# 글로벌 경로 (예시)
path = [
    (-81.193715, 19.1800045),
    (-81.235545, 21.0940825),
    (-81.17149, 23.1670345),
    (-81.20082, 25.40908),
    (-81.124765, 27.59159),
    (-81.15738, 29.463755),
    (-81.113975, 31.889125),
    (-81.190635, 34.0125),
    (-80.450775, 35.86293),
    (-79.066285, 37.50478),
    (-77.278985, 38.70318),
    (-75.32049, 39.25519),
    (-73.235175, 39.088155),
    (-71.448985, 38.338315),
    (-69.99534, 37.253495),
    (-68.704715, 35.849885),
    (-68.05677, 33.79497),
    (-67.98011, 31.671595),
    (-68.023515, 29.246225),
    (-67.9909, 27.37406),
    (-68.066955, 25.19155),
    (-68.037625, 22.9495045),
    (-68.05985, 20.8765525),
    (-68.10168, 18.9624745),
    (-68.05247, 10.811345),
]


# 장애물과 경로의 충돌 여부 확인
def check_collision(path_point, obstacles, threshold=3.0):
    for obs in obstacles:
        dist = np.linalg.norm(
            np.array(path_point) - np.array(obs)
        )  # 유클리드 거리 계산
        if dist < threshold:  # 만약 장애물이 너무 가까우면
            return True
    return False


# 경로 수정 함수
def generate_avoidance_trajectory(path, obstacles):
    new_path = []
    for i in range(len(path) - 1):
        current_point = path[i]
        next_point = path[i + 1]

        # 현재 포인트와 다음 포인트 사이에 장애물이 있으면 회피 경로 생성
        if check_collision(current_point, obstacles):
            # 간단히 좌측으로 회피하는 방법 (다른 회피 방식 가능)
            avoidance_point = (current_point[0] + 0.1, current_point[1] - 0.1)
            new_path.append(avoidance_point)
        else:
            new_path.append(current_point)

    # 마지막 포인트 추가
    new_path.append(path[-1])
    return new_path


# 경로 생성
avoidance_path = generate_avoidance_trajectory(path, obstacles)

# 시각화
plt.figure(figsize=(10, 6))

# 장애물 표시
obstacles_np = np.array(obstacles)
plt.scatter(obstacles_np[:, 0], obstacles_np[:, 1], color="red", label="Obstacles")

# 원래 경로 표시
path_np = np.array(path)
plt.plot(path_np[:, 0], path_np[:, 1], marker="o", color="blue", label="Original Path")

# 회피 경로 표시
avoidance_path_np = np.array(avoidance_path)
plt.plot(
    avoidance_path_np[:, 0],
    avoidance_path_np[:, 1],
    marker="x",
    color="green",
    label="Avoidance Path",
)

# 레이블 및 제목 추가
plt.xlabel("X")
plt.ylabel("Y")
plt.title("Path and Obstacle Avoidance")
plt.legend()

# 그래프 표시
plt.grid(True)
plt.show()
