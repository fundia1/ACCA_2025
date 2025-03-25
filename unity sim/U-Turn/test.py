import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import Delaunay
from scipy.spatial.distance import cdist
import matplotlib.animation as animation

# 장애물 좌표
obstacles = np.array(
    [
        [-87.66267, 36.81775],
        [-86.18295, 40.51861],
        [-83.41397, 43.80231],
        [-79.83937, 46.19911],
        [-75.92238, 47.30313],
        [-71.75175, 46.96906],
        [-68.17937, 45.46938],
        [-65.27208, 43.29974],
        [-62.69083, 40.49252],
        [-61.39494, 36.38269],
        [-74.7186, 31.20725],
        [-74.56528, 26.9605],
        [-74.65209, 22.10976],
        [-74.58686, 18.36543],
        [-74.73897, 14.00041],
        [-74.68031, 9.516319],
        [-74.72476, 5.370415],
        [-74.80842, 1.542259],
    ]
)


# 직선에 가까운 점들을 필터링하는 함수
def filter_straight_line_points(obstacles, threshold=0.01):
    """
    직선에 가까운 점들을 필터링합니다.
    `threshold` 값보다 기울기가 작은 점들을 제거합니다.
    """
    filtered_points = []
    for i in range(1, len(obstacles)):
        x_diff = np.abs(obstacles[i][0] - obstacles[i - 1][0])
        y_diff = np.abs(obstacles[i][1] - obstacles[i - 1][1])

        if x_diff > threshold and y_diff > threshold:  # 직선에 너무 가까운 점 제외
            filtered_points.append(obstacles[i])

    return np.array(filtered_points)


# Delaunay 삼각분할 수행
def get_delaunay_tri(obstacles):
    tri = Delaunay(obstacles)
    return tri


# 긴 두 변의 중점 찾기
def get_midpoints(tri, obstacles):
    midpoints = set()
    for simplex in tri.simplices:
        edges = [
            (simplex[0], simplex[1]),
            (simplex[1], simplex[2]),
            (simplex[2], simplex[0]),
        ]
        edge_lengths = [
            np.linalg.norm(obstacles[e[0]] - obstacles[e[1]]) for e in edges
        ]
        longest_edges = sorted(zip(edge_lengths, edges))[-2:]  # 가장 긴 두 변 선택

        for _, edge in longest_edges:
            mid = tuple(((obstacles[edge[0]] + obstacles[edge[1]]) / 2).tolist())
            midpoints.add(mid)  # 중복 제거됨

    return np.array(list(midpoints))


# 장애물 랜덤 섞기
np.random.shuffle(obstacles)

# 직선에 가까운 점들을 필터링하여 Delaunay 삼각분할
filtered_obstacles = filter_straight_line_points(obstacles)
tri = get_delaunay_tri(filtered_obstacles)
midpoints = get_midpoints(tri, filtered_obstacles)

# 거리 행렬 생성 (모든 점들 사이의 거리 계산)
dist_matrix = cdist(midpoints, midpoints)

# 한붓그리기(가장 가까운 점을 따라 이동)
unvisited = set(range(len(midpoints)))  # 방문하지 않은 점들
path = [0]  # 시작점 (임의의 점에서 시작)
unvisited.remove(0)

while unvisited:
    last_point = path[-1]
    # 방문하지 않은 점 중 가장 가까운 점 찾기
    nearest = min(unvisited, key=lambda i: dist_matrix[last_point, i])
    path.append(nearest)
    unvisited.remove(nearest)

# 정렬된 경로 좌표
ordered_midpoints = midpoints[path]

# 시각화 설정
fig, ax = plt.subplots(figsize=(8, 8))

# 경로 라인과 장애물 표시
(path_line,) = ax.plot([], [], "go-", label="Path")
(obstacle_plot,) = ax.plot([], [], "ro", label="Obstacles")
(triangles_plot,) = ax.plot([], [], "b-", alpha=0.5, label="Delaunay Triangles")


# 경로 점, 장애물, 삼각형을 애니메이션으로 업데이트
def update(frame):
    # 새로 보일 장애물 점들 추가 (2~3개씩)
    obstacles_to_show = filtered_obstacles[: min(frame + 3, len(filtered_obstacles))]
    obstacle_plot.set_data(obstacles_to_show[:, 0], obstacles_to_show[:, 1])

    # 새로운 Delaunay 삼각분할을 계산
    tri = get_delaunay_tri(obstacles_to_show)

    # 기존 삼각분할을 삭제하고 새로운 삼각분할 그리기
    for simplex in tri.simplices:
        simplex_points = obstacles_to_show[simplex]
        for i in range(3):
            ax.plot(
                [simplex_points[i, 0], simplex_points[(i + 1) % 3, 0]],
                [simplex_points[i, 1], simplex_points[(i + 1) % 3, 1]],
                "b-",
                alpha=0.5,
            )

    # 중점 계산 (장애물이 추가될 때마다 업데이트)
    midpoints = get_midpoints(tri, obstacles_to_show)

    # 거리 행렬 업데이트
    dist_matrix = cdist(midpoints, midpoints)

    # 한붓그리기 경로 업데이트
    unvisited = set(range(len(midpoints)))  # 방문하지 않은 점들
    path = [0]  # 시작점 (임의의 점에서 시작)
    unvisited.remove(0)

    while unvisited:
        last_point = path[-1]
        # 방문하지 않은 점 중 가장 가까운 점 찾기
        nearest = min(unvisited, key=lambda i: dist_matrix[last_point, i])
        path.append(nearest)
        unvisited.remove(nearest)

    # 정렬된 경로 좌표
    ordered_midpoints = midpoints[path]

    # 경로 점 점차적으로 추가
    if frame < len(ordered_midpoints):
        path_line.set_data(
            ordered_midpoints[: frame + 1, 0], ordered_midpoints[: frame + 1, 1]
        )

    return obstacle_plot, triangles_plot, path_line


# 애니메이션 실행 (1초마다 1개씩 경로 추가)
ani = animation.FuncAnimation(
    fig, update, frames=len(filtered_obstacles), interval=1000, blit=True
)

plt.legend()
plt.show()
