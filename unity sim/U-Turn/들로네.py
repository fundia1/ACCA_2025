import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import Delaunay, distance_matrix
from scipy.spatial.distance import cdist

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
        [-74.71, -14.76],
    ]
)

# Delaunay 삼각분할 수행
tri = Delaunay(obstacles)

# 긴 두 변의 중점 찾기
midpoints = set()
for simplex in tri.simplices:
    edges = [
        (simplex[0], simplex[1]),
        (simplex[1], simplex[2]),
        (simplex[2], simplex[0]),
    ]
    edge_lengths = [np.linalg.norm(obstacles[e[0]] - obstacles[e[1]]) for e in edges]
    longest_edges = sorted(zip(edge_lengths, edges))[-2:]  # 가장 긴 두 변 선택

    for _, edge in longest_edges:
        mid = tuple(((obstacles[edge[0]] + obstacles[edge[1]]) / 2).tolist())
        midpoints.add(mid)  # 중복 제거됨

# 리스트로 변환
midpoints = np.array(list(midpoints))

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

print(ordered_midpoints[:, 0])
print(ordered_midpoints[:, 1])
# 시각화
plt.figure(figsize=(8, 8))
plt.triplot(obstacles[:, 0], obstacles[:, 1], tri.simplices, color="blue", alpha=0.5)
plt.scatter(obstacles[:, 0], obstacles[:, 1], color="red", label="Obstacles")
plt.plot(
    ordered_midpoints[:, 0], ordered_midpoints[:, 1], "g-", marker="o", label="Path"
)

plt.xlabel("X")
plt.ylabel("Y")
plt.title("Eulerian Path Using Midpoints of Longest Edges")
plt.legend()
plt.show()
