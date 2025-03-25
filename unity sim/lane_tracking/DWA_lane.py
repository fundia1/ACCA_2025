import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull

# 점들 생성 (예시 점들)
points = np.random.rand(30, 2) * 10  # 30개의 랜덤 점들

# Convex Hull 계산
hull = ConvexHull(points)

# 결과 시각화 (점들과 Convex Hull 경계 그리기)
plt.plot(points[:, 0], points[:, 1], "o")  # 점들 그리기
for simplex in hull.simplices:
    plt.plot(points[simplex, 0], points[simplex, 1], "k-")  # Convex Hull 경계 그리기


# 선분을 거리 단위로 세분화하는 함수
def subdivide_edge_by_distance(p1, p2, distance_per_point):
    # 두 점 사이의 벡터와 길이 계산
    vector = p2 - p1
    edge_length = np.linalg.norm(vector)

    # 선분을 세분화할 수 있는 점의 개수
    num_points = int(edge_length // distance_per_point)

    # 세분화된 점들을 저장
    subdivided_points = []

    # 각 점을 세분화하여 생성
    for i in range(num_points + 1):
        t = i / num_points  # [0, 1] 사이의 값
        point = p1 + t * vector  # 선분을 따라 점 생성
        subdivided_points.append(point)

    return np.array(subdivided_points)


# 세분화된 점들 저장
subdivided_points = []

distance_per_point = 0.5  # 세분화 간격 (단위: 거리)

for i in range(len(hull.vertices)):
    p1 = points[hull.vertices[i]]
    p2 = points[
        hull.vertices[(i + 1) % len(hull.vertices)]
    ]  # 다음 점과 연결 (원형 연결)

    # 선분을 거리 단위로 세분화
    subdivided_points.extend(subdivide_edge_by_distance(p1, p2, distance_per_point))

subdivided_points = np.array(subdivided_points)
# X값 소수점 이하를 버리고 그룹화
# X값을 반올림해서 그룹화하고, 각 그룹에 대해 Y값의 평균 계산
x_rounded = np.round(subdivided_points[:, 0], 0)  # X값 소수점 이하 반올림
unique_x_vals = np.unique(x_rounded)  # 고유한 X값들

# Y값의 평균을 구할 리스트
mean_points = []

for x_val in unique_x_vals:
    # X값이 같은 점들 찾기
    y_vals = subdivided_points[x_rounded == x_val, 1]

    # Y값의 평균 계산
    mean_y = np.mean(y_vals)

    # 평균값을 빨간 점으로 저장
    mean_points.append([x_val, mean_y])

mean_points = np.array(mean_points)

# 세분화된 점들을 시각화
plt.plot(
    subdivided_points[:, 0], subdivided_points[:, 1], "bo", label="Subdivided Points"
)

# 빨간색 점을 평균 위치로 추가
plt.scatter(mean_points[:, 0], mean_points[:, 1], color="red", label="Mean Points")

# 빨간 점들을 이어서 선으로 연결
plt.plot(
    mean_points[:, 0],
    mean_points[:, 1],
    color="red",
    linestyle="-",
    linewidth=2,
    label="Mean Line",
)

plt.title("Convex Hull with Subdivided Points by Distance")
plt.xlabel("X")
plt.ylabel("Y")
plt.legend()
plt.show()
