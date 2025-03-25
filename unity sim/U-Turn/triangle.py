import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import Delaunay
from scipy.spatial.distance import cdist


class DelaunayTriangle:
    def __init__(self):
        self.obstacles = []
        self.line = 0

    def is_collinear(self, points):
        # points[0]과 points[1]을 numpy array로 변환하여 연산
        v1 = np.array(points[1]) - np.array(points[0])
        v2 = np.array(points[2]) - np.array(points[0])

        # 두 벡터의 외적을 구하여 collinear 여부를 판단
        cross_product = np.cross(v1, v2)

        return abs(cross_product) < 0.3

    def make_trangle(self):
        # 장애물의 갯수 확인
        if len(self.obstacles) < 3 or self.is_collinear(self.obstacles):
            print("장애물이 부족하거나, 직선에 가까워 삼각분할을 수행할 수 없습니다.")
            # self.obstacles = np.vstack([self.obstacles, self.start])  # 시작점 추가
            return np.array([])

        try:
            # Delaunay 삼각분할 수행
            tri = Delaunay(self.obstacles)
            self.line = tri.simplices
        except:
            print("Delaunay 삼각분할 오류 발생.")
            return np.array([])

        # 긴 두 변의 중점 찾기
        midpoints = set()
        for simplex in tri.simplices:
            edges = [
                (simplex[0], simplex[1]),
                (simplex[1], simplex[2]),
                (simplex[2], simplex[0]),
            ]
            edge_lengths = [
                np.linalg.norm(
                    np.array(self.obstacles[e[0]]) - np.array(self.obstacles[e[1]])
                )
                for e in edges
            ]
            longest_edges = sorted(zip(edge_lengths, edges))[-2:]  # 가장 긴 두 변 선택

            for _, edge in longest_edges:
                # np.array로 변환하여 벡터 연산 수행
                mid = tuple(
                    (
                        (
                            np.array(self.obstacles[edge[0]])
                            + np.array(self.obstacles[edge[1]])
                        )
                        / 2
                    ).tolist()
                )
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

        return ordered_midpoints

    def plot_triangle(self):
        # Delaunay 삼각분할
        # tri = Delaunay(self.obstacles)
        # self.line = tri.simplices  # 삼각형들의 인덱스를 self.line에 저장

        plt.figure(figsize=(8, 8))

        # 장애물 점들 그리기
        # plt.scatter(
        #     self.obstacles[:, 0], self.obstacles[:, 1], color="red", label="Obstacles"
        # )

        # 각 삼각형의 경계를 그리기
        for simplex in self.line:
            plt.plot(
                self.obstacles[simplex, 0], self.obstacles[simplex, 1], "g-", marker="o"
            )  # 삼각형의 변을 초록색 선으로 그림

        plt.xlabel("X")
        plt.ylabel("Y")
        plt.title("Delaunay Triangulation")
        plt.legend()
        plt.show()


def main():
    # 장애물 좌표 (일부 주석 처리된 장애물들)
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
    De = DelaunayTriangle()
    De.obstacles = obstacles
    ordered_midpoints = De.make_trangle()
    De.plot_triangle()

    if ordered_midpoints.size == 0:
        print("경로를 생성할 수 없습니다.")
        return

    # 시각화
    plt.figure(figsize=(8, 8))
    plt.scatter(obstacles[:, 0], obstacles[:, 1], color="red", label="Obstacles")
    plt.plot(
        ordered_midpoints[:, 0], ordered_midpoints[:, 1], "g-", marker="o", label="Path"
    )

    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Eulerian Path Using Midpoints of Longest Edges")
    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()
