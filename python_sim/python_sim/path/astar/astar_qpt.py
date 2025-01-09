import heapq
from typing import List, Tuple, Dict


class AStar:
    def __init__(self, grid: List[List[int]]):
        """
        A* 알고리즘 초기화.
        :param grid: 2D 리스트로 이루어진 맵. 0은 이동 가능, 1은 장애물.
        """
        self.grid = grid  # map
        self.rows = len(grid)  # 수직
        self.cols = len(grid[0])  # 수평
        self.open_set = []  # 우선순위 큐
        self.came_from = {}  # 각 노드의 부모 정보

    def heuristic(self, current: Tuple[int, int], goal: Tuple[int, int]) -> float:
        """맨해튼 거리 휴리스틱 계산."""
        return abs(current[0] - goal[0]) + abs(current[1] - goal[1])

    def get_neighbors(self, node: Tuple[int, int]) -> List[Tuple[int, int]]:
        """주변 노드 가져오기."""
        neighbors = []
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # 상, 하, 좌, 우
        for d in directions:
            new_row, new_col = node[0] + d[0], node[1] + d[1]
            if (
                0 <= new_row < self.rows
                and 0 <= new_col < self.cols
                and self.grid[new_row][new_col] == 0
            ):
                neighbors.append((new_row, new_col))
        return neighbors

    def update_obstacle(self, position: Tuple[int, int]):
        """실시간 장애물 추가."""
        self.grid[position[0]][position[1]] = 1

    def find_path(
        self, start: Tuple[int, int], goal: Tuple[int, int]
    ) -> List[Tuple[int, int]]:
        """
        A* 알고리즘으로 경로를 찾습니다.
        :param start: 시작 위치 (row, col)
        :param goal: 목표 위치 (row, col)
        :return: 경로 리스트
        """
        heapq.heappush(self.open_set, (0, start))  # (f값, 현재 노드)
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while self.open_set:
            # 우선순위 큐에서 f 값이 가장 낮은 노드 선택
            _, current = heapq.heappop(self.open_set)

            if current == goal:
                # 경로 복원
                return self.reconstruct_path(current)

            for neighbor in self.get_neighbors(current):
                tentative_g_score = (
                    g_score[current] + 1
                )  # 이동 비용 (여기서는 1로 고정)

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    # 더 좋은 경로 발견
                    self.came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(
                        neighbor, goal
                    )
                    heapq.heappush(self.open_set, (f_score[neighbor], neighbor))

        # 경로를 찾을 수 없음
        return []

    def reconstruct_path(self, current: Tuple[int, int]) -> List[Tuple[int, int]]:
        """경로 복원."""
        path = [current]
        while current in self.came_from:
            current = self.came_from[current]
            path.append(current)
        return path[::-1]


# 맵 예제 (0: 이동 가능, 1: 장애물)
grid = [
    [0, 0, 0, 0, 0],
    [0, 1, 1, 1, 0],
    [0, 0, 0, 0, 0],
    [0, 1, 1, 1, 0],
    [0, 0, 0, 0, 0],
]

# A* 초기화
astar = AStar(grid)

# 시작점과 목표점
start = (0, 0)
goal = (4, 4)

# 경로 찾기
path = astar.find_path(start, goal)
print("장애물 추가 전 경로:", path)

# 실시간 장애물 추가 후 경로 재탐색
astar.update_obstacle((3, 4))
path = astar.find_path(start, goal)
print("장애물 추가 후 경로:", path)
