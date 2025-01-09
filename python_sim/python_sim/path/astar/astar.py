import heapq
import math


# Node 클래스 정의
class Node:
    def __init__(self, x, y, g=0, h=0, parent=None):
        self.x = x
        self.y = y
        self.g = g  # 시작 노드부터 현재 노드까지의 거리
        self.h = h  # 현재 노드에서 목표 노드까지의 예상 거리
        self.f = g + h  # f = g + h
        self.parent = parent

    def __lt__(self, other):
        return self.f < other.f


# 8방향으로 이동 (상, 하, 좌, 우, 대각선)
MOVES = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]


def heuristic(a, b):
    """휴리스틱: 두 점 사이의 유클리드 거리"""
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)


def astar(start, goal, obstacles):
    """A* 알고리즘을 사용하여 경로를 탐색하는 함수"""
    open_list = []
    closed_list = set()

    # 시작 노드를 초기화하고 open_list에 추가
    start_node = Node(start[0], start[1], g=0, h=heuristic(start, goal))
    goal_node = Node(goal[0], goal[1])

    heapq.heappush(open_list, start_node)

    while open_list:
        current_node = heapq.heappop(open_list)

        # 목표에 도달한 경우
        if current_node.x == goal_node.x and current_node.y == goal_node.y:
            path = []
            while current_node:
                path.append((current_node.x, current_node.y))
                current_node = current_node.parent
            return path[::-1]  # 역순으로 경로 반환

        closed_list.add((current_node.x, current_node.y))

        # 8방향으로 탐색
        for move in MOVES:
            nx, ny = current_node.x + move[0], current_node.y + move[1]

            # 장애물 체크 (주변 0.5 반경 내)
            if (nx, ny) in obstacles:
                continue

            if (nx, ny) not in closed_list:
                g_cost = current_node.g + 1  # 이동 비용 (여기서는 1로 설정)
                h_cost = heuristic(Node(nx, ny), goal_node)

                neighbor_node = Node(nx, ny, g=g_cost, h=h_cost, parent=current_node)

                if all(
                    neighbor_node.f < n.f for n in open_list if n.x == nx and n.y == ny
                ):
                    heapq.heappush(open_list, neighbor_node)

    return None  # 경로를 찾을 수 없는 경우


# 장애물 위치 예시
obstacles = {(2, 2), (3, 2), (4, 3), (5, 5)}

# 시작점과 목표점 설정
start = (0, 0)
goal = (6, 6)

# 경로 탐색
path = astar(start, goal, obstacles)

# 결과 출력
if path:
    print("Found path:", path)
else:
    print("No path found.")
