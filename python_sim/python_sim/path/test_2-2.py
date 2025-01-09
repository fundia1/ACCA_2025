import heapq
import matplotlib.pyplot as plt
import math

# 8방향으로 이동: 상, 하, 좌, 우, 좌상, 좌하, 우상, 우하
d_row = [-1, 1, 0, 0, -1, -1, 1, 1]
d_col = [0, 0, -1, 1, -1, 1, -1, 1]


def a_star(matrix, start, goal):
    h = len(matrix)  # map 크기 (위 아래)
    w = len(matrix[0])  # map 크기 (좌 우)

    # 휴리스틱 코스트 테이블(전부 무한대로 초기화)
    heuristic_cost = [[float("inf")] * w for _ in range(h)]

    # 휴리스틱 코스트 구하기
    for i in range(h):
        for j in range(w):
            if matrix[i][j]:
                heuristic_cost[i][j] = get_euclidean_distance((i, j), goal)

    row, col = start
    goal_y, goal_x = goal

    visited = [[False] * w for _ in range(h)]

    heap = []
    heapq.heappush(heap, (heuristic_cost[row][col] + 0, row, col))  # h_cost + g_cost

    total_cost = 0
    # 어떤 노드에서 어떤 노드로 이동하는지 저장할 리스트
    came_from = []

    """
    heap이 비거나 목적 지점에 도착할 때까지 반복:
        heap에서 cost가 최소인 값을 꺼내서 방문처리를 한 후,
        유효한 인접 노드가 있으면 코스트를 계산해 힙에 넣는다.
    """
    while heap and (row, col) != (goal_x, goal_y):
        total_cost, row, col = heapq.heappop(heap)

        # Total Cost 에서 휴리스틱 코스트를 빼면 시작 지점에서 현재 지점까지의 실제 거리를 구할 수 있음
        depth = total_cost - heuristic_cost[row][col]

        # 방문처리
        visited[row][col] = True

        # 유효한 인접 노드가 있으면 코스트를 계산해 힙에 넣는다.
        for i in range(8):
            adjy = row + d_row[i]
            adjx = col + d_col[i]
            if is_vaild(matrix, visited, adjy, adjx):  # 이동 가능하면
                total_cost = heuristic_cost[adjy][adjx] + depth + 1  # 코스트 올리고
                came_from.append(((row, col), (adjy, adjx)))  # 리스트 넣고
                heapq.heappush(heap, (total_cost, adjy, adjx))  # 힙에 넣고

    # came_from을 역순으로 추적하여 최단 경로를 찾음
    from_y, from_x = came_from[-1][0]
    paths = []

    for i in range(len(came_from) - 1, -1, -1):
        from_coord, to_coord = came_from[i]
        to_y, to_x = to_coord

        if from_y == to_y and from_x == to_x:
            from_y, from_x = from_coord
            paths.insert(0, to_coord)

    return total_cost, paths, visited, heuristic_cost


def get_euclidean_distance(start, goal):
    return math.sqrt((start[0] - goal[0]) ** 2 + (start[1] - goal[1]) ** 2)


def is_vaild(matrix, visited, row, col):
    h = len(matrix)
    w = len(matrix[0])

    # out of bound 처리
    if not (0 <= row < h and 0 <= col < w):
        return False

    # 유효하지 않은 노드 처리
    if not matrix[row][col]:
        return False

    # 이미 방문한 노드 처리
    if visited[row][col]:
        return False

    return True


matrix = [
    [True, True, True, False, False, False, False],
    [True, False, True, False, False, False, False],
    [True, False, True, True, True, True, True],
    [True, False, True, False, False, False, True],
    [True, False, True, False, True, True, True],
    [True, False, True, False, True, False, False],
    [True, True, True, True, True, True, True],
]


start = (0, 0)
dest = (6, 6)
total_cost, paths, vis, heuristic_cost = a_star(matrix, start, dest)


def print_matrix(matrix, title="Matrix"):
    print(f"{title}:")
    for row in matrix:
        print(
            " ".join(
                f"{cell:.2f}" if isinstance(cell, float) else str(cell) for cell in row
            )
        )
    print()


def print_paths(paths, title="Paths"):
    print(f"{title}:")
    print(" -> ".join(f"{coord}" for coord in paths))
    print()


def print_visited(visited, title="Visited"):
    print(f"{title}:")
    for row in visited:
        print(" ".join("V" if cell else "." for cell in row))
    print()


def print_cost_and_heuristic(total_cost, visited, heuristic_cost, paths):
    print(f"Total Cost: {total_cost}")
    print_paths(paths)
    print_matrix(visited, title="Visited Nodes")
    print_matrix(heuristic_cost, title="Heuristic Costs")


# print_matrix(matrix)
# print_paths(paths)
print_visited(vis)


def path_to_dest(paths):
    print("Optimal Path:")
    print(" -> ".join(f"{coord}" for coord in paths))
    print()


# Print the optimal path only
path_to_dest(paths)
