from typing import List, Tuple, Callable
import math
import heapq


"""
1. 휴리스틱 코스트 H를 구한다. (목표 지점에서 각 노드까지의 유클리드 거리를 반올림)
2. 시작 지점에서 현재 지점까지의 실제 거리 G를 구한다.
3. 이하의 과정을 목표 지점에 도달할 때까지 반복한다.
4. 현재 지점을 방문 처리 후 현재 지점에 인접한 모든 노드의 Total Cost를 계산한다. Total Cost F는 G + H로 구할 수 있다.
5. 방문하지 않은 노드 중에서 Total Cost가 최소인 노드를 찾아 현재 지점을 이동한다.
"""


# 방향 벡터
d_row = (-1, 0, 1, 0)
d_col = (0, 1, 0, -1)

Coord = Tuple[int, int]


def a_star(
    matrix: List[List[int]], start: Coord, dest: Coord
) -> Tuple[int, List[Coord]]:
    global d_row
    global d_col

    h = len(matrix)
    w = len(matrix[0])

    # 휴리스틱 코스트 테이블
    heuristic_cost = [[float("inf")] * w for _ in range(h)]

    # 휴리스틱 코스트 구하기
    for i in range(h):
        for j in range(w):
            if matrix[i][j]:
                heuristic_cost[i][j] = round(get_euclidean_distance((i, j), dest))

    row, col = start
    dest_y, dest_x = dest

    vis = [[False] * w for _ in range(h)]

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
    while heap and (row, col) != (dest_y, dest_x):
        total_cost, row, col = heapq.heappop(heap)

        # Total Cost 에서 휴리스틱 코스트를 빼면 시작 지점에서 현재 지점까지의 실제 거리를 구할 수 있음
        depth = total_cost - heuristic_cost[row][col]

        # 방문 처리
        vis[row][col] = True

        # 유효한 인접 노드가 있으면 코스트를 계산해 힙에 넣는다.
        for i in range(4):
            adjy = row + d_row[i]
            adjx = col + d_col[i]
            if is_vaild(matrix, vis, adjy, adjx):
                total_cost = heuristic_cost[adjy][adjx] + depth + 1
                came_from.append(((row, col), (adjy, adjx)))
                heapq.heappush(heap, (total_cost, adjy, adjx))

    # came_from을 역순으로 추적하여 최단 경로를 찾음
    from_y, from_x = came_from[-1][0]
    paths = []

    for i in range(len(came_from) - 1, -1, -1):
        from_coord, to_coord = came_from[i]
        to_y, to_x = to_coord

        if from_y == to_y and from_x == to_x:
            from_y, from_x = from_coord
            paths.insert(0, to_coord)

    return total_cost, paths, vis, heuristic_cost


def get_euclidean_distance(pq1: Coord, pq2: Coord) -> float:
    p1, q1 = pq1
    p2, q2 = pq2

    return math.sqrt((p1 - p2) ** 2 + (q1 - q2) ** 2)


def is_vaild(
    matrix: List[List[bool]], vis: List[List[bool]], row: int, col: int
) -> bool:
    h = len(matrix)
    w = len(matrix[0])

    # out of bound 처리
    if not (0 <= row < h and 0 <= col < w):
        return False

    # 유효하지 않은 노드 처리
    if not matrix[row][col]:
        return False

    # 이미 방문한 노드 처리
    if vis[row][col]:
        return False

    return True


def _print_cost(matrix: List[List[int]]) -> None:
    h = len(matrix)
    w = len(matrix[0])

    print("- Heuristic Cost -")
    for i in range(h):
        for j in range(w):
            print("." if math.isinf(matrix[i][j]) else matrix[i][j], end=" ")
        print()
    print()


def _print_path(
    matrix: List[List[bool]], start: Coord, dest: Coord, title: str
) -> None:
    h = len(matrix)
    w = len(matrix[0])

    print(f"---- {title} ----")
    for i in range(h):
        for j in range(w):
            if (i, j) == start:
                print("S", end=" ")
            elif (i, j) == dest:
                print("G", end=" ")
            else:
                print("O" if matrix[i][j] else ".", end=" ")
        print()
    print()


_print_shortest_distance: Callable[[Coord, Coord, int], None] = (
    lambda start, dest, total_cost: print(f"{start} -> {dest} 최단 거리 : {total_cost}")
)


def _print_shortedst_path(
    matrix: List[List[bool]], paths: List[Coord], start: Coord, dest: Coord
) -> None:
    h = len(matrix)
    w = len(matrix[0])
    matrix = [["."] * w for _ in range(h)]

    for i in range(h):
        for j in range(w):
            if (i, j) == start:
                matrix[i][j] = "S"
            elif (i, j) == dest:
                matrix[i][j] = "G"

    prev_y, prev_x = start
    for path in paths:
        cur_y, cur_x = path
        # 해당 방향으로 화살표를 그림
        if prev_y < cur_y:
            matrix[cur_y][cur_x] = "↓"
        elif prev_y > cur_y:
            matrix[cur_y][cur_x] = "↑"
        elif prev_x < cur_x:
            matrix[cur_y][cur_x] = "→"
        elif prev_x > cur_x:
            matrix[cur_y][cur_x] = "←"

        prev_y, prev_x = cur_y, cur_x

    print("-- Shortest Path --")
    for i in range(h):
        for j in range(w):
            print(matrix[i][j], end=" ")
        print()
    print()


if __name__ == "__main__":
    matrix = [
        [True, True, True, False, False, False, False],
        [True, False, True, False, False, False, False],
        [True, False, True, True, True, True, True],
        [True, False, True, False, False, False, True],
        [True, False, True, False, True, True, True],
        [True, False, True, False, True, False, False],
        [True, True, True, True, True, True, True],
    ]

    start = (2, 2)
    dest = (6, 6)
    total_cost, paths, vis, heuristic_cost = a_star(matrix, start, dest)

    _print_path(matrix, start, dest, "Path")
    _print_cost(heuristic_cost)
    _print_path(vis, start, dest, "Visited")
    _print_shortest_distance(start, dest, total_cost)
    _print_shortedst_path(matrix, paths, start, dest)
