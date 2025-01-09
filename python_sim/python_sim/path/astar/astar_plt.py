import heapq
import math


class AStar:
    def __init__(self, obs_map=[], max_delta_yaw=0.5):
        self.move_distance = 0.1  # 이동 반경
        self.obs_range = 1  # 장애물 반경
        self.max_delta_yaw = max_delta_yaw  # delta yaw 제한

        self.start = (0, 0)
        self.goal = (0, 0)

        self.map = obs_map  # 장애물 맵
        self.vertical = abs(self.start[1] - self.goal[1])  # 맵 수직 길이
        self.horizontal = abs(self.start[0] - self.goal[0])  # 맵 수평 길이

        self.open_set = []  # heap 리스트
        self.came_from = {}  # path 경로 역 추적

    def heuristic(self, current, goal):  # 현재 위치에서 골까지 거리
        return math.sqrt((current[0] - goal[0]) ** 2 + (current[1] - goal[1]) ** 2)

    def reconstruct_path(self, current):
        """경로 역 추적."""
        path = [current]
        while current in self.came_from:
            current = self.came_from[current]
            path.append((current[0], current[1]))
        return path[::-1]

    def get_neighbors(self, current_pose, prev_yaw):
        neighbors = []
        directions = [
            (-self.move_distance, 0),
            (self.move_distance, 0),
            (0, -self.move_distance),
            (0, self.move_distance),
            (-self.move_distance, -self.move_distance),
            (-self.move_distance, self.move_distance),
            (self.move_distance, -self.move_distance),
            (self.move_distance, self.move_distance),
        ]  # 좌, 우, 하, 상, 왼쪽 대각선, 오른쪽 대각선

        for d in directions:
            # 한 칸 이동
            new_x = round(current_pose[0] + d[0], 1)
            new_y = round(current_pose[1] + d[1], 1)

            # 새로운 위치에서의 yaw 계산 (이동 후의 방향)
            delta_x = new_x - current_pose[0]
            delta_y = new_y - current_pose[1]
            new_yaw = math.atan2(delta_y, delta_x)

            # delta yaw가 max_delta_yaw를 넘지 않으면, 이동 가능한 경로로 추가
            if abs(new_yaw - prev_yaw) <= self.max_delta_yaw:
                if self.check_for_obstacle(current_pose):
                    neighbors.append((new_x, new_y, new_yaw))
        return neighbors

    def check_for_obstacle(self, current_pose):
        # 장애물 위치 리스트 순회
        for obstacle in self.map:
            distance = self.heuristic(current_pose, obstacle)
            if distance <= self.obs_range:
                return False  # 장애물이 범위 내에 있음
        return True  # 장애물 없음

    def is_valid_position(self, pose):
        # 시작점이나 목표점이 장애물과 너무 가까운지 확인
        for obstacle in self.map:
            if self.heuristic(pose, obstacle) <= self.obs_range:
                return False  # 너무 가까우면 유효하지 않음
        return True  # 유효한 위치

    def update_goal(self, pose):
        self.goal = pose
        return self.find_path()

    def update_start(self, pose):
        self.start = pose
        # return self.find_path()

    def find_path(self):

        if not self.is_valid_position(self.start):
            print("Starting position is too close to an obstacle.")
            return []

        if not self.is_valid_position(self.goal):
            print("Goal position is too close to an obstacle.")
            return []

        heapq.heappush(self.open_set, (0, self.start))  # (f값, 현재위치)
        g_cost = {self.start: 0}  # 이동 코스트
        f_cost = {
            self.start: self.heuristic(self.start, self.goal)
        }  # 이동 코스트 + 휴리스틱 코스트
        prev_yaw = 0  # 초기 yaw 값 (0, 0) 위치에서의 방향은 0

        while self.open_set:
            _, current = heapq.heappop(self.open_set)
            if self.heuristic(current, self.goal) <= 0.2:
                # 경로 역추적
                return self.reconstruct_path(current)

            for neighbor in self.get_neighbors(current, prev_yaw):

                try:
                    new_g_cost = g_cost[current] + self.heuristic(current, neighbor)
                except KeyError:
                    # KeyError가 발생하면 해당 위치를 무시하고 계속 진행
                    continue
                if (
                    neighbor not in g_cost or new_g_cost < g_cost[neighbor]
                ):  # 지난곳이 아니거나 코스트가 낮아지면
                    self.came_from[neighbor] = current
                    g_cost[neighbor] = new_g_cost
                    f_cost[neighbor] = new_g_cost + self.heuristic(neighbor, self.goal)
                    heapq.heappush(self.open_set, (f_cost[neighbor], neighbor))

            prev_yaw = neighbor[2]  # 이전 yaw 값을 업데이트

        return []  # 장애물이 많거나 경로가 길어질 수록 느려짐
