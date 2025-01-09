import heapq
import math
import matplotlib.pyplot as plt


class AStar:
    def __init__(self, start, goal, obs_map=[]):
        self.move_distance = 0.1
        self.obs_range = 0.5

        self.start = start
        self.goal = goal

        self.map = obs_map  # 장애물 맵
        self.vertical = abs(start[1] - goal[1])  # 맵 수직 길이
        self.horizontal = abs(start[0] - goal[0])  # 맵 수평 길이

        self.open_set = []  # heap 리스트
        self.came_from = {}  # path 경로 역 추적

    def heuristic(self, current, goal):  # 현재 위치에서 골까지 거리
        return math.sqrt((current[0] - goal[0]) ** 2 + (current[1] - goal[1]) ** 2)

    def reconstruct_path(self, current):
        """경로 역 추척."""
        path = [current]
        while current in self.came_from:
            current = self.came_from[current]
            path.append((current[0], current[1]))
        return path[::-1]

    def get_neighbors(self, current_pose):
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
        ]  # 좌, 우, 하, 상,,

        for d in directions:
            # 한칸 이동
            new_x = round(current_pose[0] + d[0], 1)
            new_y = round(current_pose[1] + d[1], 1)

            # 이동이 map 안에있고 장애물과 안 겹칠때
            if (
                # min(self.start[0], self.goal[0])
                # <= new_x
                # <= max(self.start[0], self.goal[0])
                # and min(self.start[1], self.goal[1])
                # <= new_x
                # <= max(self.start[1], self.goal[1])
                self.check_for_obstacle(current_pose)
            ):

                neighbors.append((new_x, new_y))
        return neighbors

    def check_for_obstacle(self, current_pose):
        # 장애물 위치 리스트 순회
        for obstacle in self.map:
            distance = self.heuristic(current_pose, obstacle)
            if distance <= self.obs_range:
                print(obstacle, current_pose)
                return False  # 장애물이 범위 내에 있음
            # if distance >= self.obs_range:
            #     return True  # 장애물이 범위 외에 있음
        return True  # 장애물 없음

    def update_obstacle(self, obstacle_pose):
        self.map.append(obstacle_pose)

    def find_path(self):
        heapq.heappush(self.open_set, (0, self.start))  # (f값,현재위치)
        g_cost = {self.start: 0}  # 이동 코스트
        f_cost = {
            self.start: self.heuristic(self.start, self.goal)
        }  # 이동 코스트 + 휴리스틱 코스트

        while self.open_set:
            _, current = heapq.heappop(self.open_set)
            if current == self.goal:
                # 경로 역추적
                return self.reconstruct_path(current)

            for neighbor in self.get_neighbors(current):
                new_g_cost = g_cost[current] + self.heuristic(
                    current, neighbor
                )  # 이동 비용 추가
                if (
                    neighbor not in g_cost or new_g_cost < g_cost[neighbor]
                ):  # 지난곳이 아니거나 코스트가 낮아지면
                    self.came_from[neighbor] = current
                    g_cost[neighbor] = new_g_cost
                    f_cost[neighbor] = new_g_cost + self.heuristic(neighbor, self.goal)
                    heapq.heappush(self.open_set, (f_cost[neighbor], neighbor))

        return []


obs_map = [(1, 1)]
start = (0, 0)
goal = (4, 4)


astar = AStar(start, goal, obs_map)


path = astar.find_path()
print(path)

# # 경로를 x, y 좌표로 분리
# x_coords = [point[0] for point in path]
# y_coords = [point[1] for point in path]


# x_obs = [point[0] for point in obs_map]
# y_obs = [point[1] for point in obs_map]


# # 그래프 그리기
# plt.plot(x_coords, y_coords, marker="o", color="b", label="Path")
# plt.scatter(x_obs, y_obs)
# plt.xlabel("X")
# plt.ylabel("Y")
# plt.title("Path Visualization")
# plt.grid(True)
# plt.legend()

# 그래프 표시
# plt.show()

# astar.update_obstacle((4, -3))
# path = astar.find_path()
# print(path)
