import astar


import math
import matplotlib.pyplot as plt


direction = [
    (-1, 0),
    (1, 0),
    (0, -1),
    (0, 1),
    (-1, -1),
    (-1, 1),
    (1, -1),
    (1, 1),
]  # 좌,우,아래,위,좌하,좌상,우하,우상

map = [
    [True, True, False, True, True, True, True, True, True, True],
    [True, True, False, True, True, True, True, True, False, True],
    [True, True, True, True, True, False, False, True, True, True],
]

start = (0, 0)
goal = (9, 2)
astar._print_map(map)


