import numpy as np
from PIL import Image
import os

os.chdir("/home/jinju/ws/src/python_sim/python_sim/map/occupied_grid")
# Occupancy Grid 데이터 생성 (예: 100x100 맵)

###k-city small
# map_data = np.full((300, 50), 255, dtype=np.uint8)  # 전체를 흰색 (비점유 영역)으로 설정

# # 맵 데이터를 이미지 파일로 저장
# image = Image.fromarray(map_data)
# image.save("map.pgm")


###school parking
map_data = np.full(
    (170, 100), 255, dtype=np.uint8
)  # 전체를 흰색 (비점유 영역)으로 설정

# 맵 데이터를 이미지 파일로 저장
image = Image.fromarray(map_data)
image.save("map.pgm")
