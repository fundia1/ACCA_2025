import numpy as np
from PIL import Image
import os

os.chdir("/home/jinju/ws/src/python_sim/python_sim/map/occupied_grid")
# Occupancy Grid 데이터 생성 (예: 100x100 맵)
map_data = np.full((300, 50), 255, dtype=np.uint8)  # 전체를 흰색 (비점유 영역)으로 설정

# 맵 데이터를 이미지 파일로 저장
image = Image.fromarray(map_data)
image.save("map.pgm")


# import numpy as np
# from PIL import Image
# import os

# os.chdir("/home/jinju/ws/src/python_sim/python_sim/map/occupied_grid")

# # Occupancy Grid 데이터 생성 (예: 100x100 맵)
# map_data = np.full((100, 100), 255, dtype=np.uint8)  # 기본값: 흰색 (비점유)
# map_data[25:75, 25:75] = 0  # 장애물 추가 (검은색)
# map_data[40:60, 40:60] = 128  # 알 수 없는 영역 (회색)

# # 맵 데이터를 이미지 파일로 저장
# image = Image.fromarray(map_data)
# image.save("map.pgm")
