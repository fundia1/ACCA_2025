from ultralytics import YOLO
import cv2
import matplotlib.pyplot as plt
import numpy as np

# 학습된 모델 불러오기
model = YOLO("runs/detect/train6/weights/best.pt")  # best.pt 모델 경로

# 새로운 이미지 불러오기
image_path = "/home/jinju/labelImg/images/y_cone/WIN_20240325_17_42_28_Pro_001224.jpg"  # 테스트할 이미지 경로

# 모델로 추론
results = model(image_path)

# 추론 결과에서 첫 번째 결과 확인
result = results[0]


# if len(result.boxes) > 0:
#     print(f"Detected {len(result.boxes)} objects.")

#     # 원본 이미지를 RGBA로 변환
#     original_image = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)

#     # alpha 채널 추가 (투명도를 위한 채널)
#     original_image = cv2.cvtColor(original_image, cv2.COLOR_BGR2BGRA)

#     # 배경을 모두 투명하게 만들기
#     mask = np.zeros_like(original_image)  # 검은색 배경 생성

#     for box in result.boxes:
#         x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

#         # 객체 영역을 원본 이미지에서 추출하여 마스크에 덮어 씌우기
#         mask[int(y1) : int(y2), int(x1) : int(x2)] = original_image[
#             int(y1) : int(y2), int(x1) : int(x2)
#         ]

#     # 결과 이미지 표시
#     plt.imshow(mask)
#     plt.axis("off")
#     plt.show()
# else:
#     print("No objects detected.")
if len(result.boxes) > 0:
    print(f"Detected {len(result.boxes)} objects.")

    # 원본 이미지를 RGBA로 변환
    original_image = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
    original_image = cv2.cvtColor(original_image, cv2.COLOR_BGR2BGRA)

    # 마스크 생성
    mask = np.zeros_like(original_image)  # 검은색 배경

    for box in result.boxes:
        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

        # 바운딩 박스 영역을 추출하여 객체 영역을 마스크에 덮어 씌우기
        object_region = original_image[int(y1) : int(y2), int(x1) : int(x2)]
        mask[int(y1) : int(y2), int(x1) : int(x2)] = object_region

    # 폴리곤 형태로 색을 입히기 위한 준비
    for box in result.boxes:
        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

        # 바운딩 박스 내 객체 윤곽을 폴리곤 형태로 변환
        polygon_points = np.array(
            [[x1, y1], [x2, y1], [x2, y2], [x1, y2]], dtype=np.int32
        )
        polygon_points = polygon_points.reshape((-1, 1, 2))

        # # 객체 영역을 색으로 채우기
        # cv2.fillPoly(mask, [polygon_points], (255, 182, 193))  # 연한 핑크색

    # 결과 이미지 표시
    plt.imshow(cv2.cvtColor(mask, cv2.COLOR_BGRA2RGB))
    plt.axis("off")
    plt.show()

else:
    print("No objects detected.")
