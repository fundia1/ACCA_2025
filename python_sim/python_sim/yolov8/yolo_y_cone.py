from ultralytics import YOLO
import cv2
import matplotlib.pyplot as plt

# 학습된 모델 불러오기
model = YOLO("runs/detect/train6/weights/best.pt")  # best.pt 모델 경로


# 새로운 이미지 불러오기
image_path = "/home/jinju/labelImg/images/y_cone/WIN_20240325_17_42_28_Pro_001224.jpg"  # 테스트할 이미지 경로

# 모델로 추론
results = model(image_path)

# 추론 결과에서 첫 번째 결과 확인
result = results[0]

# 검출된 객체가 있는지 확인
if len(result.boxes) > 0:
    print(f"Detected {len(result.boxes)} objects.")
else:
    print("No objects detected.")

# 결과 이미지 시각화 (바운딩 박스 포함)
annotated_image = result.plot()  # plot()을 사용하여 시각화된 이미지를 얻음

# Matplotlib으로 이미지 표시
plt.imshow(
    cv2.cvtColor(annotated_image, cv2.COLOR_BGR2RGB)
)  # OpenCV는 BGR 순서를 사용하므로 RGB로 변환
plt.axis("off")  # 축 숨기기
plt.show()

# 또는 이미지 저장하기
# cv2.imwrite("annotated_image.jpg", annotated_image)  # 저장
