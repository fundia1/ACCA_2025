import open3d as o3d
import numpy as np
import json
import torch
from torch.utils.data import Dataset, DataLoader
import torch.nn as nn
import torch.optim as optim
import torch
from sklearn.metrics import accuracy_score


# PCD 파일 로드
def load_pcd(pcd_file):
    pcd = o3d.io.read_point_cloud(pcd_file)
    points = np.asarray(pcd.points)  # points는 (N, 3) 형태
    return points


# JSON 파일에서 Traffic Cone 객체 추출
def load_json(json_file):
    with open(json_file, "r") as f:
        data = json.load(f)
    return data


# Traffic Cone의 포인트들만 추출하는 함수
def get_traffic_cone_points(points, objects):
    traffic_cone_points = []
    for obj in objects:
        if obj["name"] == "traffic_cone":
            # 객체의 꼭지점(vertex) 좌표를 가져와서 포인트를 추가
            for vertex in obj["vertices"]:
                traffic_cone_points.append(vertex)
    return np.array(traffic_cone_points)


# PointCloudDataset 클래스 정의
class PointCloudDataset(Dataset):
    def __init__(self, points, labels):
        self.points = points
        self.labels = labels

    def __len__(self):
        return len(self.points)

    def __getitem__(self, idx):
        point = self.points[idx]
        label = self.labels[idx]
        return torch.tensor(point, dtype=torch.float32), torch.tensor(
            label, dtype=torch.long
        )


# 간단한 포인트클라우드 분류 모델 정의 (예시)
class PointCloudModel(nn.Module):
    def __init__(self):
        super(PointCloudModel, self).__init__()
        self.fc1 = nn.Linear(3, 64)  # 입력은 3차원 포인트
        self.fc2 = nn.Linear(64, 64)
        self.fc3 = nn.Linear(64, 2)  # 2개의 클래스 (배경, traffic_cone)

    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        x = self.fc3(x)
        return x


# PCD 파일 경로와 JSON 파일 경로
pcd_file = "/home/jinju/ws/src/python_sim/python_sim/lidar_deep_learning/traffic_come_pcd/000000.pcd"
json_file = "/home/jinju/ws/src/python_sim/python_sim/lidar_deep_learning/traffic_come_pcd/000000.json"

# PCD 파일에서 포인트 로드
points = load_pcd(pcd_file)

# JSON 파일에서 객체 정보 로드
data = load_json(json_file)

# Traffic Cone 포인트 추출
traffic_cone_points = get_traffic_cone_points(points, data["objects"])

# Traffic Cone에 대한 레이블 (모든 포인트는 traffic_cone으로 레이블링)
labels = np.ones(len(traffic_cone_points))  # 1: traffic_cone

# Dataset 생성
dataset = PointCloudDataset(traffic_cone_points, labels)
dataloader = DataLoader(dataset, batch_size=32, shuffle=True)

# 모델, 손실 함수, 옵티마이저 설정
model = PointCloudModel()
criterion = nn.CrossEntropyLoss()
optimizer = optim.Adam(model.parameters(), lr=0.001)

# 모델 학습
epochs = 200
for epoch in range(epochs):
    model.train()
    running_loss = 0.1
    for inputs, targets in dataloader:
        optimizer.zero_grad()
        outputs = model(inputs)  # 모델에 입력
        loss = criterion(outputs, targets)  # 손실 계산
        loss.backward()  # 역전파
        optimizer.step()  # 파라미터 업데이트

        running_loss += loss.item()

    print(f"Epoch [{epoch+1}/{epochs}], Loss: {running_loss/len(dataloader):.4f}")

# 학습된 모델 저장
torch.save(
    model.state_dict(),
    "/home/jinju/ws/src/python_sim/python_sim/lidar_deep_learning/traffic_cone_model.pth",
)
print("모델 학습 완료 및 가중치 저장 완료!")


def evaluate_model(test_loader, model):
    model.eval()  # 모델을 평가 모드로 설정
    correct = 0
    total = 0
    with torch.no_grad():  # 평가 중에는 기울기를 계산하지 않음
        for inputs, targets in test_loader:
            outputs = model(inputs)  # 모델에 입력
            _, predicted = torch.max(outputs, 1)  # 예측된 클래스
            total += targets.size(0)  # 전체 샘플 수
            correct += (predicted == targets).sum().item()  # 맞춘 샘플 수

    accuracy = 100 * correct / total  # 정확도 계산
    return accuracy


test_pcd_file = "/home/jinju/ws/src/python_sim/python_sim/lidar_deep_learning/traffic_come_pcd/000001.pcd"
test_json_file = "/home/jinju/ws/src/python_sim/python_sim/lidar_deep_learning/traffic_come_pcd/000001.json"

# 테스트 데이터 로딩
test_points = load_pcd(test_pcd_file)
test_data = load_json(test_json_file)

# Traffic Cone 포인트 추출
test_traffic_cone_points = get_traffic_cone_points(test_points, test_data["objects"])

# 레이블 (테스트 데이터는 실제로는 다를 수 있습니다)
test_labels = np.ones(len(test_traffic_cone_points))  # 1: traffic_cone

# 테스트 데이터셋 생성
test_dataset = PointCloudDataset(test_traffic_cone_points, test_labels)
test_loader = DataLoader(test_dataset, batch_size=32, shuffle=False)

# 모델 평가
accuracy = evaluate_model(test_loader, model)
print(f"Test Accuracy: {accuracy:.2f}%")
