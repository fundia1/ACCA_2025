using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using System.Collections.Generic;
using RosMessageTypes.BuiltinInterfaces;

public class Velodyne32LidarPublisher : MonoBehaviour
{
    private ROSConnection rosConnection; // ROS와 통신을 위한 ROSConnection
    public string lidarTopic = "/velodyne_points"; // ROS2 토픽 이름
    public int numRaysHorizontal = 360;  // 수평 레이 개수
    public int numRaysVertical = 32;     // 수직 레이 개수 (Velodyne 32)
    public float maxDistance = 100f;     // 레이의 최대 거리
    public float rotationSpeed = 10f;    // LiDAR 회전 속도 (도/초)

    private List<Vector3> lidarPoints = new List<Vector3>(); // LiDAR 포인트 데이터 저장
    private PointCloud2Msg lidarMessage; // ROS2 메시지
    private float angleStepHorizontal;  // 수평 레이 간의 각도
    private float angleStepVertical;    // 수직 레이 간의 각도

    void Start()
    {
        // ROS 연결 초기화
        rosConnection = ROSConnection.instance;

        // ROS2 퍼블리셔 등록
        rosConnection.RegisterPublisher<PointCloud2Msg>(lidarTopic);

        // 각도 계산
        angleStepHorizontal = 360f / numRaysHorizontal;
        angleStepVertical = 40f / numRaysVertical; // Velodyne 32는 수직 40도 범위

        // PointCloud2 메시지 초기화
        lidarMessage = new PointCloud2Msg();
    }

    void Update()
    {
        lidarPoints.Clear();

        // LiDAR 포인트 데이터 생성
        for (int i = 0; i < numRaysHorizontal; i++)
        {
            for (int j = 0; j < numRaysVertical; j++)
            {
                // 수평 및 수직 각도 계산
                float horizontalAngle = i * angleStepHorizontal;
                float verticalAngle = (j - numRaysVertical / 2) * angleStepVertical;

                // 방향 계산
                Vector3 direction = new Vector3(
                    Mathf.Cos(Mathf.Deg2Rad * horizontalAngle) * Mathf.Cos(Mathf.Deg2Rad * verticalAngle),
                    Mathf.Sin(Mathf.Deg2Rad * verticalAngle),
                    Mathf.Sin(Mathf.Deg2Rad * horizontalAngle) * Mathf.Cos(Mathf.Deg2Rad * verticalAngle)
                );

                // Raycast로 장애물 감지
                if (Physics.Raycast(transform.position, direction, out RaycastHit hit, maxDistance))
                {
                    if (hit.collider.gameObject.name != "Plane")
                    {
                        lidarPoints.Add(hit.point);
                    }
                }
                else
                {
                    // 장애물이 없으면 최대 거리 포인트 추가
                    // lidarPoints.Add(transform.position + direction * maxDistance);
                }
            }
        }

        // LiDAR 메시지 업데이트
        UpdateLidarMessage();

        // ROS2로 메시지 퍼블리시
        rosConnection.Publish(lidarTopic, lidarMessage);

        // LiDAR 회전 (옵션)
        transform.Rotate(Vector3.up, rotationSpeed * Time.deltaTime);
    }

    private void UpdateLidarMessage()
    {
        int pointCount = lidarPoints.Count;
        lidarMessage.data = new byte[pointCount * 12]; // x, y, z (float32 각각 4바이트) * 포인트 수
        int index = 0;

        foreach (Vector3 point in lidarPoints)
        {
            Buffer.BlockCopy(BitConverter.GetBytes(-point.z), 0, lidarMessage.data, index, 4);
            index += 4;
            Buffer.BlockCopy(BitConverter.GetBytes(point.x), 0, lidarMessage.data, index, 4);
            index += 4;
            Buffer.BlockCopy(BitConverter.GetBytes(point.y), 0, lidarMessage.data, index, 4);
            index += 4;
        }

        // PointCloud2 필드 설정
        lidarMessage.fields = new PointFieldMsg[]
        {
            new PointFieldMsg { name = "x", offset = 0, datatype = PointFieldMsg.FLOAT32, count = 1 },
            new PointFieldMsg { name = "y", offset = 4, datatype = PointFieldMsg.FLOAT32, count = 1 },
            new PointFieldMsg { name = "z", offset = 8, datatype = PointFieldMsg.FLOAT32, count = 1 },
        };
        lidarMessage.point_step = 12; // x, y, z 각각 4바이트
        // pointCount가 int일 수 있기 때문에 uint로 명시적 변환이 필요
        lidarMessage.row_step = (uint)(lidarMessage.point_step * pointCount); // long에서 uint로 명시적 변환
        lidarMessage.height = 1;  // LiDAR는 1개의 행
        lidarMessage.width = (uint)pointCount;  // int에서 uint로 명시적 변환

        lidarMessage.is_dense = true;

        // 타임스탬프 설정
        double currentTime = Time.timeAsDouble;
        int sec = (int)currentTime;
        uint nanosec = (uint)((currentTime - sec) * 1e9);

        lidarMessage.header.stamp = new TimeMsg
        {
            sec = sec,
            nanosec = nanosec
        };

        // 프레임 ID 설정
        lidarMessage.header.frame_id = "map";
    }


    void OnDrawGizmos()
    {
        Gizmos.color = Color.green;

        foreach (Vector3 point in lidarPoints)
        {
            Gizmos.DrawSphere(point, 0.1f);
        }
    }
}
