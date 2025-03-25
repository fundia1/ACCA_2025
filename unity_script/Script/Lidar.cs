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
    public float maxDistance = 200f;     // 레이의 최대 거리
    public float rotationSpeed = 10f;    // LiDAR 회전 속도 (도/초)

    private List<Vector3> lidarPoints = new List<Vector3>(); // LiDAR 포인트 데이터 저장
    private PointCloud2Msg lidarMessage; // ROS2 메시지
    private float angleStepHorizontal;  // 수평 레이 간의 각도
    private float angleStepVertical;

    private readonly float[] vlp32_vertical_angles = new float[]
    {
        +15.000f, +10.870f, +7.230f, +4.290f, +2.370f, +1.250f, +0.330f, -0.690f,
        -1.610f, -2.530f, -3.450f, -4.370f, -5.290f, -6.210f, -7.130f, -8.050f,
        -8.970f, -9.890f, -10.810f, -11.730f, -12.650f, -13.570f, -14.490f, -15.410f,
        -16.330f, -17.250f, -18.170f, -19.090f, -20.010f, -21.930f, -23.850f, -25.000f
    };

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
            for (int j = 0; j < vlp32_vertical_angles.Length; j++)
            {
                float horizontalAngle = i * angleStepHorizontal;
                float verticalAngle = vlp32_vertical_angles[j];

                Vector3 direction = new Vector3(
                    Mathf.Cos(Mathf.Deg2Rad * horizontalAngle) * Mathf.Cos(Mathf.Deg2Rad * verticalAngle),
                    Mathf.Sin(Mathf.Deg2Rad * verticalAngle),
                    Mathf.Sin(Mathf.Deg2Rad * horizontalAngle) * Mathf.Cos(Mathf.Deg2Rad * verticalAngle)
                );

                if (Physics.Raycast(transform.position, direction, out RaycastHit hit, maxDistance))
                {
                    if (hit.collider.gameObject.tag != "Plane" && hit.collider.gameObject.tag != "road")
                    {
                        lidarPoints.Add(hit.point);
                    }
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
            // x, y, z 좌표 순으로 데이터를 byte 배열에 복사
            Buffer.BlockCopy(BitConverter.GetBytes(-point.z), 0, lidarMessage.data, index, 4); // z
            index += 4;
            Buffer.BlockCopy(BitConverter.GetBytes(point.x), 0, lidarMessage.data, index, 4);   // x
            index += 4;
            Buffer.BlockCopy(BitConverter.GetBytes(point.y), 0, lidarMessage.data, index, 4);   // y
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

        lidarMessage.row_step = (uint)(lidarMessage.point_step * pointCount); // uint로 캐스팅
        lidarMessage.width = (uint)pointCount;  // uint로 캐스팅
        lidarMessage.height = 1;  // LiDAR는 1개의 행
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
