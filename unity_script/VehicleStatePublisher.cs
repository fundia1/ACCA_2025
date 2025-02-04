using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.BuiltinInterfaces;
public class VehicleStatePublisher : MonoBehaviour
{
    [SerializeField] private string topicName = "/vehicle/state";  // ROS2에서 사용할 토픽 이름
    [SerializeField] private float publishFrequency = 0.1f;  // 초당 메시지 발행 빈도

    private ROSConnection ros;
    private float timeElapsed = 0.0f;

    private TwistStampedMsg vel_raw;
    private TransformStampedMsg vehicle_transform;

    // 차량의 각 바퀴 및 차량의 몸체
    public GameObject frontLeftWheel;
    public GameObject frontRightWheel;
    public GameObject rearLeftWheel;
    public GameObject rearRightWheel;

    private WheelCollider frontLeftWheelCollider;
    private WheelCollider frontRightWheelCollider;
    private WheelCollider rearLeftWheelCollider;
    private WheelCollider rearRightWheelCollider;

    private Rigidbody rb;

    private void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistStampedMsg>(topicName);  // 속도 메시지 등록
        ros.RegisterPublisher<TransformStampedMsg>("/vehicle/transform");  // 차량 상태 메시지 등록

        vel_raw = new TwistStampedMsg();
        vehicle_transform = new TransformStampedMsg();

        // WheelCollider 초기화
        frontLeftWheelCollider = frontLeftWheel.GetComponent<WheelCollider>();
        frontRightWheelCollider = frontRightWheel.GetComponent<WheelCollider>();
        rearLeftWheelCollider = rearLeftWheel.GetComponent<WheelCollider>();
        rearRightWheelCollider = rearRightWheel.GetComponent<WheelCollider>();

        // 차량 Rigidbody 초기화
        rb = GetComponent<Rigidbody>();

        if (frontLeftWheelCollider == null || frontRightWheelCollider == null ||
            rearLeftWheelCollider == null || rearRightWheelCollider == null)
        {
            Debug.LogError("Missing WheelCollider on one or more wheels!");
        }
    }

    private void FixedUpdate()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed >= publishFrequency)
        {
            PublishVehicleState();
            timeElapsed = 0.0f;
        }
    }

    private void PublishVehicleState()
    {
        // 현재 시간 계산 (Time.time을 초로 사용하고 나노초는 계산하여 설정)
        float currentTime = Time.time;
        int sec = (int)currentTime;
        uint nanosec = (uint)((currentTime - sec) * 1e9f);  // 나노초로 변환

        // TimeMsg 생성하여 timestamp 설정
        TimeMsg timestamp = new TimeMsg { sec = sec, nanosec = nanosec };

        // 차량 속도 및 회전 속도 계산
        Vector3 velocity = rb.linearVelocity;
        float speed = velocity.magnitude;  // 차량의 속도는 속도 벡터의 크기

        // 앞바퀴 회전 속도 및 steer 값
        float frontLeftWheelRPM = frontLeftWheelCollider.rpm;
        float frontRightWheelRPM = frontRightWheelCollider.rpm;
        float steerAngle = frontLeftWheelCollider.steerAngle;

        // 차량의 위치 및 회전 (Transform)
        Vector3 position = transform.position;
        Quaternion rotation = transform.rotation;

        // 차량 상태 메시지 작성 (위치 및 회전)
        vehicle_transform.header.stamp = timestamp;  // 시간 스탬프 설정
        vehicle_transform.header.frame_id = "map";  // 기준 프레임을 map으로 설정
        vehicle_transform.transform.translation = new Vector3Msg { x = position.x, y = position.y, z = position.z };
        vehicle_transform.transform.rotation = new QuaternionMsg { x = rotation.x, y = rotation.y, z = rotation.z, w = rotation.w };

        // 속도 메시지 작성 (linear 및 angular velocity)
        vel_raw.header.stamp = timestamp;  // 시간 스탬프 설정
        vel_raw.twist.linear.x = speed;
        vel_raw.twist.linear.y = 0;  // y 방향 속도는 사용하지 않음
        vel_raw.twist.angular.z = steerAngle;  // steer 값이 회전 속도에 대응

        // ROS2로 메시지 발행
        ros.Publish(topicName, vel_raw);
        ros.Publish("/vehicle/transform", vehicle_transform);

        // 디버그 로그로 정보 확인
        Debug.Log($"Vehicle State - Position: {position}, Speed: {speed}, Steering Angle: {steerAngle}");
    }
}
