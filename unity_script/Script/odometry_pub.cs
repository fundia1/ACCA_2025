using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.BuiltinInterfaces;

public class odometry_pub : MonoBehaviour
{
    [SerializeField] private string topicName = "/localization/kinematic_state";  // ROS2에서 사용할 토픽 이름
    [SerializeField] private float publishFrequency = 0.001f;  // 초당 메시지 발행 빈도

    private ROSConnection ros;
    private float timeElapsed = 0.0f;

    private OdometryMsg odometryMsg;

    private void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<OdometryMsg>(topicName);  // Odometry 메시지 등록

        odometryMsg = new OdometryMsg();
    }

    private void FixedUpdate()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed >= publishFrequency)
        {
            PublishOdometry();
            timeElapsed = 0.0f;
        }
    }

    private void PublishOdometry()
    {
        // 현재 시간 계산 (Time.time을 초로 사용하고 나노초는 계산하여 설정)
        float currentTime = Time.time;
        int sec = (int)currentTime;
        uint nanosec = (uint)((currentTime - sec) * 1e9f);  // 나노초로 변환

        // TimeMsg 생성하여 timestamp 설정
        TimeMsg timestamp = new TimeMsg { sec = sec, nanosec = nanosec };  // sec를 uint로 명시적 변환

        // 차량의 위치 및 회전 (Transform)
        Vector3 position = transform.position;
        Quaternion rotation = transform.rotation;

        // 차량의 속도 (Rigidbody를 이용해 속도 계산)
        Rigidbody rb = GetComponent<Rigidbody>();
        Vector3 velocity = transform.InverseTransformDirection(rb.linearVelocity);
       

        // 좌표계 변환: Unity -> ROS2
        // Unity의 Y축은 ROS2의 Z축이므로 Y와 Z를 반전시킴
        Vector3 rosPosition = new Vector3(-position.z, position.x, position.y); // Y와 Z 반전
        Quaternion rosRotation = new Quaternion(-rotation.z, rotation.x, -rotation.y, rotation.w); // Y와 Z 반전

        // 180도 회전 (PI 라디안, Y축 기준)
        Quaternion rotate180 = Quaternion.Euler(0, 0, 180);
        // 기존 회전에 180도 회전 추가
        rosRotation = rosRotation * rotate180;  // 기존 회전 + 180도 회전


        // Odometry 메시지 설정
        odometryMsg.header.stamp = timestamp;  // 시간 스탬프 설정
        odometryMsg.header.frame_id = "map";  // 기준 프레임을 map으로 설정

        // 차량의 위치 (pose.position)
        odometryMsg.pose.pose.position = new PointMsg { x = rosPosition.x, y = rosPosition.y, z = rosPosition.z };

        // 차량의 회전 (pose.orientation)
        odometryMsg.pose.pose.orientation = new QuaternionMsg
        {
            x = rosRotation.x,
            y = rosRotation.y,
            z = rosRotation.z,
            w = rosRotation.w
        };

        // 차량의 속도 (twist.linear 및 twist.angular)
        odometryMsg.twist.twist.linear = new Vector3Msg
        {
            x = velocity.z,
            y = velocity.x,
            z = velocity.y
        };

        // 회전 속도 (angular velocity)
        odometryMsg.twist.twist.angular = new Vector3Msg
        {
            x = 0,  // 회전 속도 x는 0으로 설정 (yaw 회전은 z축으로만)
            y = 0,
            z = 0  // 회전 속도 z는 0으로 설정 (여기서는 회전 속도는 차량의 회전 정보를 사용하지 않음)
        };

        // ROS2로 Odometry 메시지 발행
        ros.Publish(topicName, odometryMsg);

        // 디버그 로그로 정보 확인
        // Debug.Log($"Odometry - Position: {rosPosition}, Rotation: {rosRotation}, Velocity: {velocity}");
    }
}
