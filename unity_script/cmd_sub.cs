using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Erp42; // erp42_msgs 메시지 패키지

public class cmd_sub : MonoBehaviour
{
    public GameObject robot;  // ERP42 모델
    public GameObject frontLeftWheel;
    public GameObject frontRightWheel;
    public GameObject rearLeftWheel;
    public GameObject rearRightWheel;

    

    private WheelCollider frontLeftWheelCollider;
    private WheelCollider frontRightWheelCollider;
    private WheelCollider rearLeftWheelCollider;
    private WheelCollider rearRightWheelCollider;

    private ROSConnection ros;

    public float speedFactor = 0.01f;   // 속도 변환 (ROS -> Unity)
    public float steerFactor = 0.1f;    // 조향 변환
    public float maxBrakeForce = 5000f; // 최대 브레이크 힘
    public float wheelBase = 2.5f;      // 바퀴 간 거리 (미터)
    public float vehicleMass = 1000f;   // 차량 질량 (kg)

    public float maxSteeringRate = 80f; // 초당 최대 조향 각 속도 (도)
    




    private bool isBraking = false; // 브레이크 상태 추적
    private float lastSteerAngle = 0f;  // 마지막 조향 각도 추적
    private float lastSpeed = 0f;  // 마지막 속도 추적
    private float leftWheelBaseAngle = -90f;  // 왼쪽 바퀴의 기본 각도
    private float rightWheelBaseAngle = 90f; // 오른쪽 바퀴의 기본 각도

    void Start()
    {
        // ROS 연결 설정
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<ControlMessageMsg>("erp42_cmd", ControlMessageCallback);

        // WheelCollider 초기화
        frontLeftWheelCollider = frontLeftWheel.GetComponent<WheelCollider>();
        frontRightWheelCollider = frontRightWheel.GetComponent<WheelCollider>();
        rearLeftWheelCollider = rearLeftWheel.GetComponent<WheelCollider>();
        rearRightWheelCollider = rearRightWheel.GetComponent<WheelCollider>();

        if (frontLeftWheelCollider == null || frontRightWheelCollider == null || rearLeftWheelCollider == null || rearRightWheelCollider == null)
        {
            Debug.LogError("Missing WheelCollider on one or more wheels!");
        }
    }

  void ControlMessageCallback(ControlMessageMsg msg)
    {
        float moveSpeed = msg.speed * speedFactor;
        float steerAngle = msg.steer * steerFactor;
        isBraking = msg.brake > 0;  

        Debug.Log($"[ERP42_CMD] MORA: {msg.mora}, ESTOP: {msg.estop}, GEAR: {msg.gear}, " +
                $"SPEED: {msg.speed}, STEER: {msg.steer}, BRAKE: {msg.brake}, ALIVE: {msg.alive}");

        if (msg.estop == 1)
        {   
            lastSpeed = 0;
            ApplyBrake(maxBrakeForce);
            return;
        }

        if (msg.gear == 0)
            lastSpeed = moveSpeed;
        else if (msg.gear == 1)
            lastSpeed = -moveSpeed;

        if (isBraking)
        {
            float brakeForce = Mathf.Lerp(0, maxBrakeForce, msg.brake / 200f);
            ApplyBrake(brakeForce);
        }
        else
        {
            ApplyBrake(0);
        }

        // 조향 변화율 제한 적용
        
        float steerChange = steerAngle - lastSteerAngle;
        float maxSteerChange = maxSteeringRate * Time.deltaTime;

        
        
        steerChange = Mathf.Clamp(steerChange, -maxSteerChange, maxSteerChange);
        steerAngle = lastSteerAngle + steerChange;
        if (Mathf.Abs(steerChange) > maxSteerChange)
        {
            steerAngle = lastSteerAngle + Mathf.Sign(steerChange) * maxSteerChange;
        }

        // 속도에 따른 조향 제한 적용
        // float dynamicSteeringLimit = Mathf.Lerp(15f, 35f, 1f - Mathf.Clamp01(Mathf.Abs(lastSpeed) / 5f)); 
        // steerAngle = Mathf.Clamp(steerAngle, -dynamicSteeringLimit, dynamicSteeringLimit);

        lastSteerAngle = steerAngle;
    }

    private void ApplyRobotMovement(float moveSpeed, float steerAngle)
    {
        float deltaTime = Time.deltaTime;

        // 조향 속도 제한 (속도에 따라 조향 변화 속도 조정)
        float maxSteerChange = maxSteeringRate * deltaTime;
        float steerChange = steerAngle - lastSteerAngle;
        steerChange = Mathf.Clamp(steerChange, -maxSteerChange, maxSteerChange);
        steerAngle = lastSteerAngle + steerChange;
        lastSteerAngle = steerAngle; // 조향 값 업데이트

        // 차량의 회전 반경 계산 (steerAngle이 0이면 회전 반경 무한)
        float radius = (Mathf.Abs(steerAngle) > 0.01f) ? wheelBase / Mathf.Tan(steerAngle * Mathf.Deg2Rad) : float.MaxValue;

        // 각속도 계산 (회전 반경에 따라 차량 회전 속도 계산)
        float angularVelocity = moveSpeed / radius;  // 각속도 계산

        // 차량 이동 (앞으로 이동)
        float displacement = moveSpeed * deltaTime;
        robot.transform.Translate(Vector3.forward * displacement);

        // 차량 회전 (조향 변화 속도 적용)
        float turnAngle = angularVelocity * deltaTime * Mathf.Rad2Deg;

        // 제한된 회전 각도 적용
        turnAngle = Mathf.Clamp(turnAngle, -maxSteeringRate * deltaTime, maxSteeringRate * deltaTime);

        // Rigidbody를 이용하여 회전 처리
        Rigidbody rb = robot.GetComponent<Rigidbody>();
        if (rb != null)
        {
            // 회전 직후 미끄러짐 방지
            rb.angularVelocity = Vector3.zero;

            // 회전 적용 (물리적 회전, 회전 각도를 계산하여 적용)
            rb.MoveRotation(rb.rotation * Quaternion.Euler(0, turnAngle, 0));
        }



        // 바퀴 회전 및 조향 적용
        ApplyWheelRotation(frontLeftWheel, frontLeftWheelCollider, steerAngle, leftWheelBaseAngle);
        ApplyWheelRotation(frontRightWheel, frontRightWheelCollider, steerAngle, rightWheelBaseAngle);
        ApplyWheelRotation(rearLeftWheel, rearLeftWheelCollider, steerAngle, leftWheelBaseAngle);
        ApplyWheelRotation(rearRightWheel, rearRightWheelCollider, steerAngle, rightWheelBaseAngle);
    }



    private void ApplyWheelRotation(GameObject wheel, WheelCollider wheelCollider, float steerAngle, float baseAngle)
    {
        if (wheelCollider != null && wheel != null)
        {
            // 바퀴의 위치와 회전 정보를 가져오기 위한 WheelCollider.GetWorldPose 사용
            Vector3 wheelPos = wheelCollider.transform.position;
            Quaternion wheelRot = wheelCollider.transform.rotation;

            wheelCollider.GetWorldPose(out wheelPos, out wheelRot);

            // 바퀴 위치 업데이트
            wheel.transform.position = wheelPos;
            wheel.transform.rotation = wheelRot;

            // 앞바퀴는 steerAngle에 맞게 회전 (전륜 조향)
            if (wheelCollider == frontLeftWheelCollider || wheelCollider == frontRightWheelCollider)
            {
                // 왼쪽과 오른쪽 바퀴에 각각의 기본각도와 steerAngle을 더하여 회전
                if (wheelCollider == frontLeftWheelCollider)
                {
                    float finalSteerAngle = baseAngle + steerAngle;  // 왼쪽 바퀴
                    wheel.transform.localRotation = Quaternion.Euler(0f, finalSteerAngle, 0f);
                }
                else if (wheelCollider == frontRightWheelCollider)
                {
                    float finalSteerAngle = baseAngle + steerAngle; // 오른쪽 바퀴 (반대 방향으로 조향)
                    wheel.transform.localRotation = Quaternion.Euler(0f, finalSteerAngle, 0f);
                }
            }
        }
    }

    private void ApplyBrake(float brakeTorque)
    {
        // 모든 바퀴에 브레이크 토크 적용
        frontLeftWheelCollider.brakeTorque = brakeTorque;
        frontRightWheelCollider.brakeTorque = brakeTorque;
        rearLeftWheelCollider.brakeTorque = brakeTorque;
        rearRightWheelCollider.brakeTorque = brakeTorque;

        if (brakeTorque > 0)
        {
            Debug.Log($"Braking with force: {brakeTorque}");
        }
    }

    void FixedUpdate()
    {
        // 이전 메시지의 값을 반영하여 로봇을 움직이도록 처리
        ApplyRobotMovement(lastSpeed, lastSteerAngle);

        // 차량의 y 값을 고정
        Vector3 fixedPosition = robot.transform.position;
        fixedPosition.y = 0.31f;  // 원하는 고정된 y 값 (여기서는 0)
        robot.transform.position = fixedPosition;

        // WheelCollider 회전 업데이트
        ApplyWheelRotation(frontLeftWheel, frontLeftWheelCollider, lastSteerAngle, leftWheelBaseAngle);
        ApplyWheelRotation(frontRightWheel, frontRightWheelCollider, lastSteerAngle, rightWheelBaseAngle);
        ApplyWheelRotation(rearLeftWheel, rearLeftWheelCollider, lastSteerAngle, leftWheelBaseAngle);
        ApplyWheelRotation(rearRightWheel, rearRightWheelCollider, lastSteerAngle, rightWheelBaseAngle);
    }

}
