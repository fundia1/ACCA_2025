using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Erp42; // erp42_msgs 메시지 패키지

public class cmd_sub2 : MonoBehaviour
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
    public float maxSteeringAngle = 28f; // 최대 조향 각도 (도)

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

        // 마찰 설정 (기본값은 1.0으로 설정되어 있습니다. 이를 증가시키면 마찰력이 강해집니다.)
        SetWheelFriction(frontLeftWheelCollider);
        SetWheelFriction(frontRightWheelCollider);
        SetWheelFriction(rearLeftWheelCollider);
        SetWheelFriction(rearRightWheelCollider);
    }

    void SetWheelFriction(WheelCollider wheelCollider)
    {
        WheelFrictionCurve forwardFriction = wheelCollider.forwardFriction;
        WheelFrictionCurve sidewaysFriction = wheelCollider.sidewaysFriction;

        // 전방 마찰 (앞으로 미는 힘에 대한 마찰 설정)
        forwardFriction.stiffness = 1.5f;  // 마찰 강도를 증가시켜 미끄러짐 방지
        wheelCollider.forwardFriction = forwardFriction;

        // 측면 마찰 (회전하는 바퀴의 측면에 대한 마찰 설정)
        sidewaysFriction.stiffness = 1.5f;  // 마찰 강도를 증가시켜 미끄러짐 방지
        wheelCollider.sidewaysFriction = sidewaysFriction;
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

        // 조향 각도를 제한하여 최대 28도를 넘지 않도록 설정
        steerAngle = Mathf.Clamp(steerAngle, -maxSteeringAngle, maxSteeringAngle);

        // 조향 변화율 제한 적용
        float steerChange = steerAngle - lastSteerAngle;
        float maxSteerChange = maxSteeringRate * Time.deltaTime;
        steerChange = Mathf.Clamp(steerChange, -maxSteerChange, maxSteerChange);
        steerAngle = lastSteerAngle + steerChange;

        lastSteerAngle = steerAngle;
    }

    private void ApplyRobotMovement(float moveSpeed, float steerAngle)
    {
        float deltaTime = Time.deltaTime;

        // 각속도 계산 (회전 반경에 따라 차량 회전 속도 계산)
        float radius = (Mathf.Abs(steerAngle) > 0.01f) ? wheelBase / Mathf.Tan(steerAngle * Mathf.Deg2Rad) : float.MaxValue;
        float angularVelocity = moveSpeed / radius;  // 각속도 계산

        // 회전만 적용하고 이동은 WheelCollider 회전으로 대체
        frontLeftWheelCollider.steerAngle = steerAngle;
        frontRightWheelCollider.steerAngle = steerAngle;

        // 바퀴 회전 처리 (속도에 맞게 회전)
        frontLeftWheelCollider.motorTorque = moveSpeed;
        frontRightWheelCollider.motorTorque = moveSpeed;
        rearLeftWheelCollider.motorTorque = moveSpeed;
        rearRightWheelCollider.motorTorque = moveSpeed;

        // 바퀴 회전 상태 적용 (물리적 회전)
        ApplyWheelRotation(frontLeftWheel, frontLeftWheelCollider);
        ApplyWheelRotation(frontRightWheel, frontRightWheelCollider);
        ApplyWheelRotation(rearLeftWheel, rearLeftWheelCollider);
        ApplyWheelRotation(rearRightWheel, rearRightWheelCollider);
    }

    private void ApplyWheelRotation(GameObject wheel, WheelCollider wheelCollider)
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
        // fixedPosition.y = 0.31f;  // 원하는 고정된 y 값 (여기서는 0)
        robot.transform.position = fixedPosition;

        // WheelCollider 회전 업데이트
        ApplyWheelRotation(frontLeftWheel, frontLeftWheelCollider);
        ApplyWheelRotation(frontRightWheel, frontRightWheelCollider);
        ApplyWheelRotation(rearLeftWheel, rearLeftWheelCollider);
        ApplyWheelRotation(rearRightWheel, rearRightWheelCollider);
    }
}
