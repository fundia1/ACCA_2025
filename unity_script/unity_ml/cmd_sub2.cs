using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Erp42;

public class cmd_sub2 : MonoBehaviour
{
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

    private bool isBraking = false; // 브레이크 상태 추적
    private float lastSteerAngle = 0f;  // 마지막 조향 각도 추적
    private float lastSpeed = 0f;  // 마지막 속도 추적

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

        // 마찰 설정
        SetWheelFriction(frontLeftWheelCollider);
        SetWheelFriction(frontRightWheelCollider);
        SetWheelFriction(rearLeftWheelCollider);
        SetWheelFriction(rearRightWheelCollider);
    }

    void SetWheelFriction(WheelCollider wheelCollider)
    {
        // 서스펜션 설정 강화
        wheelCollider.suspensionDistance = 0.3f; // 서스펜션 거리 증가
        JointSpring suspensionSpring = wheelCollider.suspensionSpring;
        suspensionSpring.spring = 3500;  // 서스펜션 스프링 강도 증가
        suspensionSpring.damper = 900;  // 서스펜션 댐퍼 강도 증가
        suspensionSpring.targetPosition = 0.5f;  // 목표 위치 설정
        wheelCollider.suspensionSpring = suspensionSpring;

        // WheelCollider 속성 설정
        wheelCollider.mass = 25;  // 바퀴 질량 증가
        wheelCollider.wheelDampingRate = 1f;

        // 마찰력 설정 강화
        wheelCollider.forwardFriction = GetFrictionCurve(0.5f, 1f, 0.3f, 0.8f); // 전방 마찰력 강도 증가
        wheelCollider.sidewaysFriction = GetFrictionCurve(0.5f, 1f, 0.3f, 0.8f); // 측면 마찰력 강도 증가
    }
    private WheelFrictionCurve GetFrictionCurve(float extremumSlip, float extremumValue, float asymptoteSlip, float asymptoteValue)
    {
        WheelFrictionCurve frictionCurve = new WheelFrictionCurve
        {
            extremumSlip = extremumSlip,
            extremumValue = extremumValue,
            asymptoteSlip = asymptoteSlip,
            asymptoteValue = asymptoteValue,
            stiffness = 1.5f // 마찰력 강도 증가
        };
        return frictionCurve;
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
        steerAngle = Mathf.Clamp(steerAngle, -28f, 28f);

        lastSteerAngle = steerAngle;
    }

    private void ApplyWheelMovement(float moveSpeed, float steerAngle)
    {
        // WheelCollider를 사용하여 바퀴에 회전력 적용
        frontLeftWheelCollider.motorTorque = moveSpeed;
        frontRightWheelCollider.motorTorque = moveSpeed;
        rearLeftWheelCollider.motorTorque = moveSpeed;
        rearRightWheelCollider.motorTorque = moveSpeed;

        // 조향 각도 적용 (스티어링)
        frontLeftWheelCollider.steerAngle = steerAngle;
        frontRightWheelCollider.steerAngle = steerAngle;

       
    }

   
    private void ApplyBrake(float brakeTorque)
    {
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
        ApplyWheelMovement(lastSpeed, lastSteerAngle);

        
    }
}
