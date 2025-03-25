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

    public float maxLinearSpeed = 25f; // km/h
    
    private bool isBraking = false; // 브레이크 상태 추적
    private float lastSteerAngle = 0f;  // 마지막 조향 각도 추적
    private float lastSpeed = 0f;  // 마지막 속도 추적
    
    private Rigidbody rb;

    public float currentSpeed = 0f;    // 현재 속도 (km/h)
    public float speedSmoothingFactor = 10f; // 속도 조정 비율 (0에서 1 사이)
    private float torqueFactor = 1f;   // 토크 계수 (최대 1로 설정)

    void Start()
    {
        // ROS 연결 설정
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<ControlMessageMsg>("cmd_msg", ControlMessageCallback);

        // WheelCollider 초기화
        frontLeftWheelCollider = frontLeftWheel.GetComponent<WheelCollider>();
        frontRightWheelCollider = frontRightWheel.GetComponent<WheelCollider>();
        rearLeftWheelCollider = rearLeftWheel.GetComponent<WheelCollider>();
        rearRightWheelCollider = rearRightWheel.GetComponent<WheelCollider>();

        if (frontLeftWheelCollider == null || frontRightWheelCollider == null || rearLeftWheelCollider == null || rearRightWheelCollider == null)
        {
            Debug.LogError("Missing WheelCollider on one or more wheels!");
        }

        rb = GetComponent<Rigidbody>();


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

        if (msg.gear == 2)
            lastSpeed = moveSpeed;
        else if (msg.gear == 0)
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
        float maxTorque = Mathf.Abs(lastSpeed) * 8f; 
        currentSpeed = rb.linearVelocity.magnitude * Mathf.Sign(rb.linearVelocity.z);
        Debug.Log($"Robot Input - Speed: {currentSpeed * 3.6f} km/h");
        float speedError = Mathf.Abs(lastSpeed - currentSpeed);
        float torqueFactor = 1.0f - Mathf.Log10(speedError + 1.0f); // 1 - log(n) 형태의 감속 곡선
        Debug.Log($"torqueFactor: {torqueFactor} ");
        // 토크가 0보다 작아지는 것을 방지
        torqueFactor = Mathf.Clamp(torqueFactor, 0.1f, 1.0f);
        
        float appliedTorque =Mathf.Clamp(Mathf.Abs(lastSpeed), 0.0f, 1.0f) * maxTorque * torqueFactor * Mathf.Sign(lastSpeed - currentSpeed);


        
        // 이전 메시지의 값을 반영하여 로봇을 움직이도록 처리
        ApplyWheelMovement(appliedTorque, lastSteerAngle);
        UpdateVisualWheels();
        
    }


    private void UpdateVisualWheels()
    {
        UpdateWheelRotation(frontLeftWheel, frontLeftWheelCollider);
        UpdateWheelRotation(frontRightWheel, frontRightWheelCollider);
        UpdateWheelRotation(rearLeftWheel, rearLeftWheelCollider);
        UpdateWheelRotation(rearRightWheel, rearRightWheelCollider);
    }

    private void UpdateWheelRotation(GameObject wheel, WheelCollider wheelCollider)
    {
        if (wheelCollider != null && wheel != null)
        {
            // WheelCollider의 RPM을 사용하여 실제 바퀴의 회전 각도 계산
            float wheelAngularSpeed = (wheelCollider.rpm * 0.3f * 2f * Mathf.PI) / 60f; // m/s 단위로 회전 속도 계산
            wheel.transform.Rotate(Vector3.right, wheelAngularSpeed * Time.fixedDeltaTime, Space.Self);

            // 전륜 바퀴에 steerAngle 적용
            if (wheelCollider == frontLeftWheelCollider || wheelCollider == frontRightWheelCollider)
            {
                wheel.transform.localRotation = Quaternion.Euler(0f, wheelCollider.steerAngle, 0f);
            }
        }
    }

}
