using UnityEngine;

namespace RosSharp.Control
{
    public class ERP42Controller : MonoBehaviour
    {
        public GameObject frontLeftSteer;
        public GameObject frontRightSteer;
        public GameObject frontLeftWheel;
        public GameObject frontRightWheel;
        public GameObject rearLeftWheel;
        public GameObject rearRightWheel;

        private WheelCollider frontLeftWheelCollider;
        private WheelCollider frontRightWheelCollider;
        private WheelCollider rearLeftWheelCollider;
        private WheelCollider rearRightWheelCollider;

        public float maxLinearSpeed = 20f; // m/s
        public float maxRotationalSpeed = 1f; // rad/s

        private float inputSpeed = 0f;
        private float inputRotationSpeed = 0f;

        void Start()
        {
            // WheelCollider를 바퀴에 연결
            frontLeftWheelCollider = frontLeftWheel.GetComponent<WheelCollider>();
            frontRightWheelCollider = frontRightWheel.GetComponent<WheelCollider>();
            rearLeftWheelCollider = rearLeftWheel.GetComponent<WheelCollider>();
            rearRightWheelCollider = rearRightWheel.GetComponent<WheelCollider>();

            if (frontLeftWheelCollider == null || frontRightWheelCollider == null || rearLeftWheelCollider == null || rearRightWheelCollider == null)
            {
                Debug.LogError("Missing WheelCollider on one or more wheels!");
            }

            // WheelCollider 속성 설정
            if (frontLeftWheelCollider != null) SetWheelColliderProperties(frontLeftWheelCollider);
            if (frontRightWheelCollider != null) SetWheelColliderProperties(frontRightWheelCollider);
            if (rearLeftWheelCollider != null) SetWheelColliderProperties(rearLeftWheelCollider);
            if (rearRightWheelCollider != null) SetWheelColliderProperties(rearRightWheelCollider);
        }

        void FixedUpdate()
        {
            // 키보드 입력 처리
            KeyBoardUpdate();
        }

        private void SetWheelColliderProperties(WheelCollider wheelCollider)
        {
            // 서스펜션 거리 및 스프링 설정
            wheelCollider.suspensionDistance = 0.2f;
            JointSpring suspensionSpring = wheelCollider.suspensionSpring;
            suspensionSpring.spring = 0;
            suspensionSpring.damper = 0;
            suspensionSpring.targetPosition = 0.5f;
            wheelCollider.suspensionSpring = suspensionSpring;

            // 마찰 설정
            wheelCollider.mass = 20;
            wheelCollider.wheelDampingRate = 1f;
            wheelCollider.forwardFriction = GetFrictionCurve(1f, 1f, 1f, 1f);
            wheelCollider.sidewaysFriction = GetFrictionCurve(1f, 1f, 1f, 1f);
        }

        private WheelFrictionCurve GetFrictionCurve(float extremumSlip, float extremumValue, float asymptoteSlip, float asymptoteValue)
        {
            // 마찰 곡선 설정
            WheelFrictionCurve frictionCurve = new WheelFrictionCurve
            {
                extremumSlip = extremumSlip,
                extremumValue = extremumValue,
                asymptoteSlip = asymptoteSlip,
                asymptoteValue = asymptoteValue,
                stiffness = 1f
            };
            return frictionCurve;
        }

        private void KeyBoardUpdate()
        {
            // 전진/후진 속도 설정
            if (Input.GetKey(KeyCode.W))
            {
                inputSpeed = maxLinearSpeed;
            }
            else if (Input.GetKey(KeyCode.S))
            {
                inputSpeed = maxLinearSpeed * -1;
            }
            else
            {
                inputSpeed = 0;
            }

            // 회전 속도 설정
            if (Input.GetKey(KeyCode.A))
            {
                inputRotationSpeed = maxRotationalSpeed;
            }
            else if (Input.GetKey(KeyCode.D))
            {
                inputRotationSpeed = maxRotationalSpeed * -1;
            }
            else
            {
                inputRotationSpeed = 0;
            }

            // 로봇 입력 처리
            RobotInput(inputSpeed, inputRotationSpeed);
        }

        private void RobotInput(float speed, float rotSpeed)
        {
            // 조향 설정
            if (frontLeftSteer != null)
            {
                float adjustedRotation = rotSpeed * Mathf.Rad2Deg - 90;
                frontLeftSteer.transform.localEulerAngles = new Vector3(adjustedRotation, 0, 0);
            }

            if (frontRightSteer != null)
            {
                float adjustedRotation = rotSpeed * Mathf.Rad2Deg - 90;
                frontRightSteer.transform.localEulerAngles = new Vector3(adjustedRotation, 0, 0);
            }

            // 모든 바퀴 회전 속도 설정
            ApplyMotorTorque(frontLeftWheelCollider, speed);
            ApplyMotorTorque(frontRightWheelCollider, speed);
            ApplyMotorTorque(rearLeftWheelCollider, speed);
            ApplyMotorTorque(rearRightWheelCollider, speed);

            // 조향 각도 설정
            frontLeftWheelCollider.steerAngle = rotSpeed * Mathf.Rad2Deg;
            frontRightWheelCollider.steerAngle = rotSpeed * Mathf.Rad2Deg;

            Debug.Log($"Robot Input - Speed: {speed}, Rotation Speed: {rotSpeed}");
        }

        private void ApplyMotorTorque(WheelCollider wheelCollider, float speed)
        {
            // 바퀴에 motorTorque를 적용하여 속도 설정
            if (wheelCollider != null)
            {
                wheelCollider.motorTorque = speed;
            }
        }
    }
}
