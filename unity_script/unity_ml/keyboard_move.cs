using UnityEngine;
using UnityEngine.InputSystem;

namespace RosSharp.Control
{
    public class KeyboardMove : MonoBehaviour
    {
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
        public float brakeForce = 5000f; // 브레이크 토크

        private float inputSpeed = 0f;
        private float inputRotationSpeed = 0f;
        private bool isBraking = false;

        private float wheelRadius = 0.3f; // 바퀴 반지름 (m)
        private float wheelCircumference;

        private Keyboard keyboard;

        void Start()
        {
            keyboard = Keyboard.current;

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

            wheelCircumference = 2 * Mathf.PI * wheelRadius; // 바퀴 둘레 계산
        }

        void FixedUpdate()
        {
            // 키보드 입력 처리
            KeyBoardUpdate();
            // 바퀴 시각적 업데이트
            UpdateVisualWheels();
        }

        private void SetWheelColliderProperties(WheelCollider wheelCollider)
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

        private void KeyBoardUpdate()
        {
            if (keyboard == null)
            {
                Debug.LogError("Keyboard not found.");
                return;
            }

            // 스페이스바 입력 확인 (브레이크)
            if (keyboard.spaceKey.isPressed)
            {
                isBraking = true;
                ApplyBrake(brakeForce);
                inputSpeed = 0;
                inputRotationSpeed = 0;
                return;
            }
            else
            {
                isBraking = false;
                ApplyBrake(0); // 브레이크 해제
            }

            // 전진/후진 속도 설정
            if (keyboard.wKey.isPressed)
            {
                inputSpeed = maxLinearSpeed; // 전진
            }
            else if (keyboard.sKey.isPressed)
            {
                inputSpeed = -maxLinearSpeed; // 후진
            }
            else
            {
                inputSpeed = 0;
            }

            // 회전 속도 설정
            if (keyboard.aKey.isPressed)
            {
                inputRotationSpeed = -maxRotationalSpeed; // 좌회전
            }
            else if (keyboard.dKey.isPressed)
            {
                inputRotationSpeed = maxRotationalSpeed; // 우회전
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
            // 모든 바퀴 회전 속도 설정
            ApplyMotorTorque(frontLeftWheelCollider, speed);
            ApplyMotorTorque(frontRightWheelCollider, speed);
            ApplyMotorTorque(rearLeftWheelCollider, speed);
            ApplyMotorTorque(rearRightWheelCollider, speed);

            // 조향 각도 설정 (최대 28도 제한)
            float steerAngle = Mathf.Clamp(rotSpeed * Mathf.Rad2Deg, -28f, 28f);
            frontLeftWheelCollider.steerAngle = steerAngle;
            frontRightWheelCollider.steerAngle = steerAngle;

            Debug.Log($"Robot Input - Speed: {speed}, Rotation Speed: {rotSpeed}, Steer Angle: {steerAngle}");
        }

        private void ApplyMotorTorque(WheelCollider wheelCollider, float speed)
        {
            if (wheelCollider != null && !isBraking)
            {
                // speed를 motorTorque로 적용
                wheelCollider.motorTorque = speed;
            }
            else
            {
                // 브레이크 시 motorTorque를 0으로 설정
                wheelCollider.motorTorque = 0f;
            }
        }

        private void ApplyBrake(float brakeTorque)
        {
            if (frontLeftWheelCollider != null) frontLeftWheelCollider.brakeTorque = brakeTorque;
            if (frontRightWheelCollider != null) frontRightWheelCollider.brakeTorque = brakeTorque;
            if (rearLeftWheelCollider != null) rearLeftWheelCollider.brakeTorque = brakeTorque;
            if (rearRightWheelCollider != null) rearRightWheelCollider.brakeTorque = brakeTorque;

            Debug.Log($"Braking with force: {brakeTorque}");
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
                float wheelAngularSpeed = (wheelCollider.rpm * wheelRadius * 2f * Mathf.PI) / 60f; // m/s 단위로 회전 속도 계산
                wheel.transform.Rotate(Vector3.right, wheelAngularSpeed * Time.fixedDeltaTime, Space.Self);

                // 전륜 바퀴에 steerAngle 적용
                if (wheelCollider == frontLeftWheelCollider || wheelCollider == frontRightWheelCollider)
                {
                    wheel.transform.localRotation = Quaternion.Euler(0f, wheelCollider.steerAngle, 0f);
                }
            }
        }
    }
}
