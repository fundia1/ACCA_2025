
using UnityEngine;

namespace RosSharp.Control
{
    public class keyboard_move : MonoBehaviour
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
            wheelCollider.suspensionDistance = 0.2f;
            JointSpring suspensionSpring = wheelCollider.suspensionSpring;
            suspensionSpring.spring = 0;
            suspensionSpring.damper = 0;
            suspensionSpring.targetPosition = 0.5f;
            wheelCollider.suspensionSpring = suspensionSpring;

            wheelCollider.mass = 20;
            wheelCollider.wheelDampingRate = 1f;
            wheelCollider.forwardFriction = GetFrictionCurve(1f, 1f, 1f, 1f);
            wheelCollider.sidewaysFriction = GetFrictionCurve(1f, 1f, 1f, 1f);
        }

        private WheelFrictionCurve GetFrictionCurve(float extremumSlip, float extremumValue, float asymptoteSlip, float asymptoteValue)
        {
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
            // 스페이스바 입력 확인 (브레이크)
            if (Input.GetKey(KeyCode.Space))
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
            if (Input.GetKey(KeyCode.W))
            {
                inputSpeed = maxLinearSpeed; // 전진
            }
            else if (Input.GetKey(KeyCode.S))
            {
                inputSpeed = -maxLinearSpeed; // 후진
            }
            else
            {
                inputSpeed = 0;
            }

            // 회전 속도 설정
            if (Input.GetKey(KeyCode.A))
            {
                inputRotationSpeed = -maxRotationalSpeed; // 좌회전
            }
            else if (Input.GetKey(KeyCode.D))
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
                wheelCollider.motorTorque = speed;
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
                // WheelCollider의 RPM과 반지름을 사용하여 각속도 계산
                float wheelAngularSpeed = (wheelCollider.rpm * 360f) / 60f; // deg/s
                wheel.transform.Rotate(Vector3.right, wheelAngularSpeed * Time.fixedDeltaTime, Space.Self);

                // steerAngle에 따라 바퀴 회전 각도 적용
                if (wheelCollider == frontLeftWheelCollider || wheelCollider == frontRightWheelCollider)
                {
                    // 전륜 바퀴는 steerAngle로 회전, 왼쪽 바퀴는 -90, 오른쪽 바퀴는 90을 기본 값으로 설정
                    if (wheelCollider == frontLeftWheelCollider)
                    {
                        wheel.transform.localRotation = Quaternion.Euler(0f, -90f + wheelCollider.steerAngle, 0f); // 왼쪽 바퀴
                    }
                    else if (wheelCollider == frontRightWheelCollider)
                    {
                        wheel.transform.localRotation = Quaternion.Euler(0f, 90f + wheelCollider.steerAngle, 0f); // 오른쪽 바퀴
                    }
                }
            }
        }
    }
}
