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

        public float maxLinearSpeed = 25f; // km/h
        public float maxRotationalSpeed = 1f; // rad/s
        public float brakeForce = 5000f; // 브레이크 토크

        private float inputSpeed = 0f;
        private float inputRotationSpeed = 0f;
        private bool isBraking = false;

        private float wheelRadius = 0.3f; // 바퀴 반지름 (m)
        private float wheelCircumference;

        private Keyboard keyboard;

        private Rigidbody rb;

        public float currentSpeed = 0f;    // 현재 속도 (km/h)
        public float speedSmoothingFactor = 10f; // 속도 조정 비율 (0에서 1 사이)
        private float torqueFactor = 1f;   // 토크 계수 (최대 1로 설정)

        void Start()
        {
            keyboard = Keyboard.current;


            rb = GetComponent<Rigidbody>();

            // WheelCollider를 바퀴에 연결
            frontLeftWheelCollider = frontLeftWheel.GetComponent<WheelCollider>();
            frontRightWheelCollider = frontRightWheel.GetComponent<WheelCollider>();
            rearLeftWheelCollider = rearLeftWheel.GetComponent<WheelCollider>();
            rearRightWheelCollider = rearRightWheel.GetComponent<WheelCollider>();

            if (frontLeftWheelCollider == null || frontRightWheelCollider == null || rearLeftWheelCollider == null || rearRightWheelCollider == null)
            {
                Debug.LogError("Missing WheelCollider on one or more wheels!");
            }

            
           
        }

        void FixedUpdate()
        {
            // 키보드 입력 처리
            KeyBoardUpdate();
            // 바퀴 시각적 업데이트
            UpdateVisualWheels();

            LogVehicleSpeed();
        }


        private void LogVehicleSpeed()
        {
            if (rb != null)
            {
                float speed = rb.linearVelocity.magnitude * 3.6f; // m/s 단위 속도
                Debug.Log($"현재 속도: {speed} km/h");
            }
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
            
            float maxTorque = Mathf.Abs(speed) * 80f; 
            currentSpeed = rb.linearVelocity.magnitude * Mathf.Sign(rb.linearVelocity.z);
            Debug.Log($"Robot Input - Speed: {currentSpeed * 3.6f} km/h");
            float speedError = Mathf.Abs(speed - currentSpeed);
            float torqueFactor = 1.0f - Mathf.Log10(speedError + 1.0f); // 1 - log(n) 형태의 감속 곡선
            
            // 토크가 0보다 작아지는 것을 방지
            torqueFactor = Mathf.Clamp(torqueFactor, 0.1f, 1.0f);
            
            float appliedTorque =Mathf.Clamp(Mathf.Abs(speed), 0.0f, 1.0f) * maxTorque * torqueFactor * Mathf.Sign(speed - currentSpeed);


            

            Debug.Log($"Robot Input - Speed: {appliedTorque}");
            // 뒤 바퀴 회전 속도 설정
            
            ApplyMotorTorque(rearLeftWheelCollider, appliedTorque);
            ApplyMotorTorque(rearRightWheelCollider, appliedTorque);

            // 조향 각도 설정 (최대 28도 제한)
            float steerAngle = Mathf.Clamp(rotSpeed * Mathf.Rad2Deg, -28f, 28f);
            frontLeftWheelCollider.steerAngle = steerAngle;
            frontRightWheelCollider.steerAngle = steerAngle;

            
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
