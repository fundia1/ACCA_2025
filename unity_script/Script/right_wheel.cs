using UnityEngine;

namespace RosSharp.Control
{
    public class right_wheel : MonoBehaviour
    {
        // 바퀴 연결
        public GameObject rightSteerWheel;
        public GameObject rightWheel;

        // ArticulationBody (바퀴들)
        private ArticulationBody rSW;
        private ArticulationBody rW;

        // 차량 설정
        public float maxLinearSpeed = 2f; // m/s
        public float maxRotationalSpeed = 1f; // rad/s
        public float wheelRadius = 0.033f; // meters
        public float trackWidth = 0.288f; // meters
        public float forceLimit = 10f;
        public float damping = 10f;

        private float inputSpeed = 0f;
        private float inputRotationSpeed = 0f;

        void Start()
        {
            // ArticulationBody 컴포넌트 가져오기
            rSW = rightSteerWheel.GetComponent<ArticulationBody>();
            rW = rightWheel.GetComponent<ArticulationBody>();

            // 각 바퀴에 대한 파라미터 설정
            SetParameters(rSW);
            SetParameters(rW);
        }

        void FixedUpdate()
        {
            KeyBoardUpdate();
        }

        private void SetParameters(ArticulationBody joint)
        {
            ArticulationDrive drive = joint.xDrive;
            drive.forceLimit = forceLimit;
            drive.damping = damping;
            joint.xDrive = drive;
            Debug.Log($"Set parameters for: {joint.name}");
        }

        private void SetSpeed(ArticulationBody joint, float wheelSpeed)
        {
            ArticulationDrive drive = joint.xDrive;
            drive.targetVelocity = wheelSpeed;
            joint.xDrive = drive;
            Debug.Log($"Set wheel speed: {wheelSpeed} for {joint.name}");
        }

        private void KeyBoardUpdate()
        {
            float moveDirection = Input.GetAxis("Vertical");
            float turnDirection = Input.GetAxis("Horizontal");

            // 속도 설정
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

            // 조향 설정
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

            // 바퀴 회전 설정
            RobotInput(inputSpeed, inputRotationSpeed);
        }

        private void RobotInput(float speed, float rotSpeed) // m/s and rad/s
        {
            if (speed > maxLinearSpeed)
            {
                speed = maxLinearSpeed;
            }
            if (rotSpeed > maxRotationalSpeed)
            {
                rotSpeed = maxRotationalSpeed;
            }
            float wheelRotation = (speed / wheelRadius);
            float steerRotation = rotSpeed * Mathf.Rad2Deg;

            // 바퀴 회전 속도 설정
            SetSpeed(rSW, steerRotation);
            SetSpeed(rW, wheelRotation);
            Debug.Log($"Robot Input - Speed: {speed}, Rotation Speed: {rotSpeed}");
        }
    }
}
