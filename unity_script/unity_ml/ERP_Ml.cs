using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using UnityEngine.InputSystem;

public class ERP_Ml : Agent
{
    public Rigidbody Body;
    public Transform Target;
    public LidarSensor lidarSensor; 

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

    private float previousSteerAngle = 0f;
    private Keyboard keyboard;

    void Start()
    {
        LidarSensor lidar = GameObject.Find("Cylinder").GetComponent<LidarSensor>();
        if (lidar == null)
        {
            Debug.LogError("❌ LidarSensor 컴포넌트를 찾을 수 없습니다.");
        }
        Body = GetComponent<Rigidbody>();

        // angularVelocity와 velocity 사용
        if (Body != null)
        {
            Body.angularVelocity = Vector3.zero;  // 회전 속도 초기화
            Body.linearVelocity = Vector3.zero;  // 속도 초기화
        }

        if (Target == null)
        {
            Target = GameObject.FindWithTag("Target")?.transform;
            if (Target == null)
            {
                Debug.LogError("❌ Target 오브젝트를 찾을 수 없습니다. 씬에 Target 오브젝트가 존재하는지 확인하세요.");
            }
        }

        // WheelCollider 초기화
        frontLeftWheelCollider = frontLeftWheel.GetComponent<WheelCollider>();
        frontRightWheelCollider = frontRightWheel.GetComponent<WheelCollider>();
        rearLeftWheelCollider = rearLeftWheel.GetComponent<WheelCollider>();
        rearRightWheelCollider = rearRightWheel.GetComponent<WheelCollider>();
    }


    public override void OnEpisodeBegin()
    {
        if (Body == null) return;

        // 차량이 벽에 부딪혔을때 초기화
        if (this.transform.localPosition.y < 0)
        {
            Body.angularVelocity = Vector3.zero;
            Body.linearVelocity = Vector3.zero;
            this.transform.localPosition = new Vector3(0, 0.5f, 0);
        }

        // 목표의 위치를 랜덤으로 설정
        // if (Target != null)
        // {
        //     Target.localPosition = new Vector3(Random.value * 8 - 4, 0.5f, Random.value * 8 - 4);
        // }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // 목표 위치
        if (Target != null)
        {
            sensor.AddObservation(Target.localPosition);
        }

        // 차량 위치
        sensor.AddObservation(this.transform.localPosition);

        // 차량의 속도
        if (Body != null)
        {
            sensor.AddObservation(Body.linearVelocity.x);
            sensor.AddObservation(Body.linearVelocity.z);
        }

        // Lidar의 레이 캐스트
        // if (lidarSensor != null)
        // {
        //     float[] rayDistances = lidarSensor.GetRayDistances();  // 라이다 레이 캐스트 거리 가져오기
        //     foreach (float distance in rayDistances)
        //     {
        //         sensor.AddObservation(distance);  // 각 레이 캐스트 거리 추가
        //     }
        // }
        else
        {
            Debug.LogWarning("❌ LidarSensor가 null입니다.");
        }

        // 바퀴 회전 속도 (rpm) 추가
        if (frontLeftWheelCollider != null)
        {
            sensor.AddObservation(frontLeftWheelCollider.rpm); // 왼쪽 앞바퀴 회전 속도
        }
       

        // 바퀴 조향 각도 (steerAngle) 추가
        if (frontLeftWheelCollider != null)
        {
            sensor.AddObservation(frontLeftWheelCollider.steerAngle); // 왼쪽 앞바퀴 조향 각도
        }
        
    }    



    public override void OnActionReceived(ActionBuffers actionBuffers)
{
    if (Body != null)
    {
        // 차량의 속도 및 회전 입력 받기
        inputSpeed = actionBuffers.ContinuousActions[0]; // x축 속도
        inputRotationSpeed = actionBuffers.ContinuousActions[1]; // 회전 속도

        // 차량의 회전과 이동
        float moveSpeed = Mathf.Clamp(inputSpeed * 30, -maxLinearSpeed, maxLinearSpeed);
        float rotationSpeed = Mathf.Clamp(inputRotationSpeed * 28, -maxRotationalSpeed, maxRotationalSpeed);

        // 바퀴의 회전 속도 조정
        if (frontLeftWheelCollider != null && frontRightWheelCollider != null)
        {
            frontLeftWheelCollider.motorTorque = moveSpeed;
            frontRightWheelCollider.motorTorque = moveSpeed;
        }

        // 앞바퀴의 조향 각도 설정
        if (frontLeftWheelCollider != null && frontRightWheelCollider != null)
        {
            frontLeftWheelCollider.steerAngle = rotationSpeed;
            frontRightWheelCollider.steerAngle = rotationSpeed;
        }

        // 급격한 조향 변화가 있을 경우 패널티 부여
        float steerChange = Mathf.Abs(rotationSpeed - previousSteerAngle);
        if (steerChange > 5f)  // 급격한 조향 변화 감지 (예: 5도 이상)
        {
            AddReward(-5f);  // 패널티 부여
        }

        // 이전 조향각을 현재 값으로 업데이트
        previousSteerAngle = rotationSpeed;
    }

    if (Target != null)
    {
        // 목표까지의 거리 계산
        float distanceToTarget = Vector3.Distance(this.transform.localPosition, Target.localPosition);

        // 차량의 현재 회전 각도와 목표 방향 간의 각도 계산
        Vector3 vehicleDirection = transform.forward; // 차량이 바라보는 방향
        Vector3 targetDirection = (Target.localPosition - transform.localPosition).normalized; // 목표 방향

        float angleToTarget = Vector3.Angle(vehicleDirection, targetDirection); // 차량과 목표 간의 각도 차이

        // 목표 도달 보상
        if (distanceToTarget < 1.42f)
        {
            // 목표 도달 시 큰 보상
            float timeBonus = Mathf.Max(0, 10.0f - StepCount * 0.01f); 
            SetReward(1000.0f + timeBonus);  
            this.transform.localPosition = new Vector3(-8.0f, 1.0f, -5.9f);  
            this.transform.localRotation = Quaternion.Euler(Vector3.zero);

            Body.angularVelocity = Vector3.zero;  
            Body.linearVelocity = Vector3.zero;  

            EndEpisode();
        }
        else
        {
            // 목표와의 거리 감소 보상
            float reward = Mathf.Clamp(1.0f - distanceToTarget / 10f, 0f, 1f);
            AddReward(reward);
        }

        // 목표 방향에 가까워질수록 보상
        float alignmentReward = Mathf.Max(0, 1.0f - angleToTarget / 180f); 
        AddReward(alignmentReward);

        // 후진 시 감점
        if (inputSpeed < 0)
        {
            AddReward(-2.0f);
        }

        // 속도 보상: 최적 속도 (+5), 너무 빠르거나 느리면 (-10)
        float idealSpeed = 15f; // 이상적인 속도 설정 (원하는 목표 속도)
        float speedPenalty = Mathf.Abs(Body.linearVelocity.magnitude - idealSpeed);
        if (speedPenalty > 5f) 
        {
            AddReward(-10f);
        }
        else
        {
            AddReward(5f);
        }

        // 시간이 지날수록 감점 (매 프레임 -0.01)
        AddReward(-0.01f);

        // 시간이 너무 오래 걸리면 에피소드 종료 (예: 3000 스텝 이후)
        if (StepCount >= 3000)
        {
            Debug.Log("시간 초과로 에피소드 종료");
            EndEpisode();
        }
    }
    else
    {
        Debug.LogError("❌ Target이 할당되지 않았습니다.");
    }
}


    // 충돌 감지 처리
    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("obstacle"))  
        {
            SetReward(-200000.0f);  
            this.transform.localPosition = new Vector3(-8.0f, 1.5f, -5.9f);  
            this.transform.localRotation = Quaternion.Euler(Vector3.zero);

            Body.angularVelocity = Vector3.zero;  
            Body.linearVelocity = Vector3.zero;  
            EndEpisode(); 
        }
    }

   

   



    
}
