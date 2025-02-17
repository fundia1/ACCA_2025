using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using UnityEngine.InputSystem;

public class BallAgent : Agent
{
    Rigidbody rBody;
    public Transform Target;
    public float forceMultiplier = 10;

    void Start()
    {
        rBody = GetComponent<Rigidbody>();

        // Target이 할당되지 않았으면 자동으로 찾아서 할당
        if (Target == null)
        {
            Target = GameObject.FindWithTag("Target")?.transform;
            if (Target == null)
            {
                Debug.LogError("❌ Target 오브젝트를 찾을 수 없습니다. 씬에 Target 오브젝트가 존재하는지 확인하세요.");
            }
        }
    }

    public override void OnEpisodeBegin()
    {
        if (rBody == null) return;

        // Agent가 떨어졌을 경우 초기화
        if (this.transform.localPosition.y < 0)
        {
            rBody.angularVelocity = Vector3.zero;
            rBody.linearVelocity = Vector3.zero;
            this.transform.localPosition = new Vector3(0, 0.5f, 0);
        }

        // Target이 존재하는 경우에만 위치 변경
        if (Target != null)
        {
            Target.localPosition = new Vector3(Random.value * 8 - 4, 0.5f, Random.value * 8 - 4);
        }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        if (Target != null)
        {
            sensor.AddObservation(Target.localPosition);
        }
        sensor.AddObservation(this.transform.localPosition);

        if (rBody != null)
        {
            sensor.AddObservation(rBody.linearVelocity.x);
            sensor.AddObservation(rBody.linearVelocity.z);
        }
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        if (rBody != null)
        {
            Vector3 controlSignal = Vector3.zero;
            controlSignal.x = actionBuffers.ContinuousActions[0];
            controlSignal.z = actionBuffers.ContinuousActions[1];
            rBody.AddForce(controlSignal * forceMultiplier);
        }

        if (Target == null) return;

        // 목표까지의 거리 계산
        float distanceToTarget = Vector3.Distance(this.transform.localPosition, Target.localPosition);

        // 목표 도달 보상
        if (distanceToTarget < 1.42f)
        {
            SetReward(1.0f); // 목표에 도달하면 보상 +1.0
            EndEpisode();
        }
        // 떨어지면 감점 후 에피소드 종료
        else if (this.transform.localPosition.y < 0)
        {
            SetReward(-10.0f); // 떨어지면 -1.0점 감점
            EndEpisode();
        }
        else
        {
            // 시간이 지날수록 감점 (매 프레임 -0.001)
            AddReward(-0.01f);
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;
        var keyboard = Keyboard.current;
        if (keyboard != null)
        {
            continuousActionsOut[0] = keyboard.aKey.isPressed ? -1f : keyboard.dKey.isPressed ? 1f : 0f;
            continuousActionsOut[1] = keyboard.sKey.isPressed ? -1f : keyboard.wKey.isPressed ? 1f : 0f;
        }
    }
}
