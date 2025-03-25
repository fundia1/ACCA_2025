using System.Collections.Generic;
using UnityEngine;

public class TrailRenderer : MonoBehaviour
{
    public LineRenderer lineRenderer;  // 선을 그릴 LineRenderer 컴포넌트
    public Rigidbody rb;               // Rigidbody 객체
    public float minDistance = 0.1f;   // 점을 추가할 최소 거리

    private List<Vector3> positions = new List<Vector3>();
    private Vector3 lastPosition;

    void Start()
    {
        if (lineRenderer == null)
            lineRenderer = gameObject.AddComponent<LineRenderer>();

        // LineRenderer 설정
        lineRenderer.startWidth = 0.1f;
        lineRenderer.endWidth = 0.1f;
        lineRenderer.positionCount = 0;
        lineRenderer.material = new Material(Shader.Find("Sprites/Default"));
        lineRenderer.startColor = Color.red;
        lineRenderer.endColor = Color.red;

        lastPosition = rb.position; // 초기 위치 저장
    }

    void FixedUpdate()
    {
        if (Vector3.Distance(rb.position, lastPosition) > minDistance)
        {
            positions.Add(rb.position);
            lastPosition = rb.position;

            lineRenderer.positionCount = positions.Count;
            lineRenderer.SetPositions(positions.ToArray());
        }
    }
}
