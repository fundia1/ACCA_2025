#if UNITY_EDITOR
using UnityEngine;
using UnityEditor;
using System.Collections.Generic;

[ExecuteInEditMode]
public class ScenePathDrawer : MonoBehaviour
{
    private List<Vector3> pathPoints = new List<Vector3>();
    private GameObject pathObject;
    private LineRenderer lineRenderer;
    public GameObject spherePrefab;  // 자취 지점에 배치할 구체 프리팹
    public GameObject pathParentObject;  // 구체를 담을 부모 Empty Object

    void Start()
    {
        // "Empty" 부모 GameObject가 설정되지 않으면 새로 생성
        if (pathParentObject == null)
        {
            pathParentObject = new GameObject("PathParent");
            pathParentObject.transform.SetParent(null);  // 최상위로 설정
        }
    }

    void Update()
    {
        if (Application.isPlaying)  // 플레이 모드일 때만 작동
        {
            // 새로운 자취 경로를 계속 추가
            pathPoints.Add(transform.position);

            // 첫 번째 경로가 없으면 pathObject 생성
            if (pathObject == null)
            {
                CreatePathObject();
            }

            // LineRenderer의 경로 업데이트
            UpdatePathLine();

            // 구체를 자취 지점마다 생성
            CreateSpheresForPath();
        }
    }

    void CreatePathObject()
    {
        // 새로운 GameObject 생성
        pathObject = new GameObject("PathObject");
        
        // 씬에 바로 추가되도록 설정
        pathObject.transform.SetParent(pathParentObject.transform);  // 부모로 Empty Object를 설정

        // LineRenderer 컴포넌트 추가
        lineRenderer = pathObject.AddComponent<LineRenderer>();
        lineRenderer.material = new Material(Shader.Find("Sprites/Default"));
        lineRenderer.startColor = Color.red; // 시작 색상
        lineRenderer.endColor = Color.red;   // 끝 색상
        lineRenderer.startWidth = 0.1f;      // 선의 두께
        lineRenderer.endWidth = 0.1f;        // 선의 두께
        lineRenderer.positionCount = 0;      // 초기 포지션 수
    }

    void UpdatePathLine()
    {
        // LineRenderer의 위치 업데이트
        lineRenderer.positionCount = pathPoints.Count;
        for (int i = 0; i < pathPoints.Count; i++)
        {
            lineRenderer.SetPosition(i, pathPoints[i]);
        }
    }

    void CreateSpheresForPath()
    {
        // 각 자취 포인트에 구체를 생성하여 씬에 추가
        foreach (var point in pathPoints)
        {
            // 구체 프리팹이 없다면 기본 구체 생성
            if (spherePrefab == null)
            {
                spherePrefab = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                spherePrefab.GetComponent<Collider>().enabled = false;  // 콜라이더 비활성화
                Destroy(spherePrefab.GetComponent<Renderer>());  // 렌더링 안 함
            }

            // 구체 생성
            GameObject trailSphere = Instantiate(spherePrefab, point, Quaternion.identity);
            trailSphere.transform.localScale = Vector3.one * 0.1f;  // 구 크기 조정 (원하는 크기로)

            // 구체를 부모 "Empty" 객체의 자식으로 설정하지 않고, 씬에 독립적으로 배치
            trailSphere.transform.SetParent(null);  // 부모 설정하지 않음 (독립적)

            // 구체가 씬을 넘어가더라도 남도록 설정 (구체가 사라지지 않게)
            DontDestroyOnLoad(trailSphere);  // 구체가 씬에 계속 남도록 함
        }
    }

    void OnDrawGizmos()
    {
        // Gizmos로 경로를 그리기 (편집 모드에서만 사용)
        Gizmos.color = Color.blue;
        for (int i = 0; i < pathPoints.Count - 1; i++)
        {
            Gizmos.DrawLine(pathPoints[i], pathPoints[i + 1]);
        }
    }
}
#endif
