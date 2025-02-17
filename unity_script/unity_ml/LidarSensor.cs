using UnityEngine;
using System.Linq;
public class LidarSensor : MonoBehaviour
{
    public int numRaysHorizontal = 360;  // 수평 레이 개수
    public int numRaysVertical = 32;     // 수직 레이 개수 (Velodyne VLP-32C)
    public float maxDistance = 100f;     // 레이의 최대 거리
    public float rotationSpeed = 10f;    // LiDAR 회전 속도 (도/초)

    private float angleStepHorizontal;  // 수평 레이 간의 각도
    private float angleStepVertical;    // 수직 레이 간의 각도

    private float[] rayDistances;       // 각 레이의 거리 값

    void Start()
    {
        // 각도 계산
        angleStepHorizontal = 360f / numRaysHorizontal;
        angleStepVertical = 30f / numRaysVertical; // Velodyne VLP-32C는 수직 -15도에서 +15도까지 범위 (총 30도 범위)

        // 레이의 수는 수평 레이 * 수직 레이
        rayDistances = new float[numRaysHorizontal * numRaysVertical];
    }

    void Update()
    {
        // LiDAR 회전 처리
        transform.Rotate(Vector3.up * rotationSpeed * Time.deltaTime);

        int rayIndex = 0; // rayDistances 배열의 인덱스

        // 수직 방향 레이캔 처리 (Velodyne 32처럼 수직 방향도 처리)
        for (int v = 0; v < numRaysVertical; v++)
        {
            // 수직 각도를 -15도에서 +15도 사이로 계산
            float verticalAngle = -15f + v * angleStepVertical;  // 수직 각도

            // 수평 방향 레이캔 처리
            for (int h = 0; h < numRaysHorizontal; h++)
            {
                float horizontalAngle = h * angleStepHorizontal; // 수평 각도

                // 수평과 수직 각도를 적용한 방향 벡터 계산
                Vector3 direction = Quaternion.Euler(verticalAngle, horizontalAngle, 0) * transform.forward;

                RaycastHit hit;
                if (Physics.Raycast(transform.position, direction, out hit, maxDistance))
                {
                    // 'road' 태그를 가진 물체는 무시
                    if (hit.collider.CompareTag("road"))
                    {
                        continue; // 장애물이 없다고 간주하고 무시
                    }
                    else
                    {
                        rayDistances[rayIndex] = hit.distance; // 장애물이 있을 경우 거리 기록
                    }
                }
                else
                {
                    rayDistances[rayIndex] = maxDistance; // 장애물이 없으면 최대 거리 기록
                }

                rayIndex++;  // 레이의 인덱스 증가
            }
        }
    }

    // Gizmos를 사용하여 레이 시각화
//     void OnDrawGizmos()
//     {
//         // 레이의 색상 설정 (빨간색으로 표시)
//         Gizmos.color = Color.red;

//         int rayIndex = 0; // rayDistances 배열의 인덱스

//         // 수직 방향 레이캔 처리
//         for (int v = 0; v < numRaysVertical; v++)
//         {
//             float verticalAngle = (v * angleStepVertical) - (40f / 2);  // 수직 각도

//             // 수평 방향 레이캔 처리
//             for (int h = 0; h < numRaysHorizontal; h++)
//             {
//                 float horizontalAngle = h * angleStepHorizontal; // 수평 각도

//                 // 수평과 수직 각도를 적용한 방향 벡터 계산
//                 Vector3 direction = Quaternion.Euler(verticalAngle, horizontalAngle, 0) * transform.forward;

//                 RaycastHit hit;
//                 if (Physics.Raycast(transform.position, direction, out hit, maxDistance))
//                 {
//                     // 'road' 태그를 가진 물체는 무시
//                     if (!hit.collider.CompareTag("road"))
//                     {
//                         // 장애물이 있을 경우 레이의 끝 점을 표시
//                         Vector3 rayEnd = transform.position + direction * hit.distance;
//                         Gizmos.DrawLine(transform.position, rayEnd);  // Gizmos로 레이 표시
//                     }
//                 }
//                 rayIndex++;  // 레이의 인덱스 증가
//             }
//         }
//     }

   public float[] GetRayDistances()
    {
        // 반환할 데이터의 수
        int numToReturn = 10;

        // rayDistances 배열을 정렬하여 가장 가까운 10개 값을 선택
        var closestDistances = rayDistances
            .Where(distance => distance < maxDistance) // 최대 거리 이하의 값만 선택
            .OrderBy(distance => distance) // 가까운 순서대로 정렬
            .Take(numToReturn) // 상위 10개 선택
            .ToArray(); // 배열로 변환

        return closestDistances;
    }
}
