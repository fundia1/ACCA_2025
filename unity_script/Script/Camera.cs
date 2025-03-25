using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces; // TimeMsg 네임스페이스 추가

public class ImageRawPublisher : MonoBehaviour
{
    [SerializeField] private RenderTexture renderTexture = null;
    [SerializeField] private string topicName = "img_raw";
    [SerializeField] private float publishFrequency = 0.1f;

    ROSConnection ros;
    private float timeElapsed = 0.0f;

    private void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(topicName);
    }

    private void FixedUpdate()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed >= publishFrequency)
        {
            Texture2D texture = new Texture2D(renderTexture.width, renderTexture.height, TextureFormat.RGBA32, false);
            RenderTexture.active = renderTexture;
            texture.ReadPixels(new Rect(0, 0, renderTexture.width, renderTexture.height), 0, 0, false);

            // 좌우 반전 처리
            for (int y = 0; y < texture.height; y++)
            {
                for (int x = 0; x < texture.width / 2; x++)
                {
                    Color temp = texture.GetPixel(x, y);
                    texture.SetPixel(x, y, texture.GetPixel(texture.width - x - 1, y));
                    texture.SetPixel(texture.width - x - 1, y, temp);
                }
            }

            texture.Apply(); 

            // 명시적 캐스트 추가
            var img_raw = new ImageMsg
            {
                header = new HeaderMsg
                {
                    stamp = new TimeMsg
                    {
                        sec = (int)DateTimeOffset.UtcNow.ToUnixTimeSeconds(), // explicit cast to uint
                        nanosec = (uint)((DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() % 1000) * 1_000_000)
                    }
                },
                height = (uint)texture.height,
                width = (uint)texture.width,
                encoding = "rgba8",
                is_bigendian = 0,
                step = (uint)(texture.width * 4),
                data = texture.GetRawTextureData() 
            };

            ros.Publish(topicName, img_raw);

            timeElapsed = 0;
        }
    }
}
