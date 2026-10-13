using UnityEngine;
using System.Collections.Generic;

public class SphereSpikes : MonoBehaviour
{
    [Header("棘の設定")]
    [Min(1)] public int spikeCount = 60;
    [Min(0.01f)] public float spikeLength = 3f;
    [Range(0.005f, 0.2f)] public float spikeWidth = 0.06f;
    [Range(3, 16)] public int sides = 6;

    void Start()
    {
        GenerateSpikes();
    }

    void GenerateSpikes()
    {
        List<Vector3> vertices = new List<Vector3>();
        List<int> triangles = new List<int>();

        float goldenAngle = Mathf.PI * (3f - Mathf.Sqrt(5f));

        for (int i = 0; i < spikeCount; i++)
        {
            // Sphere表面に均等配置
            float y = 1f - 2f * (i + 0.5f) / spikeCount;
            float r = Mathf.Sqrt(1f - y * y);
            float angle = goldenAngle * i;

            Vector3 direction = new Vector3(
                Mathf.Cos(angle) * r,
                y,
                Mathf.Sin(angle) * r
            );

            Quaternion rotation =
                Quaternion.FromToRotation(Vector3.up, direction);

            // Unity標準Sphereの半径は0.5
            // 根元を少し内側に埋める
            float rootRadius =
                Mathf.Sqrt(0.25f - spikeWidth * spikeWidth) - 0.005f;

            Vector3 root = direction * rootRadius;
            Vector3 tip = root + direction * spikeLength;

            int start = vertices.Count;

            vertices.Add(tip);

            // 円錐の底面
            for (int j = 0; j < sides; j++)
            {
                float a = j * Mathf.PI * 2f / sides;

                Vector3 offset = new Vector3(
                    Mathf.Cos(a) * spikeWidth,
                    0f,
                    Mathf.Sin(a) * spikeWidth
                );

                vertices.Add(root + rotation * offset);
            }

            // 円錐の側面
            for (int j = 0; j < sides; j++)
            {
                triangles.Add(start);
                triangles.Add(start + 1 + (j + 1) % sides);
                triangles.Add(start + 1 + j);
            }
        }

        Mesh mesh = new Mesh();
        mesh.name = "Sphere Spikes";
        mesh.SetVertices(vertices);
        mesh.SetTriangles(triangles, 0);
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();

        GameObject spikes = new GameObject("Spikes");
        spikes.transform.SetParent(transform, false);

        MeshFilter filter = spikes.AddComponent<MeshFilter>();
        filter.sharedMesh = mesh;

        MeshRenderer meshRenderer =
            spikes.AddComponent<MeshRenderer>();

        Renderer sphereRenderer = GetComponent<Renderer>();

        if (sphereRenderer != null)
        {
            meshRenderer.sharedMaterial =
                sphereRenderer.sharedMaterial;
        }
    }
}