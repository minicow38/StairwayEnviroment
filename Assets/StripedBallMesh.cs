using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// テクスチャを使わず、2つのサブメッシュで縞模様の球体を生成します。
///
/// BallVisualへ追加して使用してください。
/// Rigidbody / Colliderは追加しません。
/// </summary>
[ExecuteAlways]
[DisallowMultipleComponent]
[RequireComponent(typeof(MeshFilter))]
[RequireComponent(typeof(MeshRenderer))]
public sealed class StripedBallMesh : MonoBehaviour
{
    [Header("Sphere")]
    [SerializeField, Min(.01f)]
    float radius = .5f;

    [SerializeField, Range(8, 128)]
    int latitudeSegments = 32;

    [SerializeField, Range(8, 256)]
    int longitudeSegments = 64;

    [SerializeField]
    Vector3 localCenter = Vector3.zero;

    [Header("Stripe Mesh")]
    [Tooltip("球の周囲に作る縞の繰り返し数です。")]
    [SerializeField, Range(2, 64)]
    int stripeCount = 12;

    [Tooltip("0なら縦縞。値を増やすと斜め・らせん状になります。負数で逆方向です。")]
    [SerializeField, Range(-16, 16)]
    int diagonalTurns = 3;

    [Tooltip("ONなら縞Aと縞Bを入れ替えます。")]
    [SerializeField]
    bool invertStripes;

    [Header("Materials")]
    [SerializeField]
    Material stripeMaterialA;

    [SerializeField]
    Material stripeMaterialB;

    [SerializeField]
    Color fallbackColorA = Color.white;

    [SerializeField]
    Color fallbackColorB = Color.black;

    [Header("Generation")]
    [SerializeField]
    bool rebuildAutomatically = true;

    MeshFilter meshFilter;
    MeshRenderer meshRenderer;

    Mesh generatedMesh;
    Material generatedMaterialA;
    Material generatedMaterialB;

    void Reset()
    {
        ResolveComponents();
        RebuildMesh();
    }

    void OnEnable()
    {
        ResolveComponents();

        if (rebuildAutomatically)
            RebuildMesh();
    }

    void OnValidate()
    {
        radius = Mathf.Max(.01f, radius);
        latitudeSegments = Mathf.Clamp(latitudeSegments, 8, 128);
        longitudeSegments = Mathf.Clamp(longitudeSegments, 8, 256);
        stripeCount = Mathf.Clamp(stripeCount, 2, 64);

        ResolveComponents();

        if (rebuildAutomatically && isActiveAndEnabled)
            RebuildMesh();
    }

    void OnDestroy()
    {
        DestroyGeneratedObjects();
    }

    void ResolveComponents()
    {
        if (!meshFilter)
            meshFilter = GetComponent<MeshFilter>();

        if (!meshRenderer)
            meshRenderer = GetComponent<MeshRenderer>();
    }

    [ContextMenu("Rebuild Striped Ball Mesh")]
    public void RebuildMesh()
    {
        ResolveComponents();

        if (!meshFilter || !meshRenderer)
            return;

        DestroyGeneratedMeshOnly();

        generatedMesh = BuildSphereMesh();
        generatedMesh.name = "Generated Striped Ball Mesh";

        meshFilter.sharedMesh = generatedMesh;
        ApplyMaterials();
    }

    Mesh BuildSphereMesh()
    {
        int rowLength = longitudeSegments + 1;
        int vertexCount = (latitudeSegments + 1) * rowLength;

        Vector3[] vertices = new Vector3[vertexCount];
        Vector3[] normals = new Vector3[vertexCount];
        Vector2[] uvs = new Vector2[vertexCount];
        Vector4[] tangents = new Vector4[vertexCount];

        for (int latitude = 0; latitude <= latitudeSegments; latitude++)
        {
            float v = latitude / (float)latitudeSegments;
            float theta = v * Mathf.PI;

            float ringRadius = Mathf.Sin(theta);
            float y = Mathf.Cos(theta);

            for (int longitude = 0; longitude <= longitudeSegments; longitude++)
            {
                float u = longitude / (float)longitudeSegments;
                float phi = u * Mathf.PI * 2f;

                Vector3 normal = new Vector3(
                    ringRadius * Mathf.Cos(phi),
                    y,
                    ringRadius * Mathf.Sin(phi)
                ).normalized;

                int index = latitude * rowLength + longitude;

                vertices[index] = localCenter + normal * radius;
                normals[index] = normal;
                uvs[index] = new Vector2(u, 1f - v);

                // U方向へ進む接線です。
                Vector3 tangent = new Vector3(
                    -Mathf.Sin(phi),
                    0f,
                    Mathf.Cos(phi)
                ).normalized;

                tangents[index] = new Vector4(
                    tangent.x,
                    tangent.y,
                    tangent.z,
                    1f
                );
            }
        }

        List<int> stripeTrianglesA = new List<int>();
        List<int> stripeTrianglesB = new List<int>();

        for (int latitude = 0; latitude < latitudeSegments; latitude++)
        {
            for (int longitude = 0; longitude < longitudeSegments; longitude++)
            {
                int a = latitude * rowLength + longitude;
                int b = (latitude + 1) * rowLength + longitude;
                int c = (latitude + 1) * rowLength + longitude + 1;
                int d = latitude * rowLength + longitude + 1;

                float centerU =
                    (longitude + .5f) / longitudeSegments;

                float centerV =
                    (latitude + .5f) / latitudeSegments;

                bool useStripeA = IsStripeA(centerU, centerV);

                List<int> targetTriangles =
                    useStripeA
                        ? stripeTrianglesA
                        : stripeTrianglesB;

                // 外向きになる頂点順序です。
                targetTriangles.Add(a);
                targetTriangles.Add(d);
                targetTriangles.Add(b);

                targetTriangles.Add(d);
                targetTriangles.Add(c);
                targetTriangles.Add(b);
            }
        }

        Mesh mesh = new Mesh
        {
            indexFormat = vertexCount > 65535
                ? UnityEngine.Rendering.IndexFormat.UInt32
                : UnityEngine.Rendering.IndexFormat.UInt16
        };

        mesh.vertices = vertices;
        mesh.normals = normals;
        mesh.uv = uvs;
        mesh.tangents = tangents;

        mesh.subMeshCount = 2;
        mesh.SetTriangles(stripeTrianglesA, 0, false);
        mesh.SetTriangles(stripeTrianglesB, 1, false);

        mesh.RecalculateBounds();
        return mesh;
    }

    bool IsStripeA(float u, float v)
    {
        // U方向の縞へV方向の位相差を加えることで、
        // 球面上に斜め・らせん状の縞を作ります。
        float phase =
            u * stripeCount +
            (v - .5f) * diagonalTurns;

        bool stripeA =
            Mathf.Sin(phase * Mathf.PI * 2f) >= 0f;

        return invertStripes
            ? !stripeA
            : stripeA;
    }

    void ApplyMaterials()
    {
        Material materialA = stripeMaterialA
            ? stripeMaterialA
            : GetOrCreateGeneratedMaterial(
                ref generatedMaterialA,
                fallbackColorA,
                "Generated Stripe Material A"
            );

        Material materialB = stripeMaterialB
            ? stripeMaterialB
            : GetOrCreateGeneratedMaterial(
                ref generatedMaterialB,
                fallbackColorB,
                "Generated Stripe Material B"
            );

        meshRenderer.sharedMaterials = new[]
        {
            materialA,
            materialB
        };
    }

    Material GetOrCreateGeneratedMaterial(
        ref Material material,
        Color color,
        string materialName
    )
    {
        if (!material)
        {
            Shader shader =
                Shader.Find("Universal Render Pipeline/Lit");

            if (!shader)
                shader = Shader.Find("Standard");

            if (!shader)
                shader = Shader.Find("Unlit/Color");

            if (!shader)
            {
                Debug.LogError(
                    "[StripedBallMesh] 使用可能なShaderが見つかりません。",
                    this
                );
                return null;
            }

            material = new Material(shader)
            {
                name = materialName,
                hideFlags = HideFlags.DontSave
            };
        }

        if (material.HasProperty("_BaseColor"))
            material.SetColor("_BaseColor", color);

        if (material.HasProperty("_Color"))
            material.SetColor("_Color", color);

        return material;
    }

    void DestroyGeneratedMeshOnly()
    {
        if (!generatedMesh)
            return;

        if (meshFilter && meshFilter.sharedMesh == generatedMesh)
            meshFilter.sharedMesh = null;

        DestroyUnityObject(generatedMesh);
        generatedMesh = null;
    }

    void DestroyGeneratedObjects()
    {
        DestroyGeneratedMeshOnly();

        if (generatedMaterialA)
        {
            DestroyUnityObject(generatedMaterialA);
            generatedMaterialA = null;
        }

        if (generatedMaterialB)
        {
            DestroyUnityObject(generatedMaterialB);
            generatedMaterialB = null;
        }
    }

    static void DestroyUnityObject(Object target)
    {
        if (!target)
            return;

        if (Application.isPlaying)
            Destroy(target);
        else
            DestroyImmediate(target);
    }
}