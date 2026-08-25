using UnityEngine;
using UnityEngine.Rendering;

#if UNITY_EDITOR
using UnityEditor;
#endif

[ExecuteAlways]
[DisallowMultipleComponent]
[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer), typeof(MeshCollider))]
public sealed class FoldBakeMeshAndCollider : MonoBehaviour
{
    [Header("Source Plane")]
    [Tooltip("元PlaneのMeshです。このMeshのローカルBounds全体を山なり面へ置き換えます。")]
    [SerializeField]
    private Mesh sourcePlaneMesh;

    [Tooltip("元Meshがない場合だけ使う予備サイズです。Unity標準Planeなら通常10×10です。")]
    [SerializeField, Min(0.01f)]
    private float fallbackWidth = 10f;

    [SerializeField, Min(0.01f)]
    private float fallbackDepth = 10f;

    [Tooltip("元Planeの高さを基準に追加するローカルYオフセットです。")]
    [SerializeField]
    private float baseHeightOffset;

    [Header("Mountain Profile")]
    [Tooltip("元Planeの進行方向全体に作る山の数です。増やすと各山は自動的に小さくなり、等間隔を維持します。")]
    [SerializeField, Min(1)]
    private int mountainCount = 6;

    [Tooltip("各斜面の水平面に対する角度です。30度なら左右対称の山頂内角は120度です。")]
    [SerializeField, Range(1f, 80f)]
    private float slopeAngleDegrees = 30f;

    [Tooltip("ONで山、OFFで上下を反転した谷形状になります。")]
    [SerializeField]
    private bool mountainsPointUp = true;

    [Tooltip("ONなら山頂と谷底の中央を基準高さに置きます。OFFなら谷底を基準高さに置きます。")]
    [SerializeField]
    private bool centerProfileOnBaseHeight;

    [Header("Collider")]
    [Tooltip("生成MeshをMeshColliderにも設定します。静的な地形としての使用を推奨します。")]
    [SerializeField]
    private bool generateMeshCollider = true;

    [Tooltip("Bake時にMeshColliderを必ず保存します。OFFでもBake時だけはColliderを作成できます。")]
    [SerializeField]
    private bool forceColliderOnBake = true;

    [Tooltip("MeshColliderをConvexにします。地形用途では通常OFFです。")]
    [SerializeField]
    private bool colliderConvex = false;

    [Tooltip("MeshColliderをTriggerとして使います。通常の地形ではOFFです。")]
    [SerializeField]
    private bool colliderIsTrigger = false;
    [SerializeField]
    private PhysicMaterial colliderMaterial;

    [Tooltip("必要ならPhysic Materialを設定します。")]
    [SerializeField]
    private PhysicMaterial material;

    [Header("Editor")]
    [Tooltip("Inspectorの値を変更したときに自動再生成します。Bake後は自動再生成されません。")]
    [SerializeField]
    private bool rebuildOnValidate = true;

    [Tooltip("Sceneビューに中央断面を黄色線で表示します。")]
    [SerializeField]
    private bool drawProfileGizmos = true;

    [SerializeField, HideInInspector]
    private bool bakedMode;

    private MeshFilter meshFilter;
    private MeshCollider meshCollider;

    // Assets内に保存されていない一時Meshだけを保持します。
    // Bake後はnullになり、コンポーネントを削除しても保存済みMeshは破棄されません。
    private Mesh generatedMesh;

    public float MountainPitch
    {
        get
        {
            Bounds bounds = GetSourceBounds();
            return bounds.size.z / Mathf.Max(1, mountainCount);
        }
    }

    public float HalfMountainRun => MountainPitch * 0.5f;

    public float MountainHeight =>
        HalfMountainRun * Mathf.Tan(slopeAngleDegrees * Mathf.Deg2Rad);

    public float MountainInteriorAngle =>
        180f - 2f * slopeAngleDegrees;

    private void Reset()
    {
        ResolveComponents();
        CaptureCurrentMeshAsSourceIfPossible();
        RebuildMesh();
    }

    private void OnEnable()
    {
        ResolveComponents();

        if (!bakedMode && !Application.isPlaying)
        {
            CaptureCurrentMeshAsSourceIfPossible();
            RebuildMeshInternal();
        }
    }

    private void OnValidate()
    {
        fallbackWidth = Mathf.Max(0.01f, fallbackWidth);
        fallbackDepth = Mathf.Max(0.01f, fallbackDepth);
        mountainCount = Mathf.Max(1, mountainCount);
        slopeAngleDegrees = Mathf.Clamp(slopeAngleDegrees, 1f, 80f);

        if (!isActiveAndEnabled || bakedMode || !rebuildOnValidate)
            return;

        ResolveComponents();
        CaptureCurrentMeshAsSourceIfPossible();
        RebuildMeshInternal();
    }

    private void OnDestroy()
    {
        // Bake済みMesh AssetはgeneratedMeshに入れないため破棄されません。
        DestroyTemporaryGeneratedMesh();
    }

    [ContextMenu("Capture Current Mesh As Source")]
    public void CaptureCurrentMeshAsSource()
    {
        ResolveComponents();

        if (!meshFilter || !meshFilter.sharedMesh)
        {
            Debug.LogWarning(
                "[UnitMountainMeshGenerator] MeshFilterに取得できるMeshがありません。",
                this
            );
            return;
        }

        if (meshFilter.sharedMesh == generatedMesh)
        {
            Debug.LogWarning(
                "[UnitMountainMeshGenerator] 現在のMeshは生成済みの一時Meshです。元PlaneのMeshをMeshFilterへ戻してから実行してください。",
                this
            );
            return;
        }

        sourcePlaneMesh = meshFilter.sharedMesh;
        bakedMode = false;

        Debug.Log(
            $"[UnitMountainMeshGenerator] Source Plane Meshを取得しました: {sourcePlaneMesh.name}",
            sourcePlaneMesh
        );

#if UNITY_EDITOR
        EditorUtility.SetDirty(this);
#endif
    }

    [ContextMenu("Rebuild Mountain Mesh")]
    public void RebuildMesh()
    {
        bakedMode = false;
        ResolveComponents();
        CaptureCurrentMeshAsSourceIfPossible();
        RebuildMeshInternal();

#if UNITY_EDITOR
        EditorUtility.SetDirty(this);
#endif
    }

    [ContextMenu("Return To Procedural Mode")]
    public void ReturnToProceduralMode()
    {
        bakedMode = false;
        RebuildMesh();

        Debug.Log(
            "[UnitMountainMeshGenerator] Procedural Modeへ戻しました。Inspector変更で再生成されます。",
            this
        );
    }

    [ContextMenu("Remove Generated Mesh")]
    public void RemoveGeneratedMesh()
    {
        ResolveComponents();

        if (meshFilter)
            meshFilter.sharedMesh = null;

        if (meshCollider)
            meshCollider.sharedMesh = null;

        bakedMode = false;
        DestroyTemporaryGeneratedMesh();

#if UNITY_EDITOR
        EditorUtility.SetDirty(this);

        if (meshFilter)
            EditorUtility.SetDirty(meshFilter);

        if (meshCollider)
            EditorUtility.SetDirty(meshCollider);
#endif
    }

#if UNITY_EDITOR
    [ContextMenu("Bake Mesh As Asset")]
    public void BakeMeshAsAsset()
    {
        if (Application.isPlaying)
        {
            Debug.LogWarning(
                "[UnitMountainMeshGenerator] BakeはPlay Modeを停止してから実行してください。",
                this
            );
            return;
        }

        ResolveComponents();

        if (!meshFilter)
        {
            Debug.LogError(
                "[UnitMountainMeshGenerator] MeshFilterがありません。",
                this
            );
            return;
        }

        // まだ山Meshがない場合は先に生成します。
        if (!meshFilter.sharedMesh)
        {
            bakedMode = false;
            CaptureCurrentMeshAsSourceIfPossible();
            RebuildMeshInternal();
        }

        if (!meshFilter.sharedMesh)
        {
            Debug.LogError(
                "[UnitMountainMeshGenerator] 保存できる山なりMeshがありません。",
                this
            );
            return;
        }

        string defaultFileName =
            $"MountainMesh_{mountainCount}_{slopeAngleDegrees:0.#}deg.asset";

        string assetPath = EditorUtility.SaveFilePanelInProject(
            "山なりMeshをAssetとして保存",
            defaultFileName,
            "asset",
            "Assetsフォルダー内の保存場所を選択してください。"
        );

        if (string.IsNullOrWhiteSpace(assetPath))
            return;

        assetPath = AssetDatabase.GenerateUniqueAssetPath(assetPath);

        Mesh temporaryMesh = generatedMesh;
        Mesh bakedMesh = Instantiate(meshFilter.sharedMesh);

        bakedMesh.name =
            $"MountainMesh_{mountainCount}_{slopeAngleDegrees:0.#}deg";

        AssetDatabase.CreateAsset(bakedMesh, assetPath);
        AssetDatabase.SaveAssets();
        AssetDatabase.Refresh();

        // MeshFilterを正式なMesh Assetへ差し替えます。
        meshFilter.sharedMesh = bakedMesh;

        // Colliderは独立した.assetではなく、PrefabまたはScene内の
        // MeshColliderコンポーネントとして保存されます。
        // sharedMeshには同じ保存済みMesh Assetを割り当てます。
        if (generateMeshCollider || forceColliderOnBake)
            ConfigureMeshCollider(bakedMesh);

        // ここから先は保存済みAssetを使うため、一時Meshとして管理しません。
        generatedMesh = null;
        bakedMode = true;

        if (temporaryMesh && !EditorUtility.IsPersistent(temporaryMesh))
            DestroyImmediate(temporaryMesh);

        EditorUtility.SetDirty(this);
        EditorUtility.SetDirty(meshFilter);

        if (meshCollider)
            EditorUtility.SetDirty(meshCollider);

        EditorUtility.SetDirty(gameObject);

        // Prefabインスタンスなら変更内容をPrefab Overrideとして記録します。
        if (PrefabUtility.IsPartOfPrefabInstance(gameObject))
        {
            PrefabUtility.RecordPrefabInstancePropertyModifications(meshFilter);

            if (meshCollider)
                PrefabUtility.RecordPrefabInstancePropertyModifications(meshCollider);

            PrefabUtility.RecordPrefabInstancePropertyModifications(this);
        }

        AssetDatabase.SaveAssets();

        Debug.Log(
            "[UnitMountainMeshGenerator] Mesh Assetとして保存しました。\n" +
            $"Path: {assetPath}\n" +
            "MeshFilterとMeshColliderは同じ保存済みMesh Assetを参照しています。\n" +
            "この状態ならGeneratorコンポーネントを削除してもMeshは残ります。",
            bakedMesh
        );

        Selection.activeObject = bakedMesh;
        EditorGUIUtility.PingObject(bakedMesh);
    }


    [ContextMenu("Save Baked Object As Prefab")]
    public void SaveBakedObjectAsPrefab()
    {
        if (Application.isPlaying)
        {
            Debug.LogWarning(
                "[UnitMountainMeshGenerator] Prefab保存はPlay Modeを停止してから実行してください。",
                this
            );
            return;
        }

        ResolveComponents();

        if (!meshFilter || !meshFilter.sharedMesh)
        {
            Debug.LogWarning(
                "[UnitMountainMeshGenerator] Meshがありません。先にBake Mesh As Assetを実行してください。",
                this
            );
            return;
        }

        if (!EditorUtility.IsPersistent(meshFilter.sharedMesh))
        {
            Debug.LogWarning(
                "[UnitMountainMeshGenerator] 現在のMeshは一時Meshです。先にBake Mesh As Assetを実行してください。",
                this
            );
            return;
        }

        ConfigureMeshCollider(meshFilter.sharedMesh);

        string defaultPrefabName =
            $"{gameObject.name}_Baked.prefab";

        string prefabPath = EditorUtility.SaveFilePanelInProject(
            "MeshとColliderを含むPrefabを保存",
            defaultPrefabName,
            "prefab",
            "Assetsフォルダー内の保存場所を選択してください。"
        );

        if (string.IsNullOrWhiteSpace(prefabPath))
            return;

        prefabPath = AssetDatabase.GenerateUniqueAssetPath(prefabPath);

        GameObject savedPrefab = PrefabUtility.SaveAsPrefabAssetAndConnect(
            gameObject,
            prefabPath,
            InteractionMode.UserAction
        );

        if (!savedPrefab)
        {
            Debug.LogError(
                "[UnitMountainMeshGenerator] Prefabの保存に失敗しました。",
                this
            );
            return;
        }

        EditorUtility.SetDirty(meshFilter);
        EditorUtility.SetDirty(meshCollider);
        EditorUtility.SetDirty(gameObject);
        AssetDatabase.SaveAssets();

        Debug.Log(
            "[UnitMountainMeshGenerator] MeshColliderを含むPrefabを保存しました。\n" +
            $"Prefab: {prefabPath}\n" +
            $"Mesh Asset: {AssetDatabase.GetAssetPath(meshFilter.sharedMesh)}\n" +
            "このPrefabにはMeshFilter・MeshRenderer・MeshColliderが保存されています。",
            savedPrefab
        );

        Selection.activeObject = savedPrefab;
        EditorGUIUtility.PingObject(savedPrefab);
    }

    [ContextMenu("Bake Mesh + Collider And Save Prefab")]
    public void BakeMeshColliderAndSavePrefab()
    {
        BakeMeshAsAsset();

        if (!bakedMode ||
            !meshFilter ||
            !meshFilter.sharedMesh ||
            !EditorUtility.IsPersistent(meshFilter.sharedMesh))
        {
            return;
        }

        SaveBakedObjectAsPrefab();
    }

    [ContextMenu("Log Source Bounds")]
    public void LogSourceBounds()
    {
        Bounds bounds = GetSourceBounds();

        Debug.Log(
            "[UnitMountainMeshGenerator]\n" +
            $"Source Mesh = {(sourcePlaneMesh ? sourcePlaneMesh.name : "Fallback Size")}\n" +
            $"Center = {FormatVector(bounds.center)}\n" +
            $"Size = {FormatVector(bounds.size)}\n" +
            $"Min = {FormatVector(bounds.min)}\n" +
            $"Max = {FormatVector(bounds.max)}\n" +
            $"Mountain Count = {mountainCount}\n" +
            $"Mountain Pitch = {MountainPitch:R}\n" +
            $"Mountain Height = {MountainHeight:R}\n" +
            $"Slope Angle = {slopeAngleDegrees:R}°\n" +
            $"Interior Angle = {MountainInteriorAngle:R}°",
            this
        );
    }
#endif

    private void RebuildMeshInternal()
    {
        ResolveComponents();
        DestroyTemporaryGeneratedMesh();

        generatedMesh = BuildSurfaceMesh();
        generatedMesh.name =
            $"MountainSurface_{mountainCount}x_{slopeAngleDegrees:0.#}deg_TEMP";

        meshFilter.sharedMesh = generatedMesh;

        if (generateMeshCollider)
        {
            ConfigureMeshCollider(generatedMesh);
        }
        else if (meshCollider)
        {
            meshCollider.sharedMesh = null;
        }
    }

    private Mesh BuildSurfaceMesh()
    {
        Bounds sourceBounds = GetSourceBounds();

        int slopePanelCount = mountainCount * 2;
        int vertexCount = slopePanelCount * 4;
        int triangleIndexCount = slopePanelCount * 6;

        Vector3[] vertices = new Vector3[vertexCount];
        Vector2[] uvs = new Vector2[vertexCount];
        int[] triangles = new int[triangleIndexCount];

        float minX = sourceBounds.min.x;
        float maxX = sourceBounds.max.x;
        float minZ = sourceBounds.min.z;
        float depth = Mathf.Max(sourceBounds.size.z, 0.01f);

        float panelRun = depth / slopePanelCount;
        float height = panelRun * Mathf.Tan(slopeAngleDegrees * Mathf.Deg2Rad);
        float direction = mountainsPointUp ? 1f : -1f;
        float baseY = sourceBounds.center.y + baseHeightOffset;

        float lowY;
        float highY;

        if (centerProfileOnBaseHeight)
        {
            lowY = baseY - height * 0.5f * direction;
            highY = baseY + height * 0.5f * direction;
        }
        else
        {
            lowY = baseY;
            highY = baseY + height * direction;
        }

        for (int panel = 0; panel < slopePanelCount; panel++)
        {
            float z0 = minZ + panel * panelRun;
            float z1 = z0 + panelRun;

            bool risingPanel = panel % 2 == 0;
            float y0 = risingPanel ? lowY : highY;
            float y1 = risingPanel ? highY : lowY;

            int vertexStart = panel * 4;
            int triangleStart = panel * 6;

            vertices[vertexStart + 0] = new Vector3(minX, y0, z0);
            vertices[vertexStart + 1] = new Vector3(maxX, y0, z0);
            vertices[vertexStart + 2] = new Vector3(minX, y1, z1);
            vertices[vertexStart + 3] = new Vector3(maxX, y1, z1);

            float uv0 = panel / (float)slopePanelCount;
            float uv1 = (panel + 1f) / slopePanelCount;

            uvs[vertexStart + 0] = new Vector2(0f, uv0);
            uvs[vertexStart + 1] = new Vector2(1f, uv0);
            uvs[vertexStart + 2] = new Vector2(0f, uv1);
            uvs[vertexStart + 3] = new Vector2(1f, uv1);

            triangles[triangleStart + 0] = vertexStart + 0;
            triangles[triangleStart + 1] = vertexStart + 2;
            triangles[triangleStart + 2] = vertexStart + 1;

            triangles[triangleStart + 3] = vertexStart + 1;
            triangles[triangleStart + 4] = vertexStart + 2;
            triangles[triangleStart + 5] = vertexStart + 3;
        }

        Mesh mesh = new Mesh();

        if (vertexCount > 65535)
            mesh.indexFormat = IndexFormat.UInt32;

        mesh.vertices = vertices;
        mesh.uv = uvs;
        mesh.triangles = triangles;
        mesh.RecalculateNormals();
        mesh.RecalculateTangents();
        mesh.RecalculateBounds();

        return mesh;
    }

    private Bounds GetSourceBounds()
    {
        if (sourcePlaneMesh)
            return sourcePlaneMesh.bounds;

        return new Bounds(
            Vector3.zero,
            new Vector3(fallbackWidth, 0f, fallbackDepth)
        );
    }

    private void CaptureCurrentMeshAsSourceIfPossible()
    {
        if (sourcePlaneMesh || !meshFilter || !meshFilter.sharedMesh)
            return;

        Mesh currentMesh = meshFilter.sharedMesh;

        if (currentMesh == generatedMesh)
            return;

#if UNITY_EDITOR
        // Assets内のMesh、またはUnity標準PrimitiveのMeshだけを元Meshとして保持します。
        if (EditorUtility.IsPersistent(currentMesh))
            sourcePlaneMesh = currentMesh;
#else
        sourcePlaneMesh = currentMesh;
#endif
    }

    private void ResolveComponents()
    {
        if (!meshFilter)
            meshFilter = GetComponent<MeshFilter>();

        if (!meshCollider)
            meshCollider = GetComponent<MeshCollider>();
    }

    private void ConfigureMeshCollider(Mesh collisionMesh)
    {
        if (!meshCollider)
            meshCollider = GetComponent<MeshCollider>();

        if (!meshCollider)
        {
#if UNITY_EDITOR
            meshCollider = Undo.AddComponent<MeshCollider>(gameObject);
#else
            meshCollider = gameObject.AddComponent<MeshCollider>();
#endif
        }

        meshCollider.sharedMesh = null;
        meshCollider.convex = colliderConvex;
        meshCollider.isTrigger = colliderIsTrigger;
        meshCollider.sharedMaterial = colliderMaterial;
        meshCollider.sharedMesh = collisionMesh;

#if UNITY_EDITOR
        EditorUtility.SetDirty(meshCollider);

        if (!Application.isPlaying)
            UnityEditor.SceneManagement.EditorSceneManager.MarkSceneDirty(gameObject.scene);
#endif
    }

    private void DestroyTemporaryGeneratedMesh()
    {
        if (!generatedMesh)
            return;

#if UNITY_EDITOR
        // Assets内に保存されたMesh Assetは絶対に破棄しません。
        if (EditorUtility.IsPersistent(generatedMesh))
        {
            generatedMesh = null;
            return;
        }
#endif

        if (Application.isPlaying)
            Destroy(generatedMesh);
        else
            DestroyImmediate(generatedMesh);

        generatedMesh = null;
    }

    private void OnDrawGizmosSelected()
    {
        if (!drawProfileGizmos)
            return;

        Bounds sourceBounds = GetSourceBounds();

        int count = Mathf.Max(1, mountainCount);
        int panelCount = count * 2;
        float panelRun = Mathf.Max(0.01f, sourceBounds.size.z) / panelCount;
        float height = panelRun * Mathf.Tan(
            Mathf.Clamp(slopeAngleDegrees, 1f, 80f) * Mathf.Deg2Rad
        );

        float direction = mountainsPointUp ? 1f : -1f;
        float baseY = sourceBounds.center.y + baseHeightOffset;

        float lowY;
        float highY;

        if (centerProfileOnBaseHeight)
        {
            lowY = baseY - height * 0.5f * direction;
            highY = baseY + height * 0.5f * direction;
        }
        else
        {
            lowY = baseY;
            highY = baseY + height * direction;
        }

        Gizmos.color = Color.yellow;

        Vector3 previousLocal = new Vector3(
            sourceBounds.center.x,
            lowY,
            sourceBounds.min.z
        );

        for (int index = 1; index <= panelCount; index++)
        {
            bool peak = index % 2 == 1;

            Vector3 currentLocal = new Vector3(
                sourceBounds.center.x,
                peak ? highY : lowY,
                sourceBounds.min.z + index * panelRun
            );

            Gizmos.DrawLine(
                transform.TransformPoint(previousLocal),
                transform.TransformPoint(currentLocal)
            );

            previousLocal = currentLocal;
        }
    }

#if UNITY_EDITOR
    private static string FormatVector(Vector3 value)
    {
        return $"({value.x:R}, {value.y:R}, {value.z:R})";
    }
#endif
}