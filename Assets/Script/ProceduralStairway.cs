using System.Collections.Generic;
using UnityEngine;

public class ProceduralStairway : MonoBehaviour
{
    public enum TurnMode
    {
        Right,
        Left,
        Alternate,
        Random
    }

    public GameObject Player;
    [Header("Rendering")]
    [SerializeField] private Material stairMaterial;
    [SerializeField] private Material goalMaterial;

    [Header("Course")]
    [Min(1)] [SerializeField] private int totalSections = 8;
    [Min(1)] [SerializeField] private int stepsPerSection = 16;
    [Min(1)] [SerializeField] private int turnEverySteps = 8;
    [Min(1)] [SerializeField] private int initialSections = 1;
    [Min(1)] [SerializeField] private int keepAliveSections = 4;
    [SerializeField] private TurnMode turnMode = TurnMode.Right;

    [Header("Step Shape")]
    [Min(0.5f)] [SerializeField] private float stepWidth = 2.0f;
    [Min(0.3f)] [SerializeField] private float stepDepth = 1.0f;
    [Min(0.1f)] [SerializeField] private float stepHeight = 0.35f;
    [Min(0.05f)] [SerializeField] private float stepThickness = 0.30f;

    [Header("Section Trigger")]
    [Min(0.05f)] [SerializeField] private float triggerThickness = 0.18f;
    [Min(1.0f)] [SerializeField] private float triggerHeight = 2.4f;

    [Header("Goal")]
    [Min(0.5f)] [SerializeField] private float goalDepth = 1.25f;
    [Min(1.5f)] [SerializeField] private float goalHeight = 2.4f;

    private readonly Queue<GameObject> activeSections = new Queue<GameObject>();

    // 次に生成する段の「後ろ端・上面中央」
    private Vector3 cursorBackEdge;
    private Quaternion currentRotation;

    private int totalStepsGenerated;
    private int generatedSectionCount;
    private int alternatingTurnSign = 1;
    private float lowestGeneratedY;
    private GameObject goalRoot;

    public float LowestGeneratedY => lowestGeneratedY;
    public int GeneratedSectionCount => generatedSectionCount;
    public int TotalSections => totalSections;
    public bool GoalSpawned => goalRoot != null;

    public Vector3 RecommendedSpawnPosition => transform.TransformPoint(new Vector3(0f, 1.05f, stepDepth * 0.25f));
    public Quaternion RecommendedSpawnRotation => transform.rotation;

    private void Start()
    {
        var obj=Instantiate(Player.gameObject, Player.transform.position, Quaternion.identity);
        obj.transform.name = "subject";
        //obj.transform.position = new Vector3(0, 0.5f, 0);
        obj.transform.GetComponent<Rigidbody>().velocity = Vector3.zero;
       // RebuildCourse();
    }

    public void RebuildCourse()
    {
        ClearRuntimeObjects();

        stepThickness = Mathf.Min(stepThickness, stepHeight);
        initialSections = Mathf.Clamp(initialSections, 1, Mathf.Max(1, totalSections));

        cursorBackEdge = Vector3.zero;
        currentRotation = Quaternion.identity;
        totalStepsGenerated = 0;
        generatedSectionCount = 0;
        alternatingTurnSign = 1;
        lowestGeneratedY = transform.position.y;

        for (int i = 0; i < initialSections; i++)
        {
            GenerateNextSection();
        }
    }

    public void NotifySectionPassed(int sectionIndex)
    {
        if (sectionIndex < 0)
            return;

        if (generatedSectionCount >= totalSections)
            return;

        GenerateNextSection();
    }
    private void CreateHiddenRamp(Transform sectionRoot, Vector3 startTop, Vector3 endTop, Quaternion yawRotation)
    {
        GameObject ramp = new GameObject("HiddenRamp");
        ramp.transform.SetParent(sectionRoot, false);

        Vector3 mid = (startTop + endTop) * 0.5f;
        Vector3 flatForward = new Vector3(endTop.x - startTop.x, 0f, endTop.z - startTop.z);
        float horizontalLength = flatForward.magnitude;
        float verticalDrop = startTop.y - endTop.y;
        float rampLength = Mathf.Sqrt(horizontalLength * horizontalLength + verticalDrop * verticalDrop);

        ramp.transform.localPosition = mid;

        Quaternion look = flatForward.sqrMagnitude > 0.0001f
            ? Quaternion.LookRotation(flatForward.normalized, Vector3.up)
            : yawRotation;

        float pitch = -Mathf.Atan2(verticalDrop, horizontalLength) * Mathf.Rad2Deg;
        ramp.transform.localRotation = look * Quaternion.Euler(pitch, 0f, 0f);

        BoxCollider col = ramp.AddComponent<BoxCollider>();
        col.isTrigger = false;
        col.size = new Vector3(stepWidth * 0.9f, 0.2f, rampLength);
    }
    private void GenerateNextSection()
    {
        if (generatedSectionCount >= totalSections)
            return;

        int sectionIndex = generatedSectionCount;

        GameObject sectionObject = new GameObject($"StairSection_{sectionIndex}");
        sectionObject.transform.SetParent(transform, false);
        sectionObject.transform.localPosition = Vector3.zero;
        sectionObject.transform.localRotation = Quaternion.identity;

        List<Vector3> vertices = new List<Vector3>(stepsPerSection * 24);
        List<int> triangles = new List<int>(stepsPerSection * 36);
        List<Vector2> uvs = new List<Vector2>(stepsPerSection * 24);

        for (int i = 0; i < stepsPerSection; i++)
        {
            if (totalStepsGenerated > 0 && totalStepsGenerated % turnEverySteps == 0)
            {
                RotateHeading90();
            }

            Vector3 stepCenter =
                cursorBackEdge +
                currentRotation * new Vector3(0f, -stepThickness * 0.5f, stepDepth * 0.5f);

            AddBox(vertices, triangles, uvs, stepCenter, currentRotation, new Vector3(stepWidth, stepThickness, stepDepth));

            float stepBottomY = stepCenter.y - stepThickness * 0.5f;
            lowestGeneratedY = Mathf.Min(lowestGeneratedY, transform.TransformPoint(new Vector3(0f, stepBottomY, 0f)).y);

            cursorBackEdge += currentRotation * new Vector3(0f, -stepHeight, stepDepth);
            totalStepsGenerated++;
        }

        Mesh mesh = new Mesh
        {
            name = $"StairSectionMesh_{sectionIndex}"
        };
        mesh.SetVertices(vertices);
        mesh.SetTriangles(triangles, 0);
        mesh.SetUVs(0, uvs);
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();

        MeshFilter meshFilter = sectionObject.AddComponent<MeshFilter>();
        meshFilter.sharedMesh = mesh;

        MeshRenderer meshRenderer = sectionObject.AddComponent<MeshRenderer>();
        if (stairMaterial != null)
        {
            meshRenderer.sharedMaterial = stairMaterial;
        }

        MeshCollider meshCollider = sectionObject.AddComponent<MeshCollider>();
        meshCollider.sharedMesh = mesh;

        generatedSectionCount++;
        activeSections.Enqueue(sectionObject);

        Transform endPoint = CreateSectionEndPoint(sectionObject.transform, sectionIndex);

        bool hasMoreSections = generatedSectionCount < totalSections;
        if (hasMoreSections)
        {
            CreateSpawnTrigger(endPoint, sectionIndex);
        }
        else
        {
            CreateGoalGate(endPoint);
        }

        CleanupOldSections();
    }

    private void RotateHeading90()
    {
        int sign = 1;

        switch (turnMode)
        {
            case TurnMode.Right:
                sign = 1;
                break;
            case TurnMode.Left:
                sign = -1;
                break;
            case TurnMode.Alternate:
                sign = alternatingTurnSign;
                alternatingTurnSign *= -1;
                break;
            case TurnMode.Random:
                sign = Random.value < 0.5f ? -1 : 1;
                break;
        }

        currentRotation *= Quaternion.Euler(0f, sign * 90f, 0f);
    }

    private Transform CreateSectionEndPoint(Transform sectionParent, int sectionIndex)
    {
        // cursorBackEdge は「次の段の後ろ端・上面中央」
        // つまりこの位置が、その Section を抜けるための基準終点になる
        Vector3 endPointLocalPosition = cursorBackEdge + Vector3.up * stepHeight;

        GameObject endPointObject = new GameObject($"SectionEndPoint_{sectionIndex}");
        endPointObject.transform.SetParent(sectionParent, false);
        endPointObject.transform.localPosition = endPointLocalPosition;
        endPointObject.transform.localRotation = currentRotation;

        return endPointObject.transform;
    }

    private void CreateSpawnTrigger(Transform endPoint, int sectionIndex)
    {
        GameObject triggerObject = new GameObject($"SpawnLine_{sectionIndex}");
        triggerObject.transform.SetParent(endPoint, false);
        triggerObject.transform.localPosition = new Vector3(0f, triggerHeight * 0.5f - 0.15f, -triggerThickness * 0.5f);
        triggerObject.transform.localRotation = Quaternion.identity;

        BoxCollider box = triggerObject.AddComponent<BoxCollider>();
        box.isTrigger = true;
        box.size = new Vector3(stepWidth + 0.2f, triggerHeight, triggerThickness);
        box.center = Vector3.zero;

        Rigidbody rb = triggerObject.AddComponent<Rigidbody>();
        rb.isKinematic = true;
        rb.useGravity = false;

        StairSectionTrigger trigger = triggerObject.AddComponent<StairSectionTrigger>();
        trigger.Initialize(this, sectionIndex);
    }

    private void CreateGoalGate(Transform endPoint)
    {
        if (goalRoot != null)
            return;

        goalRoot = new GameObject("GoalGate");
        goalRoot.transform.SetParent(endPoint, false);
        goalRoot.transform.localPosition = new Vector3(0f, goalHeight * 0.5f - 0.2f, goalDepth * 0.5f);
        goalRoot.transform.localRotation = Quaternion.identity;

        CreateGoalPart("LeftPole", goalRoot.transform, new Vector3(-stepWidth * 0.45f, goalHeight * 0.5f, 0f), new Vector3(0.18f, goalHeight, 0.18f));
        CreateGoalPart("RightPole", goalRoot.transform, new Vector3(stepWidth * 0.45f, goalHeight * 0.5f, 0f), new Vector3(0.18f, goalHeight, 0.18f));
        CreateGoalPart("TopBar", goalRoot.transform, new Vector3(0f, goalHeight, 0f), new Vector3(stepWidth + 0.3f, 0.18f, 0.18f));
        CreateGoalPart("Pad", goalRoot.transform, new Vector3(0f, -goalHeight * 0.5f + 0.06f, 0.15f), new Vector3(stepWidth + 0.25f, 0.12f, goalDepth));
        CreateGoalPart("Marker", goalRoot.transform, new Vector3(0f, goalHeight * 0.55f, 0f), new Vector3(0.35f, 0.35f, 0.35f));

        GameObject triggerObject = new GameObject("GoalTrigger");
        triggerObject.transform.SetParent(goalRoot.transform, false);
        triggerObject.transform.localPosition = Vector3.zero;
        triggerObject.transform.localRotation = Quaternion.identity;

        BoxCollider triggerCollider = triggerObject.AddComponent<BoxCollider>();
        triggerCollider.isTrigger = true;
        triggerCollider.size = new Vector3(stepWidth + 0.3f, goalHeight, goalDepth);
        triggerCollider.center = Vector3.zero;

        Rigidbody rb = triggerObject.AddComponent<Rigidbody>();
        rb.isKinematic = true;
        rb.useGravity = false;

        triggerObject.AddComponent<StairGoalTrigger>();
    }

    private void CreateGoalPart(string partName, Transform parent, Vector3 localPosition, Vector3 localScale)
    {
        GameObject part = GameObject.CreatePrimitive(PrimitiveType.Cube);
        part.name = partName;
        part.transform.SetParent(parent, false);
        part.transform.localPosition = localPosition;
        part.transform.localRotation = Quaternion.identity;
        part.transform.localScale = localScale;

        Collider collider = part.GetComponent<Collider>();
        if (collider != null)
        {
            Destroy(collider);
        }

        if (goalMaterial != null)
        {
            MeshRenderer renderer = part.GetComponent<MeshRenderer>();
            if (renderer != null)
            {
                renderer.sharedMaterial = goalMaterial;
            }
        }
    }

    private void CleanupOldSections()
    {
        while (activeSections.Count > keepAliveSections)
        {
            GameObject oldest = activeSections.Dequeue();
            if (oldest == null)
                continue;

            MeshFilter mf = oldest.GetComponent<MeshFilter>();
            if (mf != null && mf.sharedMesh != null)
            {
                Destroy(mf.sharedMesh);
            }

            Destroy(oldest);
        }
    }

    private void ClearRuntimeObjects()
    {
        while (activeSections.Count > 0)
        {
            GameObject section = activeSections.Dequeue();
            if (section == null)
                continue;

            MeshFilter mf = section.GetComponent<MeshFilter>();
            if (mf != null && mf.sharedMesh != null)
            {
                if (Application.isPlaying)
                    Destroy(mf.sharedMesh);
                else
                    DestroyImmediate(mf.sharedMesh);
            }

            if (Application.isPlaying)
                Destroy(section);
            else
                DestroyImmediate(section);
        }

        if (goalRoot != null)
        {
            if (Application.isPlaying)
                Destroy(goalRoot);
            else
                DestroyImmediate(goalRoot);

            goalRoot = null;
        }
    }

    private static void AddBox(
        List<Vector3> vertices,
        List<int> triangles,
        List<Vector2> uvs,
        Vector3 center,
        Quaternion rotation,
        Vector3 size)
    {
        Vector3 r = rotation * Vector3.right * (size.x * 0.5f);
        Vector3 u = rotation * Vector3.up * (size.y * 0.5f);
        Vector3 f = rotation * Vector3.forward * (size.z * 0.5f);

        Vector3 lbd = center - r - u - f;
        Vector3 rbd = center + r - u - f;
        Vector3 rtd = center + r + u - f;
        Vector3 ltd = center - r + u - f;

        Vector3 lbf = center - r - u + f;
        Vector3 rbf = center + r - u + f;
        Vector3 rtf = center + r + u + f;
        Vector3 ltf = center - r + u + f;

        AddQuad(vertices, triangles, uvs, lbf, rbf, rtf, ltf); // Front
        AddQuad(vertices, triangles, uvs, rbd, lbd, ltd, rtd); // Back
        AddQuad(vertices, triangles, uvs, rbf, rbd, rtd, rtf); // Right
        AddQuad(vertices, triangles, uvs, lbd, lbf, ltf, ltd); // Left
        AddQuad(vertices, triangles, uvs, ltf, rtf, rtd, ltd); // Top
        AddQuad(vertices, triangles, uvs, lbd, rbd, rbf, lbf); // Bottom
    }

    private static void AddQuad(
        List<Vector3> vertices,
        List<int> triangles,
        List<Vector2> uvs,
        Vector3 v0,
        Vector3 v1,
        Vector3 v2,
        Vector3 v3)
    {
        int start = vertices.Count;

        vertices.Add(v0);
        vertices.Add(v1);
        vertices.Add(v2);
        vertices.Add(v3);

        uvs.Add(new Vector2(0f, 0f));
        uvs.Add(new Vector2(1f, 0f));
        uvs.Add(new Vector2(1f, 1f));
        uvs.Add(new Vector2(0f, 1f));

        triangles.Add(start + 0);
        triangles.Add(start + 1);
        triangles.Add(start + 2);
        triangles.Add(start + 0);
        triangles.Add(start + 2);
        triangles.Add(start + 3);
    }
}
