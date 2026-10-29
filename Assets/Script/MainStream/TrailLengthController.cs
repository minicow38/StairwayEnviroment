using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

// Run after CorrespondSubject (10000) and ExampleBallVisualEqualizer (12000).
[DefaultExecutionOrder(13000)]
[DisallowMultipleComponent]
public sealed class TrailLengthController : MonoBehaviour
{
    [Header("Scene references")]
    [SerializeField] Rigidbody targetBody;
    [SerializeField] CorrespondSubject coordinateSource;
    [SerializeField] ExampleBallVisualEqualizer equalizer;
    [SerializeField] TrailRenderer trail;
    [SerializeField] Camera viewCamera;
    
    [SerializeField] SlopeStickCore core;

    [SerializeField] float minTrailLength = .05f;
    [SerializeField] float maxTrailLength = 5f;
    [SerializeField] float referenceSpeed = 16f;
    [SerializeField, Range(1f, 5f)]
    float speedExponent = 3f;
    
    [Header("Constant arc length (world units)")]
    [Min(.01f)] [SerializeField] float targetLength = .8f;
    [Min(.005f)] [SerializeField] float sampleSpacing = .05f;
    [Min(.1f)] [SerializeField] float teleportThreshold = 2.5f;
    [Range(8, 512)] [SerializeField] int maximumSamples = 128;

    [Header("Ribbon geometry (appearance comes from TrailRenderer)")]
    [Tooltip("Only the mesh subdivision size. Width, Width Curve, Color and Material are controlled by the original TrailRenderer.")]
    [Range(.08f, .4f)] [SerializeField] float meshSegmentLength = .16f;

    // Stored in the NON-ROTATING physics world frame; oldest -> newest.
    readonly List<Vector3> history = new List<Vector3>(128);
    readonly List<Vector3> path = new List<Vector3>(128); // world-space, oldest -> tip
    readonly List<Vector3> meshPath = new List<Vector3>(256); // purely visual subdivision
    readonly List<Vector3> vertices = new List<Vector3>(256);
    readonly List<Vector2> uv = new List<Vector2>(256);
    readonly List<Color> colors = new List<Color>(256);
    readonly List<Vector3> normals = new List<Vector3>(256);
    readonly List<int> triangles = new List<int>(768);

    TrailRenderer displayTrail;
    GameObject ribbonObject;
    Mesh ribbonMesh;
    MeshRenderer ribbonRenderer;
    bool missingMaterialWarningShown;
    AnimationCurve widthCurve;
    Gradient colorGradient;
    bool initialized;
    bool oldSourceEnabled, oldDisplayEnabled;
    TrailRenderer disabledSource, disabledDisplay;

    public void SetTrail(TrailRenderer targetTrail)
    {
        if (initialized) RestoreNativeTrails();
        trail = targetTrail;
        if (initialized)
        {
            CacheAppearance();
            DisableNativeTrails();
        }
    }

    // Legacy API: fixed-length ribbons no longer need a time-based hold.
    // Called during a turn, it does not interrupt the position history.
    public void HoldTrailTime(float minimumTime) { }
    public void ReleaseTrailTime() { }

    // Can be called on a genuine respawn; never call it solely for a turn.
    public void ClearTrail() => ClearRibbon();

    void OnEnable()
    {
        if (initialized && ribbonObject) ribbonObject.SetActive(true);
    }

    void Awake()
    {
        Debug.Log(
            $"[Trail][AWAKE] enabled={enabled} " +
            $"active={gameObject.activeInHierarchy}",
            this
        );
    }

    void Start()
    {
        Debug.Log("[Trail][START] 実行開始", this);

        if (!equalizer) equalizer = GetComponent<ExampleBallVisualEqualizer>();
        if (!equalizer) equalizer = FindObjectOfType<ExampleBallVisualEqualizer>();
        if (!coordinateSource) coordinateSource = FindObjectOfType<CorrespondSubject>();
        if (!core && targetBody)
            core = targetBody.GetComponent<SlopeStickCore>();
        if (!trail) trail = GetComponent<TrailRenderer>();
        if (!trail && equalizer) trail = equalizer.DisplayTrail;
        if (!viewCamera) viewCamera = Camera.main;

        // The Equalizer only creates DisplayTrail when it makes a renderer-only proxy.
        // When visualTarget has no Rigidbody, the original TrailRenderer is the display trail.
        TrailRenderer equalizerTrail = equalizer ? equalizer.DisplayTrail : null;
        displayTrail = equalizerTrail ? equalizerTrail : trail;

        Debug.Log(
            $"[Trail][REFS] body={Bool(targetBody)} " +
            $"coordinate={Bool(coordinateSource)} " +
            $"equalizer={Bool(equalizer)} " +
            $"sourceTrail={Bool(trail)} " +
            $"equalizerDisplayTrail={Bool(equalizerTrail)} " +
            $"selectedTrail={Bool(displayTrail)}", this
        );

        if (!coordinateSource || !trail || !displayTrail || !targetBody)
        {
            Debug.LogError(
                "[Trail][MISSING] References are incomplete. Check [Trail][REFS] above.", this
            );
            enabled = false;
            return;
        }

        CreateRibbon();
        CacheAppearance();
        DisableNativeTrails();
        initialized = true;
       
        Debug.Log(
            $"[Trail][INIT] initialized={initialized} " +
            $"ribbon={ribbonObject?.name} " +
            $"displayTrail={displayTrail?.name} " +
            $"material={ribbonRenderer?.sharedMaterial?.name}"
        );
    }

    void LateUpdate()
    {
        if (!initialized) return;

        // BVE copies original TrailRenderer settings in its LateUpdate. Hide its
        // time-based trail AFTER BVE so only the fixed-length mesh is visible.
        // Prefer BVE's generated display trail when one exists; otherwise use the source.
        TrailRenderer currentDisplay = equalizer && equalizer.DisplayTrail
            ? equalizer.DisplayTrail : trail;
        if (currentDisplay != displayTrail)
        {
            RestoreNativeTrails();
            displayTrail = currentDisplay;
            if (!displayTrail) { ClearRibbon(); return; }
            ribbonObject.layer = displayTrail.gameObject.layer;
            CacheAppearance();
            Debug.Log($"[Trail][DISPLAY_CHANGED] using={displayTrail.name}", this);
        }
        DisableNativeTrails();

        // TrailRenderer is the single appearance authority. Live Inspector edits
        // are picked up without duplicating curve/width/material settings here.
        CacheAppearance();

        Transform head = displayTrail.transform;
        Vector3 physicsHead = coordinateSource.InverseMapPoint(head.position);
        if (!IsFinite(physicsHead)) { ClearRibbon(); return; }

        if (history.Count == 0)
        {
            history.Add(physicsHead);
        }
        else
        {
            float distance = Vector3.Distance(history[history.Count - 1], physicsHead);
            if (distance > teleportThreshold)
            {
                ClearRibbon();
                history.Add(physicsHead);
            }
            else if (distance >= sampleSpacing)
            {
                history.Add(physicsHead);
            }
        }

        if (history.Count > maximumSamples)
            history.RemoveRange(0, history.Count - maximumSamples);

        BuildArcLengthPath(physicsHead);
        BuildRibbonMesh();
        if (Time.frameCount % 60 == 0)
        {
            float length = 0f;

            for (int i = 1; i < path.Count; i++)
                length += Vector3.Distance(path[i - 1], path[i]);

            Debug.Log(
                $"[Trail][STATE] samples={history.Count} " +
                $"vertices={ribbonMesh.vertexCount} " +
                $"length={length:F3} " +
                $"renderer={ribbonRenderer.enabled}"
            );

            bool cameraSeesLayer = !viewCamera ||
                (viewCamera.cullingMask & (1 << ribbonObject.layer)) != 0;
            // At the ball end, the original TrailRenderer curve is evaluated at 0.
            float widthAtTip = trail.widthMultiplier * widthCurve.Evaluate(0f);
            float maxAlpha = Mathf.Max(colorGradient.Evaluate(0f).a,
                colorGradient.Evaluate(.5f).a,
                colorGradient.Evaluate(1f).a);
            Debug.Log(
                $"[Trail][VIS] active={ribbonObject.activeInHierarchy} " +
                $"meshLayer={LayerMask.LayerToName(ribbonObject.layer)} " +
                $"cameraSeesLayer={cameraSeesLayer} " +
                $"shader={ribbonRenderer.sharedMaterial?.shader?.name} " +
                $"sourceWidthAtTip={widthAtTip:F4} sourceAlphaMax={maxAlpha:F3} " +
                $"widthMultiplier={trail.widthMultiplier:F3} " +
                $"sourceMaterial={(trail.sharedMaterial ? trail.sharedMaterial.name : "None")} " +
                $"tip={path[path.Count - 1]} " +
                $"cameraDistance={(viewCamera ? Vector3.Distance(viewCamera.transform.position, path[path.Count - 1]) : -1f):F2}", this
            );
        }
    }

    // Constructs an EXACTLY targetLength-long polyline when enough trajectory
    // exists. When the ball stops/turns, no samples expire due to elapsed time.
    void BuildArcLengthPath(Vector3 physicsHead)
    {
        path.Clear();
        Vector3 previous = coordinateSource.MapPoint(physicsHead);
        path.Add(previous);
        

        float speed = targetBody.velocity.magnitude;
        float t = Mathf.Clamp01(speed / Mathf.Max(.01f, referenceSpeed));

        float remaining = Mathf.Lerp(
            minTrailLength,
            maxTrailLength,
            Mathf.Pow(t, speedExponent)
        );

        int oldestNeeded = 0;
        for (int i = history.Count - 1; i >= 0; i--)
        {
            Vector3 next = coordinateSource.MapPoint(history[i]);
            float length = Vector3.Distance(previous, next);
            if (length < .00001f) continue;

            if (length >= remaining)
            {
                path.Add(Vector3.Lerp(previous, next, remaining / length));
                oldestNeeded = i;
                break;
            }

            path.Add(next);
            remaining -= length;
            previous = next;
            oldestNeeded = i;
        }
        path.Reverse();

        // Retain one extra segment for smooth motion, not an indefinitely
        // growing history. The trail is clipped to targetLength.
        if (oldestNeeded > 1)
            history.RemoveRange(0, oldestNeeded - 1);
    }

    void BuildRibbonMesh()
    {
        vertices.Clear();
        uv.Clear();
        colors.Clear();
        normals.Clear();
        triangles.Clear();
        meshPath.Clear();

        if (path.Count < 2)
        {
            ribbonMesh.Clear();
            return;
        }

        // Straight subdivision keeps the EXACT centerline arc length. Curving
        // the points with a spline here would subtly change the target length.
        meshPath.Add(path[0]);
        for (int i = 1; i < path.Count; i++)
        {
            Vector3 a = path[i - 1];
            Vector3 b = path[i];
            int steps = Mathf.Clamp(
                Mathf.CeilToInt(Vector3.Distance(a, b) / meshSegmentLength), 1, 12);
            for (int j = 1; j <= steps; j++)
                meshPath.Add(Vector3.Lerp(a, b, (float)j / steps));
        }

        float total = 0f;
        for (int i = 1; i < meshPath.Count; i++)
            total += Vector3.Distance(meshPath[i - 1], meshPath[i]);
        if (total < .0001f) { ribbonMesh.Clear(); return; }

        float traveled = 0f;
        Vector3 previousSide = Vector3.zero;
        Vector3 cameraFacing = viewCamera ? -viewCamera.transform.forward : Vector3.back;
        // Mirrors the two native TrailRenderer alignment modes.
        bool useTransformZ = trail && trail.alignment == LineAlignment.TransformZ;
        Vector3 fixedFacing = useTransformZ ? displayTrail.transform.forward : cameraFacing;
        if (fixedFacing.sqrMagnitude < 1e-6f) fixedFacing = Vector3.back;
        fixedFacing.Normalize();

        for (int i = 0; i < meshPath.Count; i++)
        {
            if (i > 0)
                traveled += Vector3.Distance(meshPath[i - 1], meshPath[i]);

            // 0 = old tail, 1 = current ball. Distance fraction, not time.
            float u = Mathf.Clamp01(traveled / total);

            Vector3 before = i == 0
                ? (meshPath[1] - meshPath[0]).normalized
                : (meshPath[i] - meshPath[i - 1]).normalized;
            Vector3 after = i == meshPath.Count - 1
                ? before
                : (meshPath[i + 1] - meshPath[i]).normalized;

            // Native TrailRenderer-like camera-facing flat ribbon.
            Vector3 sideBefore = SafeSide(fixedFacing, before, previousSide);
            Vector3 sideAfter = SafeSide(fixedFacing, after, sideBefore);
            if (Vector3.Dot(sideBefore, sideAfter) < 0f)
                sideAfter = -sideAfter;

            Vector3 side = sideBefore + sideAfter;
            if (side.sqrMagnitude < 1e-6f)
                side = sideAfter;
            else side.Normalize();

            // Use the artist-authored TrailRenderer profile literally.
            // No secondary cone, taper, minimum width, or width amplification.
            // u=0: old tail; u=1: ball. The source curve's t=0 is at the ball.
            float curveT = 1f - u;
            float width = Mathf.Max(0f, trail.widthMultiplier * widthCurve.Evaluate(curveT));
            Color color = colorGradient.Evaluate(curveT);

            if (i > 0 && Vector3.Dot(side, previousSide) < 0f)
                side = -side;
            previousSide = side;

            // Do not widen sharp corners beyond the Width curve's value.
            Vector3 halfWidth = side * (width * .5f);
            Vector3 p = meshPath[i];
            vertices.Add(ribbonObject.transform.InverseTransformPoint(p - halfWidth));
            vertices.Add(ribbonObject.transform.InverseTransformPoint(p + halfWidth));
            colors.Add(color);
            colors.Add(color);
            // Consistent normals even with duplicated back-faces (native
            // TrailRenderer may use a material that reads mesh normals).
            Vector3 normal = ribbonObject.transform.InverseTransformDirection(fixedFacing);
            normals.Add(normal);
            normals.Add(normal);
            uv.Add(new Vector2(u, 0f));
            uv.Add(new Vector2(u, 1f));

            if (i == 0) continue;
            int a = (i - 1) * 2;
            triangles.Add(a);
            triangles.Add(a + 2);
            triangles.Add(a + 1);
            triangles.Add(a + 1);
            triangles.Add(a + 2);
            triangles.Add(a + 3);

            // Double-sided geometry prevents turn-dependent back-face loss.
            triangles.Add(a + 1);
            triangles.Add(a + 2);
            triangles.Add(a);
            triangles.Add(a + 3);
            triangles.Add(a + 2);
            triangles.Add(a + 1);
        }

        ribbonMesh.Clear();
        ribbonMesh.SetVertices(vertices);
        ribbonMesh.SetColors(colors);
        ribbonMesh.SetNormals(normals);
        ribbonMesh.SetUVs(0, uv);
        ribbonMesh.SetTriangles(triangles, 0);
        ribbonMesh.RecalculateBounds();
        // Avoid RecalculateNormals: reversed faces would cancel out normals.
    }

    // Perpendicular to a segment, with stable fallback when it points at camera.
    static Vector3 SafeSide(Vector3 facing, Vector3 tangent, Vector3 fallback)
    {
        Vector3 side = Vector3.Cross(facing, tangent);
        if (side.sqrMagnitude < 1e-6f)
            side = fallback.sqrMagnitude > 1e-6f ? fallback : Vector3.Cross(Vector3.up, tangent);
        if (side.sqrMagnitude < 1e-6f)
            side = Vector3.Cross(Vector3.right, tangent);
        return side.sqrMagnitude > 1e-6f ? side.normalized : Vector3.right;
    }

    void CreateRibbon()
    {
        ribbonObject = new GameObject("__Trail_FixedArcLength");
        Transform root = coordinateSource.VisualPlayerRoot;
        if (root) ribbonObject.transform.SetParent(root, false);
        // The new GameObject defaults to Default, which may be excluded by the camera.
        // Keep the same camera-culling layer as the source of the displayed trail.
        ribbonObject.layer = displayTrail.gameObject.layer;
        ribbonObject.transform.localPosition = Vector3.zero;
        ribbonObject.transform.localRotation = Quaternion.identity;
        ribbonObject.transform.localScale = Vector3.one;

        ribbonMesh = new Mesh { name = "FixedArcLengthTrail", hideFlags = HideFlags.DontSave };
        ribbonMesh.MarkDynamic();
        ribbonObject.AddComponent<MeshFilter>().sharedMesh = ribbonMesh;
        ribbonRenderer = ribbonObject.AddComponent<MeshRenderer>();
        ribbonRenderer.shadowCastingMode = ShadowCastingMode.Off;
        ribbonRenderer.receiveShadows = false;
    }

    void CacheAppearance()
    {
        if (!trail || !ribbonRenderer) return;

        // TrailRenderer owns the appearance; this component only draws the
        // geometry at a fixed arc length in the non-rotating coordinate frame.
        widthCurve = trail.widthCurve;
        colorGradient = trail.colorGradient;

        Material sourceMaterial = trail.sharedMaterial;
        if (ribbonRenderer.sharedMaterial != sourceMaterial)
            ribbonRenderer.sharedMaterial = sourceMaterial;

        if (!sourceMaterial && !missingMaterialWarningShown)
        {
            missingMaterialWarningShown = true;
            Debug.LogWarning("[Trail][MATERIAL] Assign a Material on the source TrailRenderer.", this);
        }
        if (sourceMaterial) missingMaterialWarningShown = false;

        ribbonRenderer.renderingLayerMask = trail.renderingLayerMask;
        ribbonRenderer.sortingLayerID = trail.sortingLayerID;
        ribbonRenderer.sortingOrder = trail.sortingOrder;
    }

    void DisableNativeTrails()
    {
        if (trail && disabledSource != trail)
        {
            disabledSource = trail;
            oldSourceEnabled = trail.enabled;
        }
        if (displayTrail && displayTrail != trail && disabledDisplay != displayTrail)
        {
            disabledDisplay = displayTrail;
            oldDisplayEnabled = displayTrail.enabled;
        }

        if (trail) trail.enabled = false;
        if (displayTrail) displayTrail.enabled = false;
    }

    void RestoreNativeTrails()
    {
        if (disabledSource) disabledSource.enabled = oldSourceEnabled;
        if (disabledDisplay) disabledDisplay.enabled = oldDisplayEnabled;
        disabledSource = null;
        disabledDisplay = null;
    }

    void ClearRibbon()
    {
        history.Clear();
        path.Clear();
        if (ribbonMesh) ribbonMesh.Clear();
    }

    static bool Bool(Object component) => component != null;

    static bool IsFinite(Vector3 v) =>
        !float.IsNaN(v.x) && !float.IsNaN(v.y) && !float.IsNaN(v.z) &&
        !float.IsInfinity(v.x) && !float.IsInfinity(v.y) && !float.IsInfinity(v.z);

    void OnDisable()
    {
        RestoreNativeTrails();
        if (ribbonObject) ribbonObject.SetActive(false);
    }

    void OnDestroy()
    {
        RestoreNativeTrails();
        if (ribbonObject) Destroy(ribbonObject);
        if (ribbonMesh) Destroy(ribbonMesh);
    }
}
