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

    [Header("Constant arc length (world units)")]
    [Min(.01f)] [SerializeField] float targetLength = .8f;
    [Min(.005f)] [SerializeField] float sampleSpacing = .05f;
    [Min(.1f)] [SerializeField] float teleportThreshold = 2.5f;
    [Range(8, 512)] [SerializeField] int maximumSamples = 128;

    [Header("Visibility diagnostic")]
    [Tooltip("Use an unlit cyan material and visible minimum width to isolate material/alpha problems. Disable after confirming that the mesh is visible.")]
    [SerializeField] bool diagnosticCyanMaterial = true;

    // Stored in the NON-ROTATING physics world frame; oldest -> newest.
    readonly List<Vector3> history = new List<Vector3>(128);
    readonly List<Vector3> path = new List<Vector3>(128); // world-space, oldest -> tip
    readonly List<Vector3> vertices = new List<Vector3>(256);
    readonly List<Vector2> uv = new List<Vector2>(256);
    readonly List<Color> colors = new List<Color>(256);
    readonly List<int> triangles = new List<int>(768);

    TrailRenderer displayTrail;
    GameObject ribbonObject;
    Mesh ribbonMesh;
    MeshRenderer ribbonRenderer;
    Material fallbackMaterial;
    Material diagnosticMaterial;
    bool lastDiagnosticCyanMaterial;
    AnimationCurve widthCurve;
    Gradient colorGradient;
    bool initialized;
    bool oldSourceEnabled, oldDisplayEnabled;
    TrailRenderer disabledSource, disabledDisplay;

    public void SetTrail(TrailRenderer targetTrail)
    {
        trail = targetTrail;
        if (initialized)
        {
            RestoreNativeTrails();
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
        if (!targetBody) targetBody = equalizer ? equalizer.InSubject : GetComponent<Rigidbody>();
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
        lastDiagnosticCyanMaterial = diagnosticCyanMaterial;
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

        if (lastDiagnosticCyanMaterial != diagnosticCyanMaterial)
        {
            CacheAppearance();
            lastDiagnosticCyanMaterial = diagnosticCyanMaterial;
        }

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
            float widthAtTip = trail.widthMultiplier * widthCurve.Evaluate(1f);
            float maxAlpha = Mathf.Max(colorGradient.Evaluate(0f).a,
                colorGradient.Evaluate(.5f).a,
                colorGradient.Evaluate(1f).a);
            Debug.Log(
                $"[Trail][VIS] active={ribbonObject.activeInHierarchy} " +
                $"meshLayer={LayerMask.LayerToName(ribbonObject.layer)} " +
                $"cameraSeesLayer={cameraSeesLayer} " +
                $"shader={ribbonRenderer.sharedMaterial?.shader?.name} " +
                $"widthAtTip={widthAtTip:F4} alphaMax={maxAlpha:F3} " +
                $"cyanDiagnostic={diagnosticCyanMaterial} " +
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
        float remaining = Mathf.Max(.01f, targetLength);

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
        // growing history. The exact 0.8 m tail is cut in the rendered path.
        if (oldestNeeded > 1)
            history.RemoveRange(0, oldestNeeded - 1);
    }

    void BuildRibbonMesh()
    {
        vertices.Clear();
        uv.Clear();
        colors.Clear();
        triangles.Clear();

        if (path.Count < 2)
        {
            ribbonMesh.Clear();
            return;
        }

        float total = 0f;
        for (int i = 1; i < path.Count; i++)
            total += Vector3.Distance(path[i - 1], path[i]);
        if (total < .0001f) { ribbonMesh.Clear(); return; }

        float traveled = 0f;
        Vector3 previousSide = Vector3.zero;
        Vector3 cameraPosition = viewCamera ? viewCamera.transform.position : path[path.Count - 1] + Vector3.back * 10f;

        for (int i = 0; i < path.Count; i++)
        {
            if (i > 0) traveled += Vector3.Distance(path[i - 1], path[i]);
            float u = Mathf.Clamp01(traveled / total);

            Vector3 tangent = i == 0 ? path[1] - path[0] :
                i == path.Count - 1 ? path[i] - path[i - 1] : path[i + 1] - path[i - 1];
            tangent.Normalize();

            Vector3 towardsCamera = (cameraPosition - path[i]).normalized;
            Vector3 side = Vector3.Cross(towardsCamera, tangent);
            if (side.sqrMagnitude < .000001f)
                side = Vector3.Cross(Vector3.up, tangent);
            if (side.sqrMagnitude < .000001f)
                side = Vector3.Cross(Vector3.right, tangent);
            side.Normalize();
            if (i > 0 && Vector3.Dot(side, previousSide) < 0f) side = -side;
            previousSide = side;

            float width = Mathf.Max(0f, trail.widthMultiplier * widthCurve.Evaluate(1f - u));
            if (diagnosticCyanMaterial) width = Mathf.Max(.15f, width);
            Vector3 half = side * (width * .5f);
            vertices.Add(ribbonObject.transform.InverseTransformPoint(path[i] - half));
            vertices.Add(ribbonObject.transform.InverseTransformPoint(path[i] + half));
            Color color = diagnosticCyanMaterial ? Color.white : colorGradient.Evaluate(u);
            colors.Add(color);
            colors.Add(color);
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

            // Diagnostic: include the reversed faces to rule out backface culling.
            if (diagnosticCyanMaterial)
            {
                triangles.Add(a + 1);
                triangles.Add(a + 2);
                triangles.Add(a);
                triangles.Add(a + 3);
                triangles.Add(a + 2);
                triangles.Add(a + 1);
            }
        }

        ribbonMesh.Clear();
        ribbonMesh.SetVertices(vertices);
        ribbonMesh.SetColors(colors);
        ribbonMesh.SetUVs(0, uv);
        ribbonMesh.SetTriangles(triangles, 0);
        ribbonMesh.RecalculateBounds();
        ribbonMesh.RecalculateNormals();
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
        if (!trail) return;
        widthCurve = trail.widthCurve;
        colorGradient = trail.colorGradient;
        if (diagnosticCyanMaterial)
        {
            if (!diagnosticMaterial)
            {
                Shader shader = Shader.Find("Universal Render Pipeline/Unlit");
                if (!shader) shader = Shader.Find("Unlit/Color");
                if (!shader) shader = Shader.Find("Sprites/Default");
                if (shader)
                {
                    diagnosticMaterial = new Material(shader);
                    diagnosticMaterial.name = "__Trail_DiagnosticCyan";
                    diagnosticMaterial.hideFlags = HideFlags.DontSave;
                    if (diagnosticMaterial.HasProperty("_BaseColor"))
                        diagnosticMaterial.SetColor("_BaseColor", Color.cyan);
                    if (diagnosticMaterial.HasProperty("_Color"))
                        diagnosticMaterial.SetColor("_Color", Color.cyan);
                }
                else Debug.LogWarning("[Trail][VIS] No unlit shader found; using the source material.", this);
            }
            ribbonRenderer.sharedMaterial = diagnosticMaterial ? diagnosticMaterial : trail.sharedMaterial;
        }
        else if (trail.sharedMaterial)
        {
            ribbonRenderer.sharedMaterial = trail.sharedMaterial;
        }
        else
        {
            if (!fallbackMaterial)
            {
                Shader shader = Shader.Find("Universal Render Pipeline/Unlit");
                if (!shader) shader = Shader.Find("Sprites/Default");
                if (shader) fallbackMaterial = new Material(shader);
            }
            ribbonRenderer.sharedMaterial = fallbackMaterial;
        }
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
        if (fallbackMaterial) Destroy(fallbackMaterial);
        if (diagnosticMaterial) Destroy(diagnosticMaterial);
    }
}

