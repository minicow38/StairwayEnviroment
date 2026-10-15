using System;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Splines;

/// <summary>
/// Five independently owned SplineContainers + a renderer-only measured Equalizer.
/// The five geometric splines and their entry/exit lifetime are unchanged.
/// A compact reference-BVE Hermite baseline is driven by real Knot progress;
/// actual contacts and noninertial observations drive only its bounded residual. All five
/// spline paths, the stage meshes, and the physics source are READ ONLY to the
/// Equalizer. A separate renderer-only visual object receives the display offset.
/// </summary>
[DefaultExecutionOrder(12440), DisallowMultipleComponent]
public sealed class IndependentFiveSplineWeaver : MonoBehaviour
{
    const float E = .00001f;
    const int Out = 0, Center = 1, Inside = 2, Observed = 3, Future = 4, Count = 5;
    static readonly string[] Names = {
        "OutCorner", "CenterCorn", "InCorner",
        "EqualizerObservedSpline", "EqualizerFutureBallSpline"
    };
    static readonly Color[] Colors = {
        new Color(1f,.55f,.12f), new Color(.1f,.9f,1f), new Color(.32f,1f,.28f),
        new Color(1f,.22f,.76f), new Color(.35f,.48f,1f)
    };

    [Header("Renderer geometry - READ ONLY")]
    [SerializeField] Transform generatedVisualRoot;
    [SerializeField, Min(.025f)] float rendererScanSeconds = .10f;
    [SerializeField, Min(.5f)] float fallbackPieceLength = 9.9f;
    [SerializeField] float renderLocalHeight;

    [Header("Movement reference - READ ONLY (never moved)")]
    [SerializeField] Transform visualMotionSource;    // e.g. VisualPlayerRoot/subject
    [SerializeField] Transform displayedBall;        // optional alternative input
    [SerializeField] Rigidbody inSubject;            // optional read-only fallback
    [SerializeField] Transform subjectFallback;
    [SerializeField] Transform physicsFrame, visualFrame; // explicit frame mapping
    [SerializeField] bool allowUnmappedPhysicsFallback = true;

    [Header("Entry/exit")]
    [SerializeField, Range(3f, 25f)] float lookAheadDistance = 8f;
    [SerializeField, Range(2f, 35f)] float minimumDownhillDegrees = 10f;
    [Tooltip("Allow a StairWay*_Render group even when its smoothed mesh reports little slope.")]
    [SerializeField] bool trustStairWayRendererNames = true;
    [SerializeField, Range(1f, 15f)] float maxEntryCrossTrack = 5f;
    [SerializeField, Range(.1f, 5f)] float exitConfirmDistance = .65f;
    [SerializeField, Range(.05f, 1.5f)] float cornerEntryLeadMeters = .50f;
    [SerializeField, Range(.25f, 5f)] float nextFlatArrivalTolerance = 1.6f;
    [SerializeField, Min(1f)] float verticalDetectionTolerance = 7f;

    [Header("A+B 90-degree rails (unchanged geometry approach)")]
    [SerializeField, Range(1f, 20f)] float laneHalfWidth = 2.5f;
    [SerializeField] bool turnToPositiveZ = true;
    [SerializeField, Range(1f, 8f)] float miterLimit = 4f;
    [SerializeField, Range(1f, 20f)] float maximumCornerReach = 16f;
    [SerializeField, Range(1f, 20f)] float cornerToleranceDegrees = 8f;

    [Header("Observed ball trail / Future recursive wave spline")]
    [SerializeField, Range(.02f, .5f)] float observedSpacing = .08f;
    [SerializeField, Range(8, 2048)] int maxObservedKnots = 600;
    [SerializeField, Range(0f, .5f)] float geometricLineLift = .08f;

    [Header("Measured BallVisualEqualizer - one physical observer")]
    [Tooltip("Optional RENDERER-ONLY transform. Leave empty to create a separate visible sphere. Never assign InSubject or visualMotionSource here.")]
    [SerializeField] Transform equalizerDisplay;
    [SerializeField] LayerMask equalizerGroundMask = ~0;
    [SerializeField, Range(.1f, 1f)] float equalizerProbeSpacing = .45f;
    [SerializeField, Range(.2f, 2f)] float equalizerVisualRadius = .5f;
    [SerializeField, Range(.2f, 1.5f)] float equalizerDamping = .65f;
    [SerializeField, Range(0f, 3f)] float equalizerObservationGain = 1.0f;
    [SerializeField, Range(0f, 2f)] float equalizerImpactGain = .65f;
    [SerializeField, Range(0f, 1f)] float equalizerTerrainGain = .45f;
    [SerializeField, Range(0f, 1f)] float equalizerAccelerationGain = .09f;
    [SerializeField, Range(0f, 1f)] float equalizerFrameGain = .06f;
    [SerializeField, Min(.2f)] float equalizerMaximumOffset = 3f;
    [SerializeField] bool equalizerDebug = true;

    // Numeric state is indispensable for time derivatives and an oscillator.
    // A single group ID is the only lifecycle marker: the existing five-spline
    // state machine remains the sole authority for stage entry and exit.
    readonly RaycastHit[] equalizerHits = new RaycastHit[16];
    int equalizerGroup = -1, equalizerPeakRegion = -1;
    float equalizerBase, equalizerSlow, equalizerAccelSlow;
    float equalizerOffset, equalizerSpeed, equalizerDisplayOffset, equalizerDisplaySpeed;
    float equalizerLastQSpeed, equalizerEntrySpeed, equalizerWave;
    float equalizerReferenceHeight, equalizerGroundSignal, equalizerFrameAcceleration;
    float equalizerObservedAcceleration, equalizerPhysicalImpact, equalizerNextLog, equalizerPitch = 3.3f;
    Vector3 equalizerLastOrigin, equalizerLastOriginSpeed, equalizerLastNormal = Vector3.up;
    Vector3 equalizerLastNormalSpeed, equalizerNormal = Vector3.up;
    Vector3 equalizerLastPhysicalPosition;
    GameObject equalizerOwnedDisplay;

    // Same class on InSubject receives Unity collision callbacks; no extra class.
    static IndependentFiveSplineWeaver addingContactFor;
    IndependentFiveSplineWeaver contactOwner, contactProbe;
    float contactSupport;

    [Header("Owned SplineContainer tubes")]
    [SerializeField, Range(.008f, .15f)] float tubeRadius = .035f;
    [SerializeField, Range(3, 10)] int tubeSides = 6;
    [SerializeField, Range(.02f, .5f)] float meshRefreshSeconds = .09f;
    [SerializeField] Material optionalTubeMaterial;
    [SerializeField] bool showFiveInGame = true;
    [SerializeField] bool debugSplineLifecycle = true;

    [Header("Runtime, read-only")]
    [SerializeField] int detectedPieces, detectedPairs, pairIndex = -1, stairGroup = -1;
    [SerializeField] string state = "IDLE", lastReason = "";
    [SerializeField] int[] knotCounts = new int[Count];
    [SerializeField] bool isDownhillStair, hasFlatBefore, hasRightAngleTurn;
    [SerializeField] bool cornerBuilt, exitReached;
    [SerializeField] float entryDistance, stairStation;
    [SerializeField] int railTurnCount, beforePublishReplans;
    [SerializeField] Vector3 entryCornerPoint, exitCornerPoint;
    [SerializeField] bool hasEntryCornerPoint, hasExitCornerPoint;
    [SerializeField] float signedEntryYaw, signedExitYaw;
    [SerializeField] string trackingSource = "UNRESOLVED";

    sealed class Piece {
        public Transform transform;
        public int group, part;
        public bool stair;
        public Vector3 a, b, heading;
        public float slope;
    }
    sealed class Pair {
        public int group, key, first, last, flat, partCount;
        public Transform anchor;
        public bool downhill, turn90, outTurn90;
        public Vector3 entry, exit, heading, flatHeading, side;
        public Vector3 flatStart, flatEnd, corner;
        public Vector3 outgoingFlatStart, outgoingFlatEnd, outgoingHeading, outgoingCorner;
        public int outgoingFlat = -1;
        public bool hasCorner, hasOutgoingCorner;
        public float entryYaw, exitYaw, length;
        public readonly List<Vector3> axis = new List<Vector3>(32);
        public readonly List<float> stations = new List<float>(32);
    }

    readonly List<Piece> pieces = new List<Piece>(256);
    readonly List<Pair> pairs = new List<Pair>(64);
    readonly List<Vector3>[] paths = {
        new List<Vector3>(40), new List<Vector3>(40), new List<Vector3>(40),
        new List<Vector3>(600), new List<Vector3>(48)
    };
    readonly SplineContainer[] containers = new SplineContainer[Count];
    readonly Mesh[] meshes = new Mesh[Count];
    readonly MeshRenderer[] meshRenderers = new MeshRenderer[Count];
    readonly Material[] materials = new Material[Count];
    readonly bool[] meshDirty = new bool[Count];
    Transform owner, followedVisualRoot;
    Matrix4x4 previousVisualRootMatrix;
    Pair active;
    float nextScan, nextMesh, lastObservedStation;
    Vector3 previousObserved;
    int lastCompletedGroup = -1, stageZeroFirstKey, scanHash, routeRevisions;
    float nextMissingReferenceWarning;

    void LifecycleEvent(string tag) {
        if (!debugSplineLifecycle) return;
        Debug.Log($"[IFSW][5SPLINE] {tag} state={state} group={stairGroup} " +
            $"parts={detectedPieces} pairs={detectedPairs} " +
            $"knots=[{knotCounts[0]},{knotCounts[1]},{knotCounts[2]},{knotCounts[3]},{knotCounts[4]}]", this);
    }

    void Awake() {
        if (addingContactFor) { contactOwner = addingContactFor; return; }
        owner = new GameObject("__IFSW_FIVE_SPLINES_ONLY").transform;
        for (int i = 0; i < Count; i++) {
            GameObject go = new GameObject("IFSW_NEW_" + Names[i]);
            go.transform.SetParent(owner, false);
            containers[i] = go.AddComponent<SplineContainer>();
            containers[i].Spline = new Spline();
            meshes[i] = new Mesh { name = "IFSW_Tube_" + Names[i], indexFormat = IndexFormat.UInt32 };
            go.AddComponent<MeshFilter>().sharedMesh = meshes[i];
            meshRenderers[i] = go.AddComponent<MeshRenderer>();
            Shader shader = Shader.Find("Universal Render Pipeline/Unlit");
            if (!shader) shader = Shader.Find("Unlit/Color");
            if (!shader) shader = Shader.Find("Sprites/Default");
            if (optionalTubeMaterial || shader) {
                Material m = optionalTubeMaterial ? new Material(optionalTubeMaterial) : new Material(shader);
                m.name = "IFSW_Mat_" + Names[i];
                if (m.HasProperty("_BaseColor")) m.SetColor("_BaseColor", Colors[i]);
                if (m.HasProperty("_Color")) m.SetColor("_Color", Colors[i]);
                materials[i] = m;
                meshRenderers[i].sharedMaterial = m;
            }
            meshRenderers[i].shadowCastingMode = ShadowCastingMode.Off;
            meshRenderers[i].receiveShadows = false;
            meshDirty[i] = true;
        }
    }

    void Start() {
        if (contactOwner || !inSubject) return;
        if (inSubject.gameObject == gameObject) { contactProbe = this; return; }
        contactProbe = inSubject.GetComponent<IndependentFiveSplineWeaver>();
        if (contactProbe && contactProbe != this && contactProbe.contactOwner != this) {
            Debug.LogWarning("[IFSW] InSubject already has an independent Weaver", this);
            return;
        }
        if (!contactProbe) {
            addingContactFor = this;
            try { contactProbe = inSubject.gameObject.AddComponent<IndependentFiveSplineWeaver>(); }
            finally { addingContactFor = null; }
        }
    }

    // Called on the Rigidbody host after physics. Constant supporting contacts
    // are filtered using only one previous impulse, not contact dictionaries.
    void OnCollisionEnter(Collision collision) { ReceiveImpact(collision, true); }
    void OnCollisionStay(Collision collision) { ReceiveImpact(collision, false); }

    void ReceiveImpact(Collision collision, bool entering) {
        IndependentFiveSplineWeaver w = contactOwner ? contactOwner : this;
        if (!w.inSubject || gameObject != w.inSubject.gameObject ||
            w.state != "ACTIVE" || collision.contactCount == 0) return;
        float impulse = collision.impulse.magnitude / Mathf.Max(.001f, w.inSubject.mass);
        if (!entering && contactSupport == 0f) { contactSupport = impulse; return; }
        float rise = entering ? impulse : Mathf.Max(0f, impulse - contactSupport);
        contactSupport = entering ? impulse : Mathf.Lerp(contactSupport, impulse, .2f);
        if (rise < .08f || entering &&
            Mathf.Abs(Vector3.Dot(collision.relativeVelocity, collision.GetContact(0).normal)) < .12f) return;
        Vector3 n = w.EqualizerMappedDirection(collision.GetContact(0).normal);
        float alignment = Mathf.Max(0f, Vector3.Dot(n.normalized, w.equalizerNormal));
        if (alignment < .25f) return;
        w.equalizerPhysicalImpact = Mathf.Min(3f, w.equalizerPhysicalImpact + rise * alignment);
        Debug.Log($"[IFSW][EQ_HIT] collider={collision.collider.name} layer={LayerMask.LayerToName(collision.collider.gameObject.layer)}");
    }

    void OnEnable() {
        if (contactOwner) return;
        if (owner) owner.gameObject.SetActive(true);
        if (equalizerOwnedDisplay) equalizerOwnedDisplay.SetActive(true);
    }
    void OnDisable() {
        if (contactOwner) return;
        if (owner) owner.gameObject.SetActive(false);
        if (equalizerOwnedDisplay) equalizerOwnedDisplay.SetActive(false);
    }
    void OnDestroy() {
        if (contactOwner) return;
        if (contactProbe && contactProbe != this && contactProbe.contactOwner == this)
            Destroy(contactProbe);
        if (equalizerOwnedDisplay) {
            if (Application.isPlaying) Destroy(equalizerOwnedDisplay);
            else DestroyImmediate(equalizerOwnedDisplay);
        }
        if (owner) { if (Application.isPlaying) Destroy(owner.gameObject); else DestroyImmediate(owner.gameObject); }
        for (int i = 0; i < Count; i++) {
            if (meshes[i]) { if (Application.isPlaying) Destroy(meshes[i]); else DestroyImmediate(meshes[i]); }
            if (materials[i]) { if (Application.isPlaying) Destroy(materials[i]); else DestroyImmediate(materials[i]); }
        }
    }

    void LateUpdate() {
        if (contactOwner || !owner) return;
        // Generated renderers can appear after Start or after a new chunk is emitted.
        UpdateVisualRootPose();
        if (Time.time >= nextScan || pairs.Count == 0) {
            nextScan = Time.time + rendererScanSeconds;
            ScanStage();
        }
        // Sample the real rendered Equalizer before recording its observed trail.
        UpdateEqualizerDisplay();
        // A missing player reference must not prevent container creation.
        // Once it exists, entry is evaluated in THIS frame (no FixedUpdate dependency).
        if (pairs.Count > 0 && TryGetTrackingPoint(out Vector3 point)) {
            ProcessPair(point);
        } else if (debugSplineLifecycle && Time.time >= nextMissingReferenceWarning) {
            nextMissingReferenceWarning = Time.time + 3f;
            Debug.LogWarning("[IFSW][5SPLINE] Waiting for " +
                (pairs.Count == 0 ? "StairWay*_Render geometry" :
                "a visualMotionSource, displayedBall, or mapped InSubject") +
                "; root=" + (generatedVisualRoot ? generatedVisualRoot.name : "NULL") +
                "; input=" + trackingSource, this);
        }
        FlushOwnedSplineMeshes();
    }

    // The five splines are updated in LateUpdate. InSubject is observed in
    // FixedUpdate, where velocities and finite differences have physical meaning.
    void FixedUpdate() {
        if (contactOwner) return;
        if (inSubject && active != null && state == "ACTIVE")
            UpdateMeasuredEqualizer(Time.fixedDeltaTime);
        else equalizerPhysicalImpact = 0f;
    }

    Vector3 EqualizerMappedPoint(Vector3 point) {
        return physicsFrame && visualFrame
            ? visualFrame.TransformPoint(physicsFrame.InverseTransformPoint(point))
            : point;
    }

    Vector3 EqualizerMappedDirection(Vector3 direction) {
        return physicsFrame && visualFrame
            ? visualFrame.TransformDirection(physicsFrame.InverseTransformDirection(direction))
            : direction;
    }

    // Prefer the actual StairWay renderer's collider to an overlapping smooth
    // physics ramp; fall back to the nearest ground when no stair is hit.
    bool EqualizerSurfaceHeight(Vector3 stagePoint, out float height, out bool stairHit) {
        Vector3 origin = stagePoint + Vector3.up * 2f;
        int count = Physics.RaycastNonAlloc(origin, Vector3.down, equalizerHits,
            10f, equalizerGroundMask, QueryTriggerInteraction.Ignore);
        float closest = float.PositiveInfinity;
        bool stairSelected = false;
        height = 0f;
        for (int i = 0; i < count; ++i) {
            RaycastHit hit = equalizerHits[i];
            if (!hit.collider || hit.collider.attachedRigidbody == inSubject) continue;
            bool stair = false;
            for (Transform tr = hit.collider.transform; tr; tr = tr.parent)
                if (Parse(tr.name, out bool isStair, out int group, out _) && isStair) {
                    stair = active != null && group == active.group;
                    break;
                }
            if (stairSelected && !stair || (stairSelected == stair && hit.distance >= closest))
                continue;
            closest = hit.distance;
            height = hit.point.y;
            stairSelected = stair;
        }
        stairHit = stairSelected;
        return closest < float.PositiveInfinity;
    }

    // Reference BVE's two Hermite arcs; no new phase flags or amplitude tables.
    static float WaveHermite(float a, float b, float da, float db, float u) {
        float A = 2f * (a - b) + da + db;
        float B = 3f * (b - a) - 2f * da - db;
        return ((A * u + B) * u + da) * u + a;
    }
    static float EqualizerWave(float u, float start, float peak, float end,
        float apex, float rise, float impact, float period) {
        float up = Mathf.Min(rise * period * apex, 3f * Mathf.Max(0f, peak - start));
        float down = Mathf.Min(impact * period * (1f - apex), 3f * Mathf.Max(0f, peak - end));
        return u < apex ? WaveHermite(start, peak, up, 0f, u / apex) :
            WaveHermite(peak, end, 0f, -down, (u - apex) / (1f - apex));
    }

    // Long-version topology R(n)=23+R(n-1); n is the repeated-pair depth.
    static void Resolve23Recursive(float x, int count, int depth,
        out int region, out float u, out int n) {
        if (count == 1 || x <= 1f / count) {
            float phase = Mathf.Clamp01(x * count) * 2f;
            int k = phase >= 2f ? 1 : Mathf.FloorToInt(phase);
            region = 3 + 2 * depth + k;
            u = phase >= 2f ? 1f : phase - k;
            n = depth;
            return;
        }
        Resolve23Recursive((count * x - 1f) / (count - 1f),
            count - 1, depth + 1, out region, out u, out n);
    }

    // Real recursive nth term: A(n+1) = r*A(n). No stored per-wave table.
    static float RecursiveTerm(int n, float a, float r) =>
        n <= 0 ? a : r * RecursiveTerm(n - 1, a, r);

    // One waveform authority for the display and each future Spline knot.
    float SampleRecursiveWave(Pair p, float progress, float speedAlong, float gN,
        out int region, out float u, out int depth, out float amp) {
        int repeat = Mathf.Max(0, p.partCount - 1);
        float boundary = repeat == 0 ? 1f : 3f / 7f;
        bool prefix = progress < boundary || repeat == 0;
        depth = 0;
        if (prefix) {
            float x = Mathf.Clamp01(progress / boundary) * 3f;
            region = Mathf.Min(2, Mathf.FloorToInt(x));
            u = x >= 3f ? 1f : x - region;
        } else Resolve23Recursive(Mathf.Clamp01((progress - boundary) / (1f - boundary)),
            repeat, 0, out region, out u, out depth);
        int type = region < 3 ? region : 1 + (region - 3) % 2;
        int prior = region == 0 ? 0 : region < 3 ? region - 1 : type == 1 ? 2 : 1;
        float L = p.length * (prefix ? boundary / 3f : (1f - boundary) / (2f * repeat));
        float retain = Mathf.Min(.96f, .86f * .86f * Mathf.Exp(-.02f *
            (p.length / Mathf.Max(1, p.partCount)) / 3f));
        float A1 = equalizerGroup == p.group ? equalizerReferenceHeight : .45f;
        float A2 = A1 * retain, A3 = A2 * retain;
        float r = Mathf.Exp(-.02f * p.length * (1f - boundary) / Mathf.Max(1, repeat));
        amp = (type == 0 ? A1 : type == 1 ? A2 : A3) *
            (region < 3 ? 1f : RecursiveTerm(depth, 1f, r));
        float prev = (prior == 0 ? A1 : prior == 1 ? A2 : A3) *
            (region < 3 ? 1f : RecursiveTerm(type == 1 ? Mathf.Max(0, depth - 1) : depth, 1f, r));
        float fade = repeat == 0 ? Mathf.Exp(-.02f * p.length * progress) : 1f;
        amp *= fade; prev *= fade;
        float apex = type == 0 ? .82f : .50f;
        float rise = region == 0 ? equalizerEntrySpeed : .86f * Mathf.Sqrt(2f * gN * prev * 1.75f);
        float wave = EqualizerWave(u, region == 0 ? 0f : -.75f * prev,
            amp, -.75f * amp, apex, rise,
            Mathf.Sqrt(2f * gN * amp * 1.75f), L / Mathf.Max(.1f, speedAlong));
        float window = Mathf.Sin(Mathf.PI * u) * Mathf.Sin(Mathf.PI * (u - apex));
        wave += amp * (type == 0 ? .08f : type == 1 ? .22f : .12f) *
            window * window * Mathf.Sin(2f * Mathf.PI * u);
        if (region == 2 * p.partCount)
            wave *= 1f - Mathf.SmoothStep(0f, 1f, Mathf.Clamp01((u - .8f) * 5f));
        return wave;
    }

    void UpdateMeasuredEqualizer(float dt) {
        Pair p = active;
        Vector3 physicalPosition = inSubject.position;
        Vector3 physicalVelocity = inSubject.velocity;
        Vector3 position = EqualizerMappedPoint(physicalPosition);
        Vector3 velocity = EqualizerMappedDirection(physicalVelocity);
        float station = StationOn(p, position, out _, out _);
        Vector3 origin = PointOn(p, station, out Vector3 splineTangent);
        Vector3 forward = Flat(splineTangent);
        if (forward.sqrMagnitude < E) forward = p.heading;

        // Stage renderers live in the visual frame. Physics-frame rays can sample
        // unrelated smooth collision ramps even when StairWay has step colliders.
        float spacing = Mathf.Max(equalizerProbeSpacing,
            Mathf.Clamp(p.length / Mathf.Max(1, p.partCount * 6) * .6f, .45f, 1.25f));
        bool measured =
            EqualizerSurfaceHeight(position - forward * (1.5f * spacing), out float h0, out bool s0) &
            EqualizerSurfaceHeight(position - forward * (.5f * spacing), out float h1, out bool s1) &
            EqualizerSurfaceHeight(position + forward * (.5f * spacing), out float h2, out bool s2) &
            EqualizerSurfaceHeight(position + forward * (1.5f * spacing), out float h3, out bool s3);
        int stairProbes = (s0 ? 1 : 0) + (s1 ? 1 : 0) + (s2 ? 1 : 0) + (s3 ? 1 : 0);
        float axisGrade = splineTangent.y / Mathf.Max(.01f, Flat(splineTangent).magnitude);
        float grade = measured ? Mathf.Lerp(axisGrade,
            Mathf.Clamp((h3-h0)/(3f*spacing), axisGrade-.75f, axisGrade+.75f), .5f)
            : axisGrade;
        Vector3 t = (forward + Vector3.up * grade).normalized;
        if (t.sqrMagnitude < E) t = splineTangent;
        Vector3 desiredNormal = Vector3.ProjectOnPlane(Vector3.up, t).normalized;
        if (desiredNormal.sqrMagnitude < E) desiredNormal = Vector3.up;
        Vector3 normal = equalizerGroup == p.group
            ? Vector3.Slerp(equalizerLastNormal, desiredNormal,
                1f - Mathf.Exp(-2f * Mathf.PI * 10f * dt)).normalized
            : desiredNormal;
        float ground = s0 && s1 && s2 ? (h2 - 2f*h1 + h0) * .5f :
            s1 && s2 && s3 ? (h3 - 2f*h2 + h1) * .5f : 0f;
        float q = Vector3.Dot(position - origin, normal);

        float groundPitch = p.length / Mathf.Max(1, 2 * p.partCount + 1);
        equalizerPitch = Mathf.Clamp(groundPitch, .7f, 8f);
        if (equalizerGroup != p.group) {
            equalizerGroup = p.group;
            equalizerPeakRegion = -1;
            equalizerBase = equalizerWave = 0f; // base = traveled distance, not height bias
            equalizerLastPhysicalPosition = physicalPosition;
            equalizerSlow = q; // same Knot-normal height origin for both signals
            equalizerAccelSlow = equalizerOffset = equalizerSpeed = 0f;
            equalizerDisplayOffset = equalizerDisplaySpeed = 0f;
            equalizerLastQSpeed = Vector3.Dot(velocity, normal);
            equalizerLastOrigin = origin;
            equalizerLastOriginSpeed = Vector3.zero;
            equalizerLastNormal = normal;
            equalizerLastNormalSpeed = Vector3.zero;
            equalizerNextLog = 0f;
            equalizerEntrySpeed = Mathf.Max(.1f, Vector3.Dot(velocity, normal));
            float g = Mathf.Max(.1f, -Vector3.Dot(Physics.gravity, normal));
            equalizerReferenceHeight = Mathf.Clamp(
                Mathf.Lerp(.45f, equalizerEntrySpeed * equalizerEntrySpeed / (2f * g), .35f),
                .02f, equalizerMaximumOffset);
            Debug.Log($"[IFSW][EQ_BEGIN] group={p.group} pitch={equalizerPitch:F3} " +
                $"referenceEnergyHeight={equalizerReferenceHeight:F4} " +
                $"normal={normal} source=INSUBJECT_PHYSICS_ONLY", this);
            RebuildGeometricGuide(p); // one measured-energy forecast at stair entry
        }

        Vector3 originSpeed = (origin - equalizerLastOrigin) / dt;
        Vector3 originAcceleration = (originSpeed - equalizerLastOriginSpeed) / dt;
        Vector3 normalSpeed = (normal - equalizerLastNormal) / dt;
        Vector3 normalAcceleration = (normalSpeed - equalizerLastNormalSpeed) / dt;
        float physicalSpeed = Vector3.Dot(velocity, normal);
        // q_dot = (Vball - Vorigin).N + (P - O).N_dot
        float coordinateSpeed = Vector3.Dot(velocity - originSpeed, normal) +
            Vector3.Dot(position - origin, normalSpeed);
        equalizerFrameAcceleration = -Vector3.Dot(originAcceleration, normal) +
            2f * Vector3.Dot(normalSpeed, velocity - originSpeed) +
            Vector3.Dot(normalAcceleration, position - origin);
        equalizerObservedAcceleration = (coordinateSpeed - equalizerLastQSpeed) / dt;

        // ONE authoritative wave center: measured entry energy + Knot travel.
        // Same 3/7 prefix and recursively repeated 2->3 suffix as reference BVE.
        float ds = Mathf.Max(0f, Vector3.Dot(
            EqualizerMappedDirection(physicalPosition - equalizerLastPhysicalPosition), splineTangent));
        float speedAlong = ds / Mathf.Max(.001f, dt);
        equalizerBase = Mathf.Min(p.length, equalizerBase + ds);
        int repeat = Mathf.Max(0, p.partCount - 1);
        float boundary = repeat == 0 ? 1f : 3f / 7f;
        float progress = Mathf.Clamp01(equalizerBase / Mathf.Max(.1f, p.length));
        equalizerWave = SampleRecursiveWave(p, progress, speedAlong,
            Mathf.Max(.1f, -Vector3.Dot(Physics.gravity, normal)),
            out int region, out float u, out int depth, out float amp);
        int type = region < 3 ? region : 1 + (region - 3) % 2;
        float L = p.length * (progress < boundary || repeat == 0 ? boundary / 3f :
            (1f - boundary) / (2f * repeat));
        float apex = type == 0 ? .82f : .50f;

        // Residual ONLY: avoid spring-following the Hermite wave a second time.
        float alpha = 1f - Mathf.Exp(-2f * Mathf.PI * 1.4f * dt);
        equalizerSlow += alpha * (q - equalizerSlow);
        float aPhysical = equalizerObservedAcceleration - equalizerFrameAcceleration;
        equalizerAccelSlow += alpha * (aPhysical - equalizerAccelSlow);
        equalizerGroundSignal = ground * normal.y;
        float target = Mathf.Clamp(equalizerObservationGain * (q - equalizerSlow) +
            equalizerTerrainGain * equalizerGroundSignal, -.20f, .20f);

        float kick = equalizerPhysicalImpact;
        equalizerPhysicalImpact = 0f;
        if (equalizerDebug && kick > .08f)
            Debug.Log($"[IFSW][EQ_CONTACT] group={p.group} impulseSpeed={kick:F4}", this);
        equalizerSpeed += equalizerImpactGain * kick;
        float drive = Mathf.Clamp(equalizerAccelerationGain * (aPhysical - equalizerAccelSlow) +
            equalizerFrameGain * Mathf.Clamp(equalizerFrameAcceleration, -20f, 20f), -250f, 250f);
        float frequency = Mathf.Clamp(speedAlong / Mathf.Max(.2f, L), 2f, 9f);
        float omega = 2f * Mathf.PI * frequency;
        int steps = Mathf.Clamp(Mathf.CeilToInt(dt * omega / .28f), 1, 12);
        float h = dt / steps;
        for (int i = 0; i < steps; ++i) {
            float acceleration = omega * omega * (target - equalizerOffset) -
                2f * equalizerDamping * omega * equalizerSpeed + drive;
            equalizerSpeed = Mathf.Clamp(equalizerSpeed + acceleration * h, -65f, 65f);
            equalizerOffset = Mathf.Clamp(equalizerOffset + equalizerSpeed * h, -.20f, .20f);
            if (Mathf.Abs(equalizerOffset) >= .20f && equalizerSpeed * equalizerOffset > 0f)
                equalizerSpeed = 0f;
        }

        float totalOffset = Mathf.Clamp(equalizerWave + equalizerOffset,
            -equalizerMaximumOffset, equalizerMaximumOffset);
        int waveType = type + 1;
        if (equalizerPeakRegion != region && u >= apex) {
            equalizerPeakRegion = region;
            Debug.Log($"[IFSW][EQ_APEX] group={p.group} region={region} waveType={waveType} " +
                $"travel={equalizerBase:F3} progress={progress:F4} " +
                $"baseWave={equalizerWave:F4} offset={totalOffset:F4} kind=MODEL_APEX vPhysical={physicalSpeed:F3}", this);
        }
        if (equalizerDebug && Time.fixedTime >= equalizerNextLog) {
            equalizerNextLog = Time.fixedTime + .20f;
            Debug.Log($"[IFSW][EQ_STATE] fixed={Time.fixedTime:F3} group={p.group} " +
                $"region={region} waveType={waveType} wave01={u:F4} " +
                $"station={station:F4} progress={station/Mathf.Max(.1f,p.length):F5} " +
                $"q={q:F4} vCoord={coordinateSpeed:F4} vPhysical={physicalSpeed:F4} " +
                $"vFrame={(coordinateSpeed-physicalSpeed):F4} terrain={equalizerGroundSignal:F4} " +
                $"aObserved={equalizerObservedAcceleration:F4} aFrame={equalizerFrameAcceleration:F4} " +
                $"contactKick={kick:F4} target={target:F4} baseWave={equalizerWave:F4} " +
                $"depth={depth} amp={amp:F4} residual={equalizerOffset:F4} " +
                $"offset={totalOffset:F4} travel={equalizerBase:F3} " +
                $"vOsc={equalizerSpeed:F4} refHeight={equalizerReferenceHeight:F4} " +
                $"frequency={frequency:F3} probes={measured} stairProbes={stairProbes}/4 spacing={spacing:F2} mode=MEASURED", this);
        }

        equalizerLastQSpeed = coordinateSpeed;
        equalizerLastOrigin = origin;
        equalizerLastOriginSpeed = originSpeed;
        equalizerLastNormal = normal;
        equalizerLastNormalSpeed = normalSpeed;
        equalizerLastPhysicalPosition = physicalPosition;
        equalizerNormal = normal;
    }

    void UpdateEqualizerDisplay() {
        if (!inSubject) return;
        if (!equalizerDisplay) {
            // This component owns ONLY this sphere; existing subject is never
            // moved or disabled. Remove the primitive's collider immediately.
            equalizerOwnedDisplay = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            equalizerOwnedDisplay.name = "__IFSW_EqualizerVisualOnly";
            Collider c = equalizerOwnedDisplay.GetComponent<Collider>();
            c.enabled = false;
            Destroy(c);
            equalizerDisplay = equalizerOwnedDisplay.transform;
            equalizerDisplay.localScale = Vector3.one * (equalizerVisualRadius * 2f);
        }
        if (equalizerDisplay.IsChildOf(inSubject.transform) ||
            (visualMotionSource && equalizerDisplay.IsChildOf(visualMotionSource)) ||
            (displayedBall && equalizerDisplay.IsChildOf(displayedBall))) {
            // Prevent accidental writes to a tracked input or a physics body.
            Debug.LogError("[IFSW][EQ_OUTPUT_BLOCKED] Output must be a separate renderer-only Transform.", this);
            equalizerDisplay = null;
            return;
        }
        float goal = equalizerGroup >= 0 && state == "ACTIVE" ?
            Mathf.Clamp(equalizerWave + equalizerOffset, -equalizerMaximumOffset, equalizerMaximumOffset) : 0f;
        equalizerDisplayOffset = Mathf.SmoothDamp(equalizerDisplayOffset, goal,
            ref equalizerDisplaySpeed, .018f, 100f, Time.deltaTime);
        equalizerDisplay.position = EqualizerMappedPoint(inSubject.position) +
            equalizerNormal * equalizerDisplayOffset;
        if (visualMotionSource) equalizerDisplay.rotation = visualMotionSource.rotation;
    }

    void FindVisualMotionSource() {
        if (visualMotionSource) return;
        GameObject rootObject = GameObject.Find("VisualPlayerRoot");
        if (!rootObject) return;
        Transform root = rootObject.transform;
        Transform child = root.Find("subject");
        if (!child) child = root.Find("Subject");
        if (!child) {
            foreach (Transform t in root.GetComponentsInChildren<Transform>(true)) {
                if (t.name.Equals("subject", StringComparison.OrdinalIgnoreCase)) {
                    child = t;
                    break;
                }
            }
        }
        if (child) visualMotionSource = child;
    }

    bool TryGetTrackingPoint(out Vector3 point) {
        if (!visualMotionSource) FindVisualMotionSource();
        if (visualMotionSource) {
            trackingSource = "VisualPlayerRoot/subject";
            point = visualMotionSource.position;
            return true;
        }
        if (displayedBall) {
            trackingSource = "Assigned displayedBall (read only)";
            point = displayedBall.position;
            return true;
        }
        if (inSubject && physicsFrame && visualFrame) {
            trackingSource = "Mapped InSubject";
            point = visualFrame.TransformPoint(physicsFrame.InverseTransformPoint(inSubject.position));
            return true;
        }
        if (subjectFallback) {
            trackingSource = "Assigned subjectFallback";
            point = subjectFallback.position;
            return true;
        }
        if (inSubject && allowUnmappedPhysicsFallback) {
            trackingSource = "Unmapped InSubject (check world alignment)";
            point = inSubject.position;
            return true;
        }
        trackingSource = "NO_TRACKING_SOURCE";
        point = Vector3.zero;
        return false;
    }

    void FlushOwnedSplineMeshes() {
        if (Time.time < nextMesh) return;
        nextMesh = Time.time + meshRefreshSeconds;
        for (int i=0; i<Count; i++) {
            if (meshRenderers[i]) meshRenderers[i].enabled = showFiveInGame && knotCounts[i] >= 2;
            if (meshDirty[i]) {
                BuildTube(i); // Mesh.Clear() also runs for 0 or 1 knot
                meshDirty[i] = false;
            }
        }
    }

    static bool Parse(string name, out bool stair, out int group, out int part) {
        stair=false; group=part=-1;
        int prefix;
        if(name.StartsWith("StairWay",StringComparison.OrdinalIgnoreCase)) { stair=true; prefix=8; }
        else if(name.StartsWith("ArcSlab",StringComparison.OrdinalIgnoreCase)) prefix=7;
        else return false;
        int mid=name.IndexOf('_',prefix);
        int end=mid>=0?name.IndexOf('_',mid+1):-1;
        if(mid<=prefix || end<0 ||
           !int.TryParse(name.Substring(prefix,mid-prefix),out group) ||
           !int.TryParse(name.Substring(mid+1,end-mid-1),out part)) return false;
        string suffix=name.Substring(end+1);
        return suffix.Equals("Render",StringComparison.OrdinalIgnoreCase) ||
               suffix.Equals("Renderer",StringComparison.OrdinalIgnoreCase);
    }

    static Vector3 Flat(Vector3 v) {
        v.y=0f;
        return v.sqrMagnitude>E?v.normalized:Vector3.zero;
    }
    static Vector3 Side(Vector3 heading) => Flat(new Vector3(-heading.z,0f,heading.x));
    Vector3 Snap(Vector3 v) {
        Vector3 heading=Flat(v);
        if(heading.sqrMagnitude<E) return Vector3.zero;
        Vector3 local=generatedVisualRoot ? generatedVisualRoot.InverseTransformDirection(heading) : heading;
        Vector3 axis=Mathf.Abs(local.x)>Mathf.Abs(local.z)
            ? new Vector3(Mathf.Sign(local.x),0f,0f) : new Vector3(0f,0f,Mathf.Sign(local.z));
        return Flat(generatedVisualRoot?generatedVisualRoot.TransformDirection(axis):axis);
    }

    // The mesh is used solely to estimate its own visible longitudinal bounds.
    // Stage path knots / physics spline containers are NEVER opened.
    void Locate(Piece p) {
        Transform tr=p.transform;
        float minZ=float.PositiveInfinity, maxZ=float.NegativeInfinity;
        float minX=float.PositiveInfinity, maxX=float.NegativeInfinity;
        MeshFilter[] filters=tr.GetComponentsInChildren<MeshFilter>(true);
        for(int j=0;j<filters.Length;j++) {
            MeshFilter f=filters[j];
            if(!f.sharedMesh) continue;
            Bounds b=f.sharedMesh.bounds;
            for(int k=0;k<8;k++) {
                Vector3 vertex=new Vector3((k&1)==0?b.min.x:b.max.x,
                    (k&2)==0?b.min.y:b.max.y,(k&4)==0?b.min.z:b.max.z);
                Vector3 local=tr.InverseTransformPoint(f.transform.TransformPoint(vertex));
                minX=Mathf.Min(minX,local.x); maxX=Mathf.Max(maxX,local.x);
                minZ=Mathf.Min(minZ,local.z); maxZ=Mathf.Max(maxZ,local.z);
            }
        }
        float x=minX<=maxX?.5f*(minX+maxX):0f;
        if(maxZ-minZ<.15f) { minZ=-fallbackPieceLength*.5f; maxZ=-minZ; }
        p.a=tr.TransformPoint(new Vector3(x,renderLocalHeight,minZ));
        p.b=tr.TransformPoint(new Vector3(x,renderLocalHeight,maxZ));
        Vector3 d=p.b-p.a;
        if(Flat(d).sqrMagnitude<E) {
            // The previous V6 returned WAVE_AXIS_INVALID here. Use the renderer's
            // actual forward orientation rather than retrying creation forever.
            Vector3 f=tr.forward;
            if(Flat(f).sqrMagnitude>E) {
                p.a=tr.position-f*fallbackPieceLength*.5f;
                p.b=tr.position+f*fallbackPieceLength*.5f;
                d=p.b-p.a;
            }
        }
        if(p.stair && p.a.y<p.b.y) {
            Vector3 tmp=p.a; p.a=p.b; p.b=tmp;
            d=p.b-p.a;
        }
        p.heading=Snap(d);
        float horizontal=Vector3.ProjectOnPlane(d,Vector3.up).magnitude;
        p.slope=horizontal>.05f ? Mathf.Atan2(-d.y,horizontal)*Mathf.Rad2Deg : 0f;
    }

    // If the renderer root rotates, already-published local Spline knots move
    // automatically with their parent. Reproject WORLD bookkeeping positions
    // by the identical delta so later WriteSpline calls cannot undo the turn.
    void UpdateVisualRootPose() {
        if (!followedVisualRoot || !generatedVisualRoot) return;
        Matrix4x4 now = generatedVisualRoot.localToWorldMatrix;
        if (now == previousVisualRootMatrix) return;
        Matrix4x4 delta = now * previousVisualRootMatrix.inverse;
        for (int k = 0; k < Count; k++)
            for (int i = 0; i < paths[k].Count; i++)
                paths[k][i] = delta.MultiplyPoint3x4(paths[k][i]);
        previousVisualRootMatrix = now;
        previousObserved = delta.MultiplyPoint3x4(previousObserved);
        entryCornerPoint = delta.MultiplyPoint3x4(entryCornerPoint);
        exitCornerPoint = delta.MultiplyPoint3x4(exitCornerPoint);
        // Do not parent under a generated stage root: that root may be rebuilt.
        // Match its transform while retaining an independently owned hierarchy.
        owner.SetPositionAndRotation(generatedVisualRoot.position, generatedVisualRoot.rotation);
        owner.localScale = generatedVisualRoot.lossyScale;
        nextScan = 0f; // refresh Pair/Renderer geometry in this same frame
    }

    // The Inspector may point to VisualPlayerRoot while the generated Render
    // meshes actually belong to a separate __GeneratedVisualPlayer hierarchy.
    // Find an owning root from REAL renderer objects; never use old spline roots.






    void ScanStage() {
        if(!generatedVisualRoot) {
            GameObject rootObject=GameObject.Find("__GeneratedVisualPlayer");
            if(rootObject) generatedVisualRoot=rootObject.transform;
        }
        if(!generatedVisualRoot) return;
        // Follow the stage root orientation without becoming its child.
        // Generated stage roots can be destroyed/recreated during restarts.
        if (owner && followedVisualRoot != generatedVisualRoot) {
            if (followedVisualRoot) Finish("STAGE_ROOT_CHANGED");
            followedVisualRoot = generatedVisualRoot;
            owner.SetPositionAndRotation(generatedVisualRoot.position, generatedVisualRoot.rotation);
            owner.localScale = generatedVisualRoot.lossyScale;
            previousVisualRootMatrix = generatedVisualRoot.localToWorldMatrix;
        }
        Transform[] nodes=generatedVisualRoot.GetComponentsInChildren<Transform>(true);
        List<Piece> newlyFound=new List<Piece>(nodes.Length);
        unchecked {
            int hash=17;
            for(int i=0;i<nodes.Length;i++) {
                Transform t=nodes[i];
                if(!t.gameObject.activeInHierarchy || !Parse(t.name,out bool stair,out int group,out int part)) continue;
                Piece p=new Piece {transform=t,stair=stair,group=group,part=part};
                newlyFound.Add(p);
                hash=hash*31+t.GetInstanceID();
                hash=hash*31+t.position.GetHashCode();
                hash=hash*31+t.rotation.GetHashCode();
                hash=hash*31+t.lossyScale.GetHashCode();
            }
            hash=hash*31+newlyFound.Count;
            if(hash==scanHash && pairs.Count>0) return;
            scanHash=hash;
        }
        // Mesh bounds are expensive. Only inspect them after a structural/pose
        // change, never on each unchanged 0.25-second scan.
        for(int i=0;i<newlyFound.Count;i++) Locate(newlyFound[i]);
        pieces.Clear(); pieces.AddRange(newlyFound);
        detectedPieces=pieces.Count;
        Pair oldActive=active;
        int oldKey=active!=null?active.key:0;
        Dictionary<int,Pair> byGroup=new Dictionary<int,Pair>();
        pairs.Clear();
        int latestFlat=-1;
        for(int i=0;i<pieces.Count;i++) {
            Piece p=pieces[i];
            if(!p.stair) { latestFlat=i; continue; }
            if(!byGroup.TryGetValue(p.group,out Pair pair)) {
                pair=new Pair { group=p.group, key=p.transform.GetInstanceID(),
                    first=i,last=i,flat=latestFlat,anchor=p.transform };
                byGroup.Add(p.group,pair);
                pairs.Add(pair);
                // No intervening ArcSlab => the NEXT stair has no preceding flat.
                // Do not reuse a stale ArcSlab through 0/-N rhythm transitions.
                latestFlat=-1;
            }
            pair.last=i;
            pair.partCount++;
        }
        bool sceneRestart=false;
        if(pairs.Count>0 && pairs[0].group==0) {
            int newFirstKey=pairs[0].key;
            if(stageZeroFirstKey!=0 && stageZeroFirstKey!=newFirstKey) {
                sceneRestart=true;
            }
            stageZeroFirstKey=newFirstKey;
        }
        for(int i=0;i<pairs.Count;i++) BuildPair(pairs[i],i);
        detectedPairs=pairs.Count;
        if(oldKey!=0) {
            Pair reloaded=null;
            for(int i=0;i<pairs.Count;i++) if(pairs[i].key==oldKey) { reloaded=pairs[i]; pairIndex=i; break; }
            if(reloaded!=null) {
                active=reloaded;
                isDownhillStair=active.downhill;
                hasFlatBefore=active.flat>=0;
                hasRightAngleTurn=active.turn90;
                if(active.downhill && state!="IDLE") {
                    // Appending an UNRELATED stage group must not repeatedly
                    // replace the future spline's initial conditions or
                    // redo a one-time alignment.
                    bool changed=oldActive==null ||
                        oldActive.partCount!=active.partCount ||
                        (oldActive.entry-active.entry).sqrMagnitude>.0001f ||
                        (oldActive.exit-active.exit).sqrMagnitude>.0001f ||
                        oldActive.hasCorner!=active.hasCorner ||
                        (oldActive.corner-active.corner).sqrMagnitude>.0001f ||
                        oldActive.hasOutgoingCorner!=active.hasOutgoingCorner ||
                        oldActive.outTurn90!=active.outTurn90 ||
                        (oldActive.outgoingFlatStart-active.outgoingFlatStart).sqrMagnitude>.0001f ||
                        (oldActive.outgoingFlatEnd-active.outgoingFlatEnd).sqrMagnitude>.0001f;
                    // V13 froze a PREPARED rail after shifts of 25-87m.
                    // Only the PUBLISHED rail is immutable. While PREPARED,
                    // replace the unexposed plan with the latest renderer pose.
                    if(changed) {
                        if(state=="PREPARED" && !cornerBuilt) {
                            BuildRails(active); // buffered until entry; no SplineContainer writes
                            beforePublishReplans++;
                        }
                        if (state=="ACTIVE") RebuildGeometricGuide(active);
                    }
                }
            } else {
                Finish("STAGE_REMOVED");
                if(pairs.Count==0) lastCompletedGroup=-1;
            }
        }
        if(sceneRestart) lastCompletedGroup=-1;
        routeRevisions++;
    }

    static void AppendAxis(List<Vector3> list, Vector3 p) {
        if(list.Count==0 || (list[list.Count-1]-p).sqrMagnitude>1e-7f) list.Add(p);
    }
    static float PlanarDistance(Vector3 a,Vector3 b) {
        return Vector3.ProjectOnPlane(a-b,Vector3.up).magnitude;
    }
    // This is only a SIGNED descriptor; the actual corner is always computed
    // from oriented renderer straight-line intersections, not a global flag.
    static bool SignedRightAngle(Vector3 from,Vector3 to,float tolerance,out float signedYaw) {
        signedYaw=Vector3.SignedAngle(from,to,Vector3.up);
        return from.sqrMagnitude>E && to.sqrMagnitude>E &&
               Mathf.Abs(Mathf.Abs(signedYaw)-90f)<=tolerance;
    }
    static bool IsForwardSegment(Vector3 from,Vector3 to,Vector3 direction) {
        Vector3 delta=Vector3.ProjectOnPlane(to-from,Vector3.up);
        return delta.magnitude>.05f &&
               Vector3.Dot(delta.normalized,direction)>.99f;
    }
    static Vector3 Level(Vector3 point,float height) => new Vector3(point.x,height,point.z);
    static Vector3 OnStraight(Vector3 point,Vector3 origin,Vector3 heading) {
        Vector3 d=point-origin;
        float along=Vector3.Dot(new Vector3(d.x,0f,d.z),heading);
        return new Vector3(origin.x+heading.x*along,point.y,origin.z+heading.z*along);
    }
    static bool Intersect90(Vector3 a,Vector3 da,Vector3 b,Vector3 db,float y,out Vector3 hit) {
        float cross=da.x*db.z-da.z*db.x;
        hit=Vector3.zero;
        if(Mathf.Abs(cross)<E) return false;
        Vector3 difference=b-a;
        float u=(difference.x*db.z-difference.z*db.x)/cross;
        hit=new Vector3(a.x+da.x*u,y,a.z+da.z*u);
        return true;
    }

    void BuildPair(Pair pair,int index) {
        pair.axis.Clear(); pair.stations.Clear();
        Vector3 heading=Vector3.zero, runOrigin=Vector3.zero;
        float maxSlope=0f;
        for(int i=pair.first;i<=pair.last;i++) {
            Piece p=pieces[i];
            if(!p.stair || p.group!=pair.group) continue;
            maxSlope=Mathf.Max(maxSlope,p.slope);
            if(p.heading.sqrMagnitude<E) continue;
            if(pair.axis.Count==0) {
                heading=p.heading; runOrigin=p.a;
                pair.heading=heading;
                AppendAxis(pair.axis,p.a);
                AppendAxis(pair.axis,OnStraight(p.b,runOrigin,heading));
                continue;
            }
            float angle=Vector3.Angle(heading,p.heading);
            if(angle<cornerToleranceDegrees) {
                // Do not add a drifted mid-stair kink. Align all same-direction
                // renderer pieces to one geometric support line.
                Vector3 corrected=OnStraight(p.b,runOrigin,heading);
                Vector3 last=pair.axis[pair.axis.Count-1];
                if(Vector3.Dot(corrected-last,heading)>.03f) AppendAxis(pair.axis,corrected);
            } else if(Mathf.Abs(angle-90f)<=cornerToleranceDegrees) {
                Vector3 last=pair.axis[pair.axis.Count-1];
                if(Intersect90(last,heading,p.a,p.heading,.5f*(last.y+p.a.y),out Vector3 corner))
                    AppendAxis(pair.axis,corner);
                AppendAxis(pair.axis,OnStraight(p.b,p.a,p.heading));
                runOrigin=p.a; heading=p.heading;
            } else {
                // Do not force an invalid U-turn/non-orthogonal path into A+B.
                // Preserve a straight segment until the next valid stair group.
                Vector3 corrected=OnStraight(p.b,runOrigin,heading);
                if(Vector3.Dot(corrected-pair.axis[pair.axis.Count-1],heading)>.03f)
                    AppendAxis(pair.axis,corrected);
            }
        }
        // Some generated StairWay meshes are split into almost-flat steps.
        // Detect the total descent as well as each renderer's local pitch.
        float totalRun = 0f;
        for (int j=1; j<pair.axis.Count; j++)
            totalRun += PlanarDistance(pair.axis[j-1], pair.axis[j]);
        float totalDescent = pair.axis.Count>=2 ?
            pair.axis[0].y - pair.axis[pair.axis.Count-1].y : 0f;
        float aggregateSlope = Mathf.Atan2(totalDescent, Mathf.Max(.01f,totalRun)) * Mathf.Rad2Deg;
        pair.downhill = pair.axis.Count>=2 &&
            (maxSlope>=minimumDownhillDegrees ||
             aggregateSlope>=minimumDownhillDegrees ||
             (trustStairWayRendererNames && aggregateSlope>=-2f));
        if(pair.axis.Count<2) {
            pair.length=0f;
            return;
        }
        pair.entry=pair.axis[0]; pair.exit=pair.axis[pair.axis.Count-1];
        pair.side=Side(pair.heading);
        if(pair.flat>=0) {
            Piece flatPiece=pieces[pair.flat];
            // Orient the incoming flat by BOTH neighboring stage segments.
            // Guessing from distance to StairWay entry alone can select the
            // wrong end when stage pieces are moved or turn +/-90 degrees.
            float scoreAB=PlanarDistance(flatPiece.b,pair.entry);
            float scoreBA=PlanarDistance(flatPiece.a,pair.entry);
            if(index>0 && pairs[index-1].axis.Count>=2) {
                Vector3 priorExit=pairs[index-1].exit;
                scoreAB+=PlanarDistance(priorExit,flatPiece.a);
                scoreBA+=PlanarDistance(priorExit,flatPiece.b);
            }
            pair.flatHeading=scoreAB<=scoreBA ? flatPiece.heading : -flatPiece.heading;
        } else if(index>0 && pairs[index-1].axis.Count>=2)
            pair.flatHeading=Flat(pairs[index-1].exit-
                pairs[index-1].axis[pairs[index-1].axis.Count-2]);
        if(pair.flatHeading.sqrMagnitude<E) pair.flatHeading=pair.heading;
        pair.turn90=SignedRightAngle(pair.flatHeading,pair.heading,
            cornerToleranceDegrees,out pair.entryYaw);
        pair.hasCorner=false;
        if(pair.flat>=0) {
            Piece flatPart=pieces[pair.flat];
            bool forwardAB=Vector3.Dot(flatPart.b-flatPart.a,pair.flatHeading)>=0f;
            pair.flatStart=forwardAB?flatPart.a:flatPart.b;
            pair.flatEnd=forwardAB?flatPart.b:flatPart.a;
            if(pair.turn90 && Intersect90(pair.flatEnd,pair.flatHeading,
                pair.entry,pair.heading,pair.flatEnd.y,out Vector3 hit)) {
                float incoming=Vector3.Dot(hit-pair.flatStart,pair.flatHeading);
                float flatLength=PlanarDistance(pair.flatStart,pair.flatEnd);
                float entryReach=PlanarDistance(hit,pair.entry);
                bool valid=incoming>=-.25f && incoming<=flatLength+maximumCornerReach &&
                    entryReach<=maximumCornerReach &&
                    PlanarDistance(hit,pair.flatEnd)<=maximumCornerReach &&
                    IsForwardSegment(pair.flatStart,hit,pair.flatHeading) &&
                    IsForwardSegment(hit,pair.entry,pair.heading);
                if(valid) { pair.hasCorner=true; pair.corner=hit; }
            }
        }

        // Find the FIRST ArcSlab after this entire stair group and before
        // the next StairWay. Distinct from the incoming ArcSlab; its
        // direction is determined by the endpoint nearest the STAIR EXIT.
        pair.outgoingFlat=-1;
        pair.outTurn90=false;
        pair.hasOutgoingCorner=false;
        Vector3 finalHeading=Flat(pair.exit-pair.axis[pair.axis.Count-2]);
        for(int i=pair.last+1;i<pieces.Count;i++) {
            Piece candidate=pieces[i];
            if(candidate.stair) break;
            if(candidate.heading.sqrMagnitude<E) continue;
            pair.outgoingFlat=i;
            Vector3 nextEntry=Vector3.zero;
            bool hasNextEntry=false;
            for(int n=i+1;n<pieces.Count;n++) {
                if(pieces[n].stair) {
                    nextEntry=pieces[n].a;
                    hasNextEntry=true;
                    break;
                }
            }
            float scoreAB=PlanarDistance(pair.exit,candidate.a) +
                (hasNextEntry?PlanarDistance(candidate.b,nextEntry):0f);
            float scoreBA=PlanarDistance(pair.exit,candidate.b) +
                (hasNextEntry?PlanarDistance(candidate.a,nextEntry):0f);
            bool aIsNear=scoreAB<=scoreBA;
            pair.outgoingFlatStart=aIsNear?candidate.a:candidate.b;
            pair.outgoingFlatEnd=aIsNear?candidate.b:candidate.a;
            pair.outgoingHeading=Flat(pair.outgoingFlatEnd-pair.outgoingFlatStart);
            break;
        }
        if(pair.outgoingFlat>=0) {
            pair.outTurn90=SignedRightAngle(finalHeading,pair.outgoingHeading,
                cornerToleranceDegrees,out pair.exitYaw);
            if(pair.outTurn90 && Intersect90(pair.exit,finalHeading,
                pair.outgoingFlatStart,pair.outgoingHeading,
                pair.outgoingFlatStart.y,out Vector3 hit)) {
                float distanceIn=PlanarDistance(pair.exit,hit);
                float distanceOut=PlanarDistance(hit,pair.outgoingFlatEnd);
                bool valid=distanceIn<=maximumCornerReach &&
                    distanceOut<=maximumCornerReach &&
                    IsForwardSegment(pair.exit,hit,finalHeading) &&
                    IsForwardSegment(hit,pair.outgoingFlatEnd,pair.outgoingHeading);
                if(valid) { pair.hasOutgoingCorner=true; pair.outgoingCorner=hit; }
            }
        }
        pair.stations.Add(0f);
        for(int i=1;i<pair.axis.Count;i++)
            pair.stations.Add(pair.stations[i-1]+Vector3.Distance(pair.axis[i-1],pair.axis[i]));
        pair.length=pair.stations[pair.stations.Count-1];
    }

    // Project the ball on the current stair axis, never on a random nearby U-turn.
    // The vertical error is used to reject another height level, but not to move
    // the wave station when the displayed ball bounces vertically.
    float StationOn(Pair pair,Vector3 pos,out Vector3 foot,out float crossTrack) {
        foot=pair.entry;
        crossTrack=float.PositiveInfinity;
        float best=float.PositiveInfinity, station=0f;
        for(int i=0;i<pair.axis.Count-1;i++) {
            Vector3 a=pair.axis[i], d=pair.axis[i+1]-a;
            Vector3 flat=new Vector3(d.x,0f,d.z);
            float denom=flat.sqrMagnitude;
            if(denom<E) continue;
            Vector3 delta=new Vector3(pos.x-a.x,0f,pos.z-a.z);
            float t=Mathf.Clamp01(Vector3.Dot(delta,flat)/denom);
            Vector3 q=a+d*t;
            float x=pos.x-q.x, z=pos.z-q.z, y=pos.y-q.y;
            float score=x*x+z*z+.12f*y*y;
            if(score>=best) continue;
            best=score;
            foot=q;
            crossTrack=Mathf.Sqrt(x*x+z*z);
            station=Mathf.Lerp(pair.stations[i],pair.stations[i+1],t);
        }
        return station;
    }

    Vector3 PointOn(Pair pair,float traveled,out Vector3 tangent) {
        float s=Mathf.Clamp(traveled,0f,pair.length);
        int i=0;
        while(i<pair.stations.Count-2 && pair.stations[i+1]<s) i++;
        Vector3 a=pair.axis[i], d=pair.axis[i+1]-a;
        tangent=d.sqrMagnitude>E?d.normalized:Vector3.forward;
        float span=pair.stations[i+1]-pair.stations[i];
        float t=span>E?Mathf.Clamp01((s-pair.stations[i])/span):0f;
        return Vector3.Lerp(a,pair.axis[i+1],t);
    }

    // Choose the next stage group in build order; tolerate renderer/physics height gaps.
    int FindCandidate(Vector3 point) {
        for (int i = 0; i < pairs.Count; i++) {
            Pair p = pairs[i];
            if (!p.downhill || p.axis.Count < 2 || p.group <= lastCompletedGroup ||
                (active != null && i < pairIndex)) continue;
            float station = StationOn(p, point, out Vector3 foot, out float cross);
            Vector3 toEntry = Vector3.ProjectOnPlane(p.entry - point, Vector3.up);
            float gap = toEntry.magnitude;
            float ahead = Vector3.Dot(toEntry, p.flatHeading);
            bool approaching = gap <= lookAheadDistance && ahead >= -1.5f &&
                Mathf.Abs(point.y - p.entry.y) <= verticalDetectionTolerance &&
                Mathf.Abs(Vector3.Dot(toEntry, Side(p.flatHeading))) <=
                    maxEntryCrossTrack + laneHalfWidth;
            bool inside = station > .08f && station < p.length + .1f &&
                cross < maxEntryCrossTrack + laneHalfWidth &&
                Mathf.Abs(point.y - foot.y) <= verticalDetectionTolerance;
            if (approaching || inside) return i;
        }
        return -1;
    }

    void ProcessPair(Vector3 point) {
        int candidate = FindCandidate(point);
        if (active == null) {
            if (candidate < 0) return;
            Prepare(candidate);
        }
        if (active == null) return;
        Pair p = active;
        float station = StationOn(p, point, out Vector3 foot, out float across);
        stairStation = station;
        entryDistance = PlanarDistance(point, p.entry);
        // Plane crossing works even when stair mesh sits above the physical collider.
        Vector3 heading = Flat(p.axis[1] - p.axis[0]);
        Vector3 delta = Vector3.ProjectOnPlane(point - p.entry, Vector3.up);
        float signed = Vector3.Dot(delta, heading);
        bool entryPlaneCrossed = signed >= -cornerEntryLeadMeters &&
            signed <= Mathf.Max(4f, Mathf.Min(8f, p.length * .7f)) &&
            delta.magnitude <= Mathf.Max(5f, maxEntryCrossTrack + laneHalfWidth);
        bool inside = station > .08f && station <= p.length + .1f &&
            across <= maxEntryCrossTrack + laneHalfWidth &&
            Mathf.Abs(point.y - foot.y) <= verticalDetectionTolerance;

        // Candidate may be found after the player has entered, e.g. spawned
        // on a stair or an ArcSlab->StairWay frame was skipped. Either test
        // is enough to publish; all five write in the SAME frame.
        if (state == "PREPARED" && (entryPlaneCrossed || inside)) {
            PublishFive(p, point);
        }
        if (state == "PREPARED") {
            // Avoid permanently sticking to a never-entered, old candidate.
            if (candidate >= 0 && candidate > pairIndex &&
                entryDistance > lookAheadDistance * 1.25f) {
                Finish("MISSED_ENTRY", false);
                Prepare(candidate);
                ProcessPair(point);
            }
            return;
        }
        if (state == "ACTIVE") {
            // An adjacent stair may be reached without a detectable flat frame.
            int successor = FindReachedSuccessor(point);
            if (successor >= 0) {
                Finish("DIRECT_NEXT_STAIR");
                Prepare(successor);
                ProcessPair(point);
                return;
            }
            if (inside) AppendObservedPoint(equalizerDisplay ? equalizerDisplay.position : point, station);
            lastObservedStation = Mathf.Max(lastObservedStation, station);
            bool pastExit = Vector3.Dot(point - p.exit,
                Flat(p.exit - p.axis[p.axis.Count-2])) >= exitConfirmDistance;
            if (lastObservedStation >= p.length * .7f &&
                (pastExit || IsOnFollowingFlat(p, point))) {
                exitReached = true;
                state = "CORNER_HOLD";
                paths[Observed].Clear(); WriteSpline(Observed);
                paths[Future].Clear(); WriteSpline(Future);
                LifecycleEvent("STAIR_EXIT_KEEP_CORNERS");
            }
        }
        if (state == "CORNER_HOLD") {
            if (IsOnFollowingFlat(p, point)) {
                Finish("NEXT_FLAT_ARRIVAL");
                if (candidate >= 0 && pairs[candidate].group > p.group) {
                    Prepare(candidate);
                    ProcessPair(point);
                }
            } else {
                int reached = FindReachedSuccessor(point);
                if (reached >= 0) {
                    Finish("NEXT_STAIR_REACHED");
                    Prepare(reached);
                    ProcessPair(point);
                }
            }
        }
    }

    void Prepare(int index) {
        Pair p = pairs[index];
        if (!p.downhill || p.axis.Count < 2) return;
        active = p;
        pairIndex = index;
        stairGroup = p.group;
        state = "PREPARED";
        isDownhillStair = p.downhill;
        hasFlatBefore = p.flat >= 0;
        hasRightAngleTurn = p.turn90;
        cornerBuilt = exitReached = false;
        lastObservedStation = 0f;
        beforePublishReplans = railTurnCount = 0;
        lastReason = "";
        // Precompute A+B in memory but do not show prematurely on the flat.
        for (int i = 0; i < Count; i++) {
            paths[i].Clear();
            WriteSpline(i);
        }
        BuildRails(p);
        LifecycleEvent("PREPARED_NOT_PUBLISHED");
    }

    void PublishFive(Pair p, Vector3 observedPoint) {
        if (cornerBuilt) return;
        equalizerGroup = -1; // one reset on the shared flat -> stair publication
        // Recheck plan using latest renderer geometry (including 90-degree turns).
        BuildRails(p);
        RebuildGeometricGuide(p);
        // Two coincident-in-time seed knots on the actual rendered ball path.
        paths[Observed].Clear();
        Vector3 start = equalizerDisplay ? equalizerDisplay.position : observedPoint;
        paths[Observed].Add(start + Vector3.up * geometricLineLift);
        paths[Observed].Add(start + (p.axis[1] - p.axis[0]).normalized * .02f +
            Vector3.up * geometricLineLift);
        previousObserved = start;
        for (int i = 0; i < Count; i++) WriteSpline(i);
        cornerBuilt = true;
        state = "ACTIVE";
        LifecycleEvent("PUBLISHED_ALL_FIVE_AT_ENTRY");
        // From this point the read-only observed path can replace its seed.
        float station = StationOn(p, observedPoint, out _, out _);
        if (station > .3f) AppendObservedPoint(equalizerDisplay ? equalizerDisplay.position : observedPoint, station);
    }

    // Forecast only: same recursive Hermite baseline, never predicts actual impacts.
    // Sample the whole stair at constant arc-length intervals, not only its corners.
    void RebuildGeometricGuide(Pair p) {
        paths[Future].Clear();
        int count = Mathf.Clamp(Mathf.CeilToInt(p.length / .20f), 24, 256);
        Vector3 velocity = inSubject ? EqualizerMappedDirection(inSubject.velocity) : Vector3.zero;
        float baseHeight = equalizerGroup == p.group ? equalizerSlow : equalizerVisualRadius;
        for (int i = 0; i <= count; ++i) {
            float s = p.length * i / count;
            Vector3 axis = PointOn(p, s, out Vector3 tangent);
            Vector3 normal = Vector3.ProjectOnPlane(Vector3.up, tangent).normalized;
            if (normal.sqrMagnitude < E) normal = Vector3.up;
            float speed = Mathf.Max(.1f, Vector3.Dot(velocity, tangent));
            float gravity = Mathf.Max(.1f, -Vector3.Dot(Physics.gravity, normal));
            float wave = SampleRecursiveWave(p, s / Mathf.Max(.1f, p.length), speed,
                gravity, out _, out _, out _, out _);
            paths[Future].Add(axis + normal * (baseHeight + wave + geometricLineLift));
        }
        if (state == "ACTIVE") WriteSpline(Future);
    }

    // Fourth curve: record a read-only position trail, never move the subject.
    void AppendObservedPoint(Vector3 point, float station) {
        if (active == null || station < .05f || station > active.length) return;
        if (Vector3.Distance(previousObserved, point) < observedSpacing) return;
        // Lift ONLY geometry, not player Transform/Rigidbody.
        Vector3 p = point + Vector3.up * geometricLineLift;
        paths[Observed].Add(p);
        if (paths[Observed].Count > maxObservedKnots)
            paths[Observed].RemoveAt(0);
        previousObserved = point;
        WriteSpline(Observed);
    }

    int FindReachedSuccessor(Vector3 point) {
        if (active == null) return -1;
        for (int i = pairIndex + 1; i < pairs.Count; i++) {
            Pair p = pairs[i];
            if (!p.downhill || p.axis.Count < 2) continue;
            float s = StationOn(p, point, out Vector3 foot, out float cross);
            if (s > .15f && s < p.length - .15f &&
                cross < maxEntryCrossTrack + laneHalfWidth &&
                Mathf.Abs(point.y - foot.y) <= verticalDetectionTolerance) return i;
        }
        return -1;
    }

    bool IsOnFollowingFlat(Pair p,Vector3 ball) {
        for(int i=p.last+1;i<pieces.Count;i++) {
            Piece next=pieces[i];
            if(next.stair) return false;
            Vector3 a=next.a, b=next.b, ab=b-a;
            Vector3 planar=Flat(ab);
            float length=Vector3.ProjectOnPlane(ab,Vector3.up).magnitude;
            if(length<E || planar.sqrMagnitude<E) continue;
            float station=Vector3.Dot(Vector3.ProjectOnPlane(ball-a,Vector3.up),planar);
            float u=Mathf.Clamp01(station/length);
            Vector3 foot=Vector3.Lerp(a,b,u);
            float horizontal=Vector3.ProjectOnPlane(ball-foot,Vector3.up).magnitude;
            float vertical=Mathf.Abs(ball.y-foot.y);
            if(station>=-nextFlatArrivalTolerance && station<=length+nextFlatArrivalTolerance &&
               horizontal<=nextFlatArrivalTolerance && vertical<=2.5f) return true;
        }
        return false;
    }

    void Finish(string reason, bool markComplete = true) {
        equalizerGroup = -1;
        if (markComplete && active != null)
            lastCompletedGroup = Mathf.Max(lastCompletedGroup, active.group);
        active = null;
        pairIndex = stairGroup = -1;
        state = "IDLE";
        lastReason = reason;
        isDownhillStair = hasFlatBefore = hasRightAngleTurn = false;
        cornerBuilt = exitReached = false;
        lastObservedStation = 0f;
        for (int i = 0; i < Count; i++) {
            paths[i].Clear();
            WriteSpline(i);
        }
        LifecycleEvent("FINISH_" + reason);
    }

    void BuildRails(Pair pair) {
        for(int i=Out;i<=Inside;i++) paths[i].Clear();
        List<Vector3> center=paths[Center];
        // The flat segment and its actual right-angle X/Z intersection are
        // explicitly included in A+B. A+b remains STRICTLY on the stair axis.
        hasEntryCornerPoint=pair.hasCorner;
        entryCornerPoint=pair.corner;
        hasExitCornerPoint=pair.hasOutgoingCorner;
        exitCornerPoint=pair.outgoingCorner;
        signedEntryYaw=pair.entryYaw;
        signedExitYaw=pair.exitYaw;
        if(pair.flat>=0 && pair.hasCorner) {
            AppendAxis(center,pair.flatStart);
            AppendAxis(center,pair.corner);
        } else if(pair.flat>=0 && !pair.turn90) {
            // On a straight flat->stair entry, stray renderer centers must not
            // introduce a lateral kink. Preserve height but project both flat
            // endpoints onto the stair's constant X/Z centerline.
            AppendAxis(center,OnStraight(pair.flatStart,pair.entry,pair.heading));
            AppendAxis(center,OnStraight(pair.flatEnd,pair.entry,pair.heading));
        }
        // The stair mesh top commonly sits ~0.35m above its preceding flat.
        // Directly joining (corner.y=flatY) to (entry.y=meshTopY) creates a
        // diagonal *pre-stair* segment. Flatten that approach on the 3 A+B
        // rails only. The A+b stair axis and wave origin remain unchanged.
        bool useLevelApproach = pair.flat>=0 && (pair.hasCorner || !pair.turn90);
        if(pair.turn90 && pair.hasCorner) {
            Vector3 incoming=Vector3.ProjectOnPlane(pair.corner-pair.flatStart,Vector3.up);
            Vector3 outgoing=Vector3.ProjectOnPlane(pair.entry-pair.corner,Vector3.up);
            bool validTurn=incoming.magnitude>.05f && outgoing.magnitude>.05f &&
                Vector3.Dot(incoming.normalized,pair.flatHeading)>.99f &&
                Vector3.Dot(outgoing.normalized,pair.heading)>.99f &&
                Mathf.Abs(Vector3.Angle(incoming,outgoing)-90f)<=cornerToleranceDegrees;
            if(!validTurn) {
                // Don't connect a rejected Corner through a diagonal fallback.
                // Keep only the stair geometry when the renderer orientation
                // cannot supply two forward-going orthogonal segments.
                center.Clear();
                useLevelApproach=false;
                hasEntryCornerPoint=false;
            }
        }
        if(useLevelApproach) {
            float flatHeight=pair.hasCorner ? pair.corner.y : pair.flatEnd.y;
            Vector3 levelEntry=new Vector3(pair.entry.x,flatHeight,pair.entry.z);
            AppendAxis(center,levelEntry);
            // Avoid a vertical splice at the renderer's first endpoint.
            for(int i=1;i<pair.axis.Count;i++) AppendAxis(center,pair.axis[i]);
        } else {
            // Rejected intersections must never fabricate a diagonal connector.
            for(int i=0;i<pair.axis.Count;i++) AppendAxis(center,pair.axis[i]);
        }
        // A+B exit: one explicit stair exit -> 90-degree intersection ->
        // next flat outbound segment. A+b stair-only axis is untouched.
        // Never join the exit DIRECTLY to a non-collinear flat endpoint.
        if(pair.outTurn90) {
            if(pair.hasOutgoingCorner) {
                float flatY=pair.outgoingFlatStart.y;
                Vector3 last=center[center.Count-1];
                // Renderer top/bottom usually differs from the adjacent flat
                // by a small step. Move the LAST rail knot to flat height
                // (not a vertical segment that would invent a second corner).
                if(Mathf.Abs(last.y-flatY)<=1.0f) {
                    center[center.Count-1]=Level(last,flatY);
                    AppendAxis(center,Level(pair.outgoingCorner,flatY));
                    AppendAxis(center,Level(pair.outgoingFlatEnd,flatY));
                }
            }
        }
        float sign=turnToPositiveZ?1f:-1f;
        int turns=0;
        for(int i=0;i<center.Count;i++) {
            Vector3 c=center[i];
            Vector3 previous=i>0?Flat(c-center[i-1]):Vector3.zero;
            Vector3 following=i<center.Count-1?Flat(center[i+1]-c):Vector3.zero;
            Vector3 offset;
            if(previous.sqrMagnitude<E) offset=Side(following)*laneHalfWidth;
            else if(following.sqrMagnitude<E || Vector3.Dot(previous,following)>.999f)
                offset=Side(previous)*laneHalfWidth;
            else if(Mathf.Abs(Vector3.Angle(previous,following)-90f)<=cornerToleranceDegrees) {
                Vector3 offsetA=Side(previous)*laneHalfWidth;
                Vector3 offsetB=Side(following)*laneHalfWidth;
                if(Intersect90(c+offsetA,previous,c+offsetB,following,c.y,out Vector3 intersection) &&
                   (intersection-c).magnitude<=laneHalfWidth*miterLimit)
                    offset=intersection-c;
                else offset=(offsetA+offsetB).normalized*laneHalfWidth;
                turns++;
            } else offset=Side(following.sqrMagnitude>E?following:previous)*laneHalfWidth;
            paths[Out].Add(c+offset*sign);
            paths[Inside].Add(c-offset*sign);
        }
        railTurnCount=turns;
    }

    void WriteSpline(int index) {
        if(!containers[index]) return;
        Spline spline=containers[index].Spline;
        spline.Clear(); spline.Closed=false;
        Transform frame=containers[index].transform;
        List<Vector3> path=paths[index];
        for(int i=0;i<path.Count;i++) {
            Vector3 local=frame.InverseTransformPoint(path[i]);
            spline.Add(new BezierKnot(new float3(local.x,local.y,local.z)),TangentMode.Linear);
        }
        knotCounts[index]=spline.Count;
        meshDirty[index]=true;
    }

    // SplineContainer is not a Game-view renderer; a dedicated owned tube Mesh
    // draws the five independently generated splines without any LineRenderer.
    void BuildTube(int index) {
        Mesh mesh=meshes[index];
        mesh.Clear();
        Spline spline=containers[index].Spline;
        if(spline.Count<2) return;
        // The three A+B rails MUST include each actual Spline knot as a
        // tube ring. Uniform SplineUtility.EvaluatePosition(t=i/N) samples
        // do not land on unevenly spaced knots and visually cut corners.
        bool sharpRail=index<=Inside;
        int segmentCount=sharpRail?spline.Count-1:
            Mathf.Max(1,(spline.Count-1)*2);
        int rings=segmentCount+1, sides=Mathf.Max(3,tubeSides);
        var vertices=new Vector3[rings*sides];
        var normals=new Vector3[vertices.Length];
        var uv=new Vector2[vertices.Length];
        var triangles=new int[segmentCount*sides*6];
        for(int i=0;i<rings;i++) {
            float t=i/(float)segmentCount;
            float3 x=sharpRail?spline[i].Position: SplineUtility.EvaluatePosition(spline,t);
            float3 left=sharpRail?spline[Mathf.Max(0,i-1)].Position:
                SplineUtility.EvaluatePosition(spline,Mathf.Max(0f,t-.0005f));
            float3 right=sharpRail?spline[Mathf.Min(spline.Count-1,i+1)].Position:
                SplineUtility.EvaluatePosition(spline,Mathf.Min(1f,t+.0005f));
            Vector3 p=new Vector3(x.x,x.y,x.z);
            Vector3 direction=new Vector3(right.x-left.x,right.y-left.y,right.z-left.z).normalized;
            if(direction.sqrMagnitude<E) direction=Vector3.forward;
            Vector3 side=Vector3.Cross(direction,Vector3.up).normalized;
            if(side.sqrMagnitude<E) side=Vector3.Cross(direction,Vector3.right).normalized;
            Vector3 vertical=Vector3.Cross(direction,side).normalized;
            for(int j=0;j<sides;j++) {
                float angle=2f*Mathf.PI*j/sides;
                Vector3 n=side*Mathf.Cos(angle)+vertical*Mathf.Sin(angle);
                int k=i*sides+j;
                vertices[k]=p+n*tubeRadius;
                normals[k]=n;
                uv[k]=new Vector2(j/(float)sides,t);
            }
        }
        int next=0;
        for(int i=0;i<segmentCount;i++) for(int j=0;j<sides;j++) {
            int a=i*sides+j,b=i*sides+(j+1)%sides;
            int c=a+sides,d=b+sides;
            triangles[next++]=a;triangles[next++]=b;triangles[next++]=c;
            triangles[next++]=b;triangles[next++]=d;triangles[next++]=c;
        }
        mesh.vertices=vertices;mesh.normals=normals;
        mesh.uv=uv;mesh.triangles=triangles;
        mesh.RecalculateBounds();
    }
}
