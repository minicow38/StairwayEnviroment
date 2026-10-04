using UnityEngine;
using System.Collections.Generic;
[DefaultExecutionOrder(12000)]
[DisallowMultipleComponent]
public sealed class ExampleBallVisualEqualizer : MonoBehaviour {
    const float Eps = 0.000001f;
    const int BaseWaves = 3;
    const int MaxPatternMultiplier = 8;
    readonly RaycastHit[] hits = new RaycastHit[16];

    // Mathematical state containers. They keep related values together
    // without changing the public/Inspector-facing runtime fields.
    struct WaveAmplitudes {
        public float A1, A2, A3;

        public float At(int type) {
            return type <= 0 ? A1 : type == 1 ? A2 : A3;
        }
    }

    struct FCoordinate {
        // -1 = E->F side, +1 = F->G side. F itself belongs to the right side.
        public int Side;
        public float Distance01;

        public float LegacySignedValue {
            get { return Side < 0 ? Distance01 : -Distance01; }
        }
    }

    [Header("Required: only primary motion injection")]
    [SerializeField] Rigidbody inSubject;
    [SerializeField] Transform visualTarget;

    [Header("Output wiring: no writes to the existing Rigidbody")]
    [SerializeField] bool autoCreateDisplayForRigidbody = true;
    [SerializeField] bool hideOriginalRendererWhileTesting = true;
    [SerializeField] Transform actualDisplayTarget;
    readonly List<Renderer> originalRenderers = new List<Renderer>();
    readonly List<bool> originalRendererStates = new List<bool>();

    sealed class RendererBinding {
        public Renderer source;
        public Renderer display;
    }

    sealed class TrailRendererBinding {
        public TrailRenderer source;
        public TrailRenderer display;
        public bool primed;
        public bool lastSourceEmitting;
    }

    readonly List<RendererBinding> rendererBindings = new List<RendererBinding>();
    readonly List<TrailRendererBinding> trailRendererBindings = new List<TrailRendererBinding>();
    GameObject generatedDisplay;

    public TrailRenderer DisplayTrail =>
        trailRendererBindings.Count > 0 ? trailRendererBindings[0].display : null;

    [Header("Subject-space coordinate map (auto-resolves CorrespondSubject)")]
    [SerializeField] CorrespondSubject coordinateSource;
    [SerializeField] Transform physicsFrame, visualFrame;

    [Header("Subject: read-only visual anchor and rotation reference")]
    [SerializeField] Transform subjectRotation;
    [SerializeField] bool followSubjectRotation = true;
    [SerializeField] bool addVisualRoll;
    [SerializeField, Min(.01f)] float visualRadius = .5f;
    [SerializeField, Min(.01f)] float rotationFollowHz = 8f;

    [Header("Four surface heights -> three slopes")]
    [SerializeField] LayerMask surfaceLayers = ~0;
    [SerializeField, Min(.05f)] float probeSpacing = .45f;
    [SerializeField, Min(.1f)] float probeHeight = 2f;
    [SerializeField, Min(.2f)] float probeDepth = 8f;
    [SerializeField, Min(.01f)] float frameFollowHz = 10f;
    [SerializeField, Min(.01f)] float roughFollowHz = 12f;
    [SerializeField, Min(.01f)] float roughnessReference = .40f;

    [Header("Section / entry")]
    [SerializeField] bool autoBeginOnDownhill = true;
    [SerializeField] bool autoEndAtSection = true;
    [SerializeField, Min(.1f)] float sectionLength = 9.9f;
    [SerializeField, Range(1f, 60f)] float entrySlopeDegrees = 12f;
    [SerializeField, Min(0f)] float minimumEntryPlanarSpeed = 2f;
    [SerializeField, Min(0f)] float teleportThreshold = 2.5f;

    [Header("InitialStartPattern multiplier / F boundary")]
    [Tooltip("1 = 1->2->3. 2 = 1->2->3 | 2->3. 3 = 1->2->3 | 2->3 | 2->3.")]
    [SerializeField, Range(1, MaxPatternMultiplier)] int initialStartPatternMultiplier = 1;
    [Tooltip("Fixed F boundary. 0=E, 1=G. E->F is always 1->2->3; F->G is repeated 2->3 chunks.")]
    [SerializeField, Range(.05f, .95f)] float targetSlopeProgressPerecent = 3f / 7f;

    [Header("Vertical boundary / first-wave initial conditions")]
    [Tooltip("Physics-Y integration constant f(0)=CY. This NEVER changes X/Z.")]
    [SerializeField] float initialCY;
    [SerializeField, Min(.01f)] float initialHeight = .45f;
    [SerializeField, Min(0f)] float fallbackInitialNormalSpeed = 3.1367f;
    [SerializeField, Range(0f, 1f)] float energyBlend = .35f;
    [SerializeField, Range(.05f, .99f)] float restitution = .86f;
    [SerializeField, Range(.05f, 1f)] float lowerRatio = .75f;
    [SerializeField, Range(.15f, .95f)] float firstApexFraction = .82f;
    [SerializeField, Range(.15f, .85f)] float laterApexFraction = .50f;
    [SerializeField, Min(0f)] float dampingPerMeter = .02f;
    [SerializeField, Min(.01f)] float exitDamping = 9f;

    [Header("Unified motion character")]
    [Tooltip("Reference planar speed used only to classify a safe deterministic wave character.")]
    [SerializeField, Min(1f)] float referencePlanarSpeed = 18f;
    [Tooltip("Maximum A1 correction from planar speed / slope / roughness / entry alignment.")]
    [SerializeField, Range(0f, .15f)] float motionHeightInfluence = .06f;
    [Tooltip("Maximum A2/A3 ratio variation. This does not change wave topology.")]
    [SerializeField, Range(0f, .15f)] float patternVariation = .08f;
    [Tooltip("Small compression->rebound after G. Visual only; Rigidbody is untouched.")]
    [SerializeField, Range(0f, .5f)] float terminalBounceRatio = .22f;
    [SerializeField, Min(.10f)] float terminalBounceDuration = .32f;

    [Header("Measured roughness + 3-wave character")]
    [SerializeField, Range(0f, .5f)] float firstRoughness = .08f;
    [SerializeField, Range(0f, .5f)] float secondRoughness = .22f;
    [SerializeField, Range(0f, .5f)] float thirdRoughness = .12f;
    [SerializeField, Range(0f, .5f)] float measuredRoughnessGain = .10f;
    [SerializeField, Min(0f)] float signedRoughAccelerationGain = 90f;
    [SerializeField, Min(0f)] float accelerationResidualGain = .18f;
    [SerializeField, Min(.01f)] float residualLowPassHz = 2f;

    [Header("Residual oscillator: pullback / damping / jerk")]
    [SerializeField, Min(0f)] float springK = 420f;
    [SerializeField, Min(0f)] float damperC = 18f;
    [SerializeField, Min(0f)] float maximumResidual = .20f;
    [SerializeField, Min(.1f)] float maximumAcceleration = 450f;
    [SerializeField, Min(.1f)] float maximumJerk = 2500f;
    [SerializeField, Min(.01f)] float entryBlendSeconds = .10f;

    [Header("Presentation only: smooth renderer without changing InSubject")]
    [Tooltip("Smooth only the displayed Stable-N offset. InSubject, X/Z progression, wave phase and Colliders remain exact.")]
    [SerializeField] bool softenVisualWave = true;
    [Tooltip("Fast response near a wave contact so stair impact remains visible.")]
    [SerializeField, Range(.005f, .08f)] float impactSmoothSeconds = .015f;
    [Tooltip("Softer response between contacts, including the Apex.")]
    [SerializeField, Range(.005f, .10f)] float flightSmoothSeconds = .035f;
    [Tooltip("Fraction of the local wave near u=0/1 treated as the contact zone.")]
    [SerializeField, Range(.02f, .30f)] float contactBlendFraction = .16f;

    [Header("Adaptive Visual Filter: renderer-only")]
    [Tooltip("Shortens the display SmoothDamp time when the visual offset changes quickly. Physics is never filtered.")]
    [SerializeField] bool adaptiveVisualFilter = true;
    [SerializeField, Min(0f)] float adaptiveSpeedGain = 1.4f;
    [SerializeField, Range(.5f, 30f)] float adaptiveDerivativeHz = 8f;
    [SerializeField, Range(.005f, .04f)] float adaptiveMinimumSmoothSeconds = .008f;
    [Tooltip("Reset presentation lag only when Stable-N itself rotates sharply. T -> -T alone does not reset.")]
    [SerializeField] bool resetFilterOnSharpNormalChange = true;
    [SerializeField, Range(30f, 170f)] float filterNormalResetDegrees = 65f;

    [Header("Debug / trajectory analysis")]
    [SerializeField] bool enableDebugLog = true;
    [SerializeField] bool debugPeriodicState = true;
    [SerializeField] bool debugWaveTransitions = true;
    [SerializeField] bool debugProbeTransitions = true;
    [SerializeField, Min(.05f)] float debugLogInterval = .25f;

    [Header("Read-only runtime")]
    [SerializeField] bool active, exiting, probeValid, firstWavePeakReady;
    [SerializeField] int waveIndex = -1, waveType = -1, activeWaveCount = BaseWaves;
    [SerializeField] int repeat23ChunkCount, recursiveChunkIndex = -1, firstWavePeakRevision;
    [SerializeField] float traveled, progress01, currentAmplitude;
    [SerializeField] float efLocal01, fgLocal01, signedFLimit;
    [SerializeField] int fSide;
    [SerializeField] float fDistance01;
    [SerializeField] float slope0, slope1, slope2, signedCurvature, roughness, filteredRoughness;
    [SerializeField] float baseOffset, residualOffset, residualSpeed, residualAcceleration, visualOffset;
    [SerializeField] float presentedOffset, presentedSpeed, presentationSmoothSeconds;
    [SerializeField] float adaptiveVisualSpeed, adaptiveCutoffHz, currentWaveLocal01;
    [SerializeField] int visualFilterResetCount;
    [SerializeField] float upperNormal, lowerNormal, firstWavePeakCY, firstWavePeakProgress01;
    [SerializeField] Vector3 tangent = Vector3.forward, normal = Vector3.up, binormal = Vector3.right;

    [SerializeField] bool outputWritable;
    [SerializeField] float[] sampledHeights = new float[4];
    readonly Collider[] sampledColliders = new Collider[4];

    [Header("Runtime stair program")]
    [SerializeField] float effectiveSectionLength;
    [SerializeField] float effectiveFBoundary;
    [SerializeField] float recursiveA1, recursiveA2, recursiveA3;
    [SerializeField] int motionPattern;
    [SerializeField] float motionCharacter01, entryPlanarSpeed, entryAlignment01;
    [SerializeField] float terminalAmplitude, terminalOffset;
    Vector3 heading = Vector3.forward, previousCarrier, previousVelocity, previousApexPosition;
    Vector3 previousNormal = Vector3.up;
    Quaternion rotationOffset = Quaternion.identity, roll = Quaternion.identity;
    float gradient, filteredSignedCurvature, lowPassedNormalAcceleration;
    float initialWaveHeight, initialWaveSpeed, gravityN, elapsed, exitElapsed, flatElapsed;
    float previousOffset, previousOffsetSpeed, lastBegin = -999f, lastProbeSlopeDegrees;
    float previousFilterTarget;
    Vector3 previousFilterNormal = Vector3.up;
    bool visualFilterReady, presentationFrameReady;
    bool initialized, armed = true;

    // Debug state only. These do not affect trajectory calculation.
    float nextDebugLogTime;
    int lastDebugWaveIndex = int.MinValue;
    int lastDebugFSide;
    bool lastDebugProbeValid;
    public Rigidbody InSubject => inSubject;
    public bool IsActive => active;
    public int WaveIndex => waveIndex;
    public int WaveType => waveType + 1;
    public int ActiveWaveCount => activeWaveCount;
    public int Repeat23ChunkCount => repeat23ChunkCount;
    public int RecursiveChunkIndex => recursiveChunkIndex;
    public float TargetSlopeProgressPerecent => targetSlopeProgressPerecent;
    public float SignedFLimit => signedFLimit;
    public int FSide => fSide;
    public float FDistance01 => fDistance01;
    public float Progress01 => progress01;
    public float Roughness => filteredRoughness;
    public float SignedCurvature => filteredSignedCurvature;
    public float NormalOffset => visualOffset;
    public Vector3 StableT => tangent;
    public Vector3 StableN => normal;
    public Vector3 StableB => binormal;
    public float UpperNormal => upperNormal;
    public float LowerNormal => lowerNormal;
    public bool FirstWavePeakReady => firstWavePeakReady;
    public float FirstWavePeakCY => firstWavePeakCY;
    public float FirstWavePeakProgress01 => firstWavePeakProgress01;
    public int FirstWavePeakRevision => firstWavePeakRevision;
    public Vector3 FirstWavePeakWorld => previousApexPosition;

    void DebugEvent(string tag, string message) {
        if (!enableDebugLog) return;
        Debug.Log($"[ExampleBVE][{tag}] t={Time.fixedTime:F3} {message}", this);
    }

    void DebugPeriodic(
        Vector3 p,
        Vector3 v,
        float u,
        float localWaveLength,
        float forwardSpeed,
        float waveRough) {

        if (!enableDebugLog || !debugPeriodicState)
            return;

        if (Time.fixedTime < nextDebugLogTime)
            return;

        nextDebugLogTime =
            Time.fixedTime +
            Mathf.Max(.05f, debugLogInterval);

        Debug.Log(
            $"[ExampleBVE][STATE] " +
            $"t={Time.fixedTime:F3} " +
            $"active={active} exiting={exiting} probe={probeValid} armed={armed} " +
            $"progress={progress01:F4} traveled={traveled:F3}/{effectiveSectionLength:F3} " +
            $"waveIndex={waveIndex} waveType={waveType + 1} chunk={recursiveChunkIndex} u={u:F4} " +
            $"Fside={fSide} Fdist={fDistance01:F4} Fsigned={signedFLimit:F4} " +
            $"amp={currentAmplitude:F4} base={baseOffset:F4} residual={residualOffset:F4} " +
            $"terminal={terminalOffset:F4} visual={visualOffset:F4} " +
            $"lower={lowerNormal:F4} upper={upperNormal:F4} " +
            $"forwardSpeed={forwardSpeed:F3} localWaveLength={localWaveLength:F3} " +
            $"slopeDeg={lastProbeSlopeDegrees:F3} rough={filteredRoughness:F5} " +
            $"waveRough={waveRough:F5} curvature={filteredSignedCurvature:F5} " +
            $"pos={p:F4} vel={v:F4} " +
            $"T={tangent:F4} N={normal:F4} B={binormal:F4}",
            this);
    }

    void DebugProbeTransition() {
        if (!enableDebugLog || !debugProbeTransitions)
            return;

        if (probeValid == lastDebugProbeValid)
            return;

        lastDebugProbeValid = probeValid;

        DebugEvent(
            "PROBE",
            $"valid={probeValid} " +
            $"slopeDeg={lastProbeSlopeDegrees:F3} " +
            $"heights=[{sampledHeights[0]:F3},{sampledHeights[1]:F3},{sampledHeights[2]:F3},{sampledHeights[3]:F3}] " +
            $"rough={filteredRoughness:F5}");
    }

    public void SetInitialStartPatternValue(int patternValue) {
        int magnitude = patternValue == int.MinValue ? MaxPatternMultiplier : Mathf.Abs(patternValue);
        initialStartPatternMultiplier = Mathf.Clamp(magnitude, 1, MaxPatternMultiplier);
    }

    public void SetInitialStartPatternMultiplier(int multiplier) {
        initialStartPatternMultiplier = Mathf.Clamp(multiplier, 1, MaxPatternMultiplier);
    }

    public void Inject(Rigidbody primary, Transform secondaryRotation = null) {
        ReleaseGeneratedDisplay();
        inSubject = primary;
        subjectRotation = secondaryRotation;
        if (!visualTarget) visualTarget = transform;
        ResolveCoordinateSource();
        initialized = ValidateReferences();
        if (initialized) InitializeFrame();
    }

    void Awake() {
        if (!visualTarget) visualTarget = transform;
        ResolveCoordinateSource();
        initialized = ValidateReferences();
        if (initialized) InitializeFrame();
    }

    void OnEnable() {
        if (Application.isPlaying && initialized && !actualDisplayTarget) {
            ResolveDisplayTarget();
            outputWritable = actualDisplayTarget && !actualDisplayTarget.GetComponent<Rigidbody>();
        }
    }
    void OnDisable() => ReleaseGeneratedDisplay();
    void OnDestroy() => ReleaseGeneratedDisplay();

    void Start() {
        if (!coordinateSource) {
            ResolveCoordinateSource();
            if (coordinateSource && initialized) ValidateReferences();
        }
    }

    void ResolveCoordinateSource() {
        if (coordinateSource && inSubject && coordinateSource.InSubjectBody != inSubject) {
            Debug.LogWarning("[ExampleBVE][MAP_SOURCE_MISMATCH] Assigned CorrespondSubject drives a different InSubject; searching for a matching source.", this);
            coordinateSource = null;
        }
        if (!coordinateSource) {
            CorrespondSubject[] sources = FindObjectsOfType<CorrespondSubject>();
            for (int i = 0; i < sources.Length; i++)
                if (sources[i].InSubjectBody == inSubject &&
                    (!subjectRotation || !sources[i].SubjectBody ||
                     sources[i].SubjectBody.transform == subjectRotation)) {
                    coordinateSource = sources[i];
                    break;
                }
        }
        if (!subjectRotation && coordinateSource && coordinateSource.SubjectBody) subjectRotation = coordinateSource.SubjectBody.transform;
        if (coordinateSource && subjectRotation && coordinateSource.SubjectBody &&
            coordinateSource.SubjectBody.transform != subjectRotation) {
            Debug.LogWarning("[ExampleBVE][MAP_SUBJECT_MISMATCH] Selected Subject differs from CorrespondSubject's Subject. " +
                "Falling back to the assigned Subject anchor unless an explicit frame pair is provided.", this);
            coordinateSource = null;
        }
    }

    bool ValidateReferences() {
        if (!inSubject || !visualTarget) {
            Debug.LogError("[ExampleBVE] Assign the InSubject Rigidbody and a separate visualTarget.", this);
            return false;
        }

        if (visualTarget == inSubject.transform ||
            visualTarget.IsChildOf(inSubject.transform)) {
            Debug.LogError("[ExampleBVE] visualTarget must NOT be InSubject or its child.", this);
            return false;
        }

        ResolveDisplayTarget();

        outputWritable =
            actualDisplayTarget &&
            !actualDisplayTarget.GetComponent<Rigidbody>();

        if (!outputWritable) {
            Debug.LogError("[ExampleBVE][OUTPUT_UNAVAILABLE] Could not create a renderer-only display.", this);
            return false;
        }

        if (subjectRotation == visualTarget)
            subjectRotation = null;

        rotationOffset =
            subjectRotation
                ? Quaternion.Inverse(subjectRotation.rotation) *
                  visualTarget.rotation
                : visualTarget.rotation;

        bool hasCoordinateMap =
            (coordinateSource && coordinateSource.UsesRootFrames) ||
            (physicsFrame && visualFrame) ||
            subjectRotation;

        if (!hasCoordinateMap) {
            Debug.LogError("[ExampleBVE][NO_VISUAL_FRAME] Assign Subject Rotation or a valid root mapping.", this);
            return false;
        }

        return true;
    }

    void ResolveDisplayTarget() {
        if (!visualTarget) { actualDisplayTarget = null; return; }

        Rigidbody oldBody = visualTarget.GetComponent<Rigidbody>();
        if (!oldBody) {
            ReleaseGeneratedDisplay();
            actualDisplayTarget = visualTarget;
            return;
        }

        if (generatedDisplay) {
            actualDisplayTarget = generatedDisplay.transform;
            return;
        }

        if (!autoCreateDisplayForRigidbody) {
            actualDisplayTarget = null;
            return;
        }

        generatedDisplay = new GameObject("ExampleBVE_Display_" + visualTarget.name);
        Transform display = generatedDisplay.transform;

        // Keep the renderer-only proxy outside any moving Rigidbody hierarchy.
        // A normal VisualPlayerRoot parent is preserved when it is not physics-owned.
        Transform safeParent =
            visualTarget.parent &&
            !visualTarget.parent.GetComponentInParent<Rigidbody>()
                ? visualTarget.parent
                : null;

        display.SetParent(safeParent, false);
        display.SetPositionAndRotation(visualTarget.position, visualTarget.rotation);
        display.localScale = safeParent ? visualTarget.localScale : visualTarget.lossyScale;

        int count = CopyPresentation(visualTarget, display);
        if (count == 0) {
            Debug.LogError("[ExampleBVE][PROXY_EMPTY] No MeshRenderer+MeshFilter, SpriteRenderer, or TrailRenderer " +
                "was found beneath Visual Target. Existing renderers have not been hidden.", this);
            ReleaseGeneratedDisplay();
            actualDisplayTarget = null;
            return;
        }

        if (hideOriginalRendererWhileTesting) {
            Renderer[] all = visualTarget.GetComponentsInChildren<Renderer>(true);
            for (int i = 0; i < all.Length; i++) TrackOriginal(all[i]);
            for (int i = 0; i < originalRenderers.Count; i++)
                if (originalRenderers[i]) originalRenderers[i].enabled = false;
        }

        actualDisplayTarget = display;
    }

    int CopyPresentation(Transform source, Transform destination) {
        int copied = 0;

        TrailRenderer sourceTrail = source.GetComponent<TrailRenderer>();
        if (sourceTrail) {
            TrailRenderer displayTrail = destination.gameObject.AddComponent<TrailRenderer>();
            CopyTrailSettings(sourceTrail, displayTrail);
            
            TrailLengthController lengthController =
                source.GetComponent<TrailLengthController>();

            if (lengthController)
                lengthController.SetTrail(displayTrail);

            BallVisualTrailTurnReset turnReset =
                source.GetComponent<BallVisualTrailTurnReset>();

            if (turnReset)
                turnReset.SetTrail(displayTrail);
            displayTrail.enabled = sourceTrail.enabled;
            displayTrail.emitting = false;
            displayTrail.Clear();

            TrackOriginal(sourceTrail);
            trailRendererBindings.Add(new TrailRendererBinding {
                source = sourceTrail,
                display = displayTrail,
                primed = false,
                lastSourceEmitting = sourceTrail.emitting
            });
            copied++;
        }

        MeshRenderer sourceMesh = source.GetComponent<MeshRenderer>();
        MeshFilter sourceFilter = source.GetComponent<MeshFilter>();
        if (sourceMesh && sourceFilter && sourceFilter.sharedMesh) {
            destination.gameObject.AddComponent<MeshFilter>().sharedMesh = sourceFilter.sharedMesh;
            MeshRenderer renderer = destination.gameObject.AddComponent<MeshRenderer>();
            renderer.sharedMaterials = sourceMesh.sharedMaterials;
            renderer.shadowCastingMode = sourceMesh.shadowCastingMode;
            renderer.receiveShadows = sourceMesh.receiveShadows;
            renderer.lightProbeUsage = sourceMesh.lightProbeUsage;
            renderer.reflectionProbeUsage = sourceMesh.reflectionProbeUsage;
            renderer.sortingLayerID = sourceMesh.sortingLayerID;
            renderer.sortingOrder = sourceMesh.sortingOrder;
            renderer.enabled = sourceMesh.enabled;
            TrackOriginal(sourceMesh);
            BindRenderer(sourceMesh, renderer);
            copied++;
        }

        SpriteRenderer sourceSprite = source.GetComponent<SpriteRenderer>();
        if (sourceSprite && sourceSprite.sprite) {
            SpriteRenderer renderer = destination.gameObject.AddComponent<SpriteRenderer>();
            renderer.sprite = sourceSprite.sprite;
            renderer.sharedMaterial = sourceSprite.sharedMaterial;
            renderer.color = sourceSprite.color;
            renderer.flipX = sourceSprite.flipX;
            renderer.flipY = sourceSprite.flipY;
            renderer.drawMode = sourceSprite.drawMode;
            renderer.size = sourceSprite.size;
            renderer.maskInteraction = sourceSprite.maskInteraction;
            renderer.sortingLayerID = sourceSprite.sortingLayerID;
            renderer.sortingOrder = sourceSprite.sortingOrder;
            renderer.enabled = sourceSprite.enabled;
            TrackOriginal(sourceSprite);
            BindRenderer(sourceSprite, renderer);
            copied++;
        }

        for (int i = 0; i < source.childCount; i++) {
            Transform child = source.GetChild(i);
            GameObject node = new GameObject(child.name + "_Display");
            Transform dst = node.transform;
            dst.SetParent(destination, false);
            dst.localPosition = child.localPosition;
            dst.localRotation = child.localRotation;
            dst.localScale = child.localScale;
            dst.gameObject.SetActive(child.gameObject.activeSelf);
            copied += CopyPresentation(child, dst);
        }

        return copied;
    }

    static void CopyTrailSettings(TrailRenderer source, TrailRenderer display) {
        if (!source || !display) return;

        // Source-side TrailLengthController / TrailTurnReset remain the authority.
        display.time = source.time;
        display.minVertexDistance = source.minVertexDistance;
        display.widthMultiplier = source.widthMultiplier;
        display.widthCurve = source.widthCurve;
        display.colorGradient = source.colorGradient;
        display.numCornerVertices = source.numCornerVertices;
        display.numCapVertices = source.numCapVertices;
        display.alignment = source.alignment;
        display.textureMode = source.textureMode;
        display.generateLightingData = source.generateLightingData;
        display.shadowBias = source.shadowBias;
        display.autodestruct = false;

        display.sharedMaterials = source.sharedMaterials;
        display.shadowCastingMode = source.shadowCastingMode;
        display.receiveShadows = source.receiveShadows;
        display.lightProbeUsage = source.lightProbeUsage;
        display.reflectionProbeUsage = source.reflectionProbeUsage;
        display.sortingLayerID = source.sortingLayerID;
        display.sortingOrder = source.sortingOrder;
    }

    void TrackOriginal(Renderer renderer) {
        if (!renderer || originalRenderers.Contains(renderer)) return;
        originalRenderers.Add(renderer);
        originalRendererStates.Add(renderer.enabled);
    }

    void BindRenderer(Renderer source, Renderer display) {
        if (!source || !display) return;
        rendererBindings.Add(new RendererBinding { source = source, display = display });
    }

    // External systems may continue editing the original renderer/material.
    // The proxy is visible, so mirror appearance only. Renderer.enabled is intentionally
    // not copied because the original renderer is hidden by this component.
    void SyncPresentationFromOriginal() {
        for (int i = 0; i < rendererBindings.Count; i++) {
            RendererBinding binding = rendererBindings[i];
            if (binding == null || !binding.source || !binding.display) continue;

            MeshRenderer sourceMesh = binding.source as MeshRenderer;
            MeshRenderer displayMesh = binding.display as MeshRenderer;
            if (sourceMesh && displayMesh) {
                displayMesh.sharedMaterials = sourceMesh.sharedMaterials;
                displayMesh.shadowCastingMode = sourceMesh.shadowCastingMode;
                displayMesh.receiveShadows = sourceMesh.receiveShadows;
                displayMesh.lightProbeUsage = sourceMesh.lightProbeUsage;
                displayMesh.reflectionProbeUsage = sourceMesh.reflectionProbeUsage;
                displayMesh.sortingLayerID = sourceMesh.sortingLayerID;
                displayMesh.sortingOrder = sourceMesh.sortingOrder;
                continue;
            }

            SpriteRenderer sourceSprite = binding.source as SpriteRenderer;
            SpriteRenderer displaySprite = binding.display as SpriteRenderer;
            if (sourceSprite && displaySprite) {
                displaySprite.sprite = sourceSprite.sprite;
                displaySprite.sharedMaterial = sourceSprite.sharedMaterial;
                displaySprite.color = sourceSprite.color;
                displaySprite.flipX = sourceSprite.flipX;
                displaySprite.flipY = sourceSprite.flipY;
                displaySprite.drawMode = sourceSprite.drawMode;
                displaySprite.size = sourceSprite.size;
                displaySprite.maskInteraction = sourceSprite.maskInteraction;
                displaySprite.sortingLayerID = sourceSprite.sortingLayerID;
                displaySprite.sortingOrder = sourceSprite.sortingOrder;
            }
        }
    }

    // The proxy trail must be updated after the proxy pose. It therefore traces the
    // displayed Equalizer motion rather than the hidden source Rigidbody position.
    void SyncTrailPresentationFromOriginal(bool poseWritten) {
        for (int i = 0; i < trailRendererBindings.Count; i++) {
            TrailRendererBinding binding = trailRendererBindings[i];
            if (binding == null || !binding.source || !binding.display) continue;

            TrailRenderer source = binding.source;
            TrailRenderer display = binding.display;
            CopyTrailSettings(source, display);

            if (!poseWritten) {
                display.emitting = false;
                continue;
            }

            bool sourceEmitting = source.emitting;

            if (!binding.primed) {
                // First point is created only after the display has reached its real pose.
                display.Clear();
                display.emitting = sourceEmitting;
                binding.lastSourceEmitting = sourceEmitting;
                binding.primed = true;
                continue;
            }

            // TrailRenderer.Clear() on the source cannot be observed directly. The
            // TrailTurnReset OFF -> Clear -> ON sequence is reproduced on the proxy.
            if (!binding.lastSourceEmitting && sourceEmitting)
                display.Clear();

            display.emitting = sourceEmitting;
            binding.lastSourceEmitting = sourceEmitting;
        }
    }

    void ReleaseGeneratedDisplay() {
        for (int i = 0; i < originalRenderers.Count; i++)
            if (originalRenderers[i]) originalRenderers[i].enabled = originalRendererStates[i];

        originalRenderers.Clear();
        originalRendererStates.Clear();
        rendererBindings.Clear();
        trailRendererBindings.Clear();
        actualDisplayTarget = null;

        if (generatedDisplay) {
            if (Application.isPlaying) Destroy(generatedDisplay);
            else DestroyImmediate(generatedDisplay);
            generatedDisplay = null;
        }
    }

    void InitializeFrame() {
        previousCarrier = inSubject.position;
        previousVelocity = inSubject.velocity;
        Vector3 planar = Vector3.ProjectOnPlane(previousVelocity, Vector3.up);
        if (planar.sqrMagnitude > .01f) heading = planar.normalized;
        tangent = heading;
        normal = Vector3.up;
        binormal = Vector3.Cross(normal, tangent).normalized;
        previousNormal = normal;
        visualOffset = 0f;
        currentWaveLocal01 = 1f;
        ResetVisualFilterState(0f);
        active = exiting = false;

        lastDebugWaveIndex = int.MinValue;
        lastDebugFSide = 0;
        lastDebugProbeValid = probeValid;
        nextDebugLogTime = Time.fixedTime;

        DebugEvent(
            "INIT",
            $"pos={inSubject.position:F4} vel={inSubject.velocity:F4} " +
            $"heading={heading:F4} T={tangent:F4} N={normal:F4} B={binormal:F4}");
    }


    bool ProbeHeight(Vector3 point, int sampleIndex, out float y) {
        Vector3 origin = point + Vector3.up * probeHeight;
        int count = Physics.RaycastNonAlloc(origin, Vector3.down, hits,
            probeHeight + probeDepth, surfaceLayers, QueryTriggerInteraction.Ignore);
        float closest = float.PositiveInfinity;
        y = 0f;
        sampledColliders[sampleIndex] = null;
        for (int i = 0; i < count; i++) {
            Collider c = hits[i].collider;
            if (!c || c.attachedRigidbody == inSubject ||
                (visualTarget && c.transform.IsChildOf(visualTarget))) continue;
            if (hits[i].distance >= closest) continue;
            closest = hits[i].distance;
            y = hits[i].point.y;
            sampledColliders[sampleIndex] = c;
        }
        sampledHeights[sampleIndex] = y;
        return closest < float.PositiveInfinity;
    }

    static bool TryParseGeneratedStairName(
        string objectName,
        out string groupName) {

        groupName = null;

        const string suffix = "_Physics";
        if (string.IsNullOrEmpty(objectName) ||
            !objectName.EndsWith(
                suffix,
                System.StringComparison.Ordinal))
            return false;

        string core =
            objectName.Substring(
                0,
                objectName.Length - suffix.Length);

        int lastUnderscore = core.LastIndexOf('_');
        if (lastUnderscore <= 0)
            return false;

        int childIndex;
        if (!int.TryParse(
                core.Substring(lastUnderscore + 1),
                out childIndex))
            return false;

        groupName = core.Substring(0, lastUnderscore);

        return groupName.StartsWith(
            "StairWay",
            System.StringComparison.Ordinal);
    }

    int ResolveCurrentStairMultiplier() {
        for (int k = 0; k < 4; k++) {
            int sampleIndex =
                k == 0 ? 2 :
                k == 1 ? 3 :
                k == 2 ? 1 : 0;

            Collider hitCollider =
                sampledColliders[sampleIndex];

            if (!hitCollider)
                continue;

            Transform stair =
                hitCollider.transform;

            string group;

            while (stair &&
                   !TryParseGeneratedStairName(
                       stair.name,
                       out group))
                stair = stair.parent;

            if (!stair || stair.parent == null)
                continue;

            if (!TryParseGeneratedStairName(
                    stair.name,
                    out group))
                continue;

            int count = 0;
            Transform root = stair.parent;

            for (int i = 0; i < root.childCount; i++) {
                string siblingGroup;

                if (TryParseGeneratedStairName(
                        root.GetChild(i).name,
                        out siblingGroup) &&
                    siblingGroup == group)
                    count++;
            }

            if (count > 0)
                return Mathf.Clamp(
                    count,
                    1,
                    MaxPatternMultiplier);
        }

        return 1;
    }

    static float FollowAlpha(float hz, float dt) {
        return 1f - Mathf.Exp(-2f * Mathf.PI * Mathf.Max(.0001f, hz) * Mathf.Max(0f, dt));
    }

    void AnalyzeHeightSamples(float d) {
        // Discrete differential operator:
        // h --D--> three slopes --D--> signed curvature / roughness.
        float h0 = sampledHeights[0];
        float h1 = sampledHeights[1];
        float h2 = sampledHeights[2];
        float h3 = sampledHeights[3];

        slope0 = Mathf.Atan2(h1 - h0, d);
        slope1 = Mathf.Atan2(h2 - h1, d);
        slope2 = Mathf.Atan2(h3 - h2, d);

        gradient = (h3 - h0) / (3f * d);
        signedCurvature = slope1 - .5f * (slope0 + slope2);
        roughness = Mathf.Abs(signedCurvature) + .5f * Mathf.Abs(slope2 - slope0);
        lastProbeSlopeDegrees = -Mathf.Atan(gradient) * Mathf.Rad2Deg;
    }

    void TransportMovingFrame(Vector3 velocity, float dt) {
        Vector3 desiredT = probeValid
            ? (heading + Vector3.up * gradient).normalized
            : Vector3.ProjectOnPlane(velocity.sqrMagnitude > .01f ? velocity : heading, normal).normalized;

        if (desiredT.sqrMagnitude < Eps) desiredT = tangent;

        Vector3 transportedN = Quaternion.FromToRotation(tangent, desiredT) * normal;
        Vector3 rawN = probeValid ? Vector3.up : transportedN;
        rawN = Vector3.ProjectOnPlane(rawN, desiredT).normalized;

        if (rawN.sqrMagnitude < Eps) rawN = transportedN.normalized;
        if (Vector3.Dot(rawN, transportedN) < 0f) rawN = -rawN;

        tangent = desiredT;
        normal = Vector3.Slerp(transportedN, rawN, FollowAlpha(frameFollowHz, dt)).normalized;
        normal = Vector3.ProjectOnPlane(normal, tangent).normalized;
        binormal = Vector3.Cross(normal, tangent).normalized;
    }

    void ObserveGeometry(Vector3 position, Vector3 velocity, float dt) {
        Vector3 planar = Vector3.ProjectOnPlane(velocity, Vector3.up);
        if (planar.sqrMagnitude > .01f) heading = planar.normalized;

        float d = Mathf.Max(.05f, probeSpacing);
        float ignored;
        bool r0 = ProbeHeight(position - heading * (1.5f * d), 0, out ignored);
        bool r1 = ProbeHeight(position - heading * (.5f * d), 1, out ignored);
        bool r2 = ProbeHeight(position + heading * (.5f * d), 2, out ignored);
        bool r3 = ProbeHeight(position + heading * (1.5f * d), 3, out ignored);
        probeValid = r0 && r1 && r2 && r3;

        if (probeValid)
            AnalyzeHeightSamples(d);
        else {
            signedCurvature = roughness = 0f;
            gradient = 0f;
            lastProbeSlopeDegrees = 0f;
        }

        float roughA = FollowAlpha(roughFollowHz, dt);
        filteredSignedCurvature = Mathf.Lerp(filteredSignedCurvature, signedCurvature, roughA);
        filteredRoughness = Mathf.Lerp(filteredRoughness, roughness, roughA);

        TransportMovingFrame(velocity, dt);
    }

    float ResolveMotionCharacter(Vector3 velocity) {
        Vector3 planar =
            Vector3.ProjectOnPlane(
                velocity,
                Vector3.up);

        entryPlanarSpeed =
            planar.magnitude;

        float speed01 =
            Mathf.InverseLerp(
                minimumEntryPlanarSpeed,
                referencePlanarSpeed * 1.35f,
                entryPlanarSpeed);

        float slope01 =
            Mathf.InverseLerp(
                entrySlopeDegrees,
                45f,
                Mathf.Clamp(
                    lastProbeSlopeDegrees,
                    entrySlopeDegrees,
                    45f));

        float rough01 =
            Mathf.Clamp01(
                filteredRoughness /
                Mathf.Max(
                    .01f,
                    roughnessReference));

        Vector3 planarTangent =
            Vector3.ProjectOnPlane(
                tangent,
                Vector3.up);

        entryAlignment01 = 1f;

        if (planar.sqrMagnitude > .01f &&
            planarTangent.sqrMagnitude > .01f) {

            entryAlignment01 =
                Mathf.Clamp01(
                    Vector3.Dot(
                        planar.normalized,
                        planarTangent.normalized));
        }

        // One common rule for every stair:
        // speed gives the main character, slope/roughness add texture,
        // and poor post-turn alignment reduces excess emphasis.
        return Mathf.Clamp01(
            .50f * speed01 +
            .25f * slope01 +
            .15f * rough01 +
            .10f * entryAlignment01);
    }

    float MotionPatternSign() {
        // Echo=-1, Balanced=0, Punch=+1.
        return Mathf.Clamp(motionPattern - 1f, -1f, 1f);
    }

    float PatternAmplitudeScale(int type) {
        if (type <= 0) return 1f;

        float typeSign = type == 1 ? 1f : -.55f;
        return 1f + MotionPatternSign() * patternVariation * typeSign;
    }

    float PatternApex(int type) {
        if (type <= 0) return firstApexFraction;

        float waveSign = type == 1 ? -1f : 1f;
        float shift = MotionPatternSign() * patternVariation * .45f * waveSign;
        return Mathf.Clamp(laterApexFraction + shift, .35f, .65f);
    }

    float EvaluateTerminalOffset() {
        if (!exiting ||
            terminalAmplitude <= 0f ||
            terminalBounceDuration <= .01f)
            return 0f;

        float t =
            Mathf.Clamp01(
                exitElapsed /
                terminalBounceDuration);

        if (t >= 1f)
            return 0f;

        // One soft compression then one rebound, converging exactly to zero.
        float envelope =
            (1f - t) *
            (1f - t);

        return
            -terminalAmplitude *
            envelope *
            Mathf.Sin(
                2f *
                Mathf.PI *
                t);
    }

    [ContextMenu("ExampleBVE / Begin Stair (Play Mode)")]

    public void BeginStair() {
        if (!Application.isPlaying || !initialized) return;

        Vector3 v = inSubject.velocity;

        // Keep the successful factor itself:
        // sample entry energy from the live probe normal once at BeginStair.
        // No half-normal, no extra launch-angle reconstruction, no u0 clamp.
        Vector3 entryNormal = normal;
        gravityN =
            Mathf.Max(
                .1f,
                -Vector3.Dot(
                    Physics.gravity,
                    entryNormal));

        float measured =
            Mathf.Max(
                0f,
                Vector3.Dot(
                    v,
                    entryNormal));

        initialWaveSpeed =
            measured > .1f
                ? measured
                : fallbackInitialNormalSpeed;

        float ballisticRise =
            initialWaveSpeed * initialWaveSpeed /
            (2f * gravityN);

        initialWaveHeight =
            Mathf.Max(
                .02f,
                Mathf.Lerp(
                    initialHeight,
                    ballisticRise,
                    energyBlend));

        motionCharacter01 =
            ResolveMotionCharacter(v);

        float characterSigned =
            (motionCharacter01 - .5f) *
            2f;

        // Normal-direction entry energy remains the authority,
        // while other motion features can only move A1 by a small bounded amount.
        initialWaveHeight *=
            1f +
            characterSigned *
            motionHeightInfluence;

        motionPattern =
            motionCharacter01 < .34f
                ? 0
                : motionCharacter01 < .67f
                    ? 1
                    : 2;

        repeat23ChunkCount =
            Mathf.Max(
                0,
                initialStartPatternMultiplier - 1);

        // Algebraic topology:
        // prefix  = 1->2->3  (3 waves)
        // suffix  = (2->3)^repeat23ChunkCount  (2 waves per chunk)
        activeWaveCount =
            BaseWaves +
            repeat23ChunkCount * 2;

        effectiveSectionLength =
            sectionLength *
            Mathf.Max(
                1,
                initialStartPatternMultiplier);

        // F is an invariant of the program. Increasing the multiplier extends
        // the repeated 2->3 suffix without moving the E/F/G boundary itself.
        effectiveFBoundary =
            Mathf.Clamp(
                targetSlopeProgressPerecent,
                .05f,
                .95f);

        // -5 and above imitate the old -1 1->2->3 heights.
        float baseWaveLength =
            sectionLength / BaseWaves;

        float oneStepRetention =
            restitution * restitution *
            Mathf.Exp(
                -dampingPerMeter *
                baseWaveLength);

        WaveAmplitudes amplitudes = BuildWaveAmplitudes(initialWaveHeight, oneStepRetention);
        recursiveA1 = amplitudes.A1;
        recursiveA2 = amplitudes.A2;
        recursiveA3 = amplitudes.A3;

        terminalAmplitude = 0f;
        terminalOffset = 0f;

        previousCarrier = inSubject.position;
        previousVelocity = v;
        traveled = progress01 = elapsed = exitElapsed = flatElapsed = 0f;
        baseOffset = residualOffset = residualSpeed = residualAcceleration = visualOffset = 0f;
        currentWaveLocal01 = 0f;
        ResetVisualFilterState(0f);
        previousOffset = previousOffsetSpeed = lowPassedNormalAcceleration = 0f;
        currentAmplitude = initialWaveHeight;
        waveIndex = waveType = -1;
        recursiveChunkIndex = -1;
        efLocal01 = fgLocal01 = 0f;
        fSide = -1;
        fDistance01 = 1f;
        signedFLimit = 1f;
        firstWavePeakReady = false;
        firstWavePeakCY = firstWavePeakProgress01 = 0f;
        previousNormal = normal;
        lastBegin = Time.fixedTime;
        active = true;
        exiting = armed = false;

        lastDebugWaveIndex = int.MinValue;
        lastDebugFSide = -1;
        nextDebugLogTime = Time.fixedTime;

        DebugEvent(
            "BEGIN",
            $"multiplier={initialStartPatternMultiplier} repeats23={repeat23ChunkCount} waves={activeWaveCount} " +
            $"sectionLength={effectiveSectionLength:F3} F={effectiveFBoundary:F4} " +
            $"entryNormal={entryNormal:F4} gravityN={gravityN:F4} initialWaveSpeed={initialWaveSpeed:F4} " +
            $"ballisticRise={ballisticRise:F4} initialWaveHeight={initialWaveHeight:F4} " +
            $"motionCharacter={motionCharacter01:F4} pattern={motionPattern} " +
            $"A1={recursiveA1:F4} A2={recursiveA2:F4} A3={recursiveA3:F4} " +
            $"entryPlanarSpeed={entryPlanarSpeed:F3} alignment={entryAlignment01:F4} " +
            $"pos={inSubject.position:F4} vel={v:F4}");
    }
    WaveAmplitudes BuildWaveAmplitudes(float a1, float oneStepRetention) {
        WaveAmplitudes a;
        a.A1 = a1;
        a.A2 = a.A1 * oneStepRetention * PatternAmplitudeScale(1);
        a.A3 = a.A2 * oneStepRetention * PatternAmplitudeScale(2);

        // Projection onto the descending cone A1 > A2 > A3.
        a.A2 = Mathf.Min(a.A2, a.A1 * .96f);
        a.A3 = Mathf.Min(a.A3, a.A2 * .96f);
        return a;
    }

    [ContextMenu("ExampleBVE / End Stair (Play Mode)")]

    public void EndStair() {
        if (!active || exiting) return;

        exiting = true;
        exitElapsed = 0f;

        terminalAmplitude =
            Mathf.Min(
                visualRadius * .20f,
                Mathf.Max(
                    .01f,
                    Mathf.Max(
                        recursiveA3,
                        currentAmplitude) *
                    terminalBounceRatio));

        terminalOffset = 0f;

        DebugEvent(
            "END-BEGIN",
            $"progress={progress01:F4} traveled={traveled:F3} " +
            $"waveIndex={waveIndex} waveType={waveType + 1} " +
            $"currentAmplitude={currentAmplitude:F4} terminalAmplitude={terminalAmplitude:F4}");
    }

    void FixedUpdate() {
        if (!initialized) return;
        float dt = Time.fixedDeltaTime;
        Vector3 p = inSubject.position;
        Vector3 v = inSubject.velocity;
        Vector3 displacement = p - previousCarrier;
        if (displacement.magnitude > teleportThreshold) {
            DebugEvent(
                "TELEPORT",
                $"distance={displacement.magnitude:F3} threshold={teleportThreshold:F3} " +
                $"from={previousCarrier:F4} to={p:F4}");
            InitializeFrame();
            return;
        }
        ObserveGeometry(p, v, dt);
        DebugProbeTransition();

        Vector3 planar = Vector3.ProjectOnPlane(v, Vector3.up);
        if (!active) {
            visualOffset = 0f;
            currentWaveLocal01 = 1f;
            flatElapsed = lastProbeSlopeDegrees < entrySlopeDegrees * .5f
                ? flatElapsed + dt : 0f;
            if (flatElapsed > .12f && Time.fixedTime - lastBegin > .4f) armed = true;
            if (autoBeginOnDownhill && armed && probeValid &&
                lastProbeSlopeDegrees >= entrySlopeDegrees &&
                planar.magnitude >= minimumEntryPlanarSpeed) {

                SetInitialStartPatternMultiplier(
                    ResolveCurrentStairMultiplier());

                BeginStair();
            }
            if (!active) {
                previousCarrier = p;
                previousVelocity = v;
                return;
            }
            displacement = Vector3.zero;
        }
        elapsed += dt;
        if (exiting) exitElapsed += dt;
        float ds = Mathf.Max(0f, Vector3.Dot(displacement, tangent));
        traveled += ds;
        progress01 =
            Mathf.Clamp01(
                traveled /
                Mathf.Max(
                    .1f,
                    effectiveSectionLength));

        float u, localWaveLength;
        int priorWaveType;
        ResolveWaveCycle(progress01, out u, out localWaveLength, out priorWaveType);
        currentWaveLocal01 = u;

        if (enableDebugLog && debugWaveTransitions) {
            if (waveIndex != lastDebugWaveIndex) {
                DebugEvent(
                    "WAVE",
                    $"waveIndex={waveIndex} waveType={waveType + 1} priorType={priorWaveType + 1} " +
                    $"chunk={recursiveChunkIndex} progress={progress01:F4} u={u:F4} " +
                    $"localWaveLength={localWaveLength:F3} Fside={fSide} Fdist={fDistance01:F4}");
                lastDebugWaveIndex = waveIndex;
            }

            if (fSide != lastDebugFSide) {
                DebugEvent(
                    "F-CROSS",
                    $"side={fSide} progress={progress01:F4} F={effectiveFBoundary:F4} " +
                    $"EF={efLocal01:F4} FG={fgLocal01:F4} chunk={recursiveChunkIndex}");
                lastDebugFSide = fSide;
            }
        }

        if (autoEndAtSection && progress01 >= 1f) EndStair();

        bool repeated23Mode =
            repeat23ChunkCount > 0;

        // exp(-a) * exp(-b) = exp(-(a+b)).
        // The two independent damping paths are one additive decay exponent.
        float decayExponent =
            (repeated23Mode
                ? 0f
                : dampingPerMeter * traveled) +
            (exiting
                ? exitDamping * exitElapsed
                : 0f);

        float fade =
            Mathf.Exp(
                -decayExponent);

        float priorAmplitude;

        WaveAmplitudes amplitudeState = new WaveAmplitudes {
            A1 = recursiveA1,
            A2 = recursiveA2,
            A3 = recursiveA3
        };

        float baseCurrentAmplitude = amplitudeState.At(waveType);
        float basePriorAmplitude = amplitudeState.At(priorWaveType);

        currentAmplitude =
            baseCurrentAmplitude *
            fade;

        priorAmplitude =
            basePriorAmplitude *
            fade;

        float start =
            waveIndex == 0
                ? 0f
                : -priorAmplitude *
                  lowerRatio;

        float end =
            -currentAmplitude *
            lowerRatio;

        float apex =
            PatternApex(
                waveType);
        float measured = Mathf.Clamp01(filteredRoughness / Mathf.Max(.01f, roughnessReference));
        float baselineRough = waveType == 0 ? firstRoughness : waveType == 1 ? secondRoughness : thirdRoughness;
        float waveRough = baselineRough + measuredRoughnessGain * measured;


        float priorImpact = Mathf.Sqrt(2f * gravityN * priorAmplitude * (1f + lowerRatio));
        float impact = Mathf.Sqrt(2f * gravityN * currentAmplitude * (1f + lowerRatio));
        float riseSpeed = waveIndex == 0 ? initialWaveSpeed : restitution * priorImpact;
        float forwardSpeed = Mathf.Max(.1f, Vector3.Dot(v, tangent));
        float period = Mathf.Max(.001f, localWaveLength) / forwardSpeed;
        baseOffset = EvaluateWave(u, start, currentAmplitude, end, apex,
            riseSpeed, impact, period, waveRough);
        float normalAcceleration = Vector3.Dot((v - previousVelocity) / dt, normal);
        float lp = FollowAlpha(residualLowPassHz, dt);
        lowPassedNormalAcceleration = Mathf.Lerp(lowPassedNormalAcceleration, normalAcceleration, lp);
        float highPass = normalAcceleration - lowPassedNormalAcceleration;
        UpdateResidualOscillator(highPass, dt);
        float lowerBase = -Mathf.Lerp(waveIndex == 0 ? currentAmplitude : priorAmplitude,
            currentAmplitude, Mathf.SmoothStep(0f, 1f, u)) * lowerRatio;
        upperNormal = currentAmplitude * (1f + Mathf.Abs(waveRough)) + maximumResidual;
        lowerNormal = lowerBase - maximumResidual;
        terminalOffset =
            EvaluateTerminalOffset();

        float rawOffset =
            baseOffset +
            residualOffset +
            terminalOffset;

        visualOffset = ProjectToEnvelope(rawOffset, lowerNormal, upperNormal);
        float entryFactor = Mathf.SmoothStep(0f, 1f, elapsed / Mathf.Max(.01f, entryBlendSeconds));
        visualOffset *= entryFactor;
        float speedN = (visualOffset - previousOffset) / dt;

        DebugPeriodic(
            p,
            v,
            u,
            localWaveLength,
            forwardSpeed,
            waveRough);

        if (!firstWavePeakReady && waveIndex == 0 &&
            u >= .15f && previousOffset >= .5f * currentAmplitude &&
            previousOffsetSpeed > .02f && speedN <= 0f) {
            firstWavePeakReady = true;
            firstWavePeakRevision++;
            firstWavePeakCY = initialCY + previousNormal.y * previousOffset;
            firstWavePeakProgress01 = Mathf.Clamp01(
                progress01 -
                ds / Mathf.Max(.1f, effectiveSectionLength));
            previousApexPosition = MapPoint(previousCarrier + Vector3.up * initialCY + previousNormal * previousOffset);

            DebugEvent(
                "APEX",
                $"revision={firstWavePeakRevision} progress={firstWavePeakProgress01:F4} " +
                $"CY={firstWavePeakCY:F4} previousOffset={previousOffset:F4} " +
                $"world={previousApexPosition:F4} speedNBefore={previousOffsetSpeed:F4} speedNNow={speedN:F4}");
        }
        previousOffsetSpeed = speedN;
        previousOffset = visualOffset;
        previousNormal = normal;
        previousCarrier = p;
        previousVelocity = v;
        if (addVisualRoll && ds > 0f) roll = Quaternion.AngleAxis(-ds / Mathf.Max(.01f, visualRadius) * Mathf.Rad2Deg, MapDirection(binormal)) * roll;
        bool terminalDone =
            !exiting ||
            exitElapsed >= terminalBounceDuration;

        if (exiting &&
            terminalDone &&
            currentAmplitude < .002f &&
            Mathf.Abs(residualOffset) < .005f &&
            Mathf.Abs(residualSpeed) < .05f) {

            DebugEvent(
                "END-COMPLETE",
                $"progress={progress01:F4} traveled={traveled:F3} " +
                $"residualOffset={residualOffset:F5} residualSpeed={residualSpeed:F5} " +
                $"exitElapsed={exitElapsed:F3}");

            active = exiting = false;
            visualOffset = 0f;
            terminalOffset = 0f;
        }
    }

    void UpdateResidualOscillator(float highPassNormalAcceleration, float dt) {
        float forcing =
            filteredSignedCurvature * signedRoughAccelerationGain +
            Mathf.Clamp(highPassNormalAcceleration, -40f, 40f) * accelerationResidualGain;

        float desiredAcc = forcing - springK * residualOffset - damperC * residualSpeed;
        desiredAcc = Mathf.Clamp(desiredAcc, -maximumAcceleration, maximumAcceleration);

        residualAcceleration = Mathf.MoveTowards(
            residualAcceleration,
            desiredAcc,
            maximumJerk * dt);

        residualSpeed += residualAcceleration * dt;
        residualOffset += residualSpeed * dt;

        if (Mathf.Abs(residualOffset) <= maximumResidual) return;

        float sign = Mathf.Sign(residualOffset);
        residualOffset = sign * maximumResidual;

        // Remove only the velocity component that continues pushing outside.
        if (residualSpeed * sign > 0f) residualSpeed = 0f;
    }

    static float ProjectToEnvelope(float value, float lower, float upper) {
        // Idempotent projection P(P(x)) = P(x).
        return Mathf.Clamp(value, lower, upper);
    }

    void ResolveWaveCycle(float progress, out float u, out float localWaveLength, out int priorWaveType) {
        float f = effectiveFBoundary;

        FCoordinate fc = ResolveFCoordinate(progress, f);
        fSide = fc.Side;
        fDistance01 = fc.Distance01;
        signedFLimit = fc.LegacySignedValue;

        // Multiplier 1: P1 = 123.
        if (repeat23ChunkCount == 0) {
            recursiveChunkIndex = -1;
            efLocal01 = progress;
            fgLocal01 = 0f;

            ResolveUniformWaveSequence(
                progress,
                BaseWaves,
                0,
                0,
                0,
                out waveIndex,
                out waveType,
                out u,
                out priorWaveType);

            localWaveLength = effectiveSectionLength / BaseWaves;
            return;
        }

        // E->F is the invariant prefix 123.
        if (fc.Side < 0) {
            recursiveChunkIndex = -1;
            efLocal01 = Mathf.Clamp01(progress / f);
            fgLocal01 = 0f;

            ResolveUniformWaveSequence(
                efLocal01,
                BaseWaves,
                0,
                0,
                0,
                out waveIndex,
                out waveType,
                out u,
                out priorWaveType);

            localWaveLength = effectiveSectionLength * f / BaseWaves;
            return;
        }

        // F->G is the recursive word R_n = 23 · R_(n-1), R_1 = 23.
        // F itself belongs to this right-hand side.
        efLocal01 = 1f;
        fgLocal01 = Mathf.Clamp01((progress - f) / (1f - f));

        Resolve23Recursive(
            fgLocal01,
            repeat23ChunkCount,
            0,
            out recursiveChunkIndex,
            out waveIndex,
            out waveType,
            out u,
            out priorWaveType);

        localWaveLength =
            effectiveSectionLength *
            (1f - f) /
            (repeat23ChunkCount * 2f);
    }

    static void Resolve23Recursive(
        float x,
        int remainingChunks,
        int depth,
        out int chunkIndex,
        out int index,
        out int type,
        out float u,
        out int priorType) {

        x = Mathf.Clamp01(x);
        remainingChunks = Mathf.Max(1, remainingChunks);

        // Base case: R_1 = 23.
        if (remainingChunks == 1) {
            chunkIndex = depth;
            ResolveUniformWaveSequence(
                x,
                2,
                1,
                BaseWaves + depth * 2,
                2,
                out index,
                out type,
                out u,
                out priorType);
            return;
        }

        float firstWidth = 1f / remainingChunks;

        // Preserve the old boundary convention: an exact right boundary
        // belongs to the chunk on its left.
        if (x <= firstWidth) {
            chunkIndex = depth;
            ResolveUniformWaveSequence(
                x / firstWidth,
                2,
                1,
                BaseWaves + depth * 2,
                2,
                out index,
                out type,
                out u,
                out priorType);
            return;
        }

        // Remove the leading 23 and renormalize the remainder back to [0,1].
        // x' = (n*x - 1)/(n - 1).
        float nextX =
            (remainingChunks * x - 1f) /
            (remainingChunks - 1f);

        Resolve23Recursive(
            nextX,
            remainingChunks - 1,
            depth + 1,
            out chunkIndex,
            out index,
            out type,
            out u,
            out priorType);
    }

    static void ResolveUniformWaveSequence(
        float local01,
        int phaseCount,
        int firstWaveType,
        int baseWaveIndex,
        int priorTypeForFirst,
        out int index,
        out int type,
        out float u,
        out int priorType) {

        local01 =
            Mathf.Clamp01(
                local01);

        phaseCount =
            Mathf.Max(
                1,
                phaseCount);

        int localIndex;

        if (local01 >= 1f) {
            localIndex =
                phaseCount - 1;

            u = 1f;
        }
        else {
            float scaled =
                local01 *
                phaseCount;

            localIndex =
                Mathf.Clamp(
                    Mathf.FloorToInt(
                        scaled),
                    0,
                    phaseCount - 1);

            u =
                Mathf.Clamp01(
                    scaled -
                    localIndex);
        }

        index =
            baseWaveIndex +
            localIndex;

        type =
            firstWaveType +
            localIndex;

        priorType =
            localIndex == 0
                ? priorTypeForFirst
                : type - 1;
    }

    static FCoordinate ResolveFCoordinate(float progress, float f) {
        FCoordinate result;

        if (progress < f) {
            result.Side = -1;
            result.Distance01 = Mathf.Clamp01((f - progress) / Mathf.Max(Eps, f));
            return result;
        }

        result.Side = 1;
        result.Distance01 = Mathf.Clamp01((progress - f) / Mathf.Max(Eps, 1f - f));
        return result;
    }

    static float EvaluateWave(float u, float start, float peak, float end, float apex,
        float riseSpeed, float impactSpeed, float period, float rough) {
        float risingTangent = Mathf.Min(Mathf.Max(0f, riseSpeed * period * apex), 3f * Mathf.Max(0f, peak - start));
        float fallingTangent = Mathf.Min(Mathf.Max(0f, impactSpeed * period * (1f - apex)), 3f * Mathf.Max(0f, peak - end));
        float y = u < apex
            ? Hermite(start, peak, risingTangent, 0f, u / apex)
            : Hermite(peak, end, 0f, -fallingTangent, (u - apex) / (1f - apex));
        float window = Mathf.Pow(Mathf.Sin(Mathf.PI * u) * Mathf.Sin(Mathf.PI * (u - apex)), 2f);
        return y + peak * rough * window * Mathf.Sin(2f * Mathf.PI * u);
    }

    static float Hermite(float a, float b, float da, float db, float u) {
        float u2 = u * u, u3 = u2 * u;
        return (2f * u3 - 3f * u2 + 1f) * a + (u3 - 2f * u2 + u) * da
             + (3f * u2 - 2f * u3) * b + (u3 - u2) * db;
    }

    void ResetVisualFilterState(float target) {
        presentedOffset = target;
        previousFilterTarget = target;
        presentedSpeed = 0f;
        adaptiveVisualSpeed = 0f;
        adaptiveCutoffHz = 0f;
        presentationSmoothSeconds = 0f;
        previousFilterNormal = normal;
        visualFilterReady = false;
        presentationFrameReady = true;
        visualFilterResetCount++;
    }

    // One-Euro-inspired adaptation without adding another position filter.
    // Only the SmoothDamp response time changes; the wave solver remains untouched.
    float GetAdaptiveSmoothSeconds(float target, float baseSmoothSeconds, float dt) {
        float h = Mathf.Max(.0001f, dt);

        if (!visualFilterReady) {
            previousFilterTarget = target;
            adaptiveVisualSpeed = 0f;
            visualFilterReady = true;
        }

        float rawVisualSpeed = (target - previousFilterTarget) / h;
        previousFilterTarget = target;

        float alpha = 1f - Mathf.Exp(-2f * Mathf.PI * adaptiveDerivativeHz * h);
        adaptiveVisualSpeed += (rawVisualSpeed - adaptiveVisualSpeed) * alpha;

        float baseSeconds = Mathf.Max(.005f, baseSmoothSeconds);
        float baseCutoff = 1f / (2f * Mathf.PI * baseSeconds);
        adaptiveCutoffHz = baseCutoff + adaptiveSpeedGain * Mathf.Abs(adaptiveVisualSpeed);

        return Mathf.Clamp(
            1f / (2f * Mathf.PI * Mathf.Max(Eps, adaptiveCutoffHz)),
            Mathf.Min(adaptiveMinimumSmoothSeconds, baseSeconds),
            baseSeconds);
    }

    Vector3 MapPoint(Vector3 physicsWorldPoint) {
        if (coordinateSource && coordinateSource.UsesRootFrames) return coordinateSource.MapPoint(physicsWorldPoint);
        if (physicsFrame && visualFrame) return visualFrame.TransformPoint(physicsFrame.InverseTransformPoint(physicsWorldPoint));
        return subjectRotation
            ? subjectRotation.position + MapDirection(physicsWorldPoint - inSubject.transform.position)
            : physicsWorldPoint;
    }

    Quaternion ResolveRotationBetweenFrames() {
        Transform sourceParent = inSubject ? inSubject.transform.parent : null;
        Transform targetParent = subjectRotation ? subjectRotation.parent : null;

        Quaternion sourceRotation =
            sourceParent ? sourceParent.rotation : Quaternion.identity;

        Quaternion targetRotation =
            targetParent ? targetParent.rotation : Quaternion.identity;

        // Convert a direction from the source parent's orientation
        // into the target parent's orientation:
        // 1) undo the source rotation
        // 2) apply the target rotation
        return targetRotation * Quaternion.Inverse(sourceRotation);
    }

    Vector3 MapDirection(Vector3 direction) {
        if (coordinateSource && coordinateSource.UsesRootFrames)
            return coordinateSource.MapDirection(direction);

        if (physicsFrame && visualFrame)
            return visualFrame.TransformDirection(
                physicsFrame.InverseTransformDirection(direction));

        return ResolveRotationBetweenFrames() * direction;
    }

    void LateUpdate() {
        if (!initialized || !outputWritable)
            return;

        // Material / renderer state remains source-authoritative even though the
        // source renderer itself is hidden while the proxy is visible.
        SyncPresentationFromOriginal();

        float targetOffset = visualOffset;

        // T and -T represent the same tangent axis. Do not reset because of a
        // tangent sign flip; reset only when Stable-N itself changes sharply.
        bool sharpNormalChange =
            resetFilterOnSharpNormalChange &&
            presentationFrameReady &&
            Vector3.Angle(previousFilterNormal, normal) >= filterNormalResetDegrees;

        if (sharpNormalChange)
            ResetVisualFilterState(targetOffset);

        previousFilterNormal = normal;

        if (softenVisualWave) {
            float u = active ? Mathf.Clamp01(currentWaveLocal01) : 1f;
            float distanceFromContact = Mathf.Min(u, 1f - u);
            float blend = Mathf.SmoothStep(
                0f,
                1f,
                distanceFromContact / Mathf.Max(.02f, contactBlendFraction));

            float baseSeconds = Mathf.Lerp(
                impactSmoothSeconds,
                flightSmoothSeconds,
                blend);

            presentationSmoothSeconds = adaptiveVisualFilter
                ? GetAdaptiveSmoothSeconds(targetOffset, baseSeconds, Time.deltaTime)
                : baseSeconds;

            if (!adaptiveVisualFilter)
                visualFilterReady = false;

            presentedOffset = Mathf.SmoothDamp(
                presentedOffset,
                targetOffset,
                ref presentedSpeed,
                presentationSmoothSeconds,
                Mathf.Infinity,
                Time.deltaTime);
        }
        else {
            presentationSmoothSeconds = 0f;
            presentedOffset = targetOffset;
            presentedSpeed = 0f;
            adaptiveVisualSpeed = 0f;
            adaptiveCutoffHz = 0f;
            visualFilterReady = false;
        }

        presentationFrameReady = true;

        actualDisplayTarget.position =
            MapPoint(
                inSubject.transform.position +
                Vector3.up * initialCY +
                normal * presentedOffset);

        if (followSubjectRotation && subjectRotation) {
            Quaternion goal =
                roll *
                subjectRotation.rotation *
                rotationOffset;

            float a = FollowAlpha(rotationFollowHz, Time.deltaTime);

            actualDisplayTarget.rotation =
                Quaternion.Slerp(
                    actualDisplayTarget.rotation,
                    goal,
                    a);
        }
        else if (addVisualRoll) {
            actualDisplayTarget.rotation =
                roll * rotationOffset;
        }

        // The proxy TrailRenderer is updated after the final display pose so its
        // history follows the visible wave. Source time/emitting stay authoritative.
        SyncTrailPresentationFromOriginal(true);
    }

}
