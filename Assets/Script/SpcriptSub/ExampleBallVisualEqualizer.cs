using UnityEngine;
using System.Collections.Generic;
using Sirenix.OdinInspector;

/// <summary>
/// InSubject-only, visual-only Stairway equalizer (Unity 2022.3 / built-in API).
/// InSubject remains the only motion authority. SlopeStickCore is read-only and optional:
/// it is used only to estimate the real landing distance. Wave 1 happens once, then
/// Wave 2 / Wave 3 repeat spatially until the landing; the final planned wave is Wave 3.
/// No EnvelopeSystem, BallVisualSlopeDrive or Splines write dependency.
/// Attach this component to an independent VISUAL GameObject, not InSubject.
/// Main motion authority: InSubject only. Output coordinates follow CorrespondSubject.
/// Subject is a read-only visual-space anchor / rotation reference.
/// No physics force, velocity, or Rigidbody state is written.
/// If the assigned visualTarget has a Rigidbody, only its mesh/sprite presentation
/// is copied into an independent visual proxy under the same parent. The original
/// Rigidbody, Collider, and motion scripts remain untouched.
/// </summary>
[Searchable]
[DefaultExecutionOrder(12000)]
[DisallowMultipleComponent]
public sealed class ExampleBallVisualEqualizer : MonoBehaviour
{
    const float Eps = 0.000001f;
    const int ReferenceWaveCount = 3;
    readonly RaycastHit[] hits = new RaycastHit[16];

    [Header("Required: only primary motion injection")]
    [SerializeField] Rigidbody inSubject;
    [SerializeField] Transform visualTarget;
    [Header("Output wiring: always use an isolated renderer-only proxy")]
    [Tooltip("Must stay enabled: original Visual Target Transform is never moved, whether or not it owns a Rigidbody.")]
    [SerializeField] bool autoCreateDisplayForRigidbody = true;
    [SerializeField] bool hideOriginalRendererWhileTesting = true;
    [SerializeField] Transform actualDisplayTarget;
    [SerializeField] string outputMode = "Uninitialized";
    readonly List<Renderer> originalRenderers = new List<Renderer>();
    readonly List<bool> originalRendererStates = new List<bool>();

    sealed class RendererBinding
    {
        public Renderer source;
        public Renderer display;
    }

    readonly List<RendererBinding> rendererBindings = new List<RendererBinding>();
    GameObject generatedDisplay;
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
    [SerializeField] bool autoEndAtLanding = true;
    [Tooltip("Reference -1 stair length. This defines the natural spatial density: 1 -> 2 -> 3 over this distance.")]
    [SerializeField, Min(.1f)] float sectionLength = 9.9f;
    [Tooltip("READ ONLY. Used only to estimate the current stair landing distance; never written by ExampleBVE.")]
    [SerializeField] SlopeStickCore slopeSectionSource;
    [Tooltip("Only sections clearly longer than the -1 reference repeat Wave2/Wave3. Near-reference sections stay exactly 1->2->3.")]
    [SerializeField, Min(1.05f)] float repeatLengthMultiplierThreshold = 1.35f;
    [Tooltip("How quickly repeated Wave2/Wave3 pairs lose energy. This is pair damping, not distance stretching.")]
    [SerializeField, Range(.50f, 1f)] float repeatPairDamping = .92f;
    [Tooltip("Keeps long stairs visibly alive instead of letting restitution drive later pairs to almost zero.")]
    [SerializeField, Range(.02f, .60f)] float minimumRepeatAmplitudeRatio = .18f;
    [Tooltip("Short confirmation time after SlopeStickCore leaves the slope before the equalizer starts settling.")]
    [SerializeField, Range(0f, .20f)] float landingConfirmSeconds = .04f;
    [SerializeField, Range(1f, 60f)] float entrySlopeDegrees = 12f;
    [SerializeField, Min(0f)] float minimumEntryPlanarSpeed = 2f;
    [SerializeField, Min(0f)] float teleportThreshold = 2.5f;

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

    [Header("Presentation safety: keep the visible ball close to the stairs")]
    [Tooltip("Bounds the wave energy even if a large ground speed is mistakenly measured as launch speed.")]
    [SerializeField, Min(.02f)] float maximumFirstWaveHeight = .65f;
    [Tooltip("Maximum extra visual displacement along Stable-N, measured from InSubject.")]
    [SerializeField, Min(.02f)] float maximumVisibleLiftNormal = .52f;
    [Tooltip("Prevents the visible ball from diving far below InSubject between waves.")]
    [SerializeField, Min(.02f)] float maximumVisibleDropNormal = .28f;
    [Tooltip("Maximum visible ball CENTER height above the sampled stair surface; this is NOT a physics constraint.")]
    [SerializeField, Min(.5f)] float maximumCenterHeightAboveGround = 1.10f;

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

    [Header("Presentation only: soften the wave without delaying InSubject")]
    [Tooltip("Smooth only the normal-axis visual offset. Carrier X/Z and the physics Rigidbody remain exact.")]
    [SerializeField] bool softenVisualWave = true;
    [Tooltip("Short response near the Lower impact: retains the stair-contact sensation.")]
    [SerializeField, Range(.005f, .08f)] float impactSmoothSeconds = .015f;
    [Tooltip("Softer response between contacts, including the Apex.")]
    [SerializeField, Range(.005f, .10f)] float flightSmoothSeconds = .035f;
    [Tooltip("Fraction of each wave near a contact to blend between the two response times.")]
    [SerializeField, Range(.02f, .30f)] float contactBlendFraction = .16f;

    [Header("Presentation turn reset")]
    [Tooltip("Do not carry display smoothing lag from the old Stable-N/T frame across a sharp turn.")]
    [SerializeField] bool resetDisplayOnSharpTurn = true;
    [SerializeField, Range(30f, 170f)] float displayTurnResetDegrees = 65f;

    [Header("Read-only runtime diagnostics")]
    [SerializeField] bool active, exiting, probeValid, firstWavePeakReady;
    [Tooltip("0=Wave1, 1=Wave2, 2=Wave3. Repeated Wave2/Wave3 keep these profile indices.")]
    [SerializeField] int waveIndex = -1;
    [Tooltip("Absolute spatial wave number: 0,1,2,3... Pattern is 1,2,3,2,3...")]
    [SerializeField] int waveCycleIndex = -1;
    [SerializeField] int plannedWaveCount = ReferenceWaveCount, firstWavePeakRevision;
    [SerializeField] float traveled, progress01, phase, currentAmplitude;
    [SerializeField] bool landingPlanValid;
    [SerializeField] float plannedLandingDistance = 9.9f;
    [SerializeField] float runtimeWaveLength = 3.3f;
    [SerializeField] float sourceSectionLength;
    [SerializeField] float sourceSectionStartProgress01;
    [SerializeField] float slope0, slope1, slope2, signedCurvature, roughness, filteredRoughness;
    [SerializeField] float baseOffset, residualOffset, residualSpeed, residualAcceleration, visualOffset;
    [SerializeField] float presentedOffset, presentedSpeed, presentationSmoothSeconds;
    [SerializeField] float entryRawNormalSpeed, entryActualRiseSpeed, entryUnclampedHeight;
    [SerializeField] float visibleUpperLimit, guardedTargetOffset;
    [SerializeField] bool heightGuardActive;
    [SerializeField] int visualFilterResetCount;
    [SerializeField] float upperNormal, lowerNormal, firstWavePeakCY, firstWavePeakProgress01;
    [SerializeField] Vector3 tangent = Vector3.forward, normal = Vector3.up, binormal = Vector3.right;
    [Header("Debug: Inspector diagnostics (read only)")]
    [SerializeField] Vector3 mappedCarrierWorld, subjectWorld, predictedVisualWorld;
    [SerializeField] float mappedSubjectGap, outputGapBeforeWrite, roundTripError;
    [SerializeField] string mappingMode = "Uninitialized";
    [SerializeField] bool outputWritable, probeWasValid;
    bool probeInitialized;
    [SerializeField] float[] sampledHeights = new float[4];
    [SerializeField] string[] sampledSurfaces = new string[4];

    [Header("Debug: categorized and rate-limited Console logs")]
    [SerializeField] bool logEvents = true;
    [SerializeField] bool logCoordinates = true;
    [SerializeField] bool logWaveSamples = true;
    [SerializeField] bool logProbeTransitions = true;
    [SerializeField, Min(.05f)] float logIntervalSeconds = .25f;
    [SerializeField, Min(.01f)] float coordinateWarningMeters = .10f;

    float nextDiagnosticLogTime;
    bool coordinateMismatchLogged, blockedOutputLogged;
    Vector3 heading = Vector3.forward, previousCarrier, previousVelocity, previousApexPosition;
    Vector3 previousNormal = Vector3.up;
    Quaternion rotationOffset = Quaternion.identity, roll = Quaternion.identity;
    float gradient, filteredSignedCurvature, lowPassedNormalAcceleration;
    float initialWaveHeight, initialWaveSpeed, gravityN, elapsed, exitElapsed, flatElapsed;
    float previousOffset, previousOffsetSpeed, lastBegin = -999f, lastProbeSlopeDegrees;
    float nextHeightGuardLog;
    bool presentationFrameReady;
    Vector3 previousFilterNormal = Vector3.up, previousFilterTangent = Vector3.forward;
    bool initialized, armed = true;
    bool observedSlopeSinceBegin;
    float offSlopeElapsed;
    int lastApexCycle = -1;

    // Unity calls OnValidate when the dropdown changes in Edit Mode or Play Mode.
    // Avoid reapplying on every Inspector refresh so manual fine-tuning is retained.
    public Rigidbody InSubject => inSubject;
    public bool IsActive => active;
    public int WaveIndex => waveIndex;
    public int WaveCycleIndex => waveCycleIndex;
    public int PlannedWaveCount => plannedWaveCount;
    public float RuntimeWaveLength => runtimeWaveLength;
    public bool LandingPlanValid => landingPlanValid;
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
    // Compatibility aliases for existing Inspector/debug callers.
    public float RuntimeSectionLength => plannedLandingDistance;
    public bool AdaptiveSectionActive => landingPlanValid && plannedWaveCount > ReferenceWaveCount;

    /// <summary>InSubject drives motion; Subject is read only for mapped-space anchoring/rotation.</summary>
    public void Inject(Rigidbody primary, Transform secondaryRotation = null)
    {
        ReleaseGeneratedDisplay();
        inSubject = primary;
        subjectRotation = secondaryRotation;
        if (!visualTarget) visualTarget = transform;
        ResolveSlopeSectionSource();
        ResolveCoordinateSource();
        initialized = ValidateReferences();
        if (initialized) InitializeFrame();
    }

    void Awake()
    {
        if (!visualTarget) visualTarget = transform;
        ResolveSlopeSectionSource();
        ResolveCoordinateSource();
        initialized = ValidateReferences();
        if (initialized) InitializeFrame();
    }

    void OnEnable()
    {
        ResetVisualFilterState(visualOffset);

        if (Application.isPlaying && initialized && !actualDisplayTarget)
        {
            ResolveDisplayTarget();
            outputWritable = IsSafeDisplayTarget(actualDisplayTarget);
            blockedOutputLogged = false;
        }
    }

    void OnDisable() => ReleaseGeneratedDisplay();

    void OnDestroy() => ReleaseGeneratedDisplay();

    void Start()
    {
        // Stage/CorrespondSubject may be instantiated after this component's Awake.
        if (!coordinateSource)
        {
            ResolveCoordinateSource();
            if (coordinateSource && initialized) ValidateReferences();
        }
    }

    void ResolveSlopeSectionSource()
    {
        if (!slopeSectionSource && inSubject)
            slopeSectionSource = inSubject.GetComponent<SlopeStickCore>();
    }

    bool SlopeSectionReadyForBegin()
    {
        // Preserve the legacy downhill entry timing. The landing plan may be captured
        // a few FixedUpdate ticks later when SlopeStickCore has stabilized on the slope.
        return true;
    }

    void ResetWavePlan()
    {
        plannedLandingDistance = Mathf.Max(.1f, sectionLength);
        plannedWaveCount = ReferenceWaveCount;
        runtimeWaveLength = plannedLandingDistance / ReferenceWaveCount;
        sourceSectionLength = 0f;
        sourceSectionStartProgress01 = 0f;
        landingPlanValid = false;
        observedSlopeSinceBegin = false;
        offSlopeElapsed = 0f;
    }

    void CaptureWavePlan()
    {
        ResetWavePlan();
        ResolveSlopeSectionSource();
        TryCaptureLandingPlan(false);
    }

    void TryCaptureLandingPlanLate()
    {
        if (!active || landingPlanValid) return;
        ResolveSlopeSectionSource();
        TryCaptureLandingPlan(true);
    }

    bool TryCaptureLandingPlan(bool estimateEntryFromTraveled)
    {
        if (!slopeSectionSource || !slopeSectionSource.BallVisualIsOnSlope)
            return false;

        float fullLength = slopeSectionSource.BallVisualSlopeSectionLength;
        if (fullLength <= Eps) return false;

        float liveProgress = Mathf.Clamp01(slopeSectionSource.BallVisualSlopeProgress01);
        float startProgress = estimateEntryFromTraveled
            ? Mathf.Clamp01(liveProgress - traveled / Mathf.Max(.1f, fullLength))
            : liveProgress;

        float remaining = fullLength * Mathf.Max(0f, 1f - startProgress);
        if (remaining <= .1f) return false;

        float referenceLength = Mathf.Max(.1f, sectionLength);
        int count = ReferenceWaveCount;
        if (remaining > referenceLength * repeatLengthMultiplierThreshold)
            count = ChooseWaveCountFromLengthMultiplier(remaining, referenceLength);

        sourceSectionLength = fullLength;
        sourceSectionStartProgress01 = startProgress;
        plannedLandingDistance = remaining;
        plannedWaveCount = Mathf.Max(ReferenceWaveCount, count);
        runtimeWaveLength = plannedLandingDistance / plannedWaveCount;
        landingPlanValid = true;
        observedSlopeSinceBegin = true;
        offSlopeElapsed = 0f;

        Trace("WAVE_PLAN",
            $"sourceLength={sourceSectionLength:F3} startP={sourceSectionStartProgress01:F4} " +
            $"landing={plannedLandingDistance:F3} count={plannedWaveCount} " +
            $"waveLength={runtimeWaveLength:F3} pattern=1,2,3,2,3...");
        return true;
    }

    static int ChooseWaveCountFromLengthMultiplier(float distance, float referenceLength)
    {
        // Generated -N stairs scale almost linearly in length. Each extra reference
        // length adds one Wave2/Wave3 pair: -1=>3 waves, -2=>5, -3=>7, -5=>11.
        int lengthMultiplier = Mathf.Max(1, Mathf.RoundToInt(
            distance / Mathf.Max(.1f, referenceLength)));
        return 2 * lengthMultiplier + 1;
    }

    int ResolveWaveProfile(int cycleIndex)
    {
        if (cycleIndex <= 0) return 0;        // Wave1 once
        return (cycleIndex & 1) == 1 ? 1 : 2; // Wave2, Wave3, Wave2, Wave3...
    }

    float EvaluateCycleAmplitude(int cycleIndex, float distanceFade)
    {
        float energyRatio = restitution * restitution;
        int profile = ResolveWaveProfile(cycleIndex);

        float profileRatio = profile == 0
            ? 1f
            : profile == 1 ? energyRatio : energyRatio * energyRatio;

        int repeatPair = cycleIndex <= 2 ? 0 : (cycleIndex - 1) / 2;
        float pairRatio = Mathf.Pow(repeatPairDamping, repeatPair);
        float ratio = profileRatio * pairRatio;

        if (cycleIndex > 2)
            ratio = Mathf.Max(minimumRepeatAmplitudeRatio, ratio);

        return initialWaveHeight * ratio * distanceFade;
    }

    void UpdateLandingState(float dt)
    {
        ResolveSlopeSectionSource();
        bool onSlope = slopeSectionSource && slopeSectionSource.BallVisualIsOnSlope;

        if (onSlope)
        {
            observedSlopeSinceBegin = true;
            offSlopeElapsed = 0f;
        }
        else if (observedSlopeSinceBegin)
        {
            offSlopeElapsed += dt;
        }

        if (!autoEndAtLanding || exiting) return;

        // Primary end: the same SlopeStickCore section reached its real endpoint.
        if (landingPlanValid && traveled >= plannedLandingDistance - .01f)
        {
            EndStair();
            return;
        }

        // Secondary end: Core has actually left the slope for a short stable interval.
        if (observedSlopeSinceBegin && !onSlope && offSlopeElapsed >= landingConfirmSeconds)
        {
            EndStair();
            return;
        }

        // Fallback when SlopeStickCore is unavailable: require a real flat probe after
        // at least the reference 3-wave distance, not merely a noisy single sample.
        if (!slopeSectionSource && probeValid && traveled >= Mathf.Max(.1f, sectionLength) &&
            lastProbeSlopeDegrees < entrySlopeDegrees * .35f)
        {
            offSlopeElapsed += dt;
            if (offSlopeElapsed >= landingConfirmSeconds) EndStair();
        }
    }

    void ResolveCoordinateSource()
    {
        if (coordinateSource && inSubject && coordinateSource.InSubjectBody != inSubject)
        {
            Debug.LogWarning("[ExampleBVE][MAP_SOURCE_MISMATCH] Assigned CorrespondSubject drives a different InSubject; searching for a matching source.", this);
            coordinateSource = null;
        }
        if (!coordinateSource)
        {
            CorrespondSubject[] sources = FindObjectsOfType<CorrespondSubject>();
            for (int i = 0; i < sources.Length; i++)
                if (sources[i].InSubjectBody == inSubject &&
                    (!subjectRotation || !sources[i].SubjectBody ||
                     sources[i].SubjectBody.transform == subjectRotation))
                {
                    coordinateSource = sources[i];
                    break;
                }
        }
        if (!subjectRotation && coordinateSource && coordinateSource.SubjectBody)
            subjectRotation = coordinateSource.SubjectBody.transform;
        if (coordinateSource && subjectRotation && coordinateSource.SubjectBody &&
            coordinateSource.SubjectBody.transform != subjectRotation)
        {
            Debug.LogWarning("[ExampleBVE][MAP_SUBJECT_MISMATCH] Selected Subject differs from CorrespondSubject's Subject. " +
                "Falling back to the assigned Subject anchor unless an explicit frame pair is provided.", this);
            coordinateSource = null;
        }
    }

    void Trace(string kind, string info, bool diagnostic = false)
    {
        if (logEvents || diagnostic)
            Debug.Log($"[ExampleBVE][{kind}] t={Time.time:F3} {info}", this);
    }

    bool ValidateReferences()
    {
        if (!inSubject || !visualTarget)
        {
            Debug.LogError("[ExampleBVE] Assign the InSubject Rigidbody and a separate visualTarget.", this);
            return false;
        }
        if (visualTarget == inSubject.transform || visualTarget.IsChildOf(inSubject.transform) ||
            inSubject.transform.IsChildOf(visualTarget))
        {
            Debug.LogError("[ExampleBVE] Visual Target must be separate from InSubject's physics hierarchy.", this);
            return false;
        }
        ResolveDisplayTarget();
        outputWritable = IsSafeDisplayTarget(actualDisplayTarget);
        if (!outputWritable)
            Debug.LogError("[ExampleBVE][OUTPUT_UNAVAILABLE] Could not create a mesh/sprite-only display. " +
                "Assign a separate renderer-only Visual Target; the dynamic Rigidbody remains untouched.", this);
        if (surfaceLayers.value == ~0)
            Debug.LogWarning("[ExampleBVE][SURFACE_MASK] Everything may sample unrelated colliders. " +
                "Select only the actual stair/slope collision layer.", this);
        if ((physicsFrame == null) != (visualFrame == null))
            Debug.LogWarning("[ExampleBVE][PARTIAL_FRAME] Manual frame mapping needs BOTH fields. " +
                "The matching CorrespondSubject or Subject-anchor fallback will be used.", this);
        if (subjectRotation == visualTarget)
        {
            Debug.LogWarning("[ExampleBVE] Subject rotation reference equals visualTarget; ignoring this reference.", this);
            subjectRotation = null;
        }
        rotationOffset = subjectRotation
            ? Quaternion.Inverse(subjectRotation.rotation) * visualTarget.rotation
            : visualTarget.rotation;
        if (coordinateSource && coordinateSource.UsesRootFrames)
            mappingMode = "CorrespondSubject.RootFrames";
        else if (physicsFrame && visualFrame)
            mappingMode = "ManualRootFrames";
        else if (subjectRotation)
            mappingMode = "SubjectAnchorFallback";
        else
        {
            mappingMode = "MISSING_SUBJECT_FRAME";
            Debug.LogError("[ExampleBVE][NO_VISUAL_FRAME] Assign Subject Rotation or a valid root mapping. " +
                "Physical world positions cannot be treated as Subject-space positions.", this);
            return false;
        }
        Trace("SETUP", $"map={mappingMode} physics={PathOf(inSubject.transform)} " +
            $"subject={PathOf(subjectRotation)} target={PathOf(visualTarget)} " +
            $"source={PathOf(coordinateSource ? coordinateSource.transform : null)} writable={outputWritable} " +
            $"outputMode={outputMode} display={PathOf(actualDisplayTarget)}");
        return true;
    }


    // Always write a renderer-only proxy. Even when visualTarget has no Rigidbody,
    // moving its Transform might move a child Collider or a stage object.
    // The source Transform, Rigidbody and Collider hierarchy is strictly read-only.
    bool IsSafeDisplayTarget(Transform candidate)
    {
        return candidate && generatedDisplay && candidate == generatedDisplay.transform &&
            !candidate.GetComponentInParent<Rigidbody>() &&
            candidate.GetComponentInChildren<Collider>(true) == null &&
            candidate.GetComponentInChildren<Rigidbody>(true) == null;
    }

    void ResolveDisplayTarget()
    {
        if (!visualTarget) { actualDisplayTarget = null; return; }
        if (generatedDisplay)
        {
            actualDisplayTarget = generatedDisplay.transform;
            outputMode = "AutoVisualProxy";
            return;
        }
        if (!autoCreateDisplayForRigidbody)
        {
            actualDisplayTarget = null;
            outputMode = "ProxyDisabled";
            Debug.LogError("[ExampleBVE][PROXY_REQUIRED] Enable Auto Create Display For Rigidbody. " +
                "This option now protects *all* original Visual Targets, including targets without a Rigidbody.", this);
            return;
        }

        // A static Collider hierarchy is a stage/physics source, not a stand-alone
        // presentation object. The supported legacy ball source has its own Rigidbody.
        if (!visualTarget.GetComponent<Rigidbody>() &&
            visualTarget.GetComponentInChildren<Collider>(true))
        {
            actualDisplayTarget = null;
            outputMode = "PhysicsSourceRejected";
            Debug.LogError("[ExampleBVE][PHYSICS_SOURCE_REJECTED] Visual Target contains Colliders " +
                "but has no own Rigidbody. Assign the ball MeshRenderer source instead of a stage/physics root.", this);
            return;
        }

        // An incorrect stage/root assignment must not create an entire staircase
        // duplicate and hide its renderers. For a ball, a compact renderer tree is expected.
        Renderer[] sourceRenderers = visualTarget.GetComponentsInChildren<Renderer>(true);
        Collider[] sourceColliders = visualTarget.GetComponentsInChildren<Collider>(true);
        if (sourceRenderers.Length > 32 || sourceColliders.Length > 8)
        {
            actualDisplayTarget = null;
            outputMode = "UnsafeSourceHierarchy";
            Debug.LogError("[ExampleBVE][UNSAFE_VISUAL_TARGET] This is too large for a ball " +
                $"(renderers={sourceRenderers.Length}, colliders={sourceColliders.Length}). " +
                "Assign the single ball Visual Target, NOT Stairway/StageRoot.", this);
            return;
        }

        generatedDisplay = new GameObject("ExampleBVE_Display_" + visualTarget.name);
        Transform display = generatedDisplay.transform;
        // Never parent the output under a moving Rigidbody: renderer-only writes
        // must stay isolated from the physics hierarchy.
        Transform safeParent = visualTarget.parent && !visualTarget.parent.GetComponentInParent<Rigidbody>()
            ? visualTarget.parent : null;
        display.SetParent(safeParent, false);
        display.SetPositionAndRotation(visualTarget.position, visualTarget.rotation);
        display.localScale = safeParent ? visualTarget.localScale : visualTarget.lossyScale;
        int count = CopyPresentation(visualTarget, display);
        if (count == 0)
        {
            Debug.LogError("[ExampleBVE][PROXY_EMPTY] No MeshRenderer+MeshFilter or SpriteRenderer " +
                "was found beneath Visual Target. Existing renderers have not been hidden.", this);
            ReleaseGeneratedDisplay();
            outputMode = "NoCopyableRenderer";
            actualDisplayTarget = null;
            return;
        }
        if (hideOriginalRendererWhileTesting)
        {
            // Also mute the legacy TrailRenderer/ParticleSystemRenderer: otherwise
            // it remains visible at the old Rigidbody position during the test.
            Renderer[] all = visualTarget.GetComponentsInChildren<Renderer>(true);
            for (int i = 0; i < all.Length; i++) TrackOriginal(all[i]);
            for (int i = 0; i < originalRenderers.Count; i++)
                if (originalRenderers[i]) originalRenderers[i].enabled = false;
        }
        actualDisplayTarget = display;
        outputMode = "AutoVisualProxy";
        Trace("OUTPUT_CONNECTED", $"legacyBody={PathOf(visualTarget)} display={PathOf(display)} " +
            $"renderers={count} hiddenOriginal={hideOriginalRendererWhileTesting} " +
            "dynamicBodyUnchanged=True colliderUnchanged=True", true);
    }

    int CopyPresentation(Transform source, Transform destination)
    {
        int copied = 0;
        MeshRenderer sourceMesh = source.GetComponent<MeshRenderer>();
        MeshFilter sourceFilter = source.GetComponent<MeshFilter>();
        if (sourceMesh && sourceFilter && sourceFilter.sharedMesh)
        {
            destination.gameObject.AddComponent<MeshFilter>().sharedMesh = sourceFilter.sharedMesh;
            MeshRenderer renderer = destination.gameObject.AddComponent<MeshRenderer>();
            renderer.sharedMaterials = sourceMesh.sharedMaterials;
            renderer.shadowCastingMode = sourceMesh.shadowCastingMode;
            renderer.receiveShadows = sourceMesh.receiveShadows;
            renderer.sortingLayerID = sourceMesh.sortingLayerID;
            renderer.sortingOrder = sourceMesh.sortingOrder;
            renderer.enabled = sourceMesh.enabled;
            TrackOriginal(sourceMesh);
            BindRenderer(sourceMesh, renderer);
            copied++;
        }
        SpriteRenderer sourceSprite = source.GetComponent<SpriteRenderer>();
        if (sourceSprite && sourceSprite.sprite)
        {
            SpriteRenderer renderer = destination.gameObject.AddComponent<SpriteRenderer>();
            renderer.sprite = sourceSprite.sprite;
            renderer.sharedMaterial = sourceSprite.sharedMaterial;
            renderer.color = sourceSprite.color;
            renderer.flipX = sourceSprite.flipX;
            renderer.flipY = sourceSprite.flipY;
            renderer.sortingLayerID = sourceSprite.sortingLayerID;
            renderer.sortingOrder = sourceSprite.sortingOrder;
            renderer.enabled = sourceSprite.enabled;
            TrackOriginal(sourceSprite);
            BindRenderer(sourceSprite, renderer);
            copied++;
        }
        for (int i = 0; i < source.childCount; i++)
        {
            Transform child = source.GetChild(i);
            // No physics, script or animation components are copied to this child.
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

    void TrackOriginal(Renderer renderer)
    {
        if (!renderer || originalRenderers.Contains(renderer)) return;
        originalRenderers.Add(renderer);
        originalRendererStates.Add(renderer.enabled);
    }

    void BindRenderer(Renderer source, Renderer display)
    {
        if (!source || !display) return;

        rendererBindings.Add(new RendererBinding
        {
            source = source,
            display = display
        });
    }

    // MainGameManager continues to edit the ORIGINAL BallVisualEqualizer Renderer.
    // The proxy is what the camera sees, so mirror presentation state every LateUpdate.
    // IMPORTANT: do NOT copy Renderer.enabled here. The original is intentionally hidden.
    void SyncPresentationFromOriginal()
    {
        for (int i = 0; i < rendererBindings.Count; i++)
        {
            RendererBinding binding = rendererBindings[i];
            if (binding == null || !binding.source || !binding.display)
                continue;

            MeshRenderer sourceMesh = binding.source as MeshRenderer;
            MeshRenderer displayMesh = binding.display as MeshRenderer;

            if (sourceMesh && displayMesh)
            {
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

            if (sourceSprite && displaySprite)
            {
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

    void ReleaseGeneratedDisplay()
    {
        for (int i = 0; i < originalRenderers.Count; i++)
            if (originalRenderers[i]) originalRenderers[i].enabled = originalRendererStates[i];
        originalRenderers.Clear();
        originalRendererStates.Clear();
        rendererBindings.Clear();
        actualDisplayTarget = null;
        if (generatedDisplay)
        {
            if (Application.isPlaying) Destroy(generatedDisplay);
            else DestroyImmediate(generatedDisplay);
            generatedDisplay = null;
        }
    }

    void InitializeFrame()
    {
        previousCarrier = inSubject.position;
        previousVelocity = inSubject.velocity;
        Vector3 planar = Vector3.ProjectOnPlane(previousVelocity, Vector3.up);
        if (planar.sqrMagnitude > .01f) heading = planar.normalized;
        // Subject rotation lives in the visual frame; never use it unconverted as
        // a physics-world Raycast direction. Wait for actual InSubject velocity.
        tangent = heading;
        normal = Vector3.up;
        binormal = Vector3.Cross(normal, tangent).normalized;
        previousNormal = normal;
        visualOffset = 0f;
        ResetVisualFilterState(0f);
        heightGuardActive = false;
        visibleUpperLimit = maximumVisibleLiftNormal;
        guardedTargetOffset = 0f;
        active = exiting = false;
        ResetWavePlan();
        UpdateCoordinateDiagnostics(inSubject.position, 0f);
        Trace("FRAME_INIT", $"physics={Fmt(inSubject.position)} subject={Fmt(subjectWorld)} " +
            $"mapped={Fmt(mappedCarrierWorld)} discrepancy={mappedSubjectGap:F4}m mode={mappingMode}");
    }

    static string Fmt(Vector3 value) => $"({value.x:F3},{value.y:F3},{value.z:F3})";
    static string PathOf(Transform tr)
    {
        if (!tr) return "None";
        string path = tr.name;
        while (tr.parent) { tr = tr.parent; path = tr.name + "/" + path; }
        return path;
    }

    bool ProbeHeight(Vector3 point, int sampleIndex, out float y)
    {
        Vector3 origin = point + Vector3.up * probeHeight;
        int count = Physics.RaycastNonAlloc(origin, Vector3.down, hits,
            probeHeight + probeDepth, surfaceLayers, QueryTriggerInteraction.Ignore);
        float closest = float.PositiveInfinity;
        y = 0f;
        sampledSurfaces[sampleIndex] = "<miss>";
        for (int i = 0; i < count; i++)
        {
            Collider c = hits[i].collider;
            if (!c || c.attachedRigidbody == inSubject ||
                (visualTarget && c.transform.IsChildOf(visualTarget))) continue;
            if (hits[i].distance >= closest) continue;
            closest = hits[i].distance;
            y = hits[i].point.y;
            sampledSurfaces[sampleIndex] = c.name;
        }
        sampledHeights[sampleIndex] = y;
        return closest < float.PositiveInfinity;
    }

    void ObserveGeometry(Vector3 position, Vector3 velocity, float dt)
    {
        Vector3 planar = Vector3.ProjectOnPlane(velocity, Vector3.up);
        if (planar.sqrMagnitude > .01f) heading = planar.normalized;
        float d = Mathf.Max(.05f, probeSpacing);
        float h0 = 0f, h1 = 0f, h2 = 0f, h3 = 0f;
        bool r0 = ProbeHeight(position - heading * (1.5f * d), 0, out h0);
        bool r1 = ProbeHeight(position - heading * (.5f * d), 1, out h1);
        bool r2 = ProbeHeight(position + heading * (.5f * d), 2, out h2);
        bool r3 = ProbeHeight(position + heading * (1.5f * d), 3, out h3);
        probeValid = r0 && r1 && r2 && r3;
        if (logProbeTransitions && (!probeInitialized || probeWasValid != probeValid))
            Trace(probeValid ? "PROBE_RECOVERED" : "PROBE_MISS",
                $"valid={r0},{r1},{r2},{r3} sources=[{string.Join(",", sampledSurfaces)}] " +
                $"origin={Fmt(position)} heading={Fmt(heading)} layer={surfaceLayers.value}", true);
        probeWasValid = probeValid;
        probeInitialized = true;
        float a = 1f - Mathf.Exp(-2f * Mathf.PI * roughFollowHz * dt);
        if (probeValid)
        {
            slope0 = Mathf.Atan2(h1 - h0, d);
            slope1 = Mathf.Atan2(h2 - h1, d);
            slope2 = Mathf.Atan2(h3 - h2, d);
            gradient = (h3 - h0) / (3f * d);
            signedCurvature = slope1 - .5f * (slope0 + slope2);
            roughness = Mathf.Abs(signedCurvature) + .5f * Mathf.Abs(slope2 - slope0);
            lastProbeSlopeDegrees = -Mathf.Atan(gradient) * Mathf.Rad2Deg;
        }
        else
        {
            signedCurvature = roughness = 0f;
            lastProbeSlopeDegrees = 0f;
        }
        filteredSignedCurvature = Mathf.Lerp(filteredSignedCurvature, signedCurvature, a);
        filteredRoughness = Mathf.Lerp(filteredRoughness, roughness, a);

        Vector3 desiredT = probeValid
            ? (heading + Vector3.up * gradient).normalized
            : Vector3.ProjectOnPlane(velocity.sqrMagnitude > .01f ? velocity : heading, normal).normalized;
        if (desiredT.sqrMagnitude < Eps) desiredT = tangent;
        Vector3 transportedN = Quaternion.FromToRotation(tangent, desiredT) * normal;
        Vector3 rawN = probeValid ? Vector3.up : transportedN;
        rawN = Vector3.ProjectOnPlane(rawN, desiredT).normalized;
        if (rawN.sqrMagnitude < Eps) rawN = transportedN.normalized;
        if (Vector3.Dot(rawN, transportedN) < 0f) rawN = -rawN;
        float frameA = 1f - Mathf.Exp(-2f * Mathf.PI * frameFollowHz * dt);
        tangent = desiredT;
        normal = Vector3.Slerp(transportedN, rawN, frameA).normalized;
        normal = Vector3.ProjectOnPlane(normal, tangent).normalized;
        binormal = Vector3.Cross(normal, tangent).normalized;
    }

    [ContextMenu("ExampleBVE / Begin Stair (Play Mode)")]
    public void BeginStair()
    {
        if (!Application.isPlaying || !initialized) return;
        Vector3 v = inSubject.velocity;
        gravityN = Mathf.Max(.1f, -Vector3.Dot(Physics.gravity, normal));
        // Ground-parallel velocity has a nonzero dot with a *tilted* normal when
        // the ball is constrained to travel horizontally. It is NOT a takeoff.
        // Only actual upward Physics-Y movement can supply observed launch energy.
        entryRawNormalSpeed = Mathf.Max(0f, Vector3.Dot(v, normal));
        entryActualRiseSpeed = Mathf.Max(0f, v.y * Mathf.Max(0f, normal.y));
        initialWaveSpeed = entryActualRiseSpeed > .1f
            ? entryActualRiseSpeed : fallbackInitialNormalSpeed;
        float ballisticRise = initialWaveSpeed * initialWaveSpeed / (2f * gravityN);
        entryUnclampedHeight = Mathf.Max(.02f, Mathf.Lerp(initialHeight, ballisticRise, energyBlend));
        initialWaveHeight = Mathf.Min(entryUnclampedHeight, Mathf.Max(.02f, maximumFirstWaveHeight));
        if (entryRawNormalSpeed > entryActualRiseSpeed + .25f || entryUnclampedHeight > initialWaveHeight + .001f)
            Trace("ENTRY_ENERGY_GUARD", $"vY={v.y:F3} rawDotVN={entryRawNormalSpeed:F3} " +
                $"trueRise={entryActualRiseSpeed:F3} usedU0={initialWaveSpeed:F3} " +
                $"rawHeight={entryUnclampedHeight:F3} usedHeight={initialWaveHeight:F3}");
        previousCarrier = inSubject.position;
        previousVelocity = v;
        CaptureWavePlan();
        traveled = progress01 = phase = elapsed = exitElapsed = flatElapsed = 0f;
        baseOffset = residualOffset = residualSpeed = residualAcceleration = visualOffset = 0f;
        ResetVisualFilterState(0f);
        previousOffset = previousOffsetSpeed = lowPassedNormalAcceleration = 0f;
        currentAmplitude = initialWaveHeight;
        waveIndex = -1;
        waveCycleIndex = -1;
        lastApexCycle = -1;
        firstWavePeakReady = false;
        firstWavePeakCY = firstWavePeakProgress01 = 0f;
        previousNormal = normal;
        lastBegin = Time.fixedTime;
        active = true;
        exiting = armed = false;
        Trace("BEGIN", $"p={Fmt(previousCarrier)} v={Fmt(v)} N={Fmt(normal)} " +
            $"subject={Fmt(subjectRotation ? subjectRotation.position : Vector3.zero)} " +
            $"map={mappingMode} h0={initialWaveHeight:F3} u0={initialWaveSpeed:F3} " +
            $"landing={plannedLandingDistance:F3} count={plannedWaveCount} waveLength={runtimeWaveLength:F3}");
    }

    [ContextMenu("ExampleBVE / End Stair (Play Mode)")]
    public void EndStair()
    {
        if (!active || exiting) return;
        exiting = true;
        exitElapsed = 0f;
        Trace("EXIT", $"s={traveled:F3}");
    }

    void FixedUpdate()
    {
        if (!initialized) return;
        float dt = Time.fixedDeltaTime;
        Vector3 p = inSubject.position;
        Vector3 v = inSubject.velocity;
        Vector3 displacement = p - previousCarrier;
        if (displacement.magnitude > teleportThreshold)
        {
            InitializeFrame();
            Trace("TELEPORT_RESET", $"p={Fmt(p)} prior={Fmt(previousCarrier)} " +
                $"delta={displacement.magnitude:F3} threshold={teleportThreshold:F3}");
            return;
        }
        ObserveGeometry(p, v, dt);
        Vector3 planar = Vector3.ProjectOnPlane(v, Vector3.up);
        if (!active)
        {
            visualOffset = 0f;
            flatElapsed = lastProbeSlopeDegrees < entrySlopeDegrees * .5f
                ? flatElapsed + dt : 0f;
            if (flatElapsed > .12f && Time.fixedTime - lastBegin > .4f) armed = true;
            if (autoBeginOnDownhill && armed && probeValid &&
                lastProbeSlopeDegrees >= entrySlopeDegrees &&
                planar.magnitude >= minimumEntryPlanarSpeed &&
                SlopeSectionReadyForBegin())
                BeginStair();
            if (!active)
            {
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

        // Capture the real landing distance once Core has a stable slope section.
        // Unlike the previous version, this NEVER stretches 3 waves across a long stair.
        TryCaptureLandingPlanLate();

        float priorProgress01 = progress01;
        progress01 = landingPlanValid
            ? Mathf.Clamp01(traveled / Mathf.Max(.1f, plannedLandingDistance))
            : Mathf.Clamp01(traveled / Mathf.Max(.1f, sectionLength));

        float wavePosition = traveled / Mathf.Max(.1f, runtimeWaveLength);
        int oldCycle = waveCycleIndex;
        int cycle = Mathf.Max(0, Mathf.FloorToInt(wavePosition));
        float u = wavePosition - cycle;

        if (landingPlanValid && cycle >= plannedWaveCount)
        {
            cycle = plannedWaveCount - 1;
            u = 1f;
        }

        waveCycleIndex = cycle;
        waveIndex = ResolveWaveProfile(waveCycleIndex);
        phase = (waveCycleIndex + Mathf.Clamp01(u)) * 2f * Mathf.PI;

        UpdateLandingState(dt);

        // Preserve the original -1 damping character only over one reference section.
        // Long-distance decay is handled by repeatPairDamping so later waves do not vanish.
        float profileDistance = Mathf.Min(traveled, Mathf.Max(.1f, sectionLength));
        float distanceFade = Mathf.Exp(-dampingPerMeter * profileDistance -
            (exiting ? exitDamping * exitElapsed : 0f));

        currentAmplitude = EvaluateCycleAmplitude(waveCycleIndex, distanceFade);
        float priorAmplitude = EvaluateCycleAmplitude(Mathf.Max(0, waveCycleIndex - 1), distanceFade);
        float start = waveCycleIndex == 0 ? 0f : -priorAmplitude * lowerRatio;
        float end = -currentAmplitude * lowerRatio;
        float apex = waveIndex == 0 ? firstApexFraction : laterApexFraction;

        // Four-point roughness remains only a small modifier. The repeated 2/3 pattern
        // now supplies the durable stair rhythm, so R=0 on a clean 45-degree collider
        // no longer collapses the long-section motion into one broad monotone wave.
        float measured = Mathf.Clamp01(filteredRoughness / Mathf.Max(.01f, roughnessReference));
        float baselineRough = waveIndex == 0 ? firstRoughness : waveIndex == 1 ? secondRoughness : thirdRoughness;
        float waveRough = baselineRough + measuredRoughnessGain * measured;

        if (waveCycleIndex != oldCycle)
            Trace("WAVE", $"cycle={waveCycleIndex+1}/{(landingPlanValid ? plannedWaveCount : -1)} " +
                $"profile={waveIndex+1} s={traveled:F3} u={u:F3} " +
                $"R={filteredRoughness:F4} signed={filteredSignedCurvature:F4} " +
                $"rough={waveRough:F3} amplitude={currentAmplitude:F4}");

        float priorImpact = Mathf.Sqrt(2f * gravityN * priorAmplitude * (1f + lowerRatio));
        float impact = Mathf.Sqrt(2f * gravityN * currentAmplitude * (1f + lowerRatio));
        float riseSpeed = waveCycleIndex == 0 ? initialWaveSpeed : restitution * priorImpact;
        float forwardSpeed = Mathf.Max(.1f, Vector3.Dot(v, tangent));
        float period = Mathf.Max(.05f, runtimeWaveLength / forwardSpeed);

        baseOffset = EvaluateWave(u, start, currentAmplitude, end, apex,
            riseSpeed, impact, period, waveRough);

        float normalAcceleration = Vector3.Dot((v - previousVelocity) / dt, normal);
        float lp = 1f - Mathf.Exp(-2f * Mathf.PI * residualLowPassHz * dt);
        lowPassedNormalAcceleration = Mathf.Lerp(lowPassedNormalAcceleration, normalAcceleration, lp);
        float highPass = normalAcceleration - lowPassedNormalAcceleration;
        float force = filteredSignedCurvature * signedRoughAccelerationGain
                    + Mathf.Clamp(highPass, -40f, 40f) * accelerationResidualGain;
        float desiredAcc = force - springK * residualOffset - damperC * residualSpeed;
        desiredAcc = Mathf.Clamp(desiredAcc, -maximumAcceleration, maximumAcceleration);
        residualAcceleration = Mathf.MoveTowards(residualAcceleration, desiredAcc, maximumJerk * dt);
        residualSpeed += residualAcceleration * dt;
        residualOffset += residualSpeed * dt;
        if (residualOffset > maximumResidual)
        {
            residualOffset = maximumResidual;
            residualSpeed = Mathf.Min(0f, residualSpeed);
        }
        if (residualOffset < -maximumResidual)
        {
            residualOffset = -maximumResidual;
            residualSpeed = Mathf.Max(0f, residualSpeed);
        }
        float lowerBase = -Mathf.Lerp(waveCycleIndex == 0 ? currentAmplitude : priorAmplitude,
            currentAmplitude, Mathf.SmoothStep(0f, 1f, u)) * lowerRatio;
        upperNormal = currentAmplitude * (1f + Mathf.Abs(waveRough)) + maximumResidual;
        lowerNormal = lowerBase - maximumResidual;
        float rawOffset = baseOffset + residualOffset;
        visualOffset = Mathf.Clamp(rawOffset, lowerNormal, upperNormal);
        float entryFactor = Mathf.SmoothStep(0f, 1f, elapsed / Mathf.Max(.01f, entryBlendSeconds));
        visualOffset *= entryFactor;

        float speedN = (visualOffset - previousOffset) / dt;
        if (!firstWavePeakReady && waveCycleIndex == 0 &&
            u >= .15f && previousOffset >= .5f * currentAmplitude &&
            previousOffsetSpeed > .02f && speedN <= 0f)
        {
            firstWavePeakReady = true;
            firstWavePeakRevision++;
            firstWavePeakCY = initialCY + previousNormal.y * previousOffset;
            firstWavePeakProgress01 = Mathf.Clamp01(priorProgress01);
            previousApexPosition = MapPoint(previousCarrier + Vector3.up * initialCY + previousNormal * previousOffset);
            lastApexCycle = waveCycleIndex;
            Trace("FIRST_APEX", $"CY={firstWavePeakCY:F4} progress={firstWavePeakProgress01:F4} " +
                $"mappedWorld={Fmt(previousApexPosition)}");
        }
        else if (waveCycleIndex != lastApexCycle && previousOffsetSpeed > .02f && speedN <= 0f)
        {
            lastApexCycle = waveCycleIndex;
            Trace("APEX", $"cycle={waveCycleIndex+1} profile={waveIndex+1} q={previousOffset:F4}");
        }
        previousOffsetSpeed = speedN;
        previousOffset = visualOffset;
        previousNormal = normal;
        previousCarrier = p;
        previousVelocity = v;
        if (addVisualRoll && ds > 0f)
            roll = Quaternion.AngleAxis(-ds / Mathf.Max(.01f, visualRadius) * Mathf.Rad2Deg, MapDirection(binormal)) * roll;
        if (exiting && currentAmplitude < .002f && Mathf.Abs(residualOffset) < .005f && Mathf.Abs(residualSpeed) < .05f)
        {
            active = exiting = false;
            visualOffset = 0f;
            Trace("SETTLED", $"s={traveled:F3}");
        }
    }

    static float EvaluateWave(float u, float start, float peak, float end, float apex,
        float riseSpeed, float impactSpeed, float period, float rough)
    {
        float risingTangent = Mathf.Min(Mathf.Max(0f, riseSpeed * period * apex), 3f * Mathf.Max(0f, peak - start));
        float fallingTangent = Mathf.Min(Mathf.Max(0f, impactSpeed * period * (1f - apex)), 3f * Mathf.Max(0f, peak - end));
        float y = u < apex
            ? Hermite(start, peak, risingTangent, 0f, u / apex)
            : Hermite(peak, end, 0f, -fallingTangent, (u - apex) / (1f - apex));
        float window = Mathf.Pow(Mathf.Sin(Mathf.PI * u) * Mathf.Sin(Mathf.PI * (u - apex)), 2f);
        return y + peak * rough * window * Mathf.Sin(2f * Mathf.PI * u);
    }

    static float Hermite(float a, float b, float da, float db, float u)
    {
        float u2 = u * u, u3 = u2 * u;
        return (2f * u3 - 3f * u2 + 1f) * a + (u3 - 2f * u2 + u) * da
             + (3f * u2 - 2f * u3) * b + (u3 - u2) * db;
    }

    // A C1 soft knee: preserve the small contact motion and progressively
    // compress an excessive excursion instead of snapping the renderer off.
    void ResetVisualFilterState(float target)
    {
        presentedOffset = target;
        presentedSpeed = 0f;
        presentationSmoothSeconds = 0f;
        previousFilterNormal = normal;
        previousFilterTangent = tangent;
        presentationFrameReady = true;
        visualFilterResetCount++;
    }

    static float SoftLimit(float value, float lower, float upper)
    {
        if (value > 0f)
        {
            if (upper <= Eps) return 0f;
            float knee = Mathf.Max(0f, upper * .65f);
            float width = Mathf.Max(.001f, upper - knee);
            return value <= knee ? value : upper - width * Mathf.Exp(-(value - knee) / width);
        }
        float lowerAbs = Mathf.Max(.001f, -lower);
        float negativeKnee = lowerAbs * .65f;
        float negativeWidth = Mathf.Max(.001f, lowerAbs - negativeKnee);
        float depth = -value;
        return depth <= negativeKnee ? value
            : -(lowerAbs - negativeWidth * Mathf.Exp(-(depth - negativeKnee) / negativeWidth));
    }

    float GetVisibleUpperLimit(Vector3 carrier)
    {
        float cap = Mathf.Max(.02f, maximumVisibleLiftNormal);
        if (!probeValid || Mathf.Abs(normal.y) < .1f) return cap;
        // The two middle probes bracket the carrier. Use the higher support to
        // avoid claiming extra airborne height at a stair edge. Disregard a
        // distant/unrelated sampled surface rather than causing a sudden snap.
        float supportY = Mathf.Max(sampledHeights[1], sampledHeights[2]);
        float groundGap = carrier.y + initialCY - supportY;
        if (groundGap < -.4f || groundGap > 2f) return cap;
        float normalHeadroom = (maximumCenterHeightAboveGround - groundGap) / normal.y;
        return Mathf.Clamp(normalHeadroom, 0f, cap);
    }

    Vector3 MapPoint(Vector3 physicsWorldPoint)
    {
        // Use EXACTLY the same point-map authority as CorrespondSubject / Subject.
        if (coordinateSource && coordinateSource.UsesRootFrames)
            return coordinateSource.MapPoint(physicsWorldPoint);
        if (physicsFrame && visualFrame)
            return visualFrame.TransformPoint(physicsFrame.InverseTransformPoint(physicsWorldPoint));
        // Fallback is explicitly Subject-anchored, not a raw physics-world output.
        // Subject position is read only; all wave physics remains InSubject-derived.
        return subjectRotation
            ? subjectRotation.position + MapDirection(physicsWorldPoint - inSubject.transform.position)
            : physicsWorldPoint;
    }

    Vector3 MapDirection(Vector3 direction)
    {
        if (coordinateSource && coordinateSource.UsesRootFrames)
            return coordinateSource.MapDirection(direction);
        if (physicsFrame && visualFrame)
            return visualFrame.TransformDirection(physicsFrame.InverseTransformDirection(direction));
        Transform sourceParent = inSubject ? inSubject.transform.parent : null;
        Transform targetParent = subjectRotation ? subjectRotation.parent : null;
        Quaternion sourceQ = sourceParent ? sourceParent.rotation : Quaternion.identity;
        Quaternion targetQ = targetParent ? targetParent.rotation : Quaternion.identity;
        return targetQ * Quaternion.Inverse(sourceQ) * direction;
    }

    void UpdateCoordinateDiagnostics(Vector3 physicsPosition, float offset)
    {
        string liveMode = coordinateSource && coordinateSource.UsesRootFrames
            ? "CorrespondSubject.RootFrames"
            : physicsFrame && visualFrame ? "ManualRootFrames" : "SubjectAnchorFallback";
        if (mappingMode != liveMode)
        {
            Trace("MAP_MODE_CHANGED", $"old={mappingMode} new={liveMode}");
            mappingMode = liveMode;
        }
        mappedCarrierWorld = MapPoint(physicsPosition);
        subjectWorld = subjectRotation ? subjectRotation.position : mappedCarrierWorld;
        mappedSubjectGap = subjectRotation ? Vector3.Distance(mappedCarrierWorld, subjectWorld) : 0f;
        predictedVisualWorld = MapPoint(physicsPosition + Vector3.up * initialCY + normal * offset);
        outputGapBeforeWrite = actualDisplayTarget ? Vector3.Distance(actualDisplayTarget.position, predictedVisualWorld) : 0f;
        roundTripError = coordinateSource && coordinateSource.UsesRootFrames
            ? Vector3.Distance(physicsPosition, coordinateSource.InverseMapPoint(mappedCarrierWorld)) : 0f;
    }

    void DiagnosticSample()
    {
        if (Time.unscaledTime < nextDiagnosticLogTime) return;
        nextDiagnosticLogTime = Time.unscaledTime + Mathf.Max(.05f, logIntervalSeconds);
        if (logCoordinates)
            Trace("MAP", $"mode={mappingMode} inP={Fmt(inSubject.transform.position)} " +
                $"subjectP={Fmt(subjectWorld)} carrierMapped={Fmt(mappedCarrierWorld)} " +
                $"targetCalculated={Fmt(predictedVisualWorld)} targetActual={Fmt(actualDisplayTarget ? actualDisplayTarget.position : Vector3.zero)} " +
                $"subjectGap={mappedSubjectGap:F4} preWriteGap={outputGapBeforeWrite:F4} " +
                $"roundTrip={roundTripError:F6} writable={outputWritable} outputMode={outputMode} " +
                $"rootP={PathOf(coordinateSource ? coordinateSource.PhysicsRoot : physicsFrame)} " +
                $"rootV={PathOf(coordinateSource ? coordinateSource.VisualPlayerRoot : visualFrame)}", true);
        if (logWaveSamples)
            Trace("SAMPLE", $"active={active} exiting={exiting} probe={probeValid} " +
                $"heights=[{sampledHeights[0]:F3},{sampledHeights[1]:F3},{sampledHeights[2]:F3},{sampledHeights[3]:F3}] " +
                $"slope=[{slope0:F3},{slope1:F3},{slope2:F3}] R={filteredRoughness:F4} " +
                $"C={filteredSignedCurvature:F4} s={traveled:F3}/{plannedLandingDistance:F3} progress={progress01:F3} " +
                $"cycle={waveCycleIndex+1}/{plannedWaveCount} profile={waveIndex+1} waveLen={runtimeWaveLength:F3} " +
                $"N={Fmt(normal)} base={baseOffset:F4} residual={residualOffset:F4} " +
                $"vResidual={residualSpeed:F4} aResidual={residualAcceleration:F3} q={visualOffset:F4} " +
                $"displayQ={presentedOffset:F4} displaySpeed={presentedSpeed:F3} smoothT={presentationSmoothSeconds:F3} " +
                $"resets={visualFilterResetCount} landingPlan={landingPlanValid} " +
                $"displayCap={visibleUpperLimit:F3} guard={heightGuardActive} " +
                $"upper={upperNormal:F3} lower={lowerNormal:F3}", true);
        if (mappedSubjectGap > coordinateWarningMeters && !coordinateMismatchLogged &&
            mappingMode != "SubjectAnchorFallback")
        {
            coordinateMismatchLogged = true;
            Debug.LogWarning($"[ExampleBVE][SUBJECT_MAP_GAP] Mapped InSubject and Subject are " +
                $"{mappedSubjectGap:F4}m apart. Verify CorrespondSubject timing/refs. " +
                $"Mapped={Fmt(mappedCarrierWorld)} Subject={Fmt(subjectWorld)}", this);
        }
        else if (mappedSubjectGap <= coordinateWarningMeters) coordinateMismatchLogged = false;
        if (!outputWritable && !blockedOutputLogged)
        {
            blockedOutputLogged = true;
            Debug.LogWarning("[ExampleBVE][OUTPUT_BLOCKED] Math and diagnostics are running, " +
                "because there is no writable renderer-only display target.", this);
        }
    }

    void LateUpdate()
    {
        if (!initialized) return;

        // MainGameManager may have changed the original BallVisualEqualizer material
        // after this proxy was created. Reflect those changes on the visible proxy.
        SyncPresentationFromOriginal();
        // InSubject supplies ALL progression, Subject supplies visual coordinate frame.
        // Smooth ONLY the displayed normal-axis offset: carrier X/Z, actual Rigidbody,
        // wave phase and the raw first-Apex observation stay untouched.
        visibleUpperLimit = GetVisibleUpperLimit(inSubject.transform.position);
        guardedTargetOffset = SoftLimit(visualOffset,
            -Mathf.Max(.02f, maximumVisibleDropNormal), visibleUpperLimit);
        heightGuardActive = Mathf.Abs(guardedTargetOffset - visualOffset) > .03f;
        if (heightGuardActive && logEvents && Time.unscaledTime >= nextHeightGuardLog)
        {
            nextHeightGuardLog = Time.unscaledTime + .5f;
            Trace("DISPLAY_HEIGHT_GUARD", $"rawQ={visualOffset:F3} guardedQ={guardedTargetOffset:F3} " +
                $"upper={visibleUpperLimit:F3} lower={-maximumVisibleDropNormal:F3} " +
                $"h0={initialWaveHeight:F3} supportProbe={probeValid}");
        }
        // Sharp turn: the previously displayed scalar q belongs to the old
        // T/N coordinate frame. Do not transport its lag into the new frame.
        bool sharpTurn = resetDisplayOnSharpTurn && presentationFrameReady &&
            (Vector3.Angle(previousFilterNormal, normal) >= displayTurnResetDegrees ||
             Vector3.Angle(previousFilterTangent, tangent) >= displayTurnResetDegrees);
        if (sharpTurn)
        {
            ResetVisualFilterState(guardedTargetOffset);
            if (logEvents) Trace("VISUAL_FILTER_TURN_RESET",
                $"N={Fmt(normal)} T={Fmt(tangent)} q={guardedTargetOffset:F3}");
        }
        previousFilterNormal = normal;
        previousFilterTangent = tangent;

        if (softenVisualWave)
        {
            float wavePosition = traveled / Mathf.Max(.1f, runtimeWaveLength);
            float u = active ? wavePosition - Mathf.Floor(wavePosition) : 1f;
            if (landingPlanValid && waveCycleIndex >= plannedWaveCount - 1 && traveled >= plannedLandingDistance)
                u = 1f;
            u = Mathf.Clamp01(u);
            float distanceFromImpact = Mathf.Min(u, 1f - u);
            float blend = Mathf.SmoothStep(0f, 1f,
                distanceFromImpact / Mathf.Max(.02f, contactBlendFraction));
            float baseSeconds = Mathf.Lerp(impactSmoothSeconds, flightSmoothSeconds, blend);
            presentationSmoothSeconds = baseSeconds;
            presentedOffset = Mathf.SmoothDamp(
                presentedOffset, guardedTargetOffset, ref presentedSpeed,
                presentationSmoothSeconds, Mathf.Infinity, Time.deltaTime);
        }
        else
        {
            presentationSmoothSeconds = 0f;
            presentedOffset = guardedTargetOffset;
            presentedSpeed = 0f;
            presentationFrameReady = true;
        }
        // SmoothDamp is not a hard safety limit when the stair height changes.
        // Bound the final *presentation* too, leaving all raw wave/Apex data intact.
        float safePresented = Mathf.Clamp(presentedOffset,
            -Mathf.Max(.02f, maximumVisibleDropNormal), visibleUpperLimit);
        if (!Mathf.Approximately(safePresented, presentedOffset))
        {
            presentedOffset = safePresented;
            presentedSpeed = 0f;
        }
        // Map carrier + presentation offset together using CorrespondSubject's map.
        UpdateCoordinateDiagnostics(inSubject.transform.position, presentedOffset);
        if (outputWritable && IsSafeDisplayTarget(actualDisplayTarget))
        {
            actualDisplayTarget.position = predictedVisualWorld;
            if (followSubjectRotation && subjectRotation)
            {
                Quaternion goal = roll * subjectRotation.rotation * rotationOffset;
                float a = 1f - Mathf.Exp(-2f * Mathf.PI * rotationFollowHz * Time.deltaTime);
                actualDisplayTarget.rotation = Quaternion.Slerp(actualDisplayTarget.rotation, goal, a);
            }
            else if (addVisualRoll)
                actualDisplayTarget.rotation = roll * rotationOffset;
        }
        DiagnosticSample();
    }
}
