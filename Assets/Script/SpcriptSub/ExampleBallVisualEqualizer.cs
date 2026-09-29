using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// InSubject-only, visual-only Stairway equalizer (Unity 2022.3 / built-in API).
/// No SlopeStickCore, EnvelopeSystem, BallVisualSlopeDrive or Splines dependency.
/// Attach this component to an independent VISUAL GameObject, not InSubject.
/// Main motion authority: InSubject only. Output coordinates follow CorrespondSubject.
/// Subject is a read-only visual-space anchor / rotation reference.
/// No physics force, velocity, or Rigidbody state is written.
/// If the assigned visualTarget has a Rigidbody, only its mesh/sprite presentation
/// is copied into an independent visual proxy under the same parent. The original
/// Rigidbody, Collider, and motion scripts remain untouched.
/// </summary>
[DefaultExecutionOrder(12000)]
[DisallowMultipleComponent]
public sealed class ExampleBallVisualEqualizer : MonoBehaviour
{
    const float Eps = 0.000001f;
    const int Waves = 3;
    readonly RaycastHit[] hits = new RaycastHit[16];

    // These are *presentation* profiles. They do not change references, layer masks,
    // InSubject movement, Subject mapping, stage progress or the section length.
    public enum ShakePreset
    {
        Custom = 0,
        BalancedContact = 1,
        SoftFlow = 2,
        FineStairRattle = 3,
        FirmImpact = 4,
        CompactSafe = 5,
        ClassicImpact = 6
    }

    [Header("Shake feel: choose a preset from the Inspector dropdown")]
    [Tooltip("Custom keeps current numeric values. Selecting another item writes its suggested tuning into the fields below; you may fine-tune those fields afterwards.")]
    [SerializeField] ShakePreset shakePreset = ShakePreset.Custom;
    [SerializeField, HideInInspector] ShakePreset lastAppliedShakePreset = ShakePreset.Custom;

    [Header("Required: only primary motion injection")]
    [SerializeField] Rigidbody inSubject;
    [SerializeField] Transform visualTarget;
    [Header("Output wiring: no writes to the existing Rigidbody")]
    [SerializeField] bool autoCreateDisplayForRigidbody = true;
    [SerializeField] bool hideOriginalRendererWhileTesting = true;
    [SerializeField] Transform actualDisplayTarget;
    [SerializeField] string outputMode = "Uninitialized";
    readonly List<Renderer> originalRenderers = new List<Renderer>();
    readonly List<bool> originalRendererStates = new List<bool>();
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
    [SerializeField] bool autoEndAtSection = true;
    [SerializeField, Min(.1f)] float sectionLength = 9.9f;
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

    [Header("Read-only runtime diagnostics")]
    [SerializeField] bool active, exiting, probeValid, firstWavePeakReady;
    [SerializeField] int waveIndex = -1, firstWavePeakRevision;
    [SerializeField] float traveled, progress01, phase, currentAmplitude;
    [SerializeField] float slope0, slope1, slope2, signedCurvature, roughness, filteredRoughness;
    [SerializeField] float baseOffset, residualOffset, residualSpeed, residualAcceleration, visualOffset;
    [SerializeField] float presentedOffset, presentedSpeed, presentationSmoothSeconds;
    [SerializeField] float entryRawNormalSpeed, entryActualRiseSpeed, entryUnclampedHeight;
    [SerializeField] float visibleUpperLimit, guardedTargetOffset;
    [SerializeField] bool heightGuardActive;
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
    bool initialized, armed = true;
    int lastApexWave = -1;

    public ShakePreset SelectedShakePreset => shakePreset;

    // Unity calls OnValidate when the dropdown changes in Edit Mode or Play Mode.
    // Avoid reapplying on every Inspector refresh so manual fine-tuning is retained.
    void OnValidate() => ApplyShakePresetIfChanged();

    // Runtime API: changes the selected profile when it differs from the last
    // applied one. Keeping this one-argument overload also makes ordinary calls
    // from other scripts simple.
    public void SetShakePreset(ShakePreset value)
    {
        SetShakePreset(value, false);
    }

    // force=true reapplies a profile even when it is already selected, restoring
    // all of its tuned fields after manual/runtime changes. Custom changes only
    // the selection and deliberately preserves the numeric fields.
    public void SetShakePreset(ShakePreset value, bool force)
    {
        if ((int)value < (int)ShakePreset.Custom ||
            (int)value > (int)ShakePreset.ClassicImpact)
        {
            Debug.LogWarning($"[ExampleBVE][INVALID_SHAKE_PRESET] value={(int)value}", this);
            return;
        }

        if (!force && shakePreset == value && lastAppliedShakePreset == value)
            return;

        shakePreset = value;
        lastAppliedShakePreset = value;
        if (value != ShakePreset.Custom)
            ApplyShakePresetValues(value);
    }

    // Safe for game events or the component context menu. Reapplying Custom is
    // intentionally a no-op: the current hand-tuned Inspector values remain.
    [ContextMenu("ExampleBVE / Reapply Selected Shake Preset")]
    public void ReapplyShakePreset()
    {
        SetShakePreset(shakePreset, true);
    }

    // Integer entry point for Unity UI Button.onClick and other UnityEvents.
    // 0 Custom, 1 BalancedContact, 2 SoftFlow, 3 FineStairRattle,
    // 4 FirmImpact, 5 CompactSafe, 6 ClassicImpact.
    public void SetShakePresetByIndex(int index)
    {
        if (index < (int)ShakePreset.Custom || index > (int)ShakePreset.ClassicImpact)
        {
            Debug.LogWarning($"[ExampleBVE][INVALID_SHAKE_PRESET_INDEX] index={index}", this);
            return;
        }
        SetShakePreset((ShakePreset)index, true);
    }

    // The Inspector enum dropdown still applies a newly selected preset once;
    // its periodic OnValidate calls do not erase manual field adjustments.
    void ApplyShakePresetIfChanged()
    {
        if (shakePreset != lastAppliedShakePreset)
            SetShakePreset(shakePreset, false);
    }

    void ApplyShakePresetValues(ShakePreset value)
    {
        // Historical reference values from this Example BVE's 3-wave, roughness,
        // pullback, presentation smoothing and subsequent height-guard iterations.
        // These are trial presets, not measured optimums from a controlled A/B test.
        // Restore a complete tuning baseline first so profiles do not inherit the
        // previous profile's hidden leftovers. Keep physical/calibration fields intact.
        initialHeight = .45f;
        energyBlend = .35f;
        restitution = .86f;
        lowerRatio = .75f;
        firstApexFraction = .82f;
        laterApexFraction = .50f;
        dampingPerMeter = .02f;
        exitDamping = 9f;
        firstRoughness = .08f;
        secondRoughness = .22f;
        thirdRoughness = .12f;
        measuredRoughnessGain = .10f;
        signedRoughAccelerationGain = 90f;
        accelerationResidualGain = .18f;
        residualLowPassHz = 2f;
        springK = 420f;
        damperC = 18f;
        maximumResidual = .20f;
        maximumAcceleration = 450f;
        maximumJerk = 2500f;
        entryBlendSeconds = .10f;
        softenVisualWave = true;
        impactSmoothSeconds = .015f;
        flightSmoothSeconds = .035f;
        contactBlendFraction = .16f;
        maximumFirstWaveHeight = .65f;
        maximumVisibleLiftNormal = .52f;
        maximumVisibleDropNormal = .28f;
        maximumCenterHeightAboveGround = 1.10f;

        switch (value)
        {
            case ShakePreset.BalancedContact:
                // Current contact feel with a slightly longer first descent.
                firstApexFraction = .76f;
                springK = 360f;
                damperC = 24f;
                maximumJerk = 1800f;
                break;

            case ShakePreset.SoftFlow:
                // Longer aerial transition, less aggressive residual and texture.
                initialHeight = .40f;
                energyBlend = .25f;
                restitution = .82f;
                lowerRatio = .65f;
                firstApexFraction = .72f;
                dampingPerMeter = .035f;
                firstRoughness = .06f;
                secondRoughness = .16f;
                thirdRoughness = .09f;
                measuredRoughnessGain = .08f;
                signedRoughAccelerationGain = 65f;
                accelerationResidualGain = .13f;
                springK = 300f;
                damperC = 28f;
                maximumResidual = .16f;
                maximumAcceleration = 300f;
                maximumJerk = 1400f;
                entryBlendSeconds = .12f;
                impactSmoothSeconds = .020f;
                flightSmoothSeconds = .055f;
                contactBlendFraction = .20f;
                maximumFirstWaveHeight = .58f;
                maximumVisibleLiftNormal = .45f;
                maximumVisibleDropNormal = .24f;
                maximumCenterHeightAboveGround = 1.03f;
                break;

            case ShakePreset.FineStairRattle:
                // Emphasize measured steps and wave 2 without raising the arc.
                initialHeight = .40f;
                energyBlend = .25f;
                firstApexFraction = .77f;
                dampingPerMeter = .025f;
                firstRoughness = .10f;
                secondRoughness = .28f;
                thirdRoughness = .16f;
                measuredRoughnessGain = .16f;
                signedRoughAccelerationGain = 120f;
                accelerationResidualGain = .23f;
                residualLowPassHz = 2.4f;
                springK = 400f;
                damperC = 24f;
                maximumAcceleration = 420f;
                maximumJerk = 2100f;
                impactSmoothSeconds = .012f;
                flightSmoothSeconds = .030f;
                contactBlendFraction = .15f;
                maximumFirstWaveHeight = .60f;
                maximumVisibleLiftNormal = .50f;
                break;

            case ShakePreset.FirmImpact:
                // A snappier contact that still softens its in-flight motion.
                initialHeight = .48f;
                firstApexFraction = .78f;
                firstRoughness = .10f;
                secondRoughness = .25f;
                thirdRoughness = .14f;
                measuredRoughnessGain = .12f;
                signedRoughAccelerationGain = 110f;
                springK = 440f;
                damperC = 20f;
                maximumJerk = 2400f;
                impactSmoothSeconds = .010f;
                flightSmoothSeconds = .025f;
                break;

            case ShakePreset.CompactSafe:
                // Suppress showy separation from the stair, retain small impacts.
                initialHeight = .32f;
                energyBlend = .15f;
                restitution = .78f;
                lowerRatio = .60f;
                firstApexFraction = .74f;
                dampingPerMeter = .045f;
                firstRoughness = .06f;
                secondRoughness = .18f;
                thirdRoughness = .10f;
                measuredRoughnessGain = .09f;
                signedRoughAccelerationGain = 85f;
                accelerationResidualGain = .14f;
                springK = 370f;
                damperC = 27f;
                maximumResidual = .12f;
                maximumAcceleration = 300f;
                maximumJerk = 1450f;
                entryBlendSeconds = .12f;
                impactSmoothSeconds = .014f;
                flightSmoothSeconds = .040f;
                maximumFirstWaveHeight = .42f;
                maximumVisibleLiftNormal = .34f;
                maximumVisibleDropNormal = .20f;
                maximumCenterHeightAboveGround = .95f;
                break;

            case ShakePreset.ClassicImpact:
                // The earlier direct (harder) presentation, with today's safety
                // caps intentionally kept enabled. No physics output is changed.
                softenVisualWave = false;
                break;
        }
    }

    public Rigidbody InSubject => inSubject;
    public bool IsActive => active;
    public int WaveIndex => waveIndex;
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

    /// <summary>InSubject drives motion; Subject is read only for mapped-space anchoring/rotation.</summary>
    public void Inject(Rigidbody primary, Transform secondaryRotation = null)
    {
        ReleaseGeneratedDisplay();
        inSubject = primary;
        subjectRotation = secondaryRotation;
        if (!visualTarget) visualTarget = transform;
        ResolveCoordinateSource();
        initialized = ValidateReferences();
        if (initialized) InitializeFrame();
    }

    void Awake()
    {
        ApplyShakePresetIfChanged();
        if (!visualTarget) visualTarget = transform;
        ResolveCoordinateSource();
        initialized = ValidateReferences();
        if (initialized) InitializeFrame();
    }

    void OnEnable()
    {
        presentedOffset = visualOffset;
        presentedSpeed = 0f;
        if (Application.isPlaying && initialized && !actualDisplayTarget)
        {
            ResolveDisplayTarget();
            outputWritable = actualDisplayTarget && !actualDisplayTarget.GetComponent<Rigidbody>();
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
        if (visualTarget == inSubject.transform || visualTarget.IsChildOf(inSubject.transform))
        {
            Debug.LogError("[ExampleBVE] visualTarget must NOT be InSubject or its child: physical motion is read-only.", this);
            return false;
        }
        ResolveDisplayTarget();
        outputWritable = actualDisplayTarget && !actualDisplayTarget.GetComponent<Rigidbody>();
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


    // The user may assign the legacy dynamic BallVisualEqualizer directly. It is
    // never moved: the presentation alone is copied to a sibling in VisualPlayerRoot.
    // The renderer visibility is restored whenever this component is disabled.
    void ResolveDisplayTarget()
    {
        if (!visualTarget) { actualDisplayTarget = null; return; }
        Rigidbody oldBody = visualTarget.GetComponent<Rigidbody>();
        if (!oldBody)
        {
            ReleaseGeneratedDisplay();
            actualDisplayTarget = visualTarget;
            outputMode = "DirectRendererOnly";
            return;
        }
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
            return;
        }

        generatedDisplay = new GameObject("ExampleBVE_Display_" + visualTarget.name);
        Transform display = generatedDisplay.transform;
        display.SetParent(visualTarget.parent, false);
        display.SetPositionAndRotation(visualTarget.position, visualTarget.rotation);
        display.localScale = visualTarget.localScale;
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

    void ReleaseGeneratedDisplay()
    {
        for (int i = 0; i < originalRenderers.Count; i++)
            if (originalRenderers[i]) originalRenderers[i].enabled = originalRendererStates[i];
        originalRenderers.Clear();
        originalRendererStates.Clear();
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
        presentedOffset = presentedSpeed = presentationSmoothSeconds = 0f;
        heightGuardActive = false;
        visibleUpperLimit = maximumVisibleLiftNormal;
        guardedTargetOffset = 0f;
        active = exiting = false;
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
        traveled = progress01 = phase = elapsed = exitElapsed = flatElapsed = 0f;
        baseOffset = residualOffset = residualSpeed = residualAcceleration = visualOffset = 0f;
        presentedOffset = presentedSpeed = 0f;
        previousOffset = previousOffsetSpeed = lowPassedNormalAcceleration = 0f;
        currentAmplitude = initialWaveHeight;
        waveIndex = -1;
        lastApexWave = -1;
        firstWavePeakReady = false;
        firstWavePeakCY = firstWavePeakProgress01 = 0f;
        previousNormal = normal;
        lastBegin = Time.fixedTime;
        active = true;
        exiting = armed = false;
        Trace("BEGIN", $"p={Fmt(previousCarrier)} v={Fmt(v)} N={Fmt(normal)} " +
            $"subject={Fmt(subjectRotation ? subjectRotation.position : Vector3.zero)} " +
            $"map={mappingMode} h0={initialWaveHeight:F3} u0={initialWaveSpeed:F3}");
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
                planar.magnitude >= minimumEntryPlanarSpeed)
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
        progress01 = Mathf.Clamp01(traveled / Mathf.Max(.1f, sectionLength));
        float cycles = traveled * Waves / Mathf.Max(.1f, sectionLength);
        int oldWave = waveIndex;
        waveIndex = Mathf.Min(Waves - 1, Mathf.FloorToInt(cycles));
        float u = Mathf.Clamp01(cycles - waveIndex);
        phase = cycles * 2f * Mathf.PI;
        if (autoEndAtSection && cycles >= Waves) EndStair();

        float fade = Mathf.Exp(-dampingPerMeter * traveled - (exiting ? exitDamping * exitElapsed : 0f));
        float energyRatio = restitution * restitution;
        currentAmplitude = initialWaveHeight * Mathf.Pow(energyRatio, waveIndex) * fade;
        float priorAmplitude = initialWaveHeight * Mathf.Pow(energyRatio, Mathf.Max(0, waveIndex - 1)) * fade;
        float start = waveIndex == 0 ? 0f : -priorAmplitude * lowerRatio;
        float end = -currentAmplitude * lowerRatio;
        float apex = waveIndex == 0 ? firstApexFraction : laterApexFraction;
        float measured = Mathf.Clamp01(filteredRoughness / Mathf.Max(.01f, roughnessReference));
        float baselineRough = waveIndex == 0 ? firstRoughness : waveIndex == 1 ? secondRoughness : thirdRoughness;
        float waveRough = baselineRough + measuredRoughnessGain * measured;
        if (waveIndex != oldWave)
            Trace("WAVE", $"index={waveIndex+1} s={traveled:F3} u={u:F3} " +
                $"R={filteredRoughness:F4} signed={filteredSignedCurvature:F4} rough={waveRough:F3} " +
                $"amplitude={currentAmplitude:F4}");
        float priorImpact = Mathf.Sqrt(2f * gravityN * priorAmplitude * (1f + lowerRatio));
        float impact = Mathf.Sqrt(2f * gravityN * currentAmplitude * (1f + lowerRatio));
        float riseSpeed = waveIndex == 0 ? initialWaveSpeed : restitution * priorImpact;
        float forwardSpeed = Mathf.Max(.1f, Vector3.Dot(v, tangent));
        float period = sectionLength / (Waves * forwardSpeed);
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
        float lowerBase = -Mathf.Lerp(waveIndex == 0 ? currentAmplitude : priorAmplitude,
            currentAmplitude, Mathf.SmoothStep(0f, 1f, u)) * lowerRatio;
        upperNormal = currentAmplitude * (1f + Mathf.Abs(waveRough)) + maximumResidual;
        lowerNormal = lowerBase - maximumResidual;
        float rawOffset = baseOffset + residualOffset;
        visualOffset = Mathf.Clamp(rawOffset, lowerNormal, upperNormal);
        float entryFactor = Mathf.SmoothStep(0f, 1f, elapsed / Mathf.Max(.01f, entryBlendSeconds));
        visualOffset *= entryFactor;

        float speedN = (visualOffset - previousOffset) / dt;
        if (!firstWavePeakReady && waveIndex == 0 &&
            u >= .15f && previousOffset >= .5f * currentAmplitude &&
            previousOffsetSpeed > .02f && speedN <= 0f)
        {
            firstWavePeakReady = true;
            firstWavePeakRevision++;
            firstWavePeakCY = initialCY + previousNormal.y * previousOffset;
            firstWavePeakProgress01 = Mathf.Clamp01(progress01 - ds / Mathf.Max(.1f, sectionLength));
            previousApexPosition = MapPoint(previousCarrier + Vector3.up * initialCY + previousNormal * previousOffset);
            lastApexWave = waveIndex;
            Trace("FIRST_APEX", $"CY={firstWavePeakCY:F4} progress={firstWavePeakProgress01:F4} " +
                $"mappedWorld={Fmt(previousApexPosition)}");
        }
        else if (waveIndex != lastApexWave && previousOffsetSpeed > .02f && speedN <= 0f)
        {
            lastApexWave = waveIndex;
            Trace("APEX", $"wave={waveIndex+1} q={previousOffset:F4}");
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
                $"C={filteredSignedCurvature:F4} s={traveled:F3} u={progress01:F3} wave={waveIndex+1} " +
                $"N={Fmt(normal)} base={baseOffset:F4} residual={residualOffset:F4} " +
                $"vResidual={residualSpeed:F4} aResidual={residualAcceleration:F3} q={visualOffset:F4} " +
                $"displayQ={presentedOffset:F4} displaySpeed={presentedSpeed:F3} smoothT={presentationSmoothSeconds:F3} " +
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
        if (softenVisualWave)
        {
            float cycles = traveled * Waves / Mathf.Max(.1f, sectionLength);
            float u = active ? Mathf.Clamp01(cycles - Mathf.Max(0, waveIndex)) : 1f;
            float distanceFromImpact = Mathf.Min(u, 1f - u);
            float blend = Mathf.SmoothStep(0f, 1f,
                distanceFromImpact / Mathf.Max(.02f, contactBlendFraction));
            presentationSmoothSeconds = Mathf.Lerp(
                impactSmoothSeconds, flightSmoothSeconds, blend);
            presentedOffset = Mathf.SmoothDamp(
                presentedOffset, guardedTargetOffset, ref presentedSpeed,
                presentationSmoothSeconds, Mathf.Infinity, Time.deltaTime);
        }
        else
        {
            presentationSmoothSeconds = 0f;
            presentedOffset = guardedTargetOffset;
            presentedSpeed = 0f;
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
        if (outputWritable)
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
