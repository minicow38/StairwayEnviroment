using UnityEngine;

/// <summary>
/// EqualizerFuture vertical-boundary provider.
///
/// Strict responsibility:
/// - Owns the Inspector boundary f(0)=CY only in Physics-Y.
/// - Observes first-wave CY(s) relative to the SlopeStickCore carrier.
/// - Builds a first-wave Pre-Apex candidate from a local spatial Taylor/quadratic model.
/// - Confirms a Taylor candidate in the SAME FixedUpdate when an independent
///   velocity-derived spatial slope dCY/ds agrees with the Taylor derivative.
/// - Later spatial residual and repeated-Taylor agreement remain secondary confirmation paths.
/// - Uses the wider multi-point quadratic fit only as optional confirmation.
/// - Real apex crossing is verification/fallback only.
/// - Release is observation-only. It never gates, resets or starts the peak solver.
/// - Never owns X/Z progression.
/// - Never modifies EqualizerFutureSpline tangents, teacher correspondence, phase or curvature.
/// - Never creates a SplineContainer.
/// </summary>
[DefaultExecutionOrder(11700)]
[DisallowMultipleComponent]
public sealed class EqualizerFutureAsyncPos : MonoBehaviour
{
    private const float Eps = 0.000001f;
    private const double SolveEps = 1e-12;
    private const int PeakSampleCapacity = 8;

    [System.Serializable]
    public struct FirstWavePeakObservation
    {
        public bool valid;
        public int revision;
        public long sectionKey;
        public int splineIndex;
        public int sectionIndex;
        public float progress01;
        public float cy;

        // Diagnostic only.
        public float relativeVerticalVelocity;

        // Solver diagnostics.
        public bool solvedPreApex;
        public bool verified;
        public bool releaseObserved;

        public float sectionDistanceMeters;
        public float fitRmseMeters;
    }

    private struct PeakSample
    {
        public float progress01;
        public float s;
        public float cy;
    }

    private struct QuadraticPeakFit
    {
        public bool valid;

        // CY(x) = a*x^2 + b*x + c
        // x = s - newestSampleS
        public float a;
        public float b;
        public float c;

        public float peakS;
        public float peakCY;

        public float rmse;
        public float spanMeters;
    }

    private struct LocalApexEstimate
    {
        public bool valid;

        public float currentS;
        public float currentCY;

        // CY(s + ds)
        // ~= CY + slope*ds + 0.5*curvature*ds^2
        public float slope;
        public float curvature;

        public float peakLeadMeters;
        public float peakS;
        public float peakCY;

        // Optional broader-window confirmation.
        public bool broadFitValid;
        public float broadPeakS;
        public float broadPeakCY;
        public float broadRmse;
        public float broadPeakDistanceError;
    }

    private enum PreApexCandidateState
    {
        Searching,
        Candidate,
        Published,
        Verified
    }

    // ================================================================
    // References
    // ================================================================

    [Header("References")]
    [SerializeField]
    private EqualizerFutureSpline directionSource;

    [SerializeField]
    private BallVisualEqualizerSync equalizerSync;

    [SerializeField]
    private CorrespondSubject correspondSubject;

    [SerializeField]
    private SlopeStickCore slopeStickCore;

    // ================================================================
    // Vertical boundary
    // ================================================================

    [Header("Vertical Boundary f(0)=CY")]
    [Tooltip(
        "Vertical offset [m] from the SlopeStickCore carrier at u=0. " +
        "X/Z are never changed by this value."
    )]
    [SerializeField]
    private float initialCY = 0f;

    [Tooltip("Observe only the first wave and publish its CY peak.")]
    [SerializeField]
    private bool observeFirstWavePeak = true;

    // ================================================================
    // Spatial Pre-Apex Solver
    // ================================================================

    [Header("First Wave Spatial Pre-Apex Solver")]

    [Tooltip(
        "Number of newest spatial CY samples used by the wider confirmation fit."
    )]
    [Range(4, PeakSampleCapacity)]
    [SerializeField]
    private int peakFitWindowSamples = 6;

    [Tooltip(
        "Samples closer than this spatial distance are not added. " +
        "This avoids FixedUpdate count becoming the authority."
    )]
    [Min(0.001f)]
    [SerializeField]
    private float minimumPeakSampleSeparationMeters = 0.015f;

    [Tooltip(
        "Minimum positive dCY/ds required to recognize first-wave ascent."
    )]
    [Min(0f)]
    [SerializeField]
    private float minimumAscentSlopePerMeter = 0.02f;

    [Tooltip(
        "Pre-Apex solve is accepted only when the predicted apex is this close " +
        "or closer, expressed as a fraction of first-wave length."
    )]
    [Range(0.01f, 0.5f)]
    [SerializeField]
    private float maximumPreApexLeadFractionOfFirstWave = 0.18f;

    [Tooltip(
        "Maximum wider-fit CY fitting error [m]. " +
        "An unreliable wider fit never blocks the local solver."
    )]
    [Min(0.001f)]
    [SerializeField]
    private float maximumPeakFitRmseMeters = 0.03f;

    [Tooltip(
        "Maximum amount by which the predicted peak may exceed the highest " +
        "CY already observed [m]. Prevents unstable far extrapolation."
    )]
    [Min(0.01f)]
    [SerializeField]
    private float maximumPeakExtrapolationMeters = 0.25f;

    [Tooltip(
        "Predicted peak must be at least this much above Inspector initialCY."
    )]
    [Min(0f)]
    [SerializeField]
    private float minimumPeakRiseAboveInitialCYMeters = 0.01f;

    [Tooltip("Minimum downward broader-fit quadratic curvature magnitude.")]
    [Min(0.000001f)]
    [SerializeField]
    private float minimumConcavityMagnitude = 0.001f;

    [Tooltip(
        "If Pre-Apex solving fails, allow the real apex crossing to publish " +
        "a compatibility fallback. Apex crossing is never the primary path."
    )]
    [SerializeField]
    private bool allowPostApexFallback = true;

    // ================================================================
    // Local Taylor solver
    // ================================================================

    [Header("Local Taylor Pre-Apex Solver")]

    [Tooltip(
        "Use the newest three spatial CY samples as the primary Pre-Apex solver. " +
        "The wider quadratic fit becomes confirmation only."
    )]
    [SerializeField]
    private bool useLocalTaylorPreApex = true;

    [Tooltip(
        "Require the newest spatial CY slope to be smaller than the previous slope. " +
        "This confirms that ascent is decelerating toward the apex."
    )]
    [SerializeField]
    private bool requireDecreasingSpatialSlope = true;

    [Tooltip("Minimum amount of negative CY curvature required [1/m].")]
    [Min(0.000001f)]
    [SerializeField]
    private float minimumTaylorCurvatureMagnitude = 0.01f;

    [Tooltip(
        "When the wider fit is itself reliable, maximum allowed difference " +
        "between its peak position and the local Taylor peak."
    )]
    [Min(0.01f)]
    [SerializeField]
    private float maximumReliableFitPeakDisagreementMeters = 0.40f;

    [Tooltip(
        "If the wider fit is reliable, require agreement with the local solver."
    )]
    [SerializeField]
    private bool requireReliableBroadFitAgreement = true;

    // ================================================================
    // Release observation
    // ================================================================

    [Header("Release Observation - Diagnostic Only")]

    [Tooltip(
        "Release is recorded but never gates, resets or starts CY measurement."
    )]
    [SerializeField]
    private bool observeReleaseDiagnostics = true;

    // ================================================================
    // Debug logging
    // ================================================================

    [Header("Pre-Apex Debug")]

    [SerializeField]
    private bool logPreApexSolver = true;

    [SerializeField]
    [Min(0.01f)]
    private float preApexRejectLogIntervalSeconds = 0.08f;

    [Header("Pre-Apex Invalidation")]

    [Tooltip(
        "If measured CY exceeds the currently published Pre-Apex peak CY " +
        "by this amount, invalidate that prediction and resume solving."
    )]
    [Min(0f)]
    [SerializeField]
    private float preApexInvalidationToleranceMeters = 0.03f;

    [Header("Pre-Apex Candidate Confirmation")]

    [Tooltip(
        "Minimum spatial distance [m] from the candidate origin to a later " +
        "observation or Taylor estimate before it may confirm the candidate. " +
        "This is spatial, not frame-count based."
    )]
    [Min(0.001f)]
    [SerializeField]
    private float minimumCandidateConfirmationSeparationMeters = 0.08f;

    [Tooltip(
        "Maximum allowed change [m] in predicted peak distance between two " +
        "spatially separated candidate estimates."
    )]
    [Min(0.001f)]
    [SerializeField]
    private float candidatePeakDistanceToleranceMeters = 0.20f;

    [Tooltip(
        "Maximum allowed change [m] in predicted peak CY between two " +
        "spatially separated candidate estimates."
    )]
    [Min(0.001f)]
    [SerializeField]
    private float candidatePeakCYToleranceMeters = 0.10f;

    [Tooltip(
        "Number of consecutive stable candidate comparisons required before " +
        "the peak is published to EqualizerFutureSpline. " +
        "This remains a secondary confirmation path. Observation residual " +
        "confirmation can publish without a second Taylor estimate."
    )]
    [Range(1, 3)]
    [SerializeField]
    private int requiredStableCandidateComparisons = 1;

    [Tooltip(
        "Maximum absolute CY residual [m] between the pending Taylor candidate " +
        "curve and a later spatial observation. If the later observation is " +
        "still before the predicted apex and still ascending, a residual at " +
        "or below this value confirms the candidate."
    )]
    [Min(0.001f)]
    [SerializeField]
    private float candidateTrajectoryResidualToleranceMeters = 0.06f;

    [Tooltip(
        "Require the later confirming observation to still have positive dCY/ds. " +
        "This prevents a post-apex sample from being used to claim a Pre-Apex confirmation."
    )]
    [SerializeField]
    private bool requireAscendingObservationForCandidateConfirmation = true;

    [Header("Instantaneous Spatial-Slope Confirmation")]

    [Tooltip(
        "Primary confirmation path. Uses Rigidbody velocities from this same FixedUpdate " +
        "to measure dCY/ds without waiting for a later positional sample."
    )]
    [SerializeField]
    private bool useInstantaneousSpatialSlopeConfirmation = true;

    [Tooltip(
        "Minimum positive carrier speed ds/dt [m/s] required before velocity-derived " +
        "dCY/ds is considered numerically reliable."
    )]
    [Min(0.01f)]
    [SerializeField]
    private float minimumInstantaneousCarrierSpatialSpeed = 0.50f;

    [Tooltip(
        "Absolute tolerance for |measured dCY/ds - Taylor dCY/ds|."
    )]
    [Min(0.001f)]
    [SerializeField]
    private float instantaneousSpatialSlopeAbsoluteTolerance = 0.18f;

    [Tooltip(
        "Relative tolerance for the instantaneous slope comparison. " +
        "The effective tolerance is max(absoluteTolerance, |TaylorSlope| * relativeTolerance)."
    )]
    [Range(0f, 2f)]
    [SerializeField]
    private float instantaneousSpatialSlopeRelativeTolerance = 0.50f;

    [Tooltip(
        "After replacing the Taylor slope with the independently measured dCY/ds, " +
        "the resulting apex position must remain within this distance of the Taylor apex."
    )]
    [Min(0.01f)]
    [SerializeField]
    private float instantaneousPeakDistanceToleranceMeters = 0.18f;

    // ================================================================
    // Completion marker
    // ================================================================

    [Header("Completion Marker")]

    [SerializeField]
    private bool drawCompletionGizmo = true;

    [Min(0.005f)]
    [SerializeField]
    private float completionGizmoRadius = 0.06f;

    // ================================================================
    // READ ONLY - First Wave
    // ================================================================

    [Header("READ ONLY - First Wave Peak")]

    [SerializeField]
    private bool firstWavePeakReady;

    [SerializeField]
    private bool firstWavePeakVerified;

    [SerializeField]
    private bool firstWavePeakSolvedPreApex;

    [SerializeField]
    private int firstWavePeakRevision;

    [SerializeField]
    private long observedSectionKey = long.MinValue;

    [SerializeField]
    private float currentRelativeCY;

    [SerializeField]
    private float currentRelativeVerticalVelocity;

    [SerializeField]
    private float currentSectionDistanceMeters;

    [SerializeField]
    private float currentSpatialCYSlope;

    [SerializeField]
    private float observedFirstWavePeakCY;

    [SerializeField]
    private float observedFirstWavePeakProgress01;

    [SerializeField]
    private float predictedPeakDistanceMeters;

    [SerializeField]
    private float predictedPeakLeadMeters;

    [SerializeField]
    private float peakFitRmseMeters;

    [SerializeField]
    private float peakFitSpanMeters;

    [SerializeField]
    private bool firstWaveSawAscent;

    // ================================================================
    // READ ONLY - Local Taylor Apex
    // ================================================================

    [Header("READ ONLY - Local Taylor Apex")]

    [SerializeField]
    private bool localTaylorValid;

    [SerializeField]
    private float localTaylorSlope;

    [SerializeField]
    private float localTaylorCurvature;

    [SerializeField]
    private float localTaylorPeakLeadMeters;

    [SerializeField]
    private float localTaylorPeakDistanceMeters;

    [SerializeField]
    private float localTaylorPeakCY;

    [SerializeField]
    private bool reliableBroadFitValid;

    [SerializeField]
    private float reliableBroadPeakDistanceMeters;

    [SerializeField]
    private float reliableBroadPeakCY;

    [SerializeField]
    private float reliableBroadFitRmse;

    [SerializeField]
    private float broadVsLocalPeakDistanceError;

    [Header("READ ONLY - Pre-Apex Candidate")]

    [SerializeField]
    private PreApexCandidateState preApexCandidateState =
        PreApexCandidateState.Searching;

    [SerializeField]
    private bool preApexCandidateActive;

    [SerializeField]
    private int preApexCandidateStableComparisons;

    [SerializeField]
    private float preApexCandidateObservationS;

    [SerializeField]
    private float preApexCandidatePeakS;

    [SerializeField]
    private float preApexCandidatePeakCY;

    [SerializeField]
    private float preApexCandidatePeakLeadMeters;

    [SerializeField]
    private float preApexCandidateSlope;

    [SerializeField]
    private float preApexCandidateCurvature;

    [SerializeField]
    private float preApexCandidateLastDistanceDelta;

    [SerializeField]
    private float preApexCandidateLastCYDelta;

    [SerializeField]
    private float preApexCandidateExpectedCYAtObservation;

    [SerializeField]
    private float preApexCandidateObservationResidualMeters;

    [SerializeField]
    private float preApexCandidateExpectedSlopeAtObservation;

    [SerializeField]
    private float preApexCandidateObservedSlopeAtObservation;

    [SerializeField]
    private bool preApexCandidateConfirmedByObservation;

    [Header("READ ONLY - Instantaneous Spatial Slope")]

    [SerializeField]
    private bool instantaneousSpatialSlopeValid;

    [SerializeField]
    private float instantaneousCarrierSpatialSpeed;

    [SerializeField]
    private float instantaneousCarrierCenterVerticalVelocity;

    [SerializeField]
    private float instantaneousEqualizerVerticalVelocityPhysics;

    [SerializeField]
    private float instantaneousRelativeCYVelocity;

    [SerializeField]
    private float instantaneousMeasuredSpatialSlope;

    [SerializeField]
    private float instantaneousTaylorSlopeError;

    [SerializeField]
    private float instantaneousTaylorSlopeTolerance;

    [SerializeField]
    private float instantaneousVelocityPeakLeadMeters;

    [SerializeField]
    private float instantaneousVelocityPeakDistanceMeters;

    [SerializeField]
    private float instantaneousVelocityPeakDistanceErrorMeters;

    [SerializeField]
    private bool preApexCandidateConfirmedByInstantaneousSlope;

    // ================================================================
    // READ ONLY - Verification
    // ================================================================

    [Header("READ ONLY - Real Apex Verification")]

    [SerializeField]
    private float verifiedFirstWavePeakCY;

    [SerializeField]
    private float verifiedFirstWavePeakProgress01;

    [SerializeField]
    private float firstWavePeakVerificationErrorMeters;

    // ================================================================
    // READ ONLY - Release
    // ================================================================

    [Header("READ ONLY - Release Observation")]

    [SerializeField]
    private bool releaseObserved;

    [SerializeField]
    private bool releaseAlreadyActiveAtSectionStart;

    [SerializeField]
    private float releaseProgress01;

    [SerializeField]
    private float releaseSectionDistanceMeters;

    [SerializeField]
    private float releaseRelativeCY;

    [SerializeField]
    private float releaseRelativeVerticalVelocity;

    [SerializeField]
    private BallVisualEqualizerSync.EqualizerPhase releaseObservedPhase;

    // ================================================================
    // READ ONLY - Completion marker
    // ================================================================

    [Header("READ ONLY - Completion Marker")]

    [SerializeField]
    private bool completionPointPhysicsValid;

    [SerializeField]
    private Vector3 completionPointPhysics;

    // ================================================================
    // Runtime
    // ================================================================

    private readonly PeakSample[] peakSamples =
        new PeakSample[PeakSampleCapacity];

    private int peakSampleCount;

    private bool previousTimeSampleValid;
    private float previousRelativeCY;

    private bool previousSpatialSlopeValid;
    private float previousSpatialSlope;

    private float maximumRelativeCY;
    private float maximumRelativeCYProgress01;

    private bool previousSynchronizedStateValid;
    private bool previousSynchronizedState;

    private string lastPreApexRejectReason = "";
    private float lastPreApexRejectLogTime = -999f;

    private LocalApexEstimate preApexCandidateEstimate;

    // ================================================================
    // Public API
    // ================================================================

    public float InitialCY => initialCY;

    public bool FirstWavePeakReady =>
        firstWavePeakReady;

    public bool FirstWavePeakVerified =>
        firstWavePeakVerified;

    public bool FirstWavePeakSolvedPreApex =>
        firstWavePeakSolvedPreApex;

    public int FirstWavePeakRevision =>
        firstWavePeakRevision;

    public float ObservedFirstWavePeakCY =>
        observedFirstWavePeakCY;

    public float ObservedFirstWavePeakProgress01 =>
        observedFirstWavePeakProgress01;

    public bool ReleaseObserved =>
        releaseObserved;

    // ================================================================
    // Unity
    // ================================================================

    private void Awake()
    {
        if (!directionSource)
            directionSource = GetComponent<EqualizerFutureSpline>();

        if (!equalizerSync)
            equalizerSync = FindObjectOfType<BallVisualEqualizerSync>();

        if (!correspondSubject)
            correspondSubject = FindObjectOfType<CorrespondSubject>();

        if (!slopeStickCore)
            slopeStickCore = FindObjectOfType<SlopeStickCore>();
    }

    private void FixedUpdate()
    {
        if (!directionSource ||
            !directionSource.DirectionReady)
        {
            return;
        }

        long sectionKey =
            directionSource.ActiveSectionKey;

        if (sectionKey == long.MinValue)
            return;

        if (sectionKey != observedSectionKey)
            ResetForSection(sectionKey);

        if (observeFirstWavePeak)
            ObserveFirstWave();
    }

    // ================================================================
    // Section reset
    // ================================================================

    private void ResetForSection(long sectionKey)
    {
        observedSectionKey = sectionKey;

        firstWavePeakReady = false;
        firstWavePeakVerified = false;
        firstWavePeakSolvedPreApex = false;

        firstWaveSawAscent = false;

        currentRelativeCY = 0f;
        currentRelativeVerticalVelocity = 0f;
        currentSectionDistanceMeters = 0f;
        currentSpatialCYSlope = 0f;

        instantaneousSpatialSlopeValid = false;
        instantaneousCarrierSpatialSpeed = 0f;
        instantaneousCarrierCenterVerticalVelocity = 0f;
        instantaneousEqualizerVerticalVelocityPhysics = 0f;
        instantaneousRelativeCYVelocity = 0f;
        instantaneousMeasuredSpatialSlope = 0f;
        instantaneousTaylorSlopeError = 0f;
        instantaneousTaylorSlopeTolerance = 0f;
        instantaneousVelocityPeakLeadMeters = 0f;
        instantaneousVelocityPeakDistanceMeters = 0f;
        instantaneousVelocityPeakDistanceErrorMeters = 0f;
        preApexCandidateConfirmedByInstantaneousSlope = false;

        observedFirstWavePeakCY = 0f;
        observedFirstWavePeakProgress01 = 0f;

        predictedPeakDistanceMeters = 0f;
        predictedPeakLeadMeters = 0f;

        peakFitRmseMeters = 0f;
        peakFitSpanMeters = 0f;

        localTaylorValid = false;
        localTaylorSlope = 0f;
        localTaylorCurvature = 0f;
        localTaylorPeakLeadMeters = 0f;
        localTaylorPeakDistanceMeters = 0f;
        localTaylorPeakCY = 0f;

        reliableBroadFitValid = false;
        reliableBroadPeakDistanceMeters = 0f;
        reliableBroadPeakCY = 0f;
        reliableBroadFitRmse = 0f;
        broadVsLocalPeakDistanceError = 0f;

        ResetPreApexCandidateState(
            PreApexCandidateState.Searching
        );

        verifiedFirstWavePeakCY = 0f;
        verifiedFirstWavePeakProgress01 = 0f;
        firstWavePeakVerificationErrorMeters = 0f;

        peakSampleCount = 0;

        previousTimeSampleValid = false;
        previousRelativeCY = 0f;

        previousSpatialSlopeValid = false;
        previousSpatialSlope = 0f;

        maximumRelativeCY =
            float.NegativeInfinity;

        maximumRelativeCYProgress01 = 0f;

        releaseObserved = false;
        releaseAlreadyActiveAtSectionStart = false;
        releaseProgress01 = 0f;
        releaseSectionDistanceMeters = 0f;
        releaseRelativeCY = 0f;
        releaseRelativeVerticalVelocity = 0f;

        previousSynchronizedStateValid = false;
        previousSynchronizedState = true;

        completionPointPhysicsValid = false;
        completionPointPhysics = Vector3.zero;

        lastPreApexRejectReason = "";
        lastPreApexRejectLogTime = -999f;

        if (logPreApexSolver)
        {
            Debug.Log(
                $"[ASYNC PEAK SECTION START] " +
                $"key={sectionKey} " +
                $"spline={directionSource.ActiveSplineIndex} " +
                $"section={directionSource.ActiveSectionIndex} " +
                $"waves={directionSource.WaveCount} " +
                $"sectionLength={directionSource.SectionLength:F4}m",
                this
            );
        }
    }

    // ================================================================
    // Main observation
    // ================================================================

    private void ObserveFirstWave()
    {
        if (firstWavePeakVerified ||
            !equalizerSync ||
            !equalizerSync.Body ||
            !correspondSubject ||
            !slopeStickCore ||
            !slopeStickCore.BallVisualHasActiveSlopeFrame)
        {
            return;
        }

        int waveCount =
            Mathf.Max(
                1,
                directionSource.WaveCount
            );

        float firstWaveEnd01 =
            1f / waveCount;

        float progress01 =
            slopeStickCore.BallVisualSlopeProgress01;

        if (!IsFinite(progress01) ||
            progress01 < 0f ||
            progress01 > firstWaveEnd01 + 0.01f)
        {
            return;
        }

        float sectionLength =
            ResolveSectionLength();

        if (!IsFinite(sectionLength) ||
            sectionLength <= Eps)
        {
            return;
        }

        float firstWaveLength =
            sectionLength / waveCount;

        currentSectionDistanceMeters =
            Mathf.Clamp01(progress01) *
            sectionLength;

        if (!slopeStickCore.TryEvaluateBallVisualSectionFramePhysics(
                progress01,
                out Vector3 carrierCenterPhysics,
                out Vector3 carrierTangentPhysics,
                out _))
        {
            return;
        }

        Vector3 equalizerPositionPhysics =
            correspondSubject.InverseMapPoint(
                equalizerSync.Body.position
            );

        if (!IsFinite(equalizerPositionPhysics) ||
            !IsFinite(carrierCenterPhysics))
        {
            return;
        }

        currentRelativeCY =
            equalizerPositionPhysics.y -
            carrierCenterPhysics.y;

        if (!IsFinite(currentRelativeCY))
            return;

        // Primary same-FixedUpdate derivative observation.
        // This does NOT extrapolate the apex in time.
        // It only measures the spatial derivative dCY/ds independently
        // from the positional Taylor fit.
        UpdateInstantaneousSpatialSlopeDiagnostic(
            carrierTangentPhysics
        );

        // Time-domain finite-difference velocity remains telemetry only.
        UpdateTimeDomainDiagnostic();

        // Release is observation-only.
        ObserveReleaseEvent(
            progress01,
            currentSectionDistanceMeters
        );

        // A previously published Pre-Apex candidate may prove too low.
        // In that case unlock the solver, but keep all accumulated samples
        // and the measured maximum so the next estimate uses newer evidence.
        CheckPreApexInvalidation();
        CheckPendingPreApexCandidateInvalidation();

        // Running maximum remains useful as verification/fallback.
        if (currentRelativeCY > maximumRelativeCY)
        {
            maximumRelativeCY =
                currentRelativeCY;

            maximumRelativeCYProgress01 =
                progress01;
        }

        // Spatial sampling.
        bool sampleAdded =
            TryAppendSpatialSample(
                progress01,
                currentSectionDistanceMeters,
                currentRelativeCY,
                out float spatialSlope
            );

        if (!sampleAdded)
            return;

        currentSpatialCYSlope =
            spatialSlope;

        // Preserve the true previous slope before changing runtime state.
        float slopeBeforeCurrent =
            previousSpatialSlope;

        bool hadPreviousSpatialSlope =
            previousSpatialSlopeValid;

        bool crossedRealApex =
            firstWaveSawAscent &&
            hadPreviousSpatialSlope &&
            slopeBeforeCurrent > 0f &&
            spatialSlope <= 0f;

        // Primary candidate confirmation path:
        // A second Taylor solution is NOT required.
        // Compare the pending Taylor curve with this later spatial observation.
        // Publication is allowed only while the observation is still pre-apex.
        if (preApexCandidateActive &&
            !firstWavePeakReady &&
            !crossedRealApex)
        {
            TryConfirmPendingPreApexCandidateFromObservation(
                sectionLength,
                spatialSlope
            );
        }

        // Secondary path:
        // if the observation did not confirm the pending candidate, another
        // Taylor estimate may still update/confirm it by estimate stability.
        if (!firstWavePeakReady)
        {
            if (TrySolvePreApexPeak(
                    firstWaveLength,
                    sectionLength,
                    out LocalApexEstimate estimate))
            {
                UpdatePreApexCandidate(
                    estimate,
                    sectionLength
                );
            }
        }

        // Update history only after the Pre-Apex solve used it.
        previousSpatialSlope =
            spatialSlope;

        previousSpatialSlopeValid =
            peakSampleCount >= 2;

        // Real apex is verification/fallback only.
        if (crossedRealApex)
        {
            if (preApexCandidateActive &&
                !firstWavePeakReady)
            {
                RejectPendingPreApexCandidate(
                    "RealApexReachedBeforeConfirmation",
                    $"candidatePeakS={preApexCandidatePeakS:F4}m " +
                    $"candidatePeakCY={preApexCandidatePeakCY:F4}m " +
                    $"observedMaxCY={maximumRelativeCY:F4}m"
                );
            }

            Debug.Log(
                $"[ASYNC PEAK REAL APEX CROSSED] " +
                $"progress={progress01:F4} " +
                $"s={currentSectionDistanceMeters:F4}m " +
                $"CY={currentRelativeCY:F4}m " +
                $"previousSlope={slopeBeforeCurrent:F5} " +
                $"currentSlope={spatialSlope:F5} " +
                $"maxCY={maximumRelativeCY:F4}m " +
                $"preApexReady={firstWavePeakReady} " +
                $"preApexSolved={firstWavePeakSolvedPreApex}",
                this
            );

            ResolveRealApexCrossing(
                firstWaveLength,
                sectionLength
            );
        }
    }

    // ================================================================
    // Same-FixedUpdate spatial derivative diagnostic
    // ================================================================

    private void UpdateInstantaneousSpatialSlopeDiagnostic(
        Vector3 carrierTangentPhysics)
    {
        instantaneousSpatialSlopeValid = false;

        instantaneousCarrierSpatialSpeed = 0f;
        instantaneousCarrierCenterVerticalVelocity = 0f;
        instantaneousEqualizerVerticalVelocityPhysics = 0f;
        instantaneousRelativeCYVelocity = 0f;
        instantaneousMeasuredSpatialSlope = 0f;

        if (!useInstantaneousSpatialSlopeConfirmation ||
            !equalizerSync ||
            !equalizerSync.Body ||
            !slopeStickCore ||
            !slopeStickCore.Body ||
            !correspondSubject)
        {
            return;
        }

        if (!IsFinite(carrierTangentPhysics) ||
            carrierTangentPhysics.sqrMagnitude <= Eps)
        {
            return;
        }

        Vector3 tangent =
            carrierTangentPhysics.normalized;

        // Equalizer Rigidbody lives in the mapped/visual frame.
        // Convert its velocity back into the unrotated Physics frame.
        Vector3 equalizerVelocityPhysics =
            correspondSubject.InverseMapDirection(
                equalizerSync.Body.velocity
            );

        // SlopeStickCore.Body already lives in the Physics frame.
        Vector3 carrierBodyVelocityPhysics =
            slopeStickCore.Body.velocity;

        if (!IsFinite(equalizerVelocityPhysics) ||
            !IsFinite(carrierBodyVelocityPhysics))
        {
            return;
        }

        // s is owned by SlopeStickCore progression.
        // Therefore ds/dt is the carrier body's velocity projected on the
        // current canonical spline tangent, not the Equalizer's own speed.
        float dsdt =
            Vector3.Dot(
                carrierBodyVelocityPhysics,
                tangent
            );

        if (!IsFinite(dsdt) ||
            dsdt <
            minimumInstantaneousCarrierSpatialSpeed)
        {
            return;
        }

        // CY(s) = EqualizerY - CarrierCenterY.
        // CarrierCenter is evaluated on the spline centerline, so its
        // instantaneous Y velocity is tangent.y * ds/dt.
        float carrierCenterYVelocity =
            tangent.y *
            dsdt;

        float equalizerYVelocity =
            equalizerVelocityPhysics.y;

        float relativeCYVelocity =
            equalizerYVelocity -
            carrierCenterYVelocity;

        float measuredSpatialSlope =
            relativeCYVelocity /
            dsdt;

        if (!IsFinite(carrierCenterYVelocity) ||
            !IsFinite(equalizerYVelocity) ||
            !IsFinite(relativeCYVelocity) ||
            !IsFinite(measuredSpatialSlope))
        {
            return;
        }

        instantaneousCarrierSpatialSpeed =
            dsdt;

        instantaneousCarrierCenterVerticalVelocity =
            carrierCenterYVelocity;

        instantaneousEqualizerVerticalVelocityPhysics =
            equalizerYVelocity;

        instantaneousRelativeCYVelocity =
            relativeCYVelocity;

        instantaneousMeasuredSpatialSlope =
            measuredSpatialSlope;

        instantaneousSpatialSlopeValid = true;
    }

    // ================================================================
    // Time diagnostic only
    // ================================================================

    private void UpdateTimeDomainDiagnostic()
    {
        if (!previousTimeSampleValid)
        {
            previousTimeSampleValid = true;
            previousRelativeCY =
                currentRelativeCY;

            currentRelativeVerticalVelocity = 0f;
            return;
        }

        currentRelativeVerticalVelocity =
            (currentRelativeCY - previousRelativeCY) /
            Mathf.Max(
                Time.fixedDeltaTime,
                Eps
            );

        previousRelativeCY =
            currentRelativeCY;
    }

    // ================================================================
    // Release observation only
    // ================================================================

    private void ObserveReleaseEvent(
        float progress01,
        float sectionDistanceMeters)
    {
        if (!observeReleaseDiagnostics ||
            !equalizerSync)
        {
            return;
        }

        bool synchronized =
            equalizerSync.IsSynchronized;

        if (!previousSynchronizedStateValid)
        {
            previousSynchronizedStateValid = true;
            previousSynchronizedState =
                synchronized;

            releaseAlreadyActiveAtSectionStart =
                !synchronized;

            return;
        }

        bool releasedNow =
            previousSynchronizedState &&
            !synchronized;

        previousSynchronizedState =
            synchronized;

        if (!releasedNow ||
            releaseObserved)
        {
            return;
        }

        releaseObserved = true;

        releaseProgress01 =
            progress01;

        releaseSectionDistanceMeters =
            sectionDistanceMeters;

        releaseRelativeCY =
            currentRelativeCY;

        releaseRelativeVerticalVelocity =
            currentRelativeVerticalVelocity;

        releaseObservedPhase =
            equalizerSync.Phase;

        Debug.Log(
            $"[EQUALIZER FUTURE RELEASE OBSERVED] " +
            $"section={directionSource.ActiveSectionIndex} " +
            $"progress={releaseProgress01:F4} " +
            $"s={releaseSectionDistanceMeters:F4}m " +
            $"CY={releaseRelativeCY:F4}m " +
            $"phase={releaseObservedPhase} " +
            $"gate=False reset=False",
            this
        );
    }

    // ================================================================
    // Spatial samples
    // ================================================================

    private bool TryAppendSpatialSample(
        float progress01,
        float s,
        float cy,
        out float spatialSlope)
    {
        spatialSlope = 0f;

        if (!IsFinite(s) ||
            !IsFinite(cy))
        {
            return false;
        }

        if (peakSampleCount > 0)
        {
            PeakSample previous =
                peakSamples[peakSampleCount - 1];

            float ds =
                s - previous.s;

            // Never use reverse or duplicated progress as a peak sample.
            if (ds <= Eps)
                return false;

            if (ds <
                minimumPeakSampleSeparationMeters)
            {
                return false;
            }

            spatialSlope =
                (cy - previous.cy) /
                ds;

            if (!IsFinite(spatialSlope))
                return false;
        }

        AppendPeakSample(
            new PeakSample
            {
                progress01 = progress01,
                s = s,
                cy = cy
            }
        );

        if (peakSampleCount >= 2 &&
            !firstWaveSawAscent &&
            spatialSlope >
            minimumAscentSlopePerMeter)
        {
            firstWaveSawAscent = true;

            // Remove old flat/pre-entry history from the local fit,
            // but keep immediate history for local curvature.
            KeepNewestPeakSamples(3);

            if (logPreApexSolver)
            {
                Debug.Log(
                    $"[ASYNC PEAK ASCENT DETECTED] " +
                    $"progress={progress01:F4} " +
                    $"s={s:F4}m " +
                    $"CY={cy:F4}m " +
                    $"dCYds={spatialSlope:F5}",
                    this
                );
            }
        }

        return true;
    }

    private void AppendPeakSample(
        PeakSample sample)
    {
        if (peakSampleCount <
            PeakSampleCapacity)
        {
            peakSamples[peakSampleCount] =
                sample;

            peakSampleCount++;
            return;
        }

        for (int i = 1;
             i < PeakSampleCapacity;
             i++)
        {
            peakSamples[i - 1] =
                peakSamples[i];
        }

        peakSamples[
            PeakSampleCapacity - 1] =
            sample;
    }

    private void KeepNewestPeakSamples(
        int requestedCount)
    {
        int keep =
            Mathf.Clamp(
                requestedCount,
                1,
                peakSampleCount
            );

        int source =
            peakSampleCount - keep;

        for (int i = 0;
             i < keep;
             i++)
        {
            peakSamples[i] =
                peakSamples[source + i];
        }

        peakSampleCount =
            keep;
    }

    // ================================================================
    // Pre-Apex solve
    // ================================================================

    private bool TrySolvePreApexPeak(
        float firstWaveLength,
        float sectionLength,
        out LocalApexEstimate estimate)
    {
        estimate = default;

        if (!firstWaveSawAscent)
        {
            LogPreApexReject(
                "NoAscent",
                $"dCYds={currentSpatialCYSlope:F5}"
            );

            return false;
        }

        if (!useLocalTaylorPreApex)
            return false;

        // Main authority:
        // newest 3 spatial CY samples only.
        if (!TryBuildLocalTaylorEstimate(
                firstWaveLength,
                out estimate))
        {
            return false;
        }

        localTaylorValid = true;
        localTaylorSlope = estimate.slope;
        localTaylorCurvature = estimate.curvature;
        localTaylorPeakLeadMeters =
            estimate.peakLeadMeters;
        localTaylorPeakDistanceMeters =
            estimate.peakS;
        localTaylorPeakCY =
            estimate.peakCY;

        // Optional wider-window fit.
        // Important: an unreliable broad fit NEVER blocks the local solver.
        if (TryGetReliableBroadPeakFit(
                firstWaveLength,
                out QuadraticPeakFit broadFit))
        {
            estimate.broadFitValid = true;
            estimate.broadPeakS =
                broadFit.peakS;
            estimate.broadPeakCY =
                broadFit.peakCY;
            estimate.broadRmse =
                broadFit.rmse;

            estimate.broadPeakDistanceError =
                Mathf.Abs(
                    broadFit.peakS -
                    estimate.peakS
                );

            reliableBroadFitValid = true;

            reliableBroadPeakDistanceMeters =
                broadFit.peakS;

            reliableBroadPeakCY =
                broadFit.peakCY;

            reliableBroadFitRmse =
                broadFit.rmse;

            broadVsLocalPeakDistanceError =
                estimate.broadPeakDistanceError;

            if (requireReliableBroadFitAgreement &&
                estimate.broadPeakDistanceError >
                maximumReliableFitPeakDisagreementMeters)
            {
                LogPreApexReject(
                    "ReliableFitDisagreement",
                    $"localPeakS={estimate.peakS:F4}m " +
                    $"broadPeakS={broadFit.peakS:F4}m " +
                    $"difference={estimate.broadPeakDistanceError:F4}m " +
                    $"limit={maximumReliableFitPeakDisagreementMeters:F4}m " +
                    $"broadRmse={broadFit.rmse:F5}m"
                );

                return false;
            }
        }
        else
        {
            reliableBroadFitValid = false;
            reliableBroadPeakDistanceMeters = 0f;
            reliableBroadPeakCY = 0f;
            reliableBroadFitRmse = 0f;
            broadVsLocalPeakDistanceError = 0f;
        }

        float peakProgress01 =
            estimate.peakS /
            Mathf.Max(
                sectionLength,
                Eps
            );

        float firstWaveEnd01 =
            firstWaveLength /
            Mathf.Max(
                sectionLength,
                Eps
            );

        if (peakProgress01 <= 0f ||
            peakProgress01 >= firstWaveEnd01)
        {
            LogPreApexReject(
                "PeakProgressOutsideFirstWave",
                $"peakProgress={peakProgress01:F4} " +
                $"firstWaveEnd={firstWaveEnd01:F4}"
            );

            return false;
        }

        return true;
    }

    private bool TryBuildLocalTaylorEstimate(
        float firstWaveLength,
        out LocalApexEstimate estimate)
    {
        estimate = default;

        if (peakSampleCount < 3)
        {
            LogPreApexReject(
                "NotEnoughLocalSamples",
                $"count={peakSampleCount} required=3"
            );

            return false;
        }

        int i0 = peakSampleCount - 3;
        int i1 = peakSampleCount - 2;
        int i2 = peakSampleCount - 1;

        PeakSample p0 =
            peakSamples[i0];

        PeakSample p1 =
            peakSamples[i1];

        PeakSample p2 =
            peakSamples[i2];

        float ds01 =
            p1.s - p0.s;

        float ds12 =
            p2.s - p1.s;

        if (ds01 <= Eps ||
            ds12 <= Eps)
        {
            LogPreApexReject(
                "InvalidSpatialSpacing",
                $"ds01={ds01:F5}m " +
                $"ds12={ds12:F5}m"
            );

            return false;
        }

        float slope01 =
            (p1.cy - p0.cy) /
            ds01;

        float slope12 =
            (p2.cy - p1.cy) /
            ds12;

        if (!IsFinite(slope01) ||
            !IsFinite(slope12))
        {
            return false;
        }

        // We still have to be rising at the latest observed interval.
        if (slope12 <= 0f)
        {
            LogPreApexReject(
                "NoLongerAscending",
                $"slope01={slope01:F5} " +
                $"slope12={slope12:F5}"
            );

            return false;
        }

        // Approaching an apex means the positive slope should be shrinking.
        if (requireDecreasingSpatialSlope &&
            slope12 >= slope01)
        {
            LogPreApexReject(
                "SlopeNotDecelerating",
                $"previousSlope={slope01:F5} " +
                $"currentSlope={slope12:F5}"
            );

            return false;
        }

        // Fit only the newest three points.
        if (!TryFitQuadratic(
                i0,
                3,
                out QuadraticPeakFit localFit))
        {
            LogPreApexReject(
                "LocalQuadraticFailed",
                $"s0={p0.s:F4} " +
                $"s1={p1.s:F4} " +
                $"s2={p2.s:F4}"
            );

            return false;
        }

        // Since TryFitQuadratic centers x=0 on p2:
        // dCY/ds at current sample = b
        // d2CY/ds2 = 2a
        float slope =
            localFit.b;

        float curvature =
            2f * localFit.a;

        if (!IsFinite(slope) ||
            !IsFinite(curvature))
        {
            return false;
        }

        if (slope <= 0f)
        {
            LogPreApexReject(
                "TaylorSlopeNotPositive",
                $"slope={slope:F6} " +
                $"secantSlope={slope12:F6}"
            );

            return false;
        }

        if (curvature >=
            -minimumTaylorCurvatureMagnitude)
        {
            LogPreApexReject(
                "TaylorCurvatureTooWeak",
                $"curvature={curvature:F6} " +
                $"required<={-minimumTaylorCurvatureMagnitude:F6}"
            );

            return false;
        }

        // Apex from local Taylor model:
        // 0 = slope + curvature * ds
        float peakLead =
            -slope /
            curvature;

        if (!IsFinite(peakLead) ||
            peakLead <= Eps)
        {
            LogPreApexReject(
                "TaylorPeakNotAhead",
                $"slope={slope:F6} " +
                $"curvature={curvature:F6} " +
                $"lead={peakLead:F4}m"
            );

            return false;
        }

        float maximumLead =
            firstWaveLength *
            maximumPreApexLeadFractionOfFirstWave;

        if (peakLead >
            maximumLead)
        {
            LogPreApexReject(
                "TaylorPeakTooFarAhead",
                $"lead={peakLead:F4}m " +
                $"maxLead={maximumLead:F4}m"
            );

            return false;
        }

        float peakS =
            p2.s +
            peakLead;

        if (peakS <= p2.s ||
            peakS >= firstWaveLength)
        {
            LogPreApexReject(
                "TaylorPeakOutsideFirstWave",
                $"currentS={p2.s:F4}m " +
                $"peakS={peakS:F4}m " +
                $"firstWaveLength={firstWaveLength:F4}m"
            );

            return false;
        }

        // Taylor prediction of CY at the apex.
        float peakCY =
            p2.cy +
            slope * peakLead +
            0.5f *
            curvature *
            peakLead *
            peakLead;

        if (!IsFinite(peakCY))
            return false;

        if (peakCY <
            initialCY +
            minimumPeakRiseAboveInitialCYMeters)
        {
            LogPreApexReject(
                "TaylorPeakCYTooLow",
                $"peakCY={peakCY:F4}m " +
                $"initialCY={initialCY:F4}m"
            );

            return false;
        }

        if (peakCY <
            maximumRelativeCY -
            0.001f)
        {
            LogPreApexReject(
                "TaylorPeakBelowObservedMaximum",
                $"peakCY={peakCY:F4}m " +
                $"observedMax={maximumRelativeCY:F4}m"
            );

            return false;
        }

        float extraHeight =
            peakCY -
            maximumRelativeCY;

        if (extraHeight >
            maximumPeakExtrapolationMeters)
        {
            LogPreApexReject(
                "TaylorPeakExtrapolationTooLarge",
                $"peakCY={peakCY:F4}m " +
                $"observedMax={maximumRelativeCY:F4}m " +
                $"extra={extraHeight:F4}m"
            );

            return false;
        }

        estimate =
            new LocalApexEstimate
            {
                valid = true,

                currentS =
                    p2.s,

                currentCY =
                    p2.cy,

                slope =
                    slope,

                curvature =
                    curvature,

                peakLeadMeters =
                    peakLead,

                peakS =
                    peakS,

                peakCY =
                    peakCY,

                broadFitValid =
                    false
            };

        if (logPreApexSolver)
        {
            Debug.Log(
                $"[ASYNC PEAK LOCAL TAYLOR] " +
                $"section={directionSource.ActiveSectionIndex} " +
                $"currentS={p2.s:F4}m " +
                $"currentCY={p2.cy:F4}m " +
                $"slope01={slope01:F5} " +
                $"slope12={slope12:F5} " +
                $"slope={slope:F5} " +
                $"curvature={curvature:F5} " +
                $"peakLead={peakLead:F4}m " +
                $"peakS={peakS:F4}m " +
                $"peakCY={peakCY:F4}m",
                this
            );
        }

        return true;
    }

    private bool TryGetReliableBroadPeakFit(
        float firstWaveLength,
        out QuadraticPeakFit fit)
    {
        fit = default;

        int requestedWindow =
            Mathf.Clamp(
                peakFitWindowSamples,
                4,
                PeakSampleCapacity
            );

        int count =
            Mathf.Min(
                requestedWindow,
                peakSampleCount
            );

        if (count < 4)
            return false;

        int start =
            peakSampleCount -
            count;

        if (!TryFitQuadratic(
                start,
                count,
                out fit))
        {
            return false;
        }

        if (fit.a >=
            -minimumConcavityMagnitude)
        {
            return false;
        }

        if (fit.rmse >
            maximumPeakFitRmseMeters)
        {
            return false;
        }

        PeakSample current =
            peakSamples[
                peakSampleCount - 1];

        if (fit.peakS <=
            current.s)
        {
            return false;
        }

        if (fit.peakS >=
            firstWaveLength)
        {
            return false;
        }

        if (!IsFinite(fit.peakCY))
            return false;

        return true;
    }

    private void UpdatePreApexCandidate(
        LocalApexEstimate estimate,
        float sectionLength)
    {
        if (!estimate.valid ||
            firstWavePeakVerified)
        {
            return;
        }

        if (!preApexCandidateActive)
        {
            SetPreApexCandidate(
                estimate,
                stableComparisons: 0
            );

            Debug.Log(
                $"[ASYNC PEAK CANDIDATE] " +
                $"section={directionSource.ActiveSectionIndex} " +
                $"currentS={estimate.currentS:F4}m " +
                $"peakS={estimate.peakS:F4}m " +
                $"peakCY={estimate.peakCY:F4}m " +
                $"lead={estimate.peakLeadMeters:F4}m " +
                $"stable=0/{requiredStableCandidateComparisons}",
                this
            );

            // PRIMARY CONFIRMATION:
            // no later FixedUpdate / positional sample is required.
            if (TryConfirmPreApexCandidateFromInstantaneousSpatialSlope(
                    estimate,
                    sectionLength))
            {
                return;
            }

            return;
        }

        // A new Taylor estimate also gets the same-FixedUpdate independent
        // velocity check before we fall back to estimate-to-estimate stability.
        if (TryConfirmPreApexCandidateFromInstantaneousSpatialSlope(
                estimate,
                sectionLength))
        {
            return;
        }

        float observationSeparation =
            estimate.currentS -
            preApexCandidateObservationS;

        if (observationSeparation <
            minimumCandidateConfirmationSeparationMeters)
        {
            return;
        }

        float peakDistanceDelta =
            Mathf.Abs(
                estimate.peakS -
                preApexCandidatePeakS
            );

        float peakCYDelta =
            Mathf.Abs(
                estimate.peakCY -
                preApexCandidatePeakCY
            );

        preApexCandidateLastDistanceDelta =
            peakDistanceDelta;

        preApexCandidateLastCYDelta =
            peakCYDelta;

        bool stable =
            peakDistanceDelta <=
                candidatePeakDistanceToleranceMeters &&
            peakCYDelta <=
                candidatePeakCYToleranceMeters;

        int nextStableComparisons =
            stable
                ? preApexCandidateStableComparisons + 1
                : 0;

        Debug.Log(
            $"[ASYNC PEAK CANDIDATE UPDATED] " +
            $"section={directionSource.ActiveSectionIndex} " +
            $"stable={stable} " +
            $"observationSeparation={observationSeparation:F4}m " +
            $"peakS={estimate.peakS:F4}m " +
            $"previousPeakS={preApexCandidatePeakS:F4}m " +
            $"deltaPeakS={peakDistanceDelta:F4}m " +
            $"limitPeakS={candidatePeakDistanceToleranceMeters:F4}m " +
            $"peakCY={estimate.peakCY:F4}m " +
            $"previousPeakCY={preApexCandidatePeakCY:F4}m " +
            $"deltaPeakCY={peakCYDelta:F4}m " +
            $"limitPeakCY={candidatePeakCYToleranceMeters:F4}m " +
            $"stableCount={nextStableComparisons}/{requiredStableCandidateComparisons}",
            this
        );

        SetPreApexCandidate(
            estimate,
            nextStableComparisons
        );

        if (!stable)
            return;

        if (preApexCandidateStableComparisons <
            requiredStableCandidateComparisons)
        {
            return;
        }

        Debug.Log(
            $"[ASYNC PEAK CANDIDATE CONFIRMED] " +
            $"section={directionSource.ActiveSectionIndex} " +
            $"peakS={estimate.peakS:F4}m " +
            $"peakCY={estimate.peakCY:F4}m " +
            $"stableCount={preApexCandidateStableComparisons} " +
            $"publish=True",
            this
        );

        preApexCandidateActive = false;
        preApexCandidateState =
            PreApexCandidateState.Published;

        PublishPreApexPeak(
            estimate,
            sectionLength
        );
    }

    private void SetPreApexCandidate(
        LocalApexEstimate estimate,
        int stableComparisons)
    {
        preApexCandidateEstimate =
            estimate;

        preApexCandidateActive = true;

        preApexCandidateState =
            PreApexCandidateState.Candidate;

        preApexCandidateStableComparisons =
            Mathf.Max(
                0,
                stableComparisons
            );

        preApexCandidateObservationS =
            estimate.currentS;

        preApexCandidatePeakS =
            estimate.peakS;

        preApexCandidatePeakCY =
            estimate.peakCY;

        preApexCandidatePeakLeadMeters =
            estimate.peakLeadMeters;

        preApexCandidateSlope =
            estimate.slope;

        preApexCandidateCurvature =
            estimate.curvature;
    }

    private bool TryConfirmPreApexCandidateFromInstantaneousSpatialSlope(
        LocalApexEstimate estimate,
        float sectionLength)
    {
        if (!useInstantaneousSpatialSlopeConfirmation ||
            !estimate.valid ||
            firstWavePeakReady ||
            firstWavePeakVerified)
        {
            return false;
        }

        if (!instantaneousSpatialSlopeValid)
        {
            if (logPreApexSolver)
            {
                Debug.Log(
                    $"[ASYNC PEAK INSTANT SLOPE] " +
                    $"section={directionSource.ActiveSectionIndex} " +
                    $"valid=False " +
                    $"reason=VelocityMeasurementUnavailable " +
                    $"carrierSpeed={instantaneousCarrierSpatialSpeed:F4}m/s",
                    this
                );
            }

            return false;
        }

        float measuredSlope =
            instantaneousMeasuredSpatialSlope;

        float taylorSlope =
            estimate.slope;

        if (!IsFinite(measuredSlope) ||
            !IsFinite(taylorSlope) ||
            !IsFinite(estimate.curvature) ||
            estimate.curvature >=
                -minimumTaylorCurvatureMagnitude)
        {
            return false;
        }

        float slopeError =
            Mathf.Abs(
                measuredSlope -
                taylorSlope
            );

        float slopeTolerance =
            Mathf.Max(
                instantaneousSpatialSlopeAbsoluteTolerance,
                Mathf.Abs(taylorSlope) *
                instantaneousSpatialSlopeRelativeTolerance
            );

        instantaneousTaylorSlopeError =
            slopeError;

        instantaneousTaylorSlopeTolerance =
            slopeTolerance;

        bool measuredStillAscending =
            measuredSlope > 0f;

        bool slopeAgrees =
            measuredStillAscending &&
            slopeError <= slopeTolerance;

        float velocityPeakLead =
            float.PositiveInfinity;

        float velocityPeakS =
            float.PositiveInfinity;

        float peakDistanceError =
            float.PositiveInfinity;

        bool velocityPeakValid = false;

        if (measuredStillAscending)
        {
            velocityPeakLead =
                -measuredSlope /
                estimate.curvature;

            velocityPeakS =
                estimate.currentS +
                velocityPeakLead;

            peakDistanceError =
                Mathf.Abs(
                    velocityPeakS -
                    estimate.peakS
                );

            int waveCount =
                Mathf.Max(
                    1,
                    directionSource.WaveCount
                );

            float firstWaveLength =
                sectionLength /
                waveCount;

            float maxLead =
                firstWaveLength *
                maximumPreApexLeadFractionOfFirstWave;

            velocityPeakValid =
                IsFinite(velocityPeakLead) &&
                IsFinite(velocityPeakS) &&
                velocityPeakLead > 0f &&
                velocityPeakLead <= maxLead &&
                velocityPeakS > estimate.currentS &&
                velocityPeakS < firstWaveLength;
        }

        instantaneousVelocityPeakLeadMeters =
            IsFinite(velocityPeakLead)
                ? velocityPeakLead
                : 0f;

        instantaneousVelocityPeakDistanceMeters =
            IsFinite(velocityPeakS)
                ? velocityPeakS
                : 0f;

        instantaneousVelocityPeakDistanceErrorMeters =
            IsFinite(peakDistanceError)
                ? peakDistanceError
                : 0f;

        bool peakPositionAgrees =
            velocityPeakValid &&
            peakDistanceError <=
                instantaneousPeakDistanceToleranceMeters;

        bool accepted =
            slopeAgrees &&
            peakPositionAgrees;

        if (logPreApexSolver)
        {
            Debug.Log(
                $"[ASYNC PEAK INSTANT SLOPE] " +
                $"section={directionSource.ActiveSectionIndex} " +
                $"valid=True " +
                $"carrierSpeed={instantaneousCarrierSpatialSpeed:F4}m/s " +
                $"relativeCYVelocity={instantaneousRelativeCYVelocity:F4}m/s " +
                $"measuredSlope={measuredSlope:F5} " +
                $"taylorSlope={taylorSlope:F5} " +
                $"slopeError={slopeError:F5} " +
                $"slopeLimit={slopeTolerance:F5} " +
                $"velocityPeakLead={instantaneousVelocityPeakLeadMeters:F4}m " +
                $"taylorPeakLead={estimate.peakLeadMeters:F4}m " +
                $"velocityPeakS={instantaneousVelocityPeakDistanceMeters:F4}m " +
                $"taylorPeakS={estimate.peakS:F4}m " +
                $"peakError={instantaneousVelocityPeakDistanceErrorMeters:F4}m " +
                $"peakLimit={instantaneousPeakDistanceToleranceMeters:F4}m " +
                $"accepted={accepted}",
                this
            );
        }

        if (!accepted)
            return false;

        preApexCandidateConfirmedByInstantaneousSlope =
            true;

        Debug.Log(
            $"[ASYNC PEAK CANDIDATE CONFIRMED] " +
            $"mode=InstantaneousSpatialSlope " +
            $"section={directionSource.ActiveSectionIndex} " +
            $"currentS={estimate.currentS:F4}m " +
            $"peakS={estimate.peakS:F4}m " +
            $"peakCY={estimate.peakCY:F4}m " +
            $"measuredSlope={measuredSlope:F5} " +
            $"taylorSlope={taylorSlope:F5} " +
            $"slopeError={slopeError:F5} " +
            $"velocityPeakS={velocityPeakS:F4}m " +
            $"peakError={peakDistanceError:F4}m " +
            $"publish=True",
            this
        );

        preApexCandidateActive = false;
        preApexCandidateState =
            PreApexCandidateState.Published;

        PublishPreApexPeak(
            estimate,
            sectionLength
        );

        return true;
    }

    private bool TryConfirmPendingPreApexCandidateFromObservation(
        float sectionLength,
        float observedSpatialSlope)
    {
        if (!preApexCandidateActive ||
            firstWavePeakReady ||
            firstWavePeakVerified)
        {
            return false;
        }

        LocalApexEstimate candidate =
            preApexCandidateEstimate;

        if (!candidate.valid)
            return false;

        float ds =
            currentSectionDistanceMeters -
            candidate.currentS;

        if (!IsFinite(ds) ||
            ds <
            minimumCandidateConfirmationSeparationMeters)
        {
            return false;
        }

        float expectedCY =
            candidate.currentCY +
            candidate.slope * ds +
            0.5f *
            candidate.curvature *
            ds *
            ds;

        float expectedSlope =
            candidate.slope +
            candidate.curvature *
            ds;

        if (!IsFinite(expectedCY) ||
            !IsFinite(expectedSlope) ||
            !IsFinite(observedSpatialSlope))
        {
            RejectPendingPreApexCandidate(
                "ObservationPredictionNonFinite",
                $"ds={ds:F4}m"
            );

            return false;
        }

        float residual =
            Mathf.Abs(
                currentRelativeCY -
                expectedCY
            );

        preApexCandidateExpectedCYAtObservation =
            expectedCY;

        preApexCandidateObservationResidualMeters =
            residual;

        preApexCandidateExpectedSlopeAtObservation =
            expectedSlope;

        preApexCandidateObservedSlopeAtObservation =
            observedSpatialSlope;

        bool candidatePeakStillAhead =
            currentSectionDistanceMeters <
            candidate.peakS;

        bool observationAscending =
            observedSpatialSlope > 0f;

        bool residualAccepted =
            residual <=
            candidateTrajectoryResidualToleranceMeters;

        Debug.Log(
            $"[ASYNC PEAK CANDIDATE OBSERVATION] " +
            $"section={directionSource.ActiveSectionIndex} " +
            $"currentS={currentSectionDistanceMeters:F4}m " +
            $"candidateOriginS={candidate.currentS:F4}m " +
            $"candidatePeakS={candidate.peakS:F4}m " +
            $"currentCY={currentRelativeCY:F4}m " +
            $"expectedCY={expectedCY:F4}m " +
            $"residual={residual:F4}m " +
            $"residualLimit={candidateTrajectoryResidualToleranceMeters:F4}m " +
            $"observedSlope={observedSpatialSlope:F5} " +
            $"expectedSlope={expectedSlope:F5} " +
            $"peakAhead={candidatePeakStillAhead} " +
            $"ascending={observationAscending}",
            this
        );

        // The local Taylor trajectory no longer matches the observed path.
        // Reject before a bad peak can ever be published.
        if (!residualAccepted)
        {
            RejectPendingPreApexCandidate(
                "TrajectoryResidualTooLarge",
                $"currentS={currentSectionDistanceMeters:F4}m " +
                $"currentCY={currentRelativeCY:F4}m " +
                $"expectedCY={expectedCY:F4}m " +
                $"residual={residual:F4}m " +
                $"limit={candidateTrajectoryResidualToleranceMeters:F4}m"
            );

            return false;
        }

        // If the predicted peak position has already been passed while the
        // observed motion is still rising, the candidate apex was too early.
        if (!candidatePeakStillAhead &&
            observationAscending)
        {
            RejectPendingPreApexCandidate(
                "CandidatePeakPassedWhileAscending",
                $"currentS={currentSectionDistanceMeters:F4}m " +
                $"candidatePeakS={candidate.peakS:F4}m " +
                $"observedSlope={observedSpatialSlope:F5}"
            );

            return false;
        }

        // A Pre-Apex publication must be confirmed before the predicted apex.
        // If the next accepted sample already reached/passed that position,
        // positional data cannot prove the candidate while still pre-apex.
        if (!candidatePeakStillAhead)
        {
            return false;
        }

        if (requireAscendingObservationForCandidateConfirmation &&
            !observationAscending)
        {
            return false;
        }

        // The later measured point agrees with the candidate trajectory while
        // the candidate apex is still ahead. This is sufficient confirmation;
        // a second Taylor peak is no longer mandatory.
        preApexCandidateConfirmedByObservation = true;

        Debug.Log(
            $"[ASYNC PEAK CANDIDATE CONFIRMED] " +
            $"mode=ObservationResidual " +
            $"section={directionSource.ActiveSectionIndex} " +
            $"currentS={currentSectionDistanceMeters:F4}m " +
            $"peakS={candidate.peakS:F4}m " +
            $"peakCY={candidate.peakCY:F4}m " +
            $"residual={residual:F4}m " +
            $"publish=True",
            this
        );

        preApexCandidateActive = false;
        preApexCandidateState =
            PreApexCandidateState.Published;

        PublishPreApexPeak(
            candidate,
            sectionLength
        );

        return true;
    }

    private void CheckPendingPreApexCandidateInvalidation()
    {
        if (!preApexCandidateActive ||
            firstWavePeakReady ||
            firstWavePeakVerified)
        {
            return;
        }

        float limitCY =
            preApexCandidatePeakCY +
            preApexInvalidationToleranceMeters;

        if (currentRelativeCY <= limitCY)
            return;

        RejectPendingPreApexCandidate(
            "MeasuredCYExceededCandidate",
            $"currentCY={currentRelativeCY:F4}m " +
            $"candidatePeakCY={preApexCandidatePeakCY:F4}m " +
            $"excess={(currentRelativeCY - preApexCandidatePeakCY):F4}m " +
            $"tolerance={preApexInvalidationToleranceMeters:F4}m"
        );
    }

    private void RejectPendingPreApexCandidate(
        string reason,
        string detail)
    {
        if (!preApexCandidateActive)
            return;

        float rejectedPeakS =
            preApexCandidatePeakS;

        float rejectedPeakCY =
            preApexCandidatePeakCY;

        int rejectedStableCount =
            preApexCandidateStableComparisons;

        ResetPreApexCandidateState(
            PreApexCandidateState.Searching
        );

        Debug.LogWarning(
            $"[ASYNC PEAK CANDIDATE REJECTED] " +
            $"reason={reason} " +
            $"section={directionSource.ActiveSectionIndex} " +
            $"rejectedPeakS={rejectedPeakS:F4}m " +
            $"rejectedPeakCY={rejectedPeakCY:F4}m " +
            $"stableCount={rejectedStableCount} " +
            $"{detail}",
            this
        );
    }

    private void ResetPreApexCandidateState(
        PreApexCandidateState nextState)
    {
        preApexCandidateEstimate = default;

        preApexCandidateActive = false;
        preApexCandidateState = nextState;

        preApexCandidateStableComparisons = 0;

        preApexCandidateObservationS = 0f;
        preApexCandidatePeakS = 0f;
        preApexCandidatePeakCY = 0f;
        preApexCandidatePeakLeadMeters = 0f;
        preApexCandidateSlope = 0f;
        preApexCandidateCurvature = 0f;

        preApexCandidateLastDistanceDelta = 0f;
        preApexCandidateLastCYDelta = 0f;

        preApexCandidateExpectedCYAtObservation = 0f;
        preApexCandidateObservationResidualMeters = 0f;
        preApexCandidateExpectedSlopeAtObservation = 0f;
        preApexCandidateObservedSlopeAtObservation = 0f;
        preApexCandidateConfirmedByObservation = false;
        preApexCandidateConfirmedByInstantaneousSlope = false;
    }

    private void PublishPreApexPeak(
        LocalApexEstimate estimate,
        float sectionLength)
    {
        observedFirstWavePeakCY =
            estimate.peakCY;

        observedFirstWavePeakProgress01 =
            estimate.peakS /
            Mathf.Max(
                sectionLength,
                Eps
            );

        predictedPeakDistanceMeters =
            estimate.peakS;

        predictedPeakLeadMeters =
            estimate.peakLeadMeters;

        peakFitRmseMeters =
            estimate.broadFitValid
                ? estimate.broadRmse
                : 0f;

        if (peakSampleCount >= 3)
        {
            peakFitSpanMeters =
                peakSamples[
                    peakSampleCount - 1].s -
                peakSamples[
                    peakSampleCount - 3].s;
        }

        firstWavePeakSolvedPreApex = true;
        firstWavePeakReady = true;

        preApexCandidateActive = false;
        preApexCandidateState =
            PreApexCandidateState.Published;

        firstWavePeakRevision++;

        RefreshCompletionPoint();

        Debug.Log(
            $"[ASYNC PEAK PRE-APEX SOLVED] " +
            $"revision={firstWavePeakRevision} " +
            $"spline={directionSource.ActiveSplineIndex} " +
            $"section={directionSource.ActiveSectionIndex} " +
            $"currentS={estimate.currentS:F4}m " +
            $"peakS={estimate.peakS:F4}m " +
            $"lead={estimate.peakLeadMeters:F4}m " +
            $"CYnow={estimate.currentCY:F4}m " +
            $"peakCY={estimate.peakCY:F4}m " +
            $"slope={estimate.slope:F5} " +
            $"curvature={estimate.curvature:F5} " +
            $"broadValid={estimate.broadFitValid} " +
            $"broadPeakS={estimate.broadPeakS:F4}m " +
            $"broadError={estimate.broadPeakDistanceError:F4}m " +
            $"releaseObserved={releaseObserved}",
            this
        );
    }

    // ================================================================
    // Pre-Apex invalidation
    // ================================================================

    private void CheckPreApexInvalidation()
    {
        // Only invalidate an unverified Pre-Apex publication.
        // Post-apex fallback values are never handled here.
        if (!firstWavePeakReady ||
            !firstWavePeakSolvedPreApex ||
            firstWavePeakVerified)
        {
            return;
        }

        float limitCY =
            observedFirstWavePeakCY +
            preApexInvalidationToleranceMeters;

        if (currentRelativeCY <= limitCY)
            return;

        float rejectedPeakCY =
            observedFirstWavePeakCY;

        float rejectedPeakProgress01 =
            observedFirstWavePeakProgress01;

        float rejectedPeakDistanceMeters =
            predictedPeakDistanceMeters;

        float excess =
            currentRelativeCY -
            rejectedPeakCY;

        // Important:
        // Do NOT reset peakSampleCount, maximumRelativeCY,
        // maximumRelativeCYProgress01, firstWaveSawAscent,
        // previousSpatialSlope, or previousSpatialSlopeValid.
        //
        // The point of invalidation is to keep the newer evidence and
        // immediately allow the spatial solver to estimate again.
        firstWavePeakReady = false;
        firstWavePeakSolvedPreApex = false;

        ResetPreApexCandidateState(
            PreApexCandidateState.Searching
        );

        localTaylorValid = false;

        reliableBroadFitValid = false;
        reliableBroadPeakDistanceMeters = 0f;
        reliableBroadPeakCY = 0f;
        reliableBroadFitRmse = 0f;
        broadVsLocalPeakDistanceError = 0f;

        // The marker represented the rejected prediction.
        completionPointPhysicsValid = false;

        Debug.LogWarning(
            $"[ASYNC PEAK PRE-APEX INVALIDATED] " +
            $"section={directionSource.ActiveSectionIndex} " +
            $"currentS={currentSectionDistanceMeters:F4}m " +
            $"currentCY={currentRelativeCY:F4}m " +
            $"rejectedPeakS={rejectedPeakDistanceMeters:F4}m " +
            $"rejectedPeakProgress={rejectedPeakProgress01:F4} " +
            $"rejectedPeakCY={rejectedPeakCY:F4}m " +
            $"excess={excess:F4}m " +
            $"tolerance={preApexInvalidationToleranceMeters:F4}m " +
            $"resumeSolver=True",
            this
        );
    }

    // ================================================================
    // Real Apex = verification / fallback only
    // ================================================================

    private void ResolveRealApexCrossing(
        float firstWaveLength,
        float sectionLength)
    {
        float resolvedS =
            maximumRelativeCYProgress01 *
            sectionLength;

        float resolvedCY =
            maximumRelativeCY;

        TryResolveInterpolatedRealApex(
            firstWaveLength,
            ref resolvedS,
            ref resolvedCY
        );

        float resolvedProgress01 =
            resolvedS /
            Mathf.Max(
                sectionLength,
                Eps
            );

        if (!firstWavePeakReady &&
            allowPostApexFallback)
        {
            PublishPostApexFallback(
                resolvedProgress01,
                resolvedS,
                resolvedCY
            );
        }

        verifiedFirstWavePeakCY =
            resolvedCY;

        verifiedFirstWavePeakProgress01 =
            resolvedProgress01;

        firstWavePeakVerified = true;

        preApexCandidateActive = false;
        preApexCandidateState =
            PreApexCandidateState.Verified;

        if (firstWavePeakReady)
        {
            firstWavePeakVerificationErrorMeters =
                verifiedFirstWavePeakCY -
                observedFirstWavePeakCY;
        }

        Debug.Log(
            $"[EQUALIZER FUTURE FIRST WAVE VERIFIED] " +
            $"revision={firstWavePeakRevision} " +
            $"preApex={firstWavePeakSolvedPreApex} " +
            $"predictedProgress={observedFirstWavePeakProgress01:F4} " +
            $"verifiedProgress={verifiedFirstWavePeakProgress01:F4} " +
            $"predictedCY={observedFirstWavePeakCY:F4}m " +
            $"verifiedCY={verifiedFirstWavePeakCY:F4}m " +
            $"error={firstWavePeakVerificationErrorMeters:F5}m",
            this
        );
    }

    private void PublishPostApexFallback(
        float progress01,
        float s,
        float cy)
    {
        if (!IsFinite(progress01) ||
            !IsFinite(cy) ||
            progress01 <= 0f)
        {
            return;
        }

        observedFirstWavePeakCY =
            cy;

        observedFirstWavePeakProgress01 =
            progress01;

        predictedPeakDistanceMeters =
            s;

        predictedPeakLeadMeters = 0f;

        firstWavePeakSolvedPreApex = false;
        firstWavePeakReady = true;

        firstWavePeakRevision++;

        RefreshCompletionPoint();

        Debug.LogWarning(
            $"[EQUALIZER FUTURE FIRST WAVE POST-APEX FALLBACK] " +
            $"revision={firstWavePeakRevision} " +
            $"progress={progress01:F4} " +
            $"CY={cy:F4}m",
            this
        );
    }

    private void TryResolveInterpolatedRealApex(
        float firstWaveLength,
        ref float resolvedS,
        ref float resolvedCY)
    {
        int count =
            Mathf.Min(
                5,
                peakSampleCount
            );

        if (count < 3)
            return;

        int start =
            peakSampleCount - count;

        if (!TryFitQuadratic(
                start,
                count,
                out QuadraticPeakFit fit))
        {
            return;
        }

        if (fit.a >=
            -minimumConcavityMagnitude)
        {
            return;
        }

        float minimumS =
            peakSamples[start].s;

        float maximumS =
            peakSamples[
                peakSampleCount - 1].s;

        if (fit.peakS <
                minimumS ||
            fit.peakS >
                maximumS ||
            fit.peakS <= 0f ||
            fit.peakS >= firstWaveLength)
        {
            return;
        }

        if (!IsFinite(fit.peakCY))
            return;

        // resolvedCY enters this method as the already-observed maximum.
        // Never allow interpolation to reduce the verified apex below it.
        if (fit.peakCY >= resolvedCY)
        {
            resolvedS =
                fit.peakS;

            resolvedCY =
                fit.peakCY;
        }
    }

    // ================================================================
    // Quadratic least-squares fit in spatial domain
    // ================================================================

    private bool TryFitQuadratic(
        int start,
        int count,
        out QuadraticPeakFit fit)
    {
        fit = default;

        if (count < 3 ||
            start < 0 ||
            start + count >
            peakSampleCount)
        {
            return false;
        }

        float centerS =
            peakSamples[
                start + count - 1].s;

        double s0 = 0.0;
        double s1 = 0.0;
        double s2 = 0.0;
        double s3 = 0.0;
        double s4 = 0.0;

        double t0 = 0.0;
        double t1 = 0.0;
        double t2 = 0.0;

        for (int i = 0;
             i < count;
             i++)
        {
            PeakSample sample =
                peakSamples[start + i];

            double x =
                sample.s -
                centerS;

            double y =
                sample.cy;

            double x2 =
                x * x;

            s0 += 1.0;
            s1 += x;
            s2 += x2;
            s3 += x2 * x;
            s4 += x2 * x2;

            t0 += y;
            t1 += x * y;
            t2 += x2 * y;
        }

        if (!TrySolve3x3(
                s4, s3, s2,
                s3, s2, s1,
                s2, s1, s0,
                t2, t1, t0,
                out double a,
                out double b,
                out double c))
        {
            return false;
        }

        if (!IsFinite(a) ||
            !IsFinite(b) ||
            !IsFinite(c) ||
            Mathf.Abs((float)a) <= Eps)
        {
            return false;
        }

        double peakX =
            -b /
            (2.0 * a);

        double peakS =
            centerS +
            peakX;

        double peakCY =
            a * peakX * peakX +
            b * peakX +
            c;

        double squaredError = 0.0;

        for (int i = 0;
             i < count;
             i++)
        {
            PeakSample sample =
                peakSamples[start + i];

            double x =
                sample.s -
                centerS;

            double predicted =
                a * x * x +
                b * x +
                c;

            double error =
                predicted -
                sample.cy;

            squaredError +=
                error * error;
        }

        double rmse =
            System.Math.Sqrt(
                squaredError /
                count
            );

        float span =
            peakSamples[
                start + count - 1].s -
            peakSamples[start].s;

        if (!IsFinite(peakS) ||
            !IsFinite(peakCY) ||
            !IsFinite(rmse))
        {
            return false;
        }

        fit =
            new QuadraticPeakFit
            {
                valid = true,

                a = (float)a,
                b = (float)b,
                c = (float)c,

                peakS =
                    (float)peakS,

                peakCY =
                    (float)peakCY,

                rmse =
                    (float)rmse,

                spanMeters =
                    span
            };

        return true;
    }

    private static bool TrySolve3x3(
        double a,
        double b,
        double c,
        double d,
        double e,
        double f,
        double g,
        double h,
        double i,
        double j,
        double k,
        double l,
        out double x,
        out double y,
        out double z)
    {
        x = 0.0;
        y = 0.0;
        z = 0.0;

        double determinant =
            a * (e * i - f * h) -
            b * (d * i - f * g) +
            c * (d * h - e * g);

        if (System.Math.Abs(
                determinant) <=
            SolveEps)
        {
            return false;
        }

        double determinantX =
            j * (e * i - f * h) -
            b * (k * i - f * l) +
            c * (k * h - e * l);

        double determinantY =
            a * (k * i - f * l) -
            j * (d * i - f * g) +
            c * (d * l - k * g);

        double determinantZ =
            a * (e * l - k * h) -
            b * (d * l - k * g) +
            j * (d * h - e * g);

        x =
            determinantX /
            determinant;

        y =
            determinantY /
            determinant;

        z =
            determinantZ /
            determinant;

        return
            IsFinite(x) &&
            IsFinite(y) &&
            IsFinite(z);
    }

    // ================================================================
    // Completion point
    // ================================================================

    private void RefreshCompletionPoint()
    {
        completionPointPhysicsValid = false;

        if (!slopeStickCore ||
            !firstWavePeakReady)
        {
            return;
        }

        if (!slopeStickCore.TryEvaluateBallVisualSectionFramePhysics(
                observedFirstWavePeakProgress01,
                out Vector3 carrierCenterPhysics,
                out _,
                out _))
        {
            return;
        }

        completionPointPhysics =
            carrierCenterPhysics +
            Vector3.up *
            observedFirstWavePeakCY;

        completionPointPhysicsValid =
            IsFinite(
                completionPointPhysics
            );
    }

    private void OnDrawGizmos()
    {
        if (!drawCompletionGizmo ||
            !completionPointPhysicsValid)
        {
            return;
        }

        Vector3 point =
            completionPointPhysics;

        if (correspondSubject)
        {
            point =
                correspondSubject.MapPoint(
                    completionPointPhysics
                );
        }

        float r =
            Mathf.Max(
                0.005f,
                completionGizmoRadius
            );

        Gizmos.DrawWireSphere(
            point,
            r
        );

        Gizmos.DrawLine(
            point - Vector3.right * r,
            point + Vector3.right * r
        );

        Gizmos.DrawLine(
            point - Vector3.up * r,
            point + Vector3.up * r
        );

        Gizmos.DrawLine(
            point - Vector3.forward * r,
            point + Vector3.forward * r
        );
    }

    // ================================================================
    // Public peak API
    // ================================================================

    public bool TryGetFirstWavePeak(
        long expectedSectionKey,
        out FirstWavePeakObservation observation)
    {
        observation = default;

        if (!firstWavePeakReady ||
            observedSectionKey !=
            expectedSectionKey)
        {
            return false;
        }

        observation =
            new FirstWavePeakObservation
            {
                valid = true,

                revision =
                    firstWavePeakRevision,

                sectionKey =
                    observedSectionKey,

                splineIndex =
                    directionSource
                        ? directionSource.ActiveSplineIndex
                        : -1,

                sectionIndex =
                    directionSource
                        ? directionSource.ActiveSectionIndex
                        : -1,

                progress01 =
                    observedFirstWavePeakProgress01,

                cy =
                    observedFirstWavePeakCY,

                relativeVerticalVelocity =
                    currentRelativeVerticalVelocity,

                solvedPreApex =
                    firstWavePeakSolvedPreApex,

                verified =
                    firstWavePeakVerified,

                releaseObserved =
                    releaseObserved,

                sectionDistanceMeters =
                    predictedPeakDistanceMeters,

                fitRmseMeters =
                    peakFitRmseMeters
            };

        return true;
    }

    // ================================================================
    // Debug
    // ================================================================

    private void LogPreApexReject(
        string reason,
        string detail)
    {
        if (!logPreApexSolver)
            return;

        bool reasonChanged =
            reason != lastPreApexRejectReason;

        bool intervalElapsed =
            Time.fixedTime -
            lastPreApexRejectLogTime >=
            preApexRejectLogIntervalSeconds;

        if (!reasonChanged &&
            !intervalElapsed)
        {
            return;
        }

        lastPreApexRejectReason =
            reason;

        lastPreApexRejectLogTime =
            Time.fixedTime;

        Debug.Log(
            $"[ASYNC PEAK REJECT] " +
            $"reason={reason} " +
            $"section={directionSource.ActiveSectionIndex} " +
            $"progress={slopeStickCore.BallVisualSlopeProgress01:F4} " +
            $"s={currentSectionDistanceMeters:F4}m " +
            $"CY={currentRelativeCY:F4}m " +
            $"samples={peakSampleCount} " +
            $"{detail}",
            this
        );
    }

    // ================================================================
    // Helpers
    // ================================================================

    private float ResolveSectionLength()
    {
        if (directionSource)
        {
            float sourceLength =
                directionSource.SectionLength;

            if (IsFinite(sourceLength) &&
                sourceLength > Eps)
            {
                return sourceLength;
            }
        }

        if (slopeStickCore)
        {
            float coreLength =
                slopeStickCore
                    .BallVisualSlopeSectionLength;

            if (IsFinite(coreLength) &&
                coreLength > Eps)
            {
                return coreLength;
            }
        }

        return 0f;
    }

    private static bool IsFinite(
        float value)
    {
        return
            !float.IsNaN(value) &&
            !float.IsInfinity(value);
    }

    private static bool IsFinite(
        double value)
    {
        return
            !double.IsNaN(value) &&
            !double.IsInfinity(value);
    }

    private static bool IsFinite(
        Vector3 value)
    {
        return
            IsFinite(value.x) &&
            IsFinite(value.y) &&
            IsFinite(value.z);
    }
}
