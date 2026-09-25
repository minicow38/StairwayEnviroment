using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Serialization;
using UnityEngine.Splines;
using Sirenix.OdinInspector;

/// <summary>
/// Generic read-only direction-field teacher contract.
///
/// The teacher may provide geometry for correspondence, but it never owns the
/// student's position constant, Rigidbody state, phase, or final spline.
/// Implementations must return all points/tangents in the same canonical Physics
/// frame expected by EqualizerFutureSpline.
/// </summary>
public interface IDirectionFieldTeacher
{
    bool TryEvaluateDirectionAtNormalizedU(
        int sourceSplineIndex,
        int sourceSectionIndex,
        float normalizedU,
        out Vector3 pointPhysics,
        out Vector3 tangentPhysics);

    bool TryGetDirectionAtArcProgress(
        int sourceSplineIndex,
        int sourceSectionIndex,
        float normalizedArcProgress,
        int arcSamples,
        out float teacherU,
        out Vector3 pointPhysics,
        out Vector3 tangentPhysics);

    bool TryProjectDirectionLocalNewton(
        int sourceSplineIndex,
        int sourceSectionIndex,
        Vector3 probePointPhysics,
        float seedU,
        float halfWindowU,
        int iterations,
        out float projectedU,
        out Vector3 pointPhysics,
        out Vector3 tangentPhysics,
        out float euclideanDistanceMeters);
}


/// <summary>
/// BallBoom direction authority.
///
/// Responsibility:
/// - InSubject / NearestKnotDetector remains the main trigger and base geometry authority.
/// - Produces the future direction field T/N/B from InSubject.
/// - May READ any IDirectionFieldTeacher as an optional tangent observation source.
/// - Never reads BallVisualEqualizer Rigidbody / phase / position / velocity / boundary state.
/// - EqualizerFutureAsyncPos is a vertical boundary provider only: Inspector f(0)=CY and a
///   read-only first-wave peak observation. It never owns X/Z progression or direction.
/// - Owns the generated direction spline. X/Z progression remains derived from the SlopeStickCore
///   carrier; the CY boundary may correct only Physics-Y over the first wave.
/// - Evaluates mass-independent differential specific energy and the resulting time loss as
///   READ ONLY kinematic diagnostics; energy never becomes a second T/N/B authority.
/// - All world -> StairWay local conversion happens here, once, at final Spline build.
/// </summary>
///
[Searchable]
[DefaultExecutionOrder(11600)]
[DisallowMultipleComponent]
public sealed class EqualizerFutureSpline : MonoBehaviour
{
    private const float Eps = 0.000001f;
    private const int WavesPerStair = 3;
    private const int MaxRetainedFutureSplines = 2;
    private const string StairWayNamePrefix = "StairWay";
    private const float StairWayParentSearchRadius = 3.0f;
    private const int StairWayParentSearchCapacity = 32;
    private const float MinimumStairWayTangentAlignment = 0.70f;

    /// <summary>
    /// Mathematical applicability of the optional external direction teacher.
    /// This is an engine-state classification, not an accuracy score.
    /// </summary>
    public enum DirectionTeacherState
    {
        Unavailable,
        Applicable,
        WeaklyApplicable,
        Rejected
    }

    [System.Serializable]
    public struct DirectionSample
    {
        public float u;
        public float s;
        public float time;
        public float speed;
        public Vector3 carrierPoint;
        public Vector3 tangent;
        public Vector3 normal;
        public Vector3 binormal;
    }

    /// <summary>
    /// Differential specific-energy diagnostics along the final direction carrier.
    ///
    /// All energy values are per unit mass so the engine remains mass-independent:
    ///   specific kinetic energy      k = 1/2 v^2                      [J/kg]
    ///   specific potential energy    phi = -g dot (x - x_entry)       [J/kg]
    ///   specific mechanical energy   e = k + phi                      [J/kg]
    ///   differential work density    de/ds = a_nonconservative        [J/(kg m)]
    ///   differential power           de/dt = (de/ds) v                [W/kg]
    ///
    /// cumulativeTimeLossSeconds compares the predicted motion with a local
    /// loss-free shadow that preserves gravity and positive non-conservative work
    /// but suppresses negative non-conservative work. It is a diagnostic only;
    /// it never modifies T/N/B or teacher applicability.
    /// </summary>
    [System.Serializable]
    public struct DifferentialEnergySample
    {
        public bool valid;
        public float u;
        public float s;
        public float time;
        public float speed;
        public float lossFreeTime;
        public float lossFreeSpeed;
        public float cumulativeTimeLossSeconds;
        public float specificKineticEnergyJPerKg;
        public float specificPotentialEnergyRelativeToEntryJPerKg;
        public float specificMechanicalEnergyJPerKg;
        public float differentialMechanicalEnergyPerMeterJPerKgPerMeter;
        public float differentialMechanicalPowerWattsPerKg;
        public float cumulativeSpecificEnergyLossJPerKg;
        public float energyBalanceResidualJPerKg;
    }

    [Header("BallBoom References")]
    [SerializeField] private Rigidbody inSubjectBody;
    [SerializeField] private NearestKnotDetector knotDetector;
    [Tooltip("Optional MonoBehaviour implementing IDirectionFieldTeacher. If empty, the component searches this GameObject.")]
    [SerializeField] private MonoBehaviour directionTeacherSource;

    [Tooltip("Vertical boundary provider: Inspector f(0)=CY and first-wave observed peak.")]
    [FormerlySerializedAs("positionConstantSource")]
    [SerializeField] private EqualizerFutureAsyncPos verticalBoundarySource;

    private IDirectionFieldTeacher directionTeacher;

    [Tooltip("PhysicsRoot/CollisionStageRoot")]
    [SerializeField] private Transform collisionStageRoot;

    [Header("Direction Field")]
    [Range(25, 129)]
    [SerializeField] private int analysisSampleCount = 49;

    [Min(0f)]
    [SerializeField] private float waveAmplitude = 0.45f;

    [Range(0f, 1f)]
    [SerializeField] private float inSubjectV0DirectionInfluence = 0.35f;

    [Range(0.02f, 0.40f)]
    [SerializeField] private float inSubjectV0DirectionFade01 = 0.15f;

    [Header("Reference Spline Tangent Correspondence")]
    [SerializeField] private bool imitateReferenceSplineTangent = true;

    [Range(32, 192)]
    [SerializeField] private int referenceArcLengthSamples = 96;

    [Range(1, 8)]
    [SerializeField] private int referenceNewtonIterations = 4;

    [Range(0.01f, 0.35f)]
    [SerializeField] private float referenceNewtonHalfWindowU = 0.12f;

    [Range(0f, 1f)]
    [SerializeField] private float referenceTangentBlend = 0.70f;

    [Header("Direction Correspondence Metric")]
    [Tooltip("Characteristic length used to nondimensionalize T/N/B position residuals. Expressed as a fraction of section arc length.")]
    [Range(0.01f, 1f)]
    [SerializeField] private float directionMetricScaleFraction = 0.15f;

    [Min(0f)]
    [SerializeField] private float directionMetricTangentWeight = 1.0f;

    [Min(0f)]
    [SerializeField] private float directionMetricNormalWeight = 0.10f;

    [Min(0f)]
    [SerializeField] private float directionMetricBinormalWeight = 1.0f;

    [Header("Direction Teacher Applicability")]
    [Tooltip("Minimum valid correspondence coverage for full teacher applicability.")]
    [Range(0.05f, 1f)]
    [SerializeField] private float applicableMinimumValidSampleRatio = 0.60f;

    [Tooltip("Minimum valid correspondence coverage for weak teacher applicability.")]
    [Range(0.01f, 1f)]
    [SerializeField] private float weakMinimumValidSampleRatio = 0.25f;

    [Tooltip("Maximum mean dimensionless T/N/B metric for full applicability.")]
    [Min(0.01f)]
    [SerializeField] private float applicableMaximumMeanDirectionMetric = 1.00f;

    [Tooltip("Maximum mean dimensionless T/N/B metric for weak applicability.")]
    [Min(0.01f)]
    [SerializeField] private float weakMaximumMeanDirectionMetric = 2.00f;

    [Tooltip("Minimum mean tangent dot for full applicability.")]
    [Range(-1f, 1f)]
    [SerializeField] private float applicableMinimumMeanForwardDot = 0.60f;

    [Tooltip("Minimum mean tangent dot for weak applicability.")]
    [Range(-1f, 1f)]
    [SerializeField] private float weakMinimumMeanForwardDot = 0.20f;

    [Range(-1f, 1f)]
    [SerializeField] private float minimumReferenceForwardDot = 0.10f;

    [Header("Independent Wave Slope")]
    [Range(1f, 45f)]
    [SerializeField] private float maximumWaveTangentAngleDegrees = 20f;

    [Header("InSubject Future Time Parameterization")]
    [Min(0.05f)]
    [SerializeField] private float minimumForwardSpeed = 0.50f;

    [Min(0.25f)]
    [SerializeField] private float maximumPredictionTimeSeconds = 3.0f;

    [Header("First Wave CY Boundary")]
    [Tooltip("Apply the Inspector f(0)=CY boundary from EqualizerFutureAsyncPos without changing X/Z progression.")]
    [SerializeField] private bool useFirstWaveCYBoundary = true;

    [Tooltip("Consume one measured first-wave peak per slope section and rebuild only the Y-boundary correction.")]
    [SerializeField] private bool refreshFromObservedFirstWavePeak = true;

    [Header("First Wave CY Boundary - READ ONLY")]
    [SerializeField] private float appliedInitialCY;
    [SerializeField] private bool firstWavePeakCorrectionActive;
    [SerializeField] private int consumedFirstWavePeakRevision = -1;
    [SerializeField] private float firstWavePeakProgress01;
    [SerializeField] private float firstWavePeakTargetCY;
    [SerializeField] private float firstWavePeakPredictedCYBeforeCorrection;
    [SerializeField] private float firstWavePeakErrorBeforeCorrection;

    [Header("Definite Integral Carrier")]
    [Tooltip("OFF: use the existing carrier march. ON: reconstruct the carrier with the composite trapezoidal integral of T(s), anchored at SlopeStickCore X/Z plus Inspector CY.")]
    [SerializeField] private bool useDefiniteIntegralPositionReconstruction = false;

    [Header("Definite Integral Carrier - READ ONLY")]
    [SerializeField] private bool definiteIntegralApplied;
    [FormerlySerializedAs("definiteIntegralUsingPositionConstant")]
    [SerializeField] private bool definiteIntegralUsingCYBoundary;
    [SerializeField] private int definiteIntegralAppliedDirectionVersion = -1;
    [FormerlySerializedAs("definiteIntegralPositionConstantC")]
    [SerializeField] private Vector3 definiteIntegralAnchorC;
    [SerializeField] private Vector3 definiteIntegralEndOffset;
    [SerializeField] private float definiteIntegralEndOffsetMagnitude;

    [Header("Differential Energy / Time Loss - READ ONLY")]
    [SerializeField] private float predictedSpecificEnergyLossJPerKg;
    [SerializeField] private float predictedLossFreeSectionDuration;
    [SerializeField] private float predictedDifferentialEnergyTimeLossSeconds;
    [SerializeField] private float predictedDifferentialEnergyTimeLossRatio;
    [SerializeField] private float meanSpecificEnergyLossGradientJPerKgPerMeter;
    [SerializeField] private float peakSpecificEnergyLossPowerWattsPerKg;
    [SerializeField] private float maximumEnergyBalanceResidualJPerKg;

    [Header("Generated Final Spline - READ ONLY")]
    [SerializeField] private SplineContainer splineContainer;
    [SerializeField] private int retainedFutureSplineCount;

    [Header("Raw Direction + CY Comparison Spline")]
    [Tooltip("Build a second comparison Spline from EqualizerFutureSpline's own raw direction field plus only the Inspector CY below. EqualizerFutureAsyncPos and BallVisualEqualizer observations are not read by this path.")]
    [SerializeField] private bool generateDirectionCYOnlySpline = true;

    [Tooltip("f(0)=CY [m] for the raw-direction comparison Spline. X/Z are never supplied by EqualizerFutureAsyncPos.")]
    [SerializeField] private float directionCYOnlyInitialCY = 0f;

    [Tooltip("Bezier knot count for the raw-direction + CY comparison output. 13 = 3 waves x 4 quarter-wave divisions + endpoint.")]
    [Range(5, 49)]
    [SerializeField] private int directionCYOnlyOutputKnotCount = 13;

    [Header("Raw Direction + CY Comparison - READ ONLY")]
    [SerializeField] private SplineContainer directionCYOnlySplineContainer;
    [SerializeField] private int directionCYOnlyConsumedRawBasisVersion = -1;
    [SerializeField] private long directionCYOnlyGeneratedSectionKey = long.MinValue;
    [SerializeField] private int directionCYOnlyGeneratedKnotCount;
    [SerializeField] private float directionCYOnlyAppliedCY;
    [SerializeField] private Vector3 directionCYOnlyStartPointPhysics;
    [SerializeField] private Vector3 directionCYOnlyEndPointPhysics;
    [SerializeField] private float directionCYOnlyIntegratedArcLength;
    [SerializeField] private int retainedDirectionCYOnlySplineCount;

    [Header("Diagnostics - READ ONLY")]
    [SerializeField] private bool directionReady;
    [SerializeField] private int directionVersion;
    [Tooltip("Increments only when EqualizerFutureSpline rebuilds its own raw direction field. Teacher/CY/first-wave observations do not change this version.")]
    [SerializeField] private int rawDirectionBasisVersion;
    [SerializeField] private int generatedSplineIndex = -1;
    [SerializeField] private int generatedSectionIndex = -1;
    [SerializeField] private int generatedKnotCount;
    [SerializeField] private float activeSectionLength;
    [SerializeField] private float predictedTimeToSlopeEntry;
    [SerializeField] private float predictedEntryTangentSpeed;
    [SerializeField] private float predictedSectionDuration;
    [SerializeField] private float capturedInSubjectClearance;
    [SerializeField] private bool builtWithReferenceTeacher;
    [SerializeField] private DirectionTeacherState directionTeacherState = DirectionTeacherState.Unavailable;
    [SerializeField] private float teacherApplicabilityScore;
    [SerializeField] private float teacherApplicabilityAuthorityScale;
    [SerializeField] private float teacherValidSampleRatio;
    [SerializeField] private float teacherMeanDirectionMetric;
    [SerializeField] private float teacherMeanForwardDot;
    [SerializeField] private int teacherSeedAvailableSamples;
    [SerializeField] private int teacherProjectedSamples;
    [SerializeField] private int referenceMatchedSamples;
    [SerializeField] private float referenceAverageAngleDegrees;
    [SerializeField] private float referenceMaximumAngleDegrees;
    [SerializeField] private float referenceMaximumDistanceMeters;
    [SerializeField] private int rawTeacherSamples;
    [SerializeField] private int teacherMonotoneAdjustedSamples;
    [SerializeField] private float rawTeacherAverageAngleDegrees;
    [SerializeField] private float rawTeacherMaximumAngleDegrees;
    [SerializeField] private int finalTeacherAngleSamples;
    [SerializeField] private float finalTeacherAverageAngleDegrees;
    [SerializeField] private float finalTeacherMaximumAngleDegrees;

    [Header("Final Teacher Max-Angle Detail - READ ONLY")]
    [SerializeField] private int finalTeacherMaximumSampleIndex = -1;
    [SerializeField] private float finalTeacherMaximumSampleU;
    [SerializeField] private float finalTeacherMaximumRawTeacherU;
    [SerializeField] private float finalTeacherMaximumMappedTeacherU;
    [SerializeField] private float finalTeacherMaximumTeacherDistanceMeters;
    [SerializeField] private float finalTeacherMaximumMappedDistanceMeters;
    [SerializeField] private float finalTeacherMaximumDeltaS;
    [SerializeField] private float finalTeacherMaximumDeltaN;
    [SerializeField] private float finalTeacherMaximumDeltaB;
    [SerializeField] private float finalTeacherMaximumDirectionMetric;
    [SerializeField] private float finalTeacherMaximumDistanceConfidence;
    [SerializeField] private float finalTeacherMaximumForwardConfidence;
    [SerializeField] private float finalTeacherMaximumGain;
    [SerializeField] private float finalTeacherMaximumRawAngleDegrees;

    [SerializeField] private string generatedStairWayParentName = string.Empty;

    [Header("Analytic Limit Warning - DEBUG ONLY")]
    [SerializeField] private bool enableAnalyticLimitWarning = true;
    [Range(1, 10)] [SerializeField] private int limitRequiredConsecutiveMatches = 3;
    [Range(0.05f, 15f)] [SerializeField] private float limitAverageAngleDegrees = 2f;
    [Range(0.05f, 20f)] [SerializeField] private float limitMaximumAngleDegrees = 5f;
    [Min(0.001f)] [SerializeField] private float limitMaximumDistanceMeters = 0.25f;
    [SerializeField] private int limitConsecutiveMatches;
    [SerializeField] private bool limitCandidate;
    private bool limitWarningLatched;

    private SlopeStickCore splineCore;
    private long generatedSectionKey = long.MinValue;
    private Transform activeStairWayParent;
    private NearestKnotDetector.GuideFrame predictionSourceGuide;
    private bool predictionSourceGuideValid;

    private float[] sampleU;
    private float[] sampleS;
    private float[] sampleTime;
    private float[] sampleSpeed;

    private float[] sampleLossFreeTime;
    private float[] sampleLossFreeSpeed;
    private float[] sampleCumulativeTimeLoss;
    private float[] sampleSpecificKineticEnergy;
    private float[] sampleSpecificPotentialEnergy;
    private float[] sampleSpecificMechanicalEnergy;
    private float[] sampleDifferentialMechanicalEnergyPerMeter;
    private float[] sampleDifferentialMechanicalPower;
    private float[] sampleCumulativeSpecificEnergyLoss;
    private float[] sampleEnergyBalanceResidual;

    private Vector3[] basePoints;
    private Vector3[] baseTangents;
    private Vector3[] baseNormals;
    private Vector3[] rawTangents;
    private Vector3[] uncorrectedDirectionTangents;
    private Vector3[] directionTangents;
    private Vector3[] directionNormals;
    private Vector3[] directionBinormals;
    private Vector3[] carrierPoints;

    // Integrated comparison output owned by this same component.
    // It uses rawTangents directly and never reads EqualizerFutureAsyncPos.
    private Vector3[] directionCYOnlyPoints;
    private Vector3[] directionCYOnlyTangents;
    private float directionCYOnlyLastBuiltCY = float.NaN;

    private Vector3[] teacherReferenceTangents;
    private float[] teacherRawU;
    private float[] teacherMappedU;
    private float[] teacherDistance;
    private float[] teacherMappedDistance;
    private float[] teacherDeltaS;
    private float[] teacherDeltaN;
    private float[] teacherDeltaB;
    private float[] teacherDirectionMetric;
    private float[] teacherDistanceConfidence;
    private float[] teacherForwardConfidence;
    private float[] teacherGain;
    private bool[] teacherCandidateValid;

    // PAVA work buffers. They are allocated once with the direction samples.
    private int[] pavaCandidateIndex;
    private int[] pavaBlockStart;
    private int[] pavaBlockEnd;
    private float[] pavaBlockMean;
    private float[] pavaBlockWeight;

    private readonly List<float> knotParameters = new List<float>(8);
    private readonly List<SplineContainer> generatedSplineHistory = new List<SplineContainer>(3);
    private readonly List<SplineContainer> directionCYOnlySplineHistory = new List<SplineContainer>(3);
    private readonly Collider[] stairWayParentSearchBuffer = new Collider[StairWayParentSearchCapacity];

    public bool DirectionReady => directionReady;
    public int DirectionVersion => directionVersion;
    public int RawDirectionBasisVersion => rawDirectionBasisVersion;
    public SplineContainer DirectionCYOnlySpline => directionCYOnlySplineContainer;
    public float DirectionCYOnlyInitialCY => directionCYOnlyInitialCY;
    public int WaveCount => WavesPerStair;
    public DirectionTeacherState TeacherState => directionTeacherState;
    public float TeacherApplicabilityScore => teacherApplicabilityScore;
    public float TeacherApplicabilityAuthorityScale => teacherApplicabilityAuthorityScale;
    public float TeacherValidSampleRatio => teacherValidSampleRatio;
    public float TeacherMeanDirectionMetric => teacherMeanDirectionMetric;
    public float TeacherMeanForwardDot => teacherMeanForwardDot;
    public long ActiveSectionKey => generatedSectionKey;
    public int ActiveSplineIndex => generatedSplineIndex;
    public int ActiveSectionIndex => generatedSectionIndex;
    public float SectionLength => activeSectionLength;
    public float PredictedTimeToSlopeEntry => predictedTimeToSlopeEntry;
    public float PredictedEntryTangentSpeed => predictedEntryTangentSpeed;
    public float PredictedSectionDuration => predictedSectionDuration;
    public float PredictedSpecificEnergyLossJPerKg => predictedSpecificEnergyLossJPerKg;
    public float PredictedLossFreeSectionDuration => predictedLossFreeSectionDuration;
    public float PredictedDifferentialEnergyTimeLossSeconds => predictedDifferentialEnergyTimeLossSeconds;
    public float PredictedDifferentialEnergyTimeLossRatio => predictedDifferentialEnergyTimeLossRatio;
    public float MeanSpecificEnergyLossGradientJPerKgPerMeter => meanSpecificEnergyLossGradientJPerKgPerMeter;
    public float PeakSpecificEnergyLossPowerWattsPerKg => peakSpecificEnergyLossPowerWattsPerKg;
    public float MaximumEnergyBalanceResidualJPerKg => maximumEnergyBalanceResidualJPerKg;
    public bool UseDefiniteIntegralPositionReconstruction => useDefiniteIntegralPositionReconstruction;
    public bool DefiniteIntegralApplied => definiteIntegralApplied;
    public bool DefiniteIntegralUsingCYBoundary => definiteIntegralUsingCYBoundary;
    public Vector3 DefiniteIntegralAnchorC => definiteIntegralAnchorC;
    public Vector3 DefiniteIntegralEndOffset => definiteIntegralEndOffset;

    // Compatibility aliases for code compiled against the previous naming.
    public bool DefiniteIntegralUsingPositionConstant => definiteIntegralUsingCYBoundary;
    public Vector3 DefiniteIntegralPositionConstantC => definiteIntegralAnchorC;
    public int DirectionSampleCount => sampleU != null ? sampleU.Length : 0;

    public float GetPredictedEnergyLossJoules(float massKg)
    {
        return Mathf.Max(0f, massKg) * predictedSpecificEnergyLossJPerKg;
    }

    public float GetPeakEnergyLossPowerWatts(float massKg)
    {
        return Mathf.Max(0f, massKg) * peakSpecificEnergyLossPowerWattsPerKg;
    }
    public SplineContainer FinalSpline => splineContainer;
    public Transform ActiveStairWayParent => activeStairWayParent;
    public int SuggestedKnotCount => knotParameters.Count;

    private void Awake()
    {
        if (!inSubjectBody)
        {
            SlopeStickCore foundCore = FindObjectOfType<SlopeStickCore>();
            if (foundCore)
                inSubjectBody = foundCore.GetComponent<Rigidbody>();
        }

        if (!knotDetector && inSubjectBody)
            knotDetector = inSubjectBody.GetComponent<NearestKnotDetector>();

        if (inSubjectBody)
            splineCore = inSubjectBody.GetComponent<SlopeStickCore>();

        ResolveDirectionTeacher();
        ResolveVerticalBoundarySource();
        EnsureBuffers();
    }

    private void ResolveVerticalBoundarySource()
    {
        if (!verticalBoundarySource)
            verticalBoundarySource = GetComponent<EqualizerFutureAsyncPos>();

        if (!verticalBoundarySource)
            verticalBoundarySource = FindObjectOfType<EqualizerFutureAsyncPos>();
    }

    private void ResolveDirectionTeacher()
    {
        directionTeacher = null;

        if (directionTeacherSource is IDirectionFieldTeacher configuredTeacher)
        {
            directionTeacher = configuredTeacher;
            return;
        }

        MonoBehaviour[] behaviours = GetComponents<MonoBehaviour>();

        for (int i = 0; i < behaviours.Length; i++)
        {
            if (behaviours[i] is IDirectionFieldTeacher candidate)
            {
                directionTeacherSource = behaviours[i];
                directionTeacher = candidate;
                return;
            }
        }
    }

    private void Start()
    {
        if (!collisionStageRoot)
        {
            GameObject root = GameObject.Find("PhysicsRoot");
            if (root)
                collisionStageRoot = root.transform;
        }
    }

    private void FixedUpdate()
    {
        if (!inSubjectBody || !knotDetector || !splineCore)
            return;

        NearestKnotDetector.GuideFrame guide = knotDetector.CurrentGuide;

        if (!guide.valid)
            return;

        if (refreshFromObservedFirstWavePeak)
            TryRefreshFromObservedFirstWavePeak();

        // Runtime switch support.  When ON, apply/re-apply the definite integral
        // whenever the authoritative direction version changes.  When switched
        // OFF, restore the legacy carrier immediately.
        if (directionReady &&
            !guide.isSlope &&
            guide.nextIsSlope)
        {
            if (useDefiniteIntegralPositionReconstruction)
            {
                if (!definiteIntegralApplied ||
                    definiteIntegralAppliedDirectionVersion != directionVersion)
                {
                    if (TryApplyDefiniteIntegralReconstruction(guide))
                        BuildFinalSplineInExistingContainer();
                }
            }
            else if (definiteIntegralApplied)
            {
                RestoreUncorrectedDirectionTangents();
                RebuildOrthonormalFrameFromDirection();
                IntegrateDirectionCarrier();
                ApplyFirstWaveCYBoundaryCorrection();
                RebuildDirectionFromCarrier();
                RebuildOrthonormalFrameFromDirection();
                PredictInSubjectTimeParameterization(guide);
                BuildFinalSplineInExistingContainer();

                Debug.Log(
                    $"[EQUALIZER FUTURE DEFINITE INTEGRAL] " +
                    $"version={directionVersion} enabled=False restoredLegacy=True",
                    this);
            }
        }

        // Main call remains InSubject -> EqualizerFutureSpline.
        if (!guide.isSlope && guide.nextIsSlope)
            BuildOrRefineDirectionField(guide);

        // The comparison Spline is now owned by EqualizerFutureSpline itself.
        // It rebuilds only when the raw direction basis, section, or its own CY changes.
        RefreshDirectionCYOnlySplineIfNeeded();
    }

    private bool BuildOrRefineDirectionField(
        NearestKnotDetector.GuideFrame guide)
    {
        if (!knotDetector.TryEvaluateForwardSlopeSection(
                guide,
                0f,
                out NearestKnotDetector.GuideSample entrySample))
        {
            return false;
        }

        long sectionKey = MakeKey(entrySample.splineIndex, entrySample.sectionIndex);

        // Same section: refine the already-built direction field when the teacher becomes available.
        if (sectionKey == generatedSectionKey)
        {
            if (imitateReferenceSplineTangent &&
                !builtWithReferenceTeacher &&
                directionReady &&
                TryApplyReferenceTeacher(
                    generatedSplineIndex,
                    generatedSectionIndex))
            {
                RebuildOrthonormalFrameFromDirection();
                CaptureUncorrectedDirectionTangents();
                IntegrateDirectionCarrier();
                ApplyFirstWaveCYBoundaryCorrection();
                RebuildDirectionFromCarrier();
                RebuildOrthonormalFrameFromDirection();
                // Re-evaluate time and differential energy on the final direction field.
                // Failure here does not invalidate already-built geometry; the previous
                // timing state remains available until the next successful evaluation.
                PredictInSubjectTimeParameterization(guide);
                builtWithReferenceTeacher = true;
                directionVersion++;

                // Optional exact-form carrier reconstruction:
                // P(s)=C+integral T ds with C anchored to base X/Z + Inspector CY.
                TryApplyDefiniteIntegralReconstruction(guide);

                BuildFinalSplineInExistingContainer();
                EvaluateReferenceDiagnosticsAndLimit();
                EvaluateFinalTeacherAngleDiagnostics();

                Debug.Log(
                    $"[EQUALIZER FUTURE DIRECTION REFINED] " +
                    $"version={directionVersion} " +
                    $"spline={generatedSplineIndex} section={generatedSectionIndex} " +
                    $"teacherState={directionTeacherState} " +
                    $"teacherScore={teacherApplicabilityScore:F3} " +
                    $"teacherAuthority={teacherApplicabilityAuthorityScale:F3} " +
                    $"teacherValidRatio={teacherValidSampleRatio:F3} " +
                    $"teacherMeanMetric={teacherMeanDirectionMetric:F3} " +
                    $"teacherMeanForwardDot={teacherMeanForwardDot:F3} " +
                    $"teacherSamples={referenceMatchedSamples} " +
                    $"rawAngleAvg={rawTeacherAverageAngleDegrees:F2}deg " +
                    $"rawAngleMax={rawTeacherMaximumAngleDegrees:F2}deg " +
                    $"finalAngleSamples={finalTeacherAngleSamples} " +
                    $"finalAngleAvg={finalTeacherAverageAngleDegrees:F2}deg " +
                    $"finalAngleMax={finalTeacherMaximumAngleDegrees:F2}deg " +
                    $"pavaAdjusted={teacherMonotoneAdjustedSamples} " +
                    $"CY={appliedInitialCY:F4}m " +
                    $"firstWavePeak={firstWavePeakCorrectionActive} " +
                    $"firstWavePeakU={firstWavePeakProgress01:F4} " +
                    $"firstWavePeakCY={firstWavePeakTargetCY:F4}m " +
                    $"definiteIntegral={definiteIntegralApplied} " +
                    $"definiteIntegralUsingCY={definiteIntegralUsingCYBoundary} " +
                    $"specificEnergyLoss={predictedSpecificEnergyLossJPerKg:F4}J/kg " +
                    $"lossFreeTime={predictedLossFreeSectionDuration:F4}s " +
                    $"timeLoss={predictedDifferentialEnergyTimeLossSeconds:F4}s " +
                    $"timeLossRatio={predictedDifferentialEnergyTimeLossRatio:F4} " +
                    $"energyResidualMax={maximumEnergyBalanceResidualJPerKg:F5}J/kg",
                    this);
            }

            return true;
        }

        if (!TryCaptureInSubjectClearance(guide, out capturedInSubjectClearance))
            return false;

        predictionSourceGuide = guide;
        predictionSourceGuideValid = true;
        firstWavePeakCorrectionActive = false;
        consumedFirstWavePeakRevision = -1;
        firstWavePeakProgress01 = 0f;
        firstWavePeakTargetCY = 0f;
        firstWavePeakPredictedCYBeforeCorrection = 0f;
        firstWavePeakErrorBeforeCorrection = 0f;

        activeSectionLength = Mathf.Max(0.01f, entrySample.sectionLength);
        EnsureBuffers();

        if (!BuildInSubjectSectionFrame(guide))
            return false;

        BuildRawDirectionField();
        rawDirectionBasisVersion++;

        builtWithReferenceTeacher =
            imitateReferenceSplineTangent &&
            TryApplyReferenceTeacher(
                entrySample.splineIndex,
                entrySample.sectionIndex);

        RebuildOrthonormalFrameFromDirection();
        CaptureUncorrectedDirectionTangents();
        IntegrateDirectionCarrier();
        ApplyFirstWaveCYBoundaryCorrection();
        RebuildDirectionFromCarrier();
        RebuildOrthonormalFrameFromDirection();

        // Time is parameterized only after the final T/N/B field exists, so
        // gravity projection, differential energy and time-loss diagnostics all
        // refer to the same final carrier geometry.
        if (!PredictInSubjectTimeParameterization(guide))
            return false;

        BuildKnotParameters();

        if (!TryResolveTargetStairWayParent(entrySample, out Transform stairWayParent))
            return false;

        if (!CreateFinalSplineContainer(stairWayParent))
            return false;

        generatedSectionKey = sectionKey;
        generatedSplineIndex = entrySample.splineIndex;
        generatedSectionIndex = entrySample.sectionIndex;
        activeStairWayParent = stairWayParent;

        splineContainer.gameObject.name =
            $"EqualizerFuture_{generatedSplineIndex}_{generatedSectionIndex}";

        directionReady = true;
        directionVersion++;

        // With the switch ON, the carrier is rebuilt by definite integration,
        // anchored at the SlopeStickCore entry X/Z plus Inspector CY only.
        TryApplyDefiniteIntegralReconstruction(guide);

        BuildFinalSplineInExistingContainer();

        EvaluateReferenceDiagnosticsAndLimit();
        EvaluateFinalTeacherAngleDiagnostics();

        Debug.Log(
            $"[EQUALIZER FUTURE FINAL BASE] " +
            $"version={directionVersion} " +
            $"spline={generatedSplineIndex} " +
            $"section={generatedSectionIndex} " +
            $"knots={generatedKnotCount} " +
            $"teacher={builtWithReferenceTeacher} " +
            $"teacherState={directionTeacherState} " +
            $"teacherScore={teacherApplicabilityScore:F3} " +
            $"teacherAuthority={teacherApplicabilityAuthorityScale:F3} " +
            $"teacherValidRatio={teacherValidSampleRatio:F3} " +
            $"teacherMeanMetric={teacherMeanDirectionMetric:F3} " +
            $"teacherMeanForwardDot={teacherMeanForwardDot:F3} " +
            $"teacherSamples={referenceMatchedSamples} " +
            $"rawTeacherSamples={rawTeacherSamples} " +
            $"rawAngleAvg={rawTeacherAverageAngleDegrees:F2}deg " +
            $"rawAngleMax={rawTeacherMaximumAngleDegrees:F2}deg " +
            $"finalAngleSamples={finalTeacherAngleSamples} " +
            $"finalAngleAvg={finalTeacherAverageAngleDegrees:F2}deg " +
            $"finalAngleMax={finalTeacherMaximumAngleDegrees:F2}deg " +
            $"pavaAdjusted={teacherMonotoneAdjustedSamples} " +
            $"CY={appliedInitialCY:F4}m " +
            $"firstWavePeak={firstWavePeakCorrectionActive} " +
            $"firstWavePeakU={firstWavePeakProgress01:F4} " +
            $"firstWavePeakCY={firstWavePeakTargetCY:F4}m " +
            $"definiteIntegral={definiteIntegralApplied} " +
            $"definiteIntegralUsingCY={definiteIntegralUsingCYBoundary} " +
            $"specificEnergyLoss={predictedSpecificEnergyLossJPerKg:F4}J/kg " +
            $"lossFreeTime={predictedLossFreeSectionDuration:F4}s " +
            $"timeLoss={predictedDifferentialEnergyTimeLossSeconds:F4}s " +
            $"timeLossRatio={predictedDifferentialEnergyTimeLossRatio:F4} " +
            $"energyResidualMax={maximumEnergyBalanceResidualJPerKg:F5}J/kg " +
            $"entryTime={predictedTimeToSlopeEntry:F4}s " +
            $"entryVT={predictedEntryTangentSpeed:F4}m/s " +
            $"parent={generatedStairWayParentName}",
            this);

        return true;
    }

    private bool BuildInSubjectSectionFrame(
        NearestKnotDetector.GuideFrame guide)
    {
        Vector3 previousTangent = Vector3.zero;
        Vector3 previousNormal = Vector3.up;

        for (int i = 0; i < sampleU.Length; i++)
        {
            float u = i / (float)(sampleU.Length - 1);

            if (!knotDetector.TryEvaluateForwardSlopeSection(
                    guide,
                    u,
                    out NearestKnotDetector.GuideSample sample))
            {
                return false;
            }

            Vector3 tangent = NormalizeSafe(
                sample.tangent,
                i > 0 ? previousTangent : guide.tangent);

            Vector3 normal;

            if (i == 0)
            {
                normal = BuildNormal(tangent, sample.normal);
            }
            else
            {
                Quaternion transport = Quaternion.FromToRotation(previousTangent, tangent);
                normal = Vector3.ProjectOnPlane(transport * previousNormal, tangent);

                if (normal.sqrMagnitude <= Eps)
                    normal = BuildNormal(tangent, sample.normal);
                else
                    normal.Normalize();

                if (sample.normal.sqrMagnitude > Eps &&
                    Vector3.Dot(normal, sample.normal.normalized) < 0f)
                {
                    normal = -normal;
                }
            }

            sampleU[i] = u;
            sampleS[i] = u * activeSectionLength;
            baseTangents[i] = tangent;
            baseNormals[i] = normal;
            basePoints[i] = sample.point + normal * capturedInSubjectClearance;

            previousTangent = tangent;
            previousNormal = normal;
        }

        return true;
    }

    private void BuildRawDirectionField()
    {
        Vector3 v0Direction = NormalizeSafe(
            inSubjectBody.velocity,
            baseTangents[0]);

        for (int i = 0; i < sampleU.Length; i++)
        {
            float dHeightDs = EvaluateWaveSlopeByDistance(sampleS[i]);

            Vector3 tangent = NormalizeSafe(
                baseTangents[i] + baseNormals[i] * dHeightDs,
                baseTangents[i]);

            float forwardDot = Mathf.Max(
                0f,
                Vector3.Dot(v0Direction, baseTangents[i]));

            float fade = 1f - Mathf.SmoothStep(
                0f,
                1f,
                Mathf.Clamp01(sampleU[i] / Mathf.Max(0.001f, inSubjectV0DirectionFade01)));

            float v0Weight =
                Mathf.Clamp01(inSubjectV0DirectionInfluence) *
                fade *
                forwardDot;

            rawTangents[i] = NormalizeSafe(
                Vector3.Slerp(tangent, v0Direction, v0Weight),
                tangent);

            directionTangents[i] = rawTangents[i];
        }
    }

    private bool TryApplyReferenceTeacher(
        int splineIndex,
        int sectionIndex)
    {
        referenceMatchedSamples = 0;
        rawTeacherSamples = 0;
        teacherMonotoneAdjustedSamples = 0;
        rawTeacherAverageAngleDegrees = 0f;
        rawTeacherMaximumAngleDegrees = 0f;
        directionTeacherState = DirectionTeacherState.Unavailable;
        teacherApplicabilityScore = 0f;
        teacherApplicabilityAuthorityScale = 0f;
        teacherValidSampleRatio = 0f;
        teacherMeanDirectionMetric = 0f;
        teacherMeanForwardDot = 0f;
        teacherSeedAvailableSamples = 0;
        teacherProjectedSamples = 0;

        if (directionTeacher == null)
            ResolveDirectionTeacher();

        CopyRawTangentsToDirection();

        if (directionTeacher == null ||
            splineIndex < 0 ||
            sectionIndex < 0)
        {
            return false;
        }

        // Phase A: arc-length seed + local Newton projection.
        // These establish a local correspondence only. Absolute 3D distance is
        // not used as tangent authority here.
        int candidateCount = 0;

        for (int i = 0; i < sampleU.Length; i++)
        {
            teacherCandidateValid[i] = false;
            teacherRawU[i] = sampleU[i];
            teacherMappedU[i] = sampleU[i];
            teacherDistance[i] = float.PositiveInfinity;
            teacherMappedDistance[i] = float.PositiveInfinity;
            teacherDeltaS[i] = 0f;
            teacherDeltaN[i] = 0f;
            teacherDeltaB[i] = 0f;
            teacherDirectionMetric[i] = float.PositiveInfinity;
            teacherDistanceConfidence[i] = 0f;
            teacherForwardConfidence[i] = 0f;
            teacherGain[i] = 0f;
            teacherReferenceTangents[i] = rawTangents[i];

            if (!directionTeacher.TryGetDirectionAtArcProgress(
                    splineIndex,
                    sectionIndex,
                    sampleU[i],
                    Mathf.Clamp(referenceArcLengthSamples, 32, 192),
                    out float seedTeacherU,
                    out _,
                    out _))
            {
                continue;
            }

            teacherSeedAvailableSamples++;

            if (!directionTeacher.TryProjectDirectionLocalNewton(
                    splineIndex,
                    sectionIndex,
                    basePoints[i],
                    seedTeacherU,
                    Mathf.Clamp(referenceNewtonHalfWindowU, 0.01f, 0.35f),
                    Mathf.Clamp(referenceNewtonIterations, 1, 8),
                    out float projectedTeacherU,
                    out _,
                    out Vector3 projectedTeacherTangent,
                    out float projectedDistance))
            {
                continue;
            }

            teacherProjectedSamples++;

            projectedTeacherTangent =
                NormalizeSafe(projectedTeacherTangent, rawTangents[i]);

            float projectedForwardDot =
                Vector3.Dot(projectedTeacherTangent, baseTangents[i]);

            // Direction reversal is a correspondence failure. Position separation
            // is not used as a hard rejection because this engine owns direction,
            // not the external position constant C.
            if (projectedForwardDot < minimumReferenceForwardDot ||
                !IsFinite(projectedDistance))
            {
                continue;
            }

            teacherRawU[i] = Mathf.Clamp01(projectedTeacherU);
            teacherMappedU[i] = teacherRawU[i];
            teacherDistance[i] = projectedDistance;
            teacherCandidateValid[i] = true;
            pavaCandidateIndex[candidateCount++] = i;
        }

        if (teacherSeedAvailableSamples <= 0)
        {
            directionTeacherState = DirectionTeacherState.Unavailable;
            return false;
        }

        int minimumCandidateCount =
            Mathf.Max(
                3,
                Mathf.CeilToInt(
                    sampleU.Length *
                    Mathf.Clamp01(weakMinimumValidSampleRatio)));

        if (candidateCount < minimumCandidateCount)
        {
            directionTeacherState = DirectionTeacherState.Rejected;
            return false;
        }

        // Phase B: preserve order. PAVA is deliberately unweighted here so that
        // absolute position distance cannot become a hidden direction authority.
        ApplyWeightedPava(candidateCount);

        float angleSum = 0f;
        float metricScale = Mathf.Max(
            0.001f,
            activeSectionLength * Mathf.Clamp(directionMetricScaleFraction, 0.01f, 1f));

        float wt = Mathf.Max(0f, directionMetricTangentWeight);
        float wn = Mathf.Max(0f, directionMetricNormalWeight);
        float wb = Mathf.Max(0f, directionMetricBinormalWeight);

        // A zero metric has no mathematical information. Fall back to the raw
        // direction field instead of inventing a confidence model.
        if (wt + wn + wb <= Eps)
        {
            directionTeacherState = DirectionTeacherState.Rejected;
            CopyRawTangentsToDirection();
            return false;
        }

        // Phase C: re-evaluate the Teacher at the PAVA-mapped parameter, then
        // build a dimensionless T/N/B correspondence metric. Only dimensionless
        // geometry, tangent orientation, and valid coverage decide applicability.
        float metricSum = 0f;
        float forwardDotSum = 0f;

        for (int k = 0; k < candidateCount; k++)
        {
            int i = pavaCandidateIndex[k];

            if (!directionTeacher.TryEvaluateDirectionAtNormalizedU(
                    splineIndex,
                    sectionIndex,
                    teacherMappedU[i],
                    out Vector3 referencePoint,
                    out Vector3 referenceTangent))
            {
                teacherCandidateValid[i] = false;
                continue;
            }

            referenceTangent =
                NormalizeSafe(referenceTangent, rawTangents[i]);

            Vector3 frameT =
                NormalizeSafe(baseTangents[i], rawTangents[i]);

            Vector3 frameN =
                BuildNormal(frameT, baseNormals[i]);

            Vector3 frameB =
                NormalizeSafe(
                    Vector3.Cross(frameN, frameT),
                    Vector3.right);

            frameN =
                NormalizeSafe(
                    Vector3.Cross(frameT, frameB),
                    frameN);

            float forwardDot =
                Vector3.Dot(referenceTangent, frameT);

            // Tangent orientation disagreement is a separate failure mode and is
            // never rescued by position proximity.
            if (forwardDot < minimumReferenceForwardDot)
            {
                teacherCandidateValid[i] = false;
                continue;
            }

            Vector3 deltaP =
                referencePoint - basePoints[i];

            float deltaS = Vector3.Dot(deltaP, frameT);
            float deltaN = Vector3.Dot(deltaP, frameN);
            float deltaB = Vector3.Dot(deltaP, frameB);

            float ns = deltaS / metricScale;
            float nn = deltaN / metricScale;
            float nb = deltaB / metricScale;

            float metricSquared =
                wt * ns * ns +
                wn * nn * nn +
                wb * nb * nb;

            if (!IsFinite(metricSquared) || metricSquared < 0f)
            {
                teacherCandidateValid[i] = false;
                continue;
            }

            float directionMetric =
                Mathf.Sqrt(metricSquared);

            float distanceConfidence =
                Mathf.Exp(-0.5f * metricSquared);

            float forwardConfidence =
                Mathf.InverseLerp(
                    minimumReferenceForwardDot,
                    1f,
                    forwardDot);

            float confidence =
                Mathf.Clamp01(
                    distanceConfidence *
                    forwardConfidence);

            float gain =
                Mathf.Clamp01(referenceTangentBlend) *
                confidence;

            teacherMappedDistance[i] = deltaP.magnitude;
            teacherDeltaS[i] = deltaS;
            teacherDeltaN[i] = deltaN;
            teacherDeltaB[i] = deltaB;
            teacherDirectionMetric[i] = directionMetric;
            teacherDistanceConfidence[i] = distanceConfidence;
            teacherForwardConfidence[i] = forwardConfidence;
            teacherGain[i] = gain;
            teacherReferenceTangents[i] = referenceTangent;

            metricSum += directionMetric;
            forwardDotSum += forwardDot;

            float rawAngle =
                Vector3.Angle(rawTangents[i], referenceTangent);

            angleSum += rawAngle;
            rawTeacherMaximumAngleDegrees =
                Mathf.Max(rawTeacherMaximumAngleDegrees, rawAngle);
            rawTeacherSamples++;

            referenceMatchedSamples++;
        }

        rawTeacherAverageAngleDegrees =
            rawTeacherSamples > 0
                ? angleSum / rawTeacherSamples
                : 0f;

        teacherValidSampleRatio =
            sampleU.Length > 0
                ? referenceMatchedSamples / (float)sampleU.Length
                : 0f;

        teacherMeanDirectionMetric =
            referenceMatchedSamples > 0
                ? metricSum / referenceMatchedSamples
                : float.PositiveInfinity;

        teacherMeanForwardDot =
            referenceMatchedSamples > 0
                ? forwardDotSum / referenceMatchedSamples
                : -1f;

        EvaluateDirectionTeacherApplicability();

        if (directionTeacherState == DirectionTeacherState.Rejected ||
            directionTeacherState == DirectionTeacherState.Unavailable)
        {
            for (int i = 0; i < teacherGain.Length; i++)
                teacherGain[i] = 0f;

            CopyRawTangentsToDirection();
            return false;
        }

        // Phase D: apply the teacher only after global mathematical
        // applicability is established. Weak applicability scales the local
        // confidence continuously; Applicable leaves it unchanged.
        for (int i = 0; i < sampleU.Length; i++)
        {
            if (!teacherCandidateValid[i])
                continue;

            float appliedGain =
                Mathf.Clamp01(
                    teacherGain[i] *
                    teacherApplicabilityAuthorityScale);

            teacherGain[i] = appliedGain;

            directionTangents[i] =
                NormalizeSafe(
                    Vector3.Slerp(
                        rawTangents[i],
                        teacherReferenceTangents[i],
                        appliedGain),
                    rawTangents[i]);
        }

        return true;
    }

    private void EvaluateDirectionTeacherApplicability()
    {
        float weakRatio = Mathf.Clamp01(weakMinimumValidSampleRatio);
        float strongRatio = Mathf.Clamp(
            applicableMinimumValidSampleRatio,
            weakRatio,
            1f);

        float strongMetric = Mathf.Max(
            0.0001f,
            applicableMaximumMeanDirectionMetric);

        float weakMetric = Mathf.Max(
            strongMetric,
            weakMaximumMeanDirectionMetric);

        float weakDot = Mathf.Clamp(
            weakMinimumMeanForwardDot,
            -1f,
            1f);

        float strongDot = Mathf.Clamp(
            applicableMinimumMeanForwardDot,
            weakDot,
            1f);

        bool finiteAggregate =
            IsFinite(teacherValidSampleRatio) &&
            IsFinite(teacherMeanDirectionMetric) &&
            IsFinite(teacherMeanForwardDot);

        if (!finiteAggregate || referenceMatchedSamples <= 0)
        {
            directionTeacherState =
                teacherSeedAvailableSamples > 0
                    ? DirectionTeacherState.Rejected
                    : DirectionTeacherState.Unavailable;

            teacherApplicabilityScore = 0f;
            teacherApplicabilityAuthorityScale = 0f;
            return;
        }

        bool stronglyApplicable =
            teacherValidSampleRatio >= strongRatio &&
            teacherMeanDirectionMetric <= strongMetric &&
            teacherMeanForwardDot >= strongDot;

        bool weaklyApplicable =
            teacherValidSampleRatio >= weakRatio &&
            teacherMeanDirectionMetric <= weakMetric &&
            teacherMeanForwardDot >= weakDot;

        float coverageScore =
            strongRatio > weakRatio + Eps
                ? Mathf.InverseLerp(
                    weakRatio,
                    strongRatio,
                    teacherValidSampleRatio)
                : (teacherValidSampleRatio >= strongRatio ? 1f : 0f);

        float metricScore =
            weakMetric > strongMetric + Eps
                ? 1f - Mathf.InverseLerp(
                    strongMetric,
                    weakMetric,
                    teacherMeanDirectionMetric)
                : (teacherMeanDirectionMetric <= strongMetric ? 1f : 0f);

        float orientationScore =
            strongDot > weakDot + Eps
                ? Mathf.InverseLerp(
                    weakDot,
                    strongDot,
                    teacherMeanForwardDot)
                : (teacherMeanForwardDot >= strongDot ? 1f : 0f);

        // Geometric mean: a collapsed condition cannot be hidden by the other
        // two. Every term is dimensionless and can be reused by other modules.
        teacherApplicabilityScore = Mathf.Pow(
            Mathf.Clamp01(coverageScore) *
            Mathf.Clamp01(metricScore) *
            Mathf.Clamp01(orientationScore),
            1f / 3f);

        if (stronglyApplicable)
        {
            directionTeacherState = DirectionTeacherState.Applicable;
            teacherApplicabilityAuthorityScale = 1f;
        }
        else if (weaklyApplicable)
        {
            directionTeacherState = DirectionTeacherState.WeaklyApplicable;
            teacherApplicabilityAuthorityScale =
                Mathf.Clamp01(teacherApplicabilityScore);
        }
        else
        {
            directionTeacherState = DirectionTeacherState.Rejected;
            teacherApplicabilityAuthorityScale = 0f;
        }
    }

    private void ApplyWeightedPava(int candidateCount)
    {
        int blockCount = 0;

        for (int k = 0; k < candidateCount; k++)
        {
            int sampleIndex = pavaCandidateIndex[k];

            pavaBlockStart[blockCount] = k;
            pavaBlockEnd[blockCount] = k;
            pavaBlockMean[blockCount] = teacherRawU[sampleIndex];

            // Standard isotonic projection in teacher-U space. Position distance
            // is intentionally not a weight because correspondence order and
            // direction authority are separate mathematical responsibilities.
            pavaBlockWeight[blockCount] = 1f;

            blockCount++;

            while (blockCount >= 2 &&
                   pavaBlockMean[blockCount - 2] >
                   pavaBlockMean[blockCount - 1])
            {
                int left = blockCount - 2;
                int right = blockCount - 1;

                float leftWeight = pavaBlockWeight[left];
                float rightWeight = pavaBlockWeight[right];
                float weight = leftWeight + rightWeight;

                pavaBlockMean[left] =
                    weight > Eps
                        ? (pavaBlockMean[left] * leftWeight +
                           pavaBlockMean[right] * rightWeight) / weight
                        : 0.5f *
                          (pavaBlockMean[left] + pavaBlockMean[right]);

                pavaBlockWeight[left] = weight;
                pavaBlockEnd[left] = pavaBlockEnd[right];
                blockCount--;
            }
        }

        for (int block = 0; block < blockCount; block++)
        {
            float mapped =
                Mathf.Clamp01(pavaBlockMean[block]);

            for (int k = pavaBlockStart[block];
                 k <= pavaBlockEnd[block];
                 k++)
            {
                int sampleIndex = pavaCandidateIndex[k];

                if (Mathf.Abs(mapped - teacherRawU[sampleIndex]) > 0.0005f)
                    teacherMonotoneAdjustedSamples++;

                teacherMappedU[sampleIndex] = mapped;
            }
        }
    }

    private void CopyRawTangentsToDirection()
    {
        for (int i = 0; i < rawTangents.Length; i++)
            directionTangents[i] = rawTangents[i];
    }

    private void RebuildOrthonormalFrameFromDirection()
    {
        Vector3 previousTangent = Vector3.zero;
        Vector3 previousNormal = Vector3.up;

        for (int i = 0; i < directionTangents.Length; i++)
        {
            Vector3 tangent = NormalizeSafe(directionTangents[i], baseTangents[i]);
            Vector3 normal;

            if (i == 0)
            {
                normal = BuildNormal(tangent, baseNormals[i]);
            }
            else
            {
                Quaternion transport = Quaternion.FromToRotation(previousTangent, tangent);
                normal = Vector3.ProjectOnPlane(transport * previousNormal, tangent);

                if (normal.sqrMagnitude <= Eps)
                    normal = BuildNormal(tangent, baseNormals[i]);
                else
                    normal.Normalize();

                if (Vector3.Dot(normal, baseNormals[i]) < 0f)
                    normal = -normal;
            }

            Vector3 binormal = NormalizeSafe(
                Vector3.Cross(normal, tangent),
                Vector3.right);

            normal = NormalizeSafe(
                Vector3.Cross(tangent, binormal),
                normal);

            directionTangents[i] = tangent;
            directionNormals[i] = normal;
            directionBinormals[i] = binormal;

            previousTangent = tangent;
            previousNormal = normal;
        }
    }

    private float ResolveInitialCY()
    {
        ResolveVerticalBoundarySource();

        float cy =
            verticalBoundarySource
                ? verticalBoundarySource.InitialCY
                : 0f;

        if (!IsFinite(cy))
            cy = 0f;

        appliedInitialCY = cy;
        return cy;
    }

    private void CaptureUncorrectedDirectionTangents()
    {
        if (uncorrectedDirectionTangents == null ||
            uncorrectedDirectionTangents.Length != directionTangents.Length)
        {
            return;
        }

        for (int i = 0; i < directionTangents.Length; i++)
            uncorrectedDirectionTangents[i] = directionTangents[i];
    }

    private void RestoreUncorrectedDirectionTangents()
    {
        if (uncorrectedDirectionTangents == null ||
            uncorrectedDirectionTangents.Length != directionTangents.Length)
        {
            return;
        }

        for (int i = 0; i < directionTangents.Length; i++)
        {
            directionTangents[i] = NormalizeSafe(
                uncorrectedDirectionTangents[i],
                baseTangents[i]);
        }
    }

    private void RebuildDirectionFromCarrier()
    {
        if (carrierPoints == null ||
            carrierPoints.Length < 2)
        {
            return;
        }

        for (int i = 0; i < carrierPoints.Length; i++)
        {
            Vector3 chord;

            if (i == 0)
            {
                chord = carrierPoints[1] - carrierPoints[0];
            }
            else if (i == carrierPoints.Length - 1)
            {
                chord = carrierPoints[i] - carrierPoints[i - 1];
            }
            else
            {
                chord = carrierPoints[i + 1] - carrierPoints[i - 1];
            }

            directionTangents[i] = NormalizeSafe(chord, directionTangents[i]);
        }
    }

    private void ApplyFirstWaveCYBoundaryCorrection()
    {
        if (!useFirstWaveCYBoundary ||
            carrierPoints == null ||
            basePoints == null ||
            sampleU == null ||
            carrierPoints.Length != basePoints.Length ||
            carrierPoints.Length != sampleU.Length ||
            carrierPoints.Length < 3)
        {
            return;
        }

        float cy = ResolveInitialCY();
        float firstWaveEnd01 = 1f / WavesPerStair;

        float currentStartOffset = carrierPoints[0].y - basePoints[0].y;
        float startCorrection = cy - currentStartOffset;

        if (!firstWavePeakCorrectionActive)
        {
            for (int i = 0; i < carrierPoints.Length; i++)
            {
                float u = sampleU[i];
                if (u > firstWaveEnd01)
                    break;

                float correction = EvaluateHermiteScalar(
                    u,
                    0f,
                    firstWaveEnd01,
                    startCorrection,
                    0f,
                    0f,
                    0f);

                carrierPoints[i] += Vector3.up * correction;
            }

            return;
        }

        int peakIndex = Mathf.Clamp(
            Mathf.RoundToInt(firstWavePeakProgress01 * (sampleU.Length - 1)),
            1,
            sampleU.Length - 2);

        float peakU = sampleU[peakIndex];

        if (peakU <= Eps ||
            peakU >= firstWaveEnd01 - Eps)
        {
            firstWavePeakCorrectionActive = false;
            ApplyFirstWaveCYBoundaryCorrection();
            return;
        }

        firstWavePeakProgress01 = peakU;

        float currentPeakCY =
            carrierPoints[peakIndex].y -
            basePoints[peakIndex].y;

        firstWavePeakPredictedCYBeforeCorrection = currentPeakCY;
        firstWavePeakErrorBeforeCorrection = firstWavePeakTargetCY - currentPeakCY;

        float peakCorrection = firstWavePeakErrorBeforeCorrection;

        float previousOffset =
            carrierPoints[peakIndex - 1].y -
            basePoints[peakIndex - 1].y;

        float nextOffset =
            carrierPoints[peakIndex + 1].y -
            basePoints[peakIndex + 1].y;

        float du = Mathf.Max(
            Eps,
            sampleU[peakIndex + 1] - sampleU[peakIndex - 1]);

        float currentPeakOffsetDerivativeU =
            (nextOffset - previousOffset) / du;

        float peakCorrectionDerivativeU =
            -currentPeakOffsetDerivativeU;

        for (int i = 0; i < carrierPoints.Length; i++)
        {
            float u = sampleU[i];
            if (u > firstWaveEnd01)
                break;

            float correction;

            if (u <= peakU)
            {
                correction = EvaluateHermiteScalar(
                    u,
                    0f,
                    peakU,
                    startCorrection,
                    peakCorrection,
                    0f,
                    peakCorrectionDerivativeU);
            }
            else
            {
                correction = EvaluateHermiteScalar(
                    u,
                    peakU,
                    firstWaveEnd01,
                    peakCorrection,
                    0f,
                    peakCorrectionDerivativeU,
                    0f);
            }

            carrierPoints[i] += Vector3.up * correction;
        }
    }

    private static float EvaluateHermiteScalar(
        float x,
        float x0,
        float x1,
        float y0,
        float y1,
        float dy0dx,
        float dy1dx)
    {
        float width = Mathf.Max(Eps, x1 - x0);
        float t = Mathf.Clamp01((x - x0) / width);
        float t2 = t * t;
        float t3 = t2 * t;

        float h00 = 2f * t3 - 3f * t2 + 1f;
        float h10 = t3 - 2f * t2 + t;
        float h01 = -2f * t3 + 3f * t2;
        float h11 = t3 - t2;

        return
            h00 * y0 +
            h10 * width * dy0dx +
            h01 * y1 +
            h11 * width * dy1dx;
    }

    private void TryRefreshFromObservedFirstWavePeak()
    {
        if (!directionReady ||
            generatedSectionKey == long.MinValue)
        {
            return;
        }

        ResolveVerticalBoundarySource();

        if (!verticalBoundarySource ||
            !verticalBoundarySource.TryGetFirstWavePeak(
                generatedSectionKey,
                out EqualizerFutureAsyncPos.FirstWavePeakObservation peak) ||
            !peak.valid ||
            peak.revision == consumedFirstWavePeakRevision)
        {
            return;
        }

        float cy = ResolveInitialCY();
        float firstWaveEnd01 = 1f / WavesPerStair;

        if (!IsFinite(peak.progress01) ||
            !IsFinite(peak.cy) ||
            peak.progress01 <= Eps ||
            peak.progress01 >= firstWaveEnd01 - Eps ||
            peak.cy <= cy + 0.0001f)
        {
            consumedFirstWavePeakRevision = peak.revision;

            Debug.LogWarning(
                $"[EQUALIZER FUTURE FIRST WAVE REFRESH REJECTED] " +
                $"revision={peak.revision} " +
                $"u={peak.progress01:F4} " +
                $"peakCY={peak.cy:F4}m " +
                $"initialCY={cy:F4}m",
                this);
            return;
        }

        firstWavePeakCorrectionActive = true;
        firstWavePeakProgress01 = peak.progress01;
        firstWavePeakTargetCY = peak.cy;

        NearestKnotDetector.GuideFrame timingGuide =
            predictionSourceGuideValid
                ? predictionSourceGuide
                : knotDetector.CurrentGuide;

        if (useDefiniteIntegralPositionReconstruction)
        {
            if (!TryApplyDefiniteIntegralReconstruction(timingGuide))
                return;
        }
        else
        {
            RestoreUncorrectedDirectionTangents();
            RebuildOrthonormalFrameFromDirection();
            IntegrateDirectionCarrier();
            ApplyFirstWaveCYBoundaryCorrection();
            RebuildDirectionFromCarrier();
            RebuildOrthonormalFrameFromDirection();
            PredictInSubjectTimeParameterization(timingGuide);
        }

        consumedFirstWavePeakRevision = peak.revision;
        directionVersion++;

        if (definiteIntegralApplied)
            definiteIntegralAppliedDirectionVersion = directionVersion;

        BuildKnotParameters();
        BuildFinalSplineInExistingContainer();
        EvaluateReferenceDiagnosticsAndLimit();
        EvaluateFinalTeacherAngleDiagnostics();

        Debug.Log(
            $"[EQUALIZER FUTURE FIRST WAVE REFRESH] " +
            $"version={directionVersion} " +
            $"revision={peak.revision} " +
            $"spline={generatedSplineIndex} " +
            $"section={generatedSectionIndex} " +
            $"initialCY={appliedInitialCY:F4}m " +
            $"peakU={firstWavePeakProgress01:F4} " +
            $"targetCY={firstWavePeakTargetCY:F4}m " +
            $"predictedBefore={firstWavePeakPredictedCYBeforeCorrection:F4}m " +
            $"errorBefore={firstWavePeakErrorBeforeCorrection:F4}m",
            this);
    }

    private void IntegrateDirectionCarrier()
    {
        // Legacy carrier used as the safe fallback and as the default when the
        // definite-integral switch is OFF. X/Z stay on the SlopeStickCore-derived
        // base anchor; Inspector CY contributes only a Physics-Y integration constant.
        carrierPoints[0] =
            basePoints[0] +
            Vector3.up * ResolveInitialCY();

        for (int i = 1; i < carrierPoints.Length; i++)
        {
            float ds = Mathf.Max(Eps, sampleS[i] - sampleS[i - 1]);
            Vector3 midTangent = NormalizeSafe(
                directionTangents[i - 1] + directionTangents[i],
                directionTangents[i]);

            carrierPoints[i] = carrierPoints[i - 1] + midTangent * ds;
        }

        definiteIntegralApplied = false;
        definiteIntegralUsingCYBoundary = false;
        definiteIntegralAppliedDirectionVersion = -1;
        definiteIntegralAnchorC = Vector3.zero;
        definiteIntegralEndOffset = carrierPoints[carrierPoints.Length - 1] - carrierPoints[0];
        definiteIntegralEndOffsetMagnitude = definiteIntegralEndOffset.magnitude;
    }

    /// <summary>
    /// Composite-trapezoidal definite integral of the final unit tangent field:
    ///
    ///   P(s) = C + integral_0^s T(sigma) d sigma
    ///
    /// The anchor uses SlopeStickCore-derived base X/Z plus Inspector CY.
    /// EqualizerFutureAsyncPos never supplies an independent X/Z translation.
    /// </summary>
    private void IntegrateDirectionCarrierByDefiniteIntegral(
        Vector3 cPhysics,
        bool usingCYBoundary)
    {
        Vector3 integral = Vector3.zero;
        carrierPoints[0] = cPhysics;

        for (int i = 1; i < carrierPoints.Length; i++)
        {
            float ds = Mathf.Max(Eps, sampleS[i] - sampleS[i - 1]);

            Vector3 t0 = NormalizeSafe(
                directionTangents[i - 1],
                baseTangents[i - 1]);

            Vector3 t1 = NormalizeSafe(
                directionTangents[i],
                baseTangents[i]);

            // Composite trapezoidal rule:
            // integral_{s0}^{s1} T(s) ds ~= 1/2 (T0 + T1) Delta s.
            Vector3 differentialIntegral =
                0.5f * (t0 + t1) * ds;

            integral += differentialIntegral;
            carrierPoints[i] = cPhysics + integral;
        }

        definiteIntegralApplied = true;
        definiteIntegralUsingCYBoundary = usingCYBoundary;
        definiteIntegralAppliedDirectionVersion = directionVersion;
        definiteIntegralAnchorC = cPhysics;
        definiteIntegralEndOffset = integral;
        definiteIntegralEndOffsetMagnitude = integral.magnitude;
    }

    /// <summary>
    /// Applies the optional definite-integral reconstruction after a direction
    /// version has become authoritative. X/Z stay anchored to the SlopeStickCore
    /// entry carrier; only Inspector CY contributes to the integration anchor.
    /// </summary>
    private bool TryApplyDefiniteIntegralReconstruction(
        NearestKnotDetector.GuideFrame guide)
    {
        if (!useDefiniteIntegralPositionReconstruction ||
            !directionReady ||
            directionVersion < 0 ||
            generatedSectionKey == long.MinValue)
        {
            return false;
        }

        ResolveVerticalBoundarySource();

        float cy = ResolveInitialCY();
        Vector3 cPhysics =
            basePoints[0] +
            Vector3.up * cy;

        RestoreUncorrectedDirectionTangents();
        RebuildOrthonormalFrameFromDirection();

        IntegrateDirectionCarrierByDefiniteIntegral(
            cPhysics,
            verticalBoundarySource != null);

        ApplyFirstWaveCYBoundaryCorrection();
        RebuildDirectionFromCarrier();
        RebuildOrthonormalFrameFromDirection();

        // Geometry changed from normalized-midpoint marching to an actual
        // trapezoidal line integral plus the vertical boundary correction.
        if (!PredictInSubjectTimeParameterization(guide))
            return false;

        Debug.Log(
            $"[EQUALIZER FUTURE DEFINITE INTEGRAL] " +
            $"version={directionVersion} " +
            $"spline={generatedSplineIndex} section={generatedSectionIndex} " +
            $"usingCY={definiteIntegralUsingCYBoundary} " +
            $"C={definiteIntegralAnchorC:F3} " +
            $"endOffset={definiteIntegralEndOffset:F3} " +
            $"endOffsetMagnitude={definiteIntegralEndOffsetMagnitude:F4}m",
            this);

        return true;
    }

    private bool PredictInSubjectTimeParameterization(
        NearestKnotDetector.GuideFrame guide)
    {
        if (!PredictPlaneLeadToSlopeEntry(
                guide,
                out predictedTimeToSlopeEntry,
                out float speed,
                out float splineDrive))
        {
            return false;
        }

        predictedEntryTangentSpeed = Mathf.Max(minimumForwardSpeed, speed);

        predictedSpecificEnergyLossJPerKg = 0f;
        predictedLossFreeSectionDuration = 0f;
        predictedDifferentialEnergyTimeLossSeconds = 0f;
        predictedDifferentialEnergyTimeLossRatio = 0f;
        meanSpecificEnergyLossGradientJPerKgPerMeter = 0f;
        peakSpecificEnergyLossPowerWattsPerKg = 0f;
        maximumEnergyBalanceResidualJPerKg = 0f;

        sampleTime[0] = predictedTimeToSlopeEntry;
        sampleSpeed[0] = predictedEntryTangentSpeed;
        sampleLossFreeTime[0] = predictedTimeToSlopeEntry;
        sampleLossFreeSpeed[0] = predictedEntryTangentSpeed;
        sampleCumulativeTimeLoss[0] = 0f;

        float initialSpecificKineticEnergy =
            0.5f * predictedEntryTangentSpeed * predictedEntryTangentSpeed;

        sampleSpecificKineticEnergy[0] = initialSpecificKineticEnergy;
        sampleSpecificPotentialEnergy[0] = 0f;
        sampleSpecificMechanicalEnergy[0] = initialSpecificKineticEnergy;
        sampleDifferentialMechanicalEnergyPerMeter[0] = splineDrive;
        sampleDifferentialMechanicalPower[0] =
            splineDrive * predictedEntryTangentSpeed;
        sampleCumulativeSpecificEnergyLoss[0] = 0f;
        sampleEnergyBalanceResidual[0] = 0f;

        float elapsedOnSlope = 0f;
        float lossFreeElapsedOnSlope = 0f;
        float currentSpeed = predictedEntryTangentSpeed;
        float lossFreeSpeed = predictedEntryTangentSpeed;
        float currentDrive = splineDrive;
        float cumulativeSpecificLoss = 0f;
        float integratedSpecificNonConservativeWork = 0f;

        Vector3 energyOrigin =
            carrierPoints != null && carrierPoints.Length > 0
                ? carrierPoints[0]
                : basePoints[0];

        for (int i = 0; i < sampleU.Length - 1; i++)
        {
            float ds = Mathf.Max(Eps, sampleS[i + 1] - sampleS[i]);

            Vector3 tangent0 =
                directionTangents != null && directionTangents[i].sqrMagnitude > Eps
                    ? directionTangents[i]
                    : baseTangents[i];

            Vector3 tangent1 =
                directionTangents != null && directionTangents[i + 1].sqrMagnitude > Eps
                    ? directionTangents[i + 1]
                    : baseTangents[i + 1];

            Vector3 segmentTangent = NormalizeSafe(
                tangent0 + tangent1,
                tangent0);

            float gravityAlong =
                Vector3.Dot(Physics.gravity, segmentTangent);

            float desiredDrive = splineCore.PredictDesiredSplineDriveReadOnly(
                true,
                currentSpeed,
                sampleU[i],
                activeSectionLength,
                gravityAlong);

            float dt = ds / Mathf.Max(minimumForwardSpeed, currentSpeed);
            float nextDrive = currentDrive;
            float nextSpeed = currentSpeed;

            // Energy-consistent fixed-point refinement:
            //   v1^2 = v0^2 + 2 a_bar ds
            // while drive dynamics still evolve in time through dt.
            for (int iteration = 0; iteration < 2; iteration++)
            {
                nextDrive = splineCore.AdvancePredictedSplineDriveReadOnly(
                    currentDrive,
                    desiredDrive,
                    dt);

                float averageDrive =
                    0.5f * (currentDrive + nextDrive);

                float acceleration =
                    gravityAlong + averageDrive;

                float nextSpeedSquared =
                    currentSpeed * currentSpeed +
                    2f * acceleration * ds;

                nextSpeed = Mathf.Sqrt(
                    Mathf.Max(0f, nextSpeedSquared));

                float averageSpeed = Mathf.Max(
                    minimumForwardSpeed,
                    0.5f * (currentSpeed + nextSpeed));

                dt = ds / averageSpeed;
            }

            dt = Mathf.Min(dt, maximumPredictionTimeSeconds);

            float segmentAverageDrive =
                0.5f * (currentDrive + nextDrive);

            float segmentAverageSpeed = Mathf.Max(
                0f,
                0.5f * (currentSpeed + nextSpeed));

            // Work-energy theorem in specific-energy form:
            //   de_mech/ds = a_nonconservative.
            // Gravity is represented by potential energy and therefore does not
            // appear in this differential mechanical-energy term.
            float differentialEnergyPerMeter = segmentAverageDrive;
            float differentialPower =
                differentialEnergyPerMeter * segmentAverageSpeed;

            float segmentSpecificWork =
                differentialEnergyPerMeter * ds;

            integratedSpecificNonConservativeWork +=
                segmentSpecificWork;

            if (segmentSpecificWork < 0f)
                cumulativeSpecificLoss += -segmentSpecificWork;

            peakSpecificEnergyLossPowerWattsPerKg = Mathf.Max(
                peakSpecificEnergyLossPowerWattsPerKg,
                Mathf.Max(0f, -differentialPower));

            // Loss-free shadow: preserve the same geometry, gravity and positive
            // non-conservative work, but suppress negative work.  This isolates
            // delay caused by energy removal without turning this diagnostic into
            // a second trajectory authority.
            float lossFreeDrive =
                Mathf.Max(0f, segmentAverageDrive);

            float lossFreeAcceleration =
                gravityAlong + lossFreeDrive;

            float lossFreeNextSpeedSquared =
                lossFreeSpeed * lossFreeSpeed +
                2f * lossFreeAcceleration * ds;

            float lossFreeNextSpeed = Mathf.Sqrt(
                Mathf.Max(0f, lossFreeNextSpeedSquared));

            float lossFreeAverageSpeed = Mathf.Max(
                minimumForwardSpeed,
                0.5f * (lossFreeSpeed + lossFreeNextSpeed));

            float lossFreeDt = Mathf.Min(
                ds / lossFreeAverageSpeed,
                maximumPredictionTimeSeconds);

            elapsedOnSlope += dt;
            lossFreeElapsedOnSlope += lossFreeDt;

            sampleTime[i + 1] =
                predictedTimeToSlopeEntry + elapsedOnSlope;

            sampleSpeed[i + 1] = nextSpeed;

            sampleLossFreeTime[i + 1] =
                predictedTimeToSlopeEntry + lossFreeElapsedOnSlope;

            sampleLossFreeSpeed[i + 1] = lossFreeNextSpeed;

            sampleCumulativeTimeLoss[i + 1] = Mathf.Max(
                0f,
                elapsedOnSlope - lossFreeElapsedOnSlope);

            float specificKineticEnergy =
                0.5f * nextSpeed * nextSpeed;

            Vector3 energyPoint =
                carrierPoints != null && carrierPoints.Length > i + 1
                    ? carrierPoints[i + 1]
                    : basePoints[i + 1];

            float specificPotentialEnergy =
                -Vector3.Dot(
                    Physics.gravity,
                    energyPoint - energyOrigin);

            float specificMechanicalEnergy =
                specificKineticEnergy + specificPotentialEnergy;

            float energyBalanceResidual = Mathf.Abs(
                (specificMechanicalEnergy - initialSpecificKineticEnergy) -
                integratedSpecificNonConservativeWork);

            sampleSpecificKineticEnergy[i + 1] =
                specificKineticEnergy;

            sampleSpecificPotentialEnergy[i + 1] =
                specificPotentialEnergy;

            sampleSpecificMechanicalEnergy[i + 1] =
                specificMechanicalEnergy;

            sampleDifferentialMechanicalEnergyPerMeter[i + 1] =
                differentialEnergyPerMeter;

            sampleDifferentialMechanicalPower[i + 1] =
                differentialPower;

            sampleCumulativeSpecificEnergyLoss[i + 1] =
                cumulativeSpecificLoss;

            sampleEnergyBalanceResidual[i + 1] =
                energyBalanceResidual;

            maximumEnergyBalanceResidualJPerKg = Mathf.Max(
                maximumEnergyBalanceResidualJPerKg,
                energyBalanceResidual);

            currentSpeed = nextSpeed;
            lossFreeSpeed = lossFreeNextSpeed;
            currentDrive = nextDrive;
        }

        predictedSectionDuration = elapsedOnSlope;
        predictedLossFreeSectionDuration = lossFreeElapsedOnSlope;
        predictedSpecificEnergyLossJPerKg = cumulativeSpecificLoss;
        predictedDifferentialEnergyTimeLossSeconds = Mathf.Max(
            0f,
            predictedSectionDuration - predictedLossFreeSectionDuration);

        predictedDifferentialEnergyTimeLossRatio =
            predictedDifferentialEnergyTimeLossSeconds /
            Mathf.Max(Eps, predictedLossFreeSectionDuration);

        meanSpecificEnergyLossGradientJPerKgPerMeter =
            predictedSpecificEnergyLossJPerKg /
            Mathf.Max(Eps, activeSectionLength);

        return
            IsFinite(predictedSectionDuration) &&
            IsFinite(predictedLossFreeSectionDuration) &&
            IsFinite(predictedSpecificEnergyLossJPerKg) &&
            IsFinite(predictedDifferentialEnergyTimeLossSeconds) &&
            IsFinite(maximumEnergyBalanceResidualJPerKg);
    }

    private bool PredictPlaneLeadToSlopeEntry(
        NearestKnotDetector.GuideFrame guide,
        out float timeSeconds,
        out float entrySpeed,
        out float entryDrive)
    {
        timeSeconds = 0f;

        Vector3 tangent = NormalizeSafe(guide.tangent, Vector3.forward);
        float speed = Mathf.Max(0f, Vector3.Dot(inSubjectBody.velocity, tangent));
        float drive = splineCore.CurrentSplineDriveAccelerationReadOnly;
        float remaining = Mathf.Max(0f, guide.distanceToNextSlope);

        if (remaining <= Eps)
        {
            entrySpeed = speed;
            entryDrive = drive;
            return true;
        }

        float simulatedDistance = 0f;
        float dt = Mathf.Clamp(Time.fixedDeltaTime * 0.5f, 0.0025f, 0.02f);
        int guard = 0;

        while (simulatedDistance < remaining &&
               timeSeconds < maximumPredictionTimeSeconds &&
               guard++ < 2048)
        {
            float gravityAlong = Vector3.Dot(Physics.gravity, tangent);

            float desiredDrive = splineCore.PredictDesiredSplineDriveReadOnly(
                false,
                speed,
                0f,
                0f,
                gravityAlong);

            float nextDrive = splineCore.AdvancePredictedSplineDriveReadOnly(
                drive,
                desiredDrive,
                dt);

            float acceleration = gravityAlong + 0.5f * (drive + nextDrive);
            float nextSpeed = Mathf.Max(0f, speed + acceleration * dt);
            float ds = Mathf.Max(0f, 0.5f * (speed + nextSpeed) * dt);

            if (simulatedDistance + ds > remaining && ds > Eps)
            {
                float fraction = Mathf.Clamp01((remaining - simulatedDistance) / ds);
                timeSeconds += dt * fraction;
                speed = Mathf.Lerp(speed, nextSpeed, fraction);
                drive = Mathf.Lerp(drive, nextDrive, fraction);
                simulatedDistance = remaining;
                break;
            }

            simulatedDistance += ds;
            timeSeconds += dt;
            speed = nextSpeed;
            drive = nextDrive;
        }

        entrySpeed = speed;
        entryDrive = drive;

        return simulatedDistance >= remaining - 0.0001f;
    }

    private float EvaluateWaveSlopeByDistance(float sMeters)
    {
        float length = Mathf.Max(0.01f, activeSectionLength);
        float u = Mathf.Clamp01(sMeters / length);

        // E(u) = sin^2(pi u): zero wave influence at entry/exit.
        float sinPiU = Mathf.Sin(Mathf.PI * u);
        float envelope = sinPiU * sinPiU;
        float envelopeDerivativeU =
            Mathf.PI * Mathf.Sin(2f * Mathf.PI * u);

        float phase =
            2f * Mathf.PI * WavesPerStair * u;

        float phaseDerivativeU =
            2f * Mathf.PI * WavesPerStair;

        // H(u) = A E(u) sin(phi)
        // dH/ds = A [E'(u) sin(phi) + E(u) cos(phi) phi'(u)] / L
        float dHeightDs =
            waveAmplitude *
            (
                envelopeDerivativeU * Mathf.Sin(phase) +
                envelope * Mathf.Cos(phase) * phaseDerivativeU
            ) /
            length;

        float maxSlope =
            Mathf.Tan(
                Mathf.Clamp(
                    maximumWaveTangentAngleDegrees,
                    1f,
                    45f) *
                Mathf.Deg2Rad);

        return Mathf.Clamp(
            dHeightDs,
            -maxSlope,
            maxSlope);
    }

    private void BuildKnotParameters()
    {
        knotParameters.Clear();
        AddUniqueKnot(0f);

        int extremaCount = WavesPerStair * 2;

        for (int i = 0; i < extremaCount; i++)
        {
            float u = (2f * i + 1f) / (4f * WavesPerStair);
            AddUniqueKnot(u);
        }

        if (firstWavePeakCorrectionActive)
        {
            AddUniqueKnot(firstWavePeakProgress01);
            AddUniqueKnot(1f / WavesPerStair);
        }

        AddUniqueKnot(1f);
        knotParameters.Sort();
    }

    private void BuildFinalSplineInExistingContainer()
    {
        if (!splineContainer)
            return;

        if (splineContainer.Spline == null)
            splineContainer.Spline = new Spline();

        Spline spline = splineContainer.Spline;
        spline.Clear();
        spline.Closed = false;

        for (int i = 0; i < knotParameters.Count; i++)
        {
            float u = knotParameters[i];

            if (!TryEvaluateDirection(
                    u,
                    out DirectionSample sample))
            {
                continue;
            }

            Vector3 worldPoint =
                sample.carrierPoint;

            float previousU =
                i > 0
                    ? knotParameters[i - 1]
                    : u;

            float nextU =
                i < knotParameters.Count - 1
                    ? knotParameters[i + 1]
                    : u;

            Vector3 previousPoint = worldPoint;
            Vector3 nextPoint = worldPoint;

            if (i > 0 &&
                TryEvaluateDirection(
                    previousU,
                    out DirectionSample previousSample))
            {
                previousPoint =
                    previousSample.carrierPoint;
            }

            if (i < knotParameters.Count - 1 &&
                TryEvaluateDirection(
                    nextU,
                    out DirectionSample nextSample))
            {
                nextPoint =
                    nextSample.carrierPoint;
            }

            float previousHandleLength =
                i > 0
                    ? Vector3.Distance(
                        previousPoint,
                        worldPoint) / 3f
                    : 0f;

            float nextHandleLength =
                i < knotParameters.Count - 1
                    ? Vector3.Distance(
                        worldPoint,
                        nextPoint) / 3f
                    : 0f;

            Vector3 tangentInWorld =
                -sample.tangent *
                previousHandleLength;

            Vector3 tangentOutWorld =
                sample.tangent *
                nextHandleLength;

            Vector3 localPoint =
                splineContainer.transform.InverseTransformPoint(
                    worldPoint);

            Vector3 localTangentIn =
                splineContainer.transform.InverseTransformVector(
                    tangentInWorld);

            Vector3 localTangentOut =
                splineContainer.transform.InverseTransformVector(
                    tangentOutWorld);

            spline.Add(
                new BezierKnot(
                    ToFloat3(localPoint),
                    ToFloat3(localTangentIn),
                    ToFloat3(localTangentOut)),
                TangentMode.Broken);
        }

        generatedKnotCount = spline.Count;
    }


    // ---------------------------------------------------------------------
    // Raw Direction + CY comparison output
    // ---------------------------------------------------------------------
    // This replaces the former EqualizerFutureDirectionCYOnlySpline
    // MonoBehaviour. The comparison remains a separate generated SplineContainer,
    // but all code and state now live inside EqualizerFutureSpline.
    //
    // Mathematical definition:
    //   P_rawCY(s) = P_base(0) + (0, CY, 0) + integral_0^s T_raw(sigma) d sigma
    //
    // T_raw is the EqualizerFutureSpline raw direction field before:
    // - external teacher blending,
    // - EqualizerFutureAsyncPos / observed first-wave CY deformation,
    // - final carrier corrections.
    //
    // Consequently this output contains no BallVisualEqualizer-derived position,
    // velocity, peak, phase, or correspondence data.

    private void RefreshDirectionCYOnlySplineIfNeeded()
    {
        if (!generateDirectionCYOnlySpline)
        {
            DestroyDirectionCYOnlySplines();
            return;
        }

        if (!directionReady ||
            generatedSectionKey == long.MinValue ||
            !activeStairWayParent ||
            sampleS == null ||
            sampleS.Length < 2 ||
            basePoints == null ||
            rawTangents == null ||
            basePoints.Length != sampleS.Length ||
            rawTangents.Length != sampleS.Length)
        {
            return;
        }

        bool rawBasisChanged =
            directionCYOnlyConsumedRawBasisVersion !=
            rawDirectionBasisVersion;

        bool sectionChanged =
            directionCYOnlyGeneratedSectionKey !=
            generatedSectionKey;

        bool cyChanged =
            !IsFinite(directionCYOnlyLastBuiltCY) ||
            Mathf.Abs(
                directionCYOnlyInitialCY -
                directionCYOnlyLastBuiltCY) >
            Eps;

        if (!rawBasisChanged &&
            !sectionChanged &&
            !cyChanged)
        {
            return;
        }

        BuildDirectionCYOnlySpline();
    }

    private bool BuildDirectionCYOnlySpline()
    {
        if (!TryIntegrateDirectionCYOnlyCarrier())
            return false;

        if (!activeStairWayParent)
            return false;

        bool needsNewContainer =
            !directionCYOnlySplineContainer ||
            directionCYOnlyGeneratedSectionKey != generatedSectionKey ||
            directionCYOnlySplineContainer.transform.parent != activeStairWayParent;

        if (needsNewContainer)
        {
            if (!CreateDirectionCYOnlySplineContainer(activeStairWayParent))
                return false;
        }

        if (!directionCYOnlySplineContainer)
            return false;

        directionCYOnlySplineContainer.gameObject.name =
            $"EqualizerFutureDirectionCYOnly_" +
            $"{generatedSplineIndex}_" +
            $"{generatedSectionIndex}";

        BuildDirectionCYOnlyBezierSpline();

        directionCYOnlyConsumedRawBasisVersion =
            rawDirectionBasisVersion;

        directionCYOnlyGeneratedSectionKey =
            generatedSectionKey;

        directionCYOnlyAppliedCY =
            directionCYOnlyInitialCY;

        directionCYOnlyLastBuiltCY =
            directionCYOnlyInitialCY;

        Debug.Log(
            $"[EQUALIZER FUTURE DIRECTION CY ONLY] " +
            $"integratedInEqualizerFutureSpline=True " +
            $"rawBasisVersion={directionCYOnlyConsumedRawBasisVersion} " +
            $"spline={generatedSplineIndex} " +
            $"section={generatedSectionIndex} " +
            $"CY={directionCYOnlyAppliedCY:F4}m " +
            $"knots={directionCYOnlyGeneratedKnotCount} " +
            $"arc={directionCYOnlyIntegratedArcLength:F4}m " +
            $"start={directionCYOnlyStartPointPhysics:F3} " +
            $"end={directionCYOnlyEndPointPhysics:F3} " +
            $"AsyncPosInput=False " +
            $"BallVisualInput=False",
            this);

        return true;
    }

    private bool TryIntegrateDirectionCYOnlyCarrier()
    {
        int count =
            sampleS != null
                ? sampleS.Length
                : 0;

        if (count < 2 ||
            basePoints == null ||
            rawTangents == null ||
            baseTangents == null ||
            basePoints.Length != count ||
            rawTangents.Length != count ||
            baseTangents.Length != count)
        {
            return false;
        }

        if (directionCYOnlyPoints == null ||
            directionCYOnlyPoints.Length != count)
        {
            directionCYOnlyPoints =
                new Vector3[count];

            directionCYOnlyTangents =
                new Vector3[count];
        }

        directionCYOnlyTangents[0] =
            NormalizeSafe(
                rawTangents[0],
                baseTangents[0]);

        directionCYOnlyPoints[0] =
            basePoints[0] +
            Vector3.up *
            directionCYOnlyInitialCY;

        directionCYOnlyIntegratedArcLength = 0f;

        for (int i = 1; i < count; i++)
        {
            directionCYOnlyTangents[i] =
                NormalizeSafe(
                    rawTangents[i],
                    directionCYOnlyTangents[i - 1]);

            float ds =
                Mathf.Max(
                    0f,
                    sampleS[i] -
                    sampleS[i - 1]);

            // Composite trapezoidal definite integral:
            // P(s_i)=P(s_0)+1/2(T_{i-1}+T_i) Delta s.
            Vector3 differential =
                0.5f *
                (directionCYOnlyTangents[i - 1] +
                 directionCYOnlyTangents[i]) *
                ds;

            directionCYOnlyPoints[i] =
                directionCYOnlyPoints[i - 1] +
                differential;

            directionCYOnlyIntegratedArcLength += ds;
        }

        directionCYOnlyStartPointPhysics =
            directionCYOnlyPoints[0];

        directionCYOnlyEndPointPhysics =
            directionCYOnlyPoints[count - 1];

        return
            IsFinite(directionCYOnlyStartPointPhysics) &&
            IsFinite(directionCYOnlyEndPointPhysics);
    }

    private void BuildDirectionCYOnlyBezierSpline()
    {
        if (!directionCYOnlySplineContainer ||
            directionCYOnlyPoints == null ||
            directionCYOnlyTangents == null ||
            directionCYOnlyPoints.Length < 2 ||
            directionCYOnlyTangents.Length != directionCYOnlyPoints.Length)
        {
            return;
        }

        if (directionCYOnlySplineContainer.Spline == null)
            directionCYOnlySplineContainer.Spline = new Spline();

        Spline spline =
            directionCYOnlySplineContainer.Spline;

        spline.Clear();
        spline.Closed = false;

        int knots =
            Mathf.Clamp(
                directionCYOnlyOutputKnotCount,
                5,
                49);

        for (int i = 0; i < knots; i++)
        {
            float u =
                i / (float)(knots - 1);

            EvaluateDirectionCYOnlyCarrier(
                u,
                out Vector3 point,
                out Vector3 tangent);

            float previousU =
                i > 0
                    ? (i - 1) / (float)(knots - 1)
                    : u;

            float nextU =
                i < knots - 1
                    ? (i + 1) / (float)(knots - 1)
                    : u;

            Vector3 previousPoint = point;
            Vector3 nextPoint = point;

            if (i > 0)
            {
                EvaluateDirectionCYOnlyCarrier(
                    previousU,
                    out previousPoint,
                    out _);
            }

            if (i < knots - 1)
            {
                EvaluateDirectionCYOnlyCarrier(
                    nextU,
                    out nextPoint,
                    out _);
            }

            float previousHandleLength =
                i > 0
                    ? Vector3.Distance(
                        previousPoint,
                        point) / 3f
                    : 0f;

            float nextHandleLength =
                i < knots - 1
                    ? Vector3.Distance(
                        point,
                        nextPoint) / 3f
                    : 0f;

            Vector3 tangentInWorld =
                -tangent *
                previousHandleLength;

            Vector3 tangentOutWorld =
                tangent *
                nextHandleLength;

            Vector3 localPoint =
                directionCYOnlySplineContainer.transform.InverseTransformPoint(
                    point);

            Vector3 localTangentIn =
                directionCYOnlySplineContainer.transform.InverseTransformVector(
                    tangentInWorld);

            Vector3 localTangentOut =
                directionCYOnlySplineContainer.transform.InverseTransformVector(
                    tangentOutWorld);

            spline.Add(
                new BezierKnot(
                    ToFloat3(localPoint),
                    ToFloat3(localTangentIn),
                    ToFloat3(localTangentOut)),
                TangentMode.Broken);
        }

        directionCYOnlyGeneratedKnotCount =
            spline.Count;
    }

    private void EvaluateDirectionCYOnlyCarrier(
        float u,
        out Vector3 point,
        out Vector3 tangent)
    {
        point = Vector3.zero;
        tangent = Vector3.forward;

        if (directionCYOnlyPoints == null ||
            directionCYOnlyTangents == null ||
            directionCYOnlyPoints.Length < 2 ||
            directionCYOnlyTangents.Length != directionCYOnlyPoints.Length)
        {
            return;
        }

        u = Mathf.Clamp01(u);

        float scaled =
            u *
            (directionCYOnlyPoints.Length - 1);

        int i0 =
            Mathf.Clamp(
                Mathf.FloorToInt(scaled),
                0,
                directionCYOnlyPoints.Length - 1);

        int i1 =
            Mathf.Min(
                i0 + 1,
                directionCYOnlyPoints.Length - 1);

        float t =
            scaled - i0;

        point =
            Vector3.Lerp(
                directionCYOnlyPoints[i0],
                directionCYOnlyPoints[i1],
                t);

        tangent =
            NormalizeSafe(
                Vector3.Slerp(
                    directionCYOnlyTangents[i0],
                    directionCYOnlyTangents[i1],
                    t),
                directionCYOnlyTangents[i0]);
    }

    private bool CreateDirectionCYOnlySplineContainer(
        Transform stairWayParent)
    {
        if (!stairWayParent ||
            !IsStairWayTransform(stairWayParent))
        {
            return false;
        }

        GameObject obj =
            new GameObject(
                "EqualizerFutureDirectionCYOnly");

        obj.transform.SetParent(
            stairWayParent,
            false);

        obj.transform.localPosition =
            Vector3.zero;

        obj.transform.localRotation =
            Quaternion.identity;

        obj.transform.localScale =
            Vector3.one;

        directionCYOnlySplineContainer =
            obj.AddComponent<SplineContainer>();

        directionCYOnlySplineContainer.Spline =
            new Spline();

        RegisterDirectionCYOnlySpline(
            directionCYOnlySplineContainer);

        return true;
    }

    private void RegisterDirectionCYOnlySpline(
        SplineContainer created)
    {
        directionCYOnlySplineHistory.RemoveAll(
            item => !item);

        directionCYOnlySplineHistory.Add(
            created);

        // Same retention rule as the final FutureSpline:
        // current + previous only.
        if (directionCYOnlySplineHistory.Count >
            MaxRetainedFutureSplines)
        {
            SplineContainer twoStepsAgo =
                directionCYOnlySplineHistory[0];

            directionCYOnlySplineHistory.RemoveAt(0);

            if (twoStepsAgo)
                Destroy(twoStepsAgo.gameObject);
        }

        retainedDirectionCYOnlySplineCount =
            directionCYOnlySplineHistory.Count;
    }

    private void DestroyDirectionCYOnlySplines()
    {
        directionCYOnlySplineHistory.RemoveAll(
            item => !item);

        for (int i = 0;
             i < directionCYOnlySplineHistory.Count;
             i++)
        {
            SplineContainer item =
                directionCYOnlySplineHistory[i];

            if (item)
                Destroy(item.gameObject);
        }

        directionCYOnlySplineHistory.Clear();
        directionCYOnlySplineContainer = null;
        retainedDirectionCYOnlySplineCount = 0;
        directionCYOnlyConsumedRawBasisVersion = -1;
        directionCYOnlyGeneratedSectionKey = long.MinValue;
        directionCYOnlyGeneratedKnotCount = 0;
        directionCYOnlyLastBuiltCY = float.NaN;
    }

    private bool CreateFinalSplineContainer(Transform stairWayParent)
    {
        if (!stairWayParent || !IsStairWayTransform(stairWayParent))
            return false;

        GameObject obj = new GameObject("EqualizerFuture");
        obj.transform.SetParent(stairWayParent, false);
        obj.transform.localPosition = Vector3.zero;
        obj.transform.localRotation = Quaternion.identity;
        obj.transform.localScale = Vector3.one;

        splineContainer = obj.AddComponent<SplineContainer>();
        splineContainer.Spline = new Spline();

        generatedStairWayParentName = stairWayParent.name;
        RegisterFinalSpline(splineContainer);
        return true;
    }

    private void RegisterFinalSpline(SplineContainer created)
    {
        generatedSplineHistory.RemoveAll(item => !item);
        generatedSplineHistory.Add(created);

        // 1st: keep. 2nd: keep. 3rd: delete only the two-steps-ago spline.
        if (generatedSplineHistory.Count > MaxRetainedFutureSplines)
        {
            SplineContainer twoStepsAgo = generatedSplineHistory[0];
            generatedSplineHistory.RemoveAt(0);

            if (twoStepsAgo)
                Destroy(twoStepsAgo.gameObject);
        }

        retainedFutureSplineCount = generatedSplineHistory.Count;
    }

    private bool TryResolveTargetStairWayParent(
        NearestKnotDetector.GuideSample entrySample,
        out Transform stairWayParent)
    {
        stairWayParent = null;

        if (!entrySample.valid || !entrySample.isSlope)
            return false;

        int hitCount = Physics.OverlapSphereNonAlloc(
            entrySample.point,
            StairWayParentSearchRadius,
            stairWayParentSearchBuffer,
            ~0,
            QueryTriggerInteraction.Ignore);

        Vector3 expectedTangent = NormalizeSafe(entrySample.tangent, Vector3.forward);
        float bestScore = float.PositiveInfinity;

        for (int i = 0; i < hitCount; i++)
        {
            Collider candidateCollider = stairWayParentSearchBuffer[i];
            if (!candidateCollider)
                continue;

            Transform owner = FindStairWayOwner(candidateCollider.transform);
            if (!owner)
                continue;

            if (collisionStageRoot &&
                owner != collisionStageRoot &&
                !owner.IsChildOf(collisionStageRoot))
            {
                continue;
            }

            Vector3 ownerForward = NormalizeSafe(owner.forward, expectedTangent);
            float tangentDot = Vector3.Dot(ownerForward, expectedTangent);

            if (tangentDot < MinimumStairWayTangentAlignment)
                continue;

            Vector3 closest = GetSafeClosestPoint(candidateCollider, entrySample.point);
            float distance = Vector3.Distance(closest, entrySample.point);
            float score = distance + (1f - tangentDot) * 0.25f;

            if (score >= bestScore)
                continue;

            bestScore = score;
            stairWayParent = owner;
        }

        return stairWayParent;
    }

    private void EvaluateFinalTeacherAngleDiagnostics()
    {
        finalTeacherAngleSamples = 0;
        finalTeacherAverageAngleDegrees = 0f;
        finalTeacherMaximumAngleDegrees = 0f;
        finalTeacherMaximumSampleIndex = -1;
        finalTeacherMaximumSampleU = 0f;
        finalTeacherMaximumRawTeacherU = 0f;
        finalTeacherMaximumMappedTeacherU = 0f;
        finalTeacherMaximumTeacherDistanceMeters = 0f;
        finalTeacherMaximumMappedDistanceMeters = 0f;
        finalTeacherMaximumDeltaS = 0f;
        finalTeacherMaximumDeltaN = 0f;
        finalTeacherMaximumDeltaB = 0f;
        finalTeacherMaximumDirectionMetric = 0f;
        finalTeacherMaximumDistanceConfidence = 0f;
        finalTeacherMaximumForwardConfidence = 0f;
        finalTeacherMaximumGain = 0f;
        finalTeacherMaximumRawAngleDegrees = 0f;

        if (directionTeacher == null ||
            !builtWithReferenceTeacher ||
            !directionReady ||
            directionTangents == null ||
            teacherCandidateValid == null ||
            teacherMappedU == null)
        {
            return;
        }

        float angleSumDegrees = 0f;

        for (int i = 0; i < directionTangents.Length; i++)
        {
            if (!teacherCandidateValid[i] ||
                !directionTeacher.TryEvaluateDirectionAtNormalizedU(
                    generatedSplineIndex,
                    generatedSectionIndex,
                    teacherMappedU[i],
                    out Vector3 teacherPoint,
                    out Vector3 teacherTangent))
            {
                continue;
            }

            Vector3 finalTangent =
                NormalizeSafe(directionTangents[i], baseTangents[i]);

            teacherTangent =
                NormalizeSafe(teacherTangent, finalTangent);

            // theta_final = arccos(T_final dot T_teacher)
            // Both tangents are normalized, so the dot product is cos(theta).
            float tangentDot = Mathf.Clamp(
                Vector3.Dot(finalTangent, teacherTangent),
                -1f,
                1f);

            float thetaFinalDegrees =
                Mathf.Acos(tangentDot) * Mathf.Rad2Deg;

            if (!IsFinite(thetaFinalDegrees))
                continue;

            angleSumDegrees += thetaFinalDegrees;

            if (finalTeacherMaximumSampleIndex < 0 ||
                thetaFinalDegrees > finalTeacherMaximumAngleDegrees)
            {
                finalTeacherMaximumAngleDegrees = thetaFinalDegrees;
                finalTeacherMaximumSampleIndex = i;
                finalTeacherMaximumSampleU = sampleU[i];
                finalTeacherMaximumRawTeacherU = teacherRawU[i];
                finalTeacherMaximumMappedTeacherU = teacherMappedU[i];
                finalTeacherMaximumTeacherDistanceMeters = teacherDistance[i];
                finalTeacherMaximumMappedDistanceMeters = teacherMappedDistance[i];
                finalTeacherMaximumDeltaS = teacherDeltaS[i];
                finalTeacherMaximumDeltaN = teacherDeltaN[i];
                finalTeacherMaximumDeltaB = teacherDeltaB[i];
                finalTeacherMaximumDirectionMetric = teacherDirectionMetric[i];
                finalTeacherMaximumDistanceConfidence = teacherDistanceConfidence[i];
                finalTeacherMaximumForwardConfidence = teacherForwardConfidence[i];
                finalTeacherMaximumGain = teacherGain[i];
                finalTeacherMaximumRawAngleDegrees =
                    Vector3.Angle(
                        NormalizeSafe(rawTangents[i], baseTangents[i]),
                        teacherTangent);
            }

            finalTeacherAngleSamples++;
        }

        if (finalTeacherAngleSamples > 0)
        {
            finalTeacherAverageAngleDegrees =
                angleSumDegrees / finalTeacherAngleSamples;

            float pavaDeltaU =
                finalTeacherMaximumMappedTeacherU -
                finalTeacherMaximumRawTeacherU;

            float warpDeltaU =
                finalTeacherMaximumMappedTeacherU -
                finalTeacherMaximumSampleU;

            Debug.Log(
                $"[EQUALIZER FUTURE FINAL MAX DETAIL] " +
                $"version={directionVersion} " +
                $"spline={generatedSplineIndex} section={generatedSectionIndex} " +
                $"sampleIndex={finalTeacherMaximumSampleIndex} " +
                $"u={finalTeacherMaximumSampleU:F4} " +
                $"teacherRawU={finalTeacherMaximumRawTeacherU:F4} " +
                $"teacherMappedU={finalTeacherMaximumMappedTeacherU:F4} " +
                $"warpDeltaU={warpDeltaU:+0.0000;-0.0000;0.0000} " +
                $"pavaDeltaU={pavaDeltaU:+0.0000;-0.0000;0.0000} " +
                $"projectedDistance={finalTeacherMaximumTeacherDistanceMeters:F4}m " +
                $"mappedDistance={finalTeacherMaximumMappedDistanceMeters:F4}m " +
                $"deltaS={finalTeacherMaximumDeltaS:+0.0000;-0.0000;0.0000}m " +
                $"deltaN={finalTeacherMaximumDeltaN:+0.0000;-0.0000;0.0000}m " +
                $"deltaB={finalTeacherMaximumDeltaB:+0.0000;-0.0000;0.0000}m " +
                $"directionMetric={finalTeacherMaximumDirectionMetric:F4} " +
                $"distanceConfidence={finalTeacherMaximumDistanceConfidence:F3} " +
                $"forwardConfidence={finalTeacherMaximumForwardConfidence:F3} " +
                $"gain={finalTeacherMaximumGain:F3} " +
                $"rawAngleAtMax={finalTeacherMaximumRawAngleDegrees:F2}deg " +
                $"finalAngleMax={finalTeacherMaximumAngleDegrees:F2}deg",
                this);
        }
    }

    private void EvaluateReferenceDiagnosticsAndLimit()
    {
        referenceAverageAngleDegrees = 0f;
        referenceMaximumAngleDegrees = 0f;
        referenceMaximumDistanceMeters = 0f;
        limitCandidate = false;

        if (directionTeacher == null ||
            !builtWithReferenceTeacher ||
            !directionReady)
        {
            limitConsecutiveMatches = 0;
            limitWarningLatched = false;
            return;
        }

        int samples = 0;
        float angleSum = 0f;

        for (int i = 0; i < sampleU.Length; i += Mathf.Max(1, (sampleU.Length - 1) / 8))
        {
            if (!teacherCandidateValid[i] ||
                !directionTeacher.TryEvaluateDirectionAtNormalizedU(
                    generatedSplineIndex,
                    generatedSectionIndex,
                    teacherMappedU[i],
                    out Vector3 referencePoint,
                    out Vector3 referenceTangent))
            {
                continue;
            }

            referenceTangent = NormalizeSafe(referenceTangent, directionTangents[i]);
            float angle = Vector3.Angle(directionTangents[i], referenceTangent);
            float distance = Vector3.Distance(carrierPoints[i], referencePoint);

            angleSum += angle;
            referenceMaximumAngleDegrees = Mathf.Max(referenceMaximumAngleDegrees, angle);
            referenceMaximumDistanceMeters = Mathf.Max(referenceMaximumDistanceMeters, distance);
            samples++;
        }

        if (samples <= 0)
        {
            limitConsecutiveMatches = 0;
            limitWarningLatched = false;
            return;
        }

        referenceAverageAngleDegrees = angleSum / samples;

        bool withinLimit =
            referenceAverageAngleDegrees <= limitAverageAngleDegrees &&
            referenceMaximumAngleDegrees <= limitMaximumAngleDegrees &&
            referenceMaximumDistanceMeters <= limitMaximumDistanceMeters;

        if (!withinLimit)
        {
            limitConsecutiveMatches = 0;
            limitWarningLatched = false;
            return;
        }

        limitConsecutiveMatches++;

        if (!enableAnalyticLimitWarning ||
            limitConsecutiveMatches < Mathf.Max(1, limitRequiredConsecutiveMatches))
        {
            return;
        }

        limitCandidate = true;

        if (limitWarningLatched)
            return;

        limitWarningLatched = true;

        Debug.LogWarning(
            $"[SPLINE ANALYTIC LIMIT CANDIDATE] " +
            $"spline={generatedSplineIndex} section={generatedSectionIndex} " +
            $"consecutive={limitConsecutiveMatches}/{limitRequiredConsecutiveMatches} " +
            $"angleAvg={referenceAverageAngleDegrees:F3}deg " +
            $"angleMax={referenceMaximumAngleDegrees:F3}deg " +
            $"refDistanceMax={referenceMaximumDistanceMeters:F4}m " +
            $"criterion=direction-field-C1-like " +
            $"note=NumericalLimitCandidateNotProof",
            this);
    }

    /// <summary>
    /// Evaluates EqualizerFutureSpline's own RAW direction basis before:
    /// - external IDirectionFieldTeacher blending,
    /// - EqualizerFutureAsyncPos CY,
    /// - observed first-wave peak deformation.
    ///
    /// This is the stable boundary for a pure comparison spline containing only:
    ///   EqualizerFutureSpline raw slope/direction + its own Inspector CY.
    ///
    /// No BallVisualEqualizer-derived data is returned through this API.
    /// basePointPhysics is the SlopeStickCore/InSubject-derived base carrier point.
    /// tangentPhysics is rawTangents, generated only from the internal analytic
    /// wave slope, base T/N frame, and InSubject initial direction.
    /// </summary>
    public bool TryEvaluateRawDirectionBasis(
        float u,
        out float s,
        out Vector3 basePointPhysics,
        out Vector3 tangentPhysics)
    {
        s = 0f;
        basePointPhysics = Vector3.zero;
        tangentPhysics = Vector3.forward;

        if (!directionReady ||
            sampleU == null ||
            sampleU.Length < 2 ||
            sampleS == null ||
            basePoints == null ||
            rawTangents == null ||
            basePoints.Length != sampleU.Length ||
            rawTangents.Length != sampleU.Length)
        {
            return false;
        }

        u = Mathf.Clamp01(u);
        float scaled = u * (sampleU.Length - 1);
        int i0 = Mathf.Clamp(
            Mathf.FloorToInt(scaled),
            0,
            sampleU.Length - 1);
        int i1 = Mathf.Min(i0 + 1, sampleU.Length - 1);
        float t = scaled - i0;

        s = Mathf.Lerp(
            sampleS[i0],
            sampleS[i1],
            t);

        basePointPhysics = Vector3.Lerp(
            basePoints[i0],
            basePoints[i1],
            t);

        tangentPhysics = NormalizeSafe(
            Vector3.Slerp(
                rawTangents[i0],
                rawTangents[i1],
                t),
            baseTangents != null &&
            baseTangents.Length > i0
                ? baseTangents[i0]
                : Vector3.forward);

        return
            IsFinite(s) &&
            IsFinite(basePointPhysics) &&
            IsFinite(tangentPhysics);
    }

    public bool TryEvaluateDirection(float u, out DirectionSample sample)
    {
        sample = default;

        if (sampleU == null ||
            sampleU.Length < 2 ||
            carrierPoints == null)
        {
            return false;
        }

        u = Mathf.Clamp01(u);
        float scaled = u * (sampleU.Length - 1);
        int i0 = Mathf.Clamp(Mathf.FloorToInt(scaled), 0, sampleU.Length - 1);
        int i1 = Mathf.Min(i0 + 1, sampleU.Length - 1);
        float t = scaled - i0;

        Vector3 tangent = NormalizeSafe(
            Vector3.Slerp(directionTangents[i0], directionTangents[i1], t),
            directionTangents[i0]);

        Vector3 normal = NormalizeSafe(
            Vector3.Slerp(directionNormals[i0], directionNormals[i1], t),
            directionNormals[i0]);

        normal = BuildNormal(tangent, normal);
        Vector3 binormal = NormalizeSafe(Vector3.Cross(normal, tangent), Vector3.right);
        normal = NormalizeSafe(Vector3.Cross(tangent, binormal), normal);

        sample = new DirectionSample
        {
            u = u,
            s = Mathf.Lerp(sampleS[i0], sampleS[i1], t),
            time = Mathf.Lerp(sampleTime[i0], sampleTime[i1], t),
            speed = Mathf.Lerp(sampleSpeed[i0], sampleSpeed[i1], t),
            carrierPoint = Vector3.Lerp(carrierPoints[i0], carrierPoints[i1], t),
            tangent = tangent,
            normal = normal,
            binormal = binormal
        };

        return true;
    }

    public bool TryEvaluateDifferentialEnergy(
        float u,
        out DifferentialEnergySample sample)
    {
        sample = default;

        if (!directionReady ||
            sampleU == null ||
            sampleU.Length < 2 ||
            sampleSpecificMechanicalEnergy == null ||
            sampleSpecificMechanicalEnergy.Length != sampleU.Length)
        {
            return false;
        }

        u = Mathf.Clamp01(u);
        float scaled = u * (sampleU.Length - 1);
        int i0 = Mathf.Clamp(
            Mathf.FloorToInt(scaled),
            0,
            sampleU.Length - 1);
        int i1 = Mathf.Min(i0 + 1, sampleU.Length - 1);
        float t = scaled - i0;

        sample = new DifferentialEnergySample
        {
            valid = true,
            u = u,
            s = Mathf.Lerp(sampleS[i0], sampleS[i1], t),
            time = Mathf.Lerp(sampleTime[i0], sampleTime[i1], t),
            speed = Mathf.Lerp(sampleSpeed[i0], sampleSpeed[i1], t),
            lossFreeTime = Mathf.Lerp(
                sampleLossFreeTime[i0],
                sampleLossFreeTime[i1],
                t),
            lossFreeSpeed = Mathf.Lerp(
                sampleLossFreeSpeed[i0],
                sampleLossFreeSpeed[i1],
                t),
            cumulativeTimeLossSeconds = Mathf.Lerp(
                sampleCumulativeTimeLoss[i0],
                sampleCumulativeTimeLoss[i1],
                t),
            specificKineticEnergyJPerKg = Mathf.Lerp(
                sampleSpecificKineticEnergy[i0],
                sampleSpecificKineticEnergy[i1],
                t),
            specificPotentialEnergyRelativeToEntryJPerKg = Mathf.Lerp(
                sampleSpecificPotentialEnergy[i0],
                sampleSpecificPotentialEnergy[i1],
                t),
            specificMechanicalEnergyJPerKg = Mathf.Lerp(
                sampleSpecificMechanicalEnergy[i0],
                sampleSpecificMechanicalEnergy[i1],
                t),
            differentialMechanicalEnergyPerMeterJPerKgPerMeter = Mathf.Lerp(
                sampleDifferentialMechanicalEnergyPerMeter[i0],
                sampleDifferentialMechanicalEnergyPerMeter[i1],
                t),
            differentialMechanicalPowerWattsPerKg = Mathf.Lerp(
                sampleDifferentialMechanicalPower[i0],
                sampleDifferentialMechanicalPower[i1],
                t),
            cumulativeSpecificEnergyLossJPerKg = Mathf.Lerp(
                sampleCumulativeSpecificEnergyLoss[i0],
                sampleCumulativeSpecificEnergyLoss[i1],
                t),
            energyBalanceResidualJPerKg = Mathf.Lerp(
                sampleEnergyBalanceResidual[i0],
                sampleEnergyBalanceResidual[i1],
                t)
        };

        return
            IsFinite(sample.time) &&
            IsFinite(sample.speed) &&
            IsFinite(sample.specificMechanicalEnergyJPerKg) &&
            IsFinite(sample.cumulativeTimeLossSeconds);
    }

    public bool TryGetSuggestedKnotProgress(int index, out float u)
    {
        u = 0f;

        if (index < 0 || index >= knotParameters.Count)
            return false;

        u = knotParameters[index];
        return true;
    }

    private bool TryCaptureInSubjectClearance(
        NearestKnotDetector.GuideFrame guide,
        out float clearance)
    {
        Vector3 tangent = NormalizeSafe(guide.tangent, Vector3.forward);
        Vector3 normal = BuildNormal(tangent, guide.normal);

        clearance = Vector3.Dot(inSubjectBody.position - guide.point, normal);
        return IsFinite(clearance);
    }

    private void EnsureBuffers()
    {
        int count = Mathf.Clamp(analysisSampleCount, 25, 129);

        if (sampleU != null &&
            sampleU.Length == count &&
            sampleLossFreeTime != null &&
            sampleLossFreeTime.Length == count &&
            sampleLossFreeSpeed != null &&
            sampleLossFreeSpeed.Length == count &&
            sampleCumulativeTimeLoss != null &&
            sampleCumulativeTimeLoss.Length == count &&
            sampleSpecificKineticEnergy != null &&
            sampleSpecificKineticEnergy.Length == count &&
            sampleSpecificPotentialEnergy != null &&
            sampleSpecificPotentialEnergy.Length == count &&
            sampleSpecificMechanicalEnergy != null &&
            sampleSpecificMechanicalEnergy.Length == count &&
            sampleDifferentialMechanicalEnergyPerMeter != null &&
            sampleDifferentialMechanicalEnergyPerMeter.Length == count &&
            sampleDifferentialMechanicalPower != null &&
            sampleDifferentialMechanicalPower.Length == count &&
            sampleCumulativeSpecificEnergyLoss != null &&
            sampleCumulativeSpecificEnergyLoss.Length == count &&
            sampleEnergyBalanceResidual != null &&
            sampleEnergyBalanceResidual.Length == count &&
            teacherMappedDistance != null &&
            teacherMappedDistance.Length == count &&
            teacherDeltaS != null &&
            teacherDeltaS.Length == count &&
            teacherDeltaN != null &&
            teacherDeltaN.Length == count &&
            teacherDeltaB != null &&
            teacherDeltaB.Length == count &&
            teacherDirectionMetric != null &&
            teacherDirectionMetric.Length == count &&
            teacherDistanceConfidence != null &&
            teacherDistanceConfidence.Length == count &&
            teacherForwardConfidence != null &&
            teacherForwardConfidence.Length == count &&
            teacherGain != null &&
            teacherGain.Length == count &&
            teacherReferenceTangents != null &&
            teacherReferenceTangents.Length == count &&
            uncorrectedDirectionTangents != null &&
            uncorrectedDirectionTangents.Length == count)
        {
            return;
        }

        sampleU = new float[count];
        sampleS = new float[count];
        sampleTime = new float[count];
        sampleSpeed = new float[count];
        sampleLossFreeTime = new float[count];
        sampleLossFreeSpeed = new float[count];
        sampleCumulativeTimeLoss = new float[count];
        sampleSpecificKineticEnergy = new float[count];
        sampleSpecificPotentialEnergy = new float[count];
        sampleSpecificMechanicalEnergy = new float[count];
        sampleDifferentialMechanicalEnergyPerMeter = new float[count];
        sampleDifferentialMechanicalPower = new float[count];
        sampleCumulativeSpecificEnergyLoss = new float[count];
        sampleEnergyBalanceResidual = new float[count];

        basePoints = new Vector3[count];
        baseTangents = new Vector3[count];
        baseNormals = new Vector3[count];
        rawTangents = new Vector3[count];
        uncorrectedDirectionTangents = new Vector3[count];
        directionTangents = new Vector3[count];
        directionNormals = new Vector3[count];
        directionBinormals = new Vector3[count];
        carrierPoints = new Vector3[count];
        teacherReferenceTangents = new Vector3[count];

        teacherRawU = new float[count];
        teacherMappedU = new float[count];
        teacherDistance = new float[count];
        teacherMappedDistance = new float[count];
        teacherDeltaS = new float[count];
        teacherDeltaN = new float[count];
        teacherDeltaB = new float[count];
        teacherDirectionMetric = new float[count];
        teacherDistanceConfidence = new float[count];
        teacherForwardConfidence = new float[count];
        teacherGain = new float[count];
        teacherCandidateValid = new bool[count];

        pavaCandidateIndex = new int[count];
        pavaBlockStart = new int[count];
        pavaBlockEnd = new int[count];
        pavaBlockMean = new float[count];
        pavaBlockWeight = new float[count];
    }

    private void AddUniqueKnot(float u)
    {
        u = Mathf.Clamp01(u);

        for (int i = 0; i < knotParameters.Count; i++)
        {
            if (Mathf.Abs(knotParameters[i] - u) <= 0.0005f)
                return;
        }

        knotParameters.Add(u);
    }

    private static Vector3 GetSafeClosestPoint(Collider collider, Vector3 point)
    {
        if (!collider)
            return point;

        if (collider is BoxCollider ||
            collider is SphereCollider ||
            collider is CapsuleCollider)
        {
            return collider.ClosestPoint(point);
        }

        if (collider is MeshCollider mesh && mesh.convex)
            return collider.ClosestPoint(point);

        return collider.bounds.ClosestPoint(point);
    }

    private static Transform FindStairWayOwner(Transform source)
    {
        Transform current = source;

        while (current)
        {
            if (!string.IsNullOrEmpty(current.name) &&
                current.name.StartsWith(StairWayNamePrefix))
            {
                return current;
            }

            current = current.parent;
        }

        return null;
    }

    private static bool IsStairWayTransform(Transform value)
    {
        return value &&
               !string.IsNullOrEmpty(value.name) &&
               value.name.StartsWith(StairWayNamePrefix);
    }

    private static Vector3 BuildNormal(Vector3 tangent, Vector3 preferredNormal)
    {
        Vector3 normal = Vector3.ProjectOnPlane(preferredNormal, tangent);

        if (normal.sqrMagnitude <= Eps)
            normal = Vector3.ProjectOnPlane(Vector3.up, tangent);

        if (normal.sqrMagnitude <= Eps)
            normal = Vector3.up;

        return normal.normalized;
    }

    private static Vector3 NormalizeSafe(Vector3 value, Vector3 fallback)
    {
        if (value.sqrMagnitude > Eps && IsFinite(value))
            return value.normalized;

        if (fallback.sqrMagnitude > Eps && IsFinite(fallback))
            return fallback.normalized;

        return Vector3.forward;
    }

    private static long MakeKey(int splineIndex, int sectionIndex)
    {
        return ((long)splineIndex << 32) | (uint)sectionIndex;
    }

    private static float3 ToFloat3(Vector3 value)
    {
        return new float3(value.x, value.y, value.z);
    }

    private static bool IsFinite(float value)
    {
        return !float.IsNaN(value) && !float.IsInfinity(value);
    }

    private static bool IsFinite(Vector3 value)
    {
        return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
    }
}
