using System;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Splines;
using Sirenix.OdinInspector;

/// <summary>
/// BallVisualEqualizer renewal / FutureSpline + collider-less Virtual Upper.
///
/// Core policy:
/// - UpperCollider / Virtual Lower / EnvelopeSystem are NOT runtime authorities here.
/// - Four bodies are READ ONLY inputs for prediction/calibration.
/// - Default mode is AnalysisOnly: no Rigidbody state is modified.
/// - Optional ApplyImpulseToEqualizer mode applies only a unilateral normal impulse to
///   BallVisualEqualizer; position/velocity are never assigned directly.
/// - Valid completed trajectories may be flattened into a statistical spatial template.
///   Raw past trajectories are never replayed as motion authority.
/// - Plane-side CURRENT state is read once per rebuild and converted into a FUTURE analytic spline.
/// - One stair slope-section owns exactly one generated SplineContainer.
/// - Spline authority is BezierKnot[]; SplineExtrude is only an optional renderer.
///
/// Semantic anchors:
///   PF : Incident-derived future prediction start (not the current Ball position)
///   PT : SlopeStickCore targetSlopeProgressPercent intermediate anchor
///   PM : BallVisualSlopeDrive missile Future Shadow anchor
///   PL : BallVisualSlopeDrive terminal/rejoin predicted destination (internal prediction end)
///
/// Visible/generated Spline policy:
/// - The SplineContainer ends at the current slope exit.
/// - PM/PL remain internal prediction anchors for validation/probes.
/// - EqualizerFutureSpline therefore never learns from the long post-slope terminal tail.
///
/// Final analytic curve (internal):
///   E(u) = C(PF,PT,PM,PL;u)
///        + [R_n(u) + H_ref(s)] N_RMF(u)
///        + R_b(u) B_RMF(u)
///
/// R_n/R_b are local finite-distance boundary residuals. H_ref(s) is either the
/// accumulated valid-sample spatial template or an analytic absolute-stair-phase
/// fallback. Receding prediction limits how far model detail is extrapolated, while
/// event/roughness-protected knots preserve meaningful Equalizer irregularity.
/// maxGroundSpeed affects time parameterization, never spatial wave count.
/// </summary>
/// 
[Searchable]
[DefaultExecutionOrder(11500)]
[DisallowMultipleComponent]
public sealed class BallVisualEqualizerFutureSplineVisualizer : MonoBehaviour, IDirectionFieldTeacher
{
    const float Eps = 0.000001f;

    public const string RuntimeBuildId =
        "FutureSpline-ObservedEventLatch-v3";

    public enum MechanicalNormalPredictionMode
    {
        ReferenceOnly,
        ShadowOnly,
        BlendIntoUpperGap
    }

    public enum VirtualUpperRuntimeMode
    {
        AnalysisOnly,
        ApplyImpulseToEqualizer
    }

    public enum VirtualUpperContactModel
    {
        ImplicitSoftConstraint,
        HuntCrossley,
        Hybrid
    }

    [Serializable]
    public struct VirtualUpperDiagnostic
    {
        public bool valid;
        public float u;
        public Vector3 carrierCenterVisual;
        public Vector3 upperCenterVisual;
        public Vector3 tangentVisual;
        public Vector3 normalVisual;
        public Vector3 sideVisual;
        public float ceilingR;
        public float retention01;
        public float spanR;
        public float spanMeters;
        public float signedGapMeters;
        public float penetrationMeters;
        public float relativeNormalSpeed;
        public float impedance01;
        public float predictedImpulseNs;
    }

    [Serializable]
    public struct KnotDiagnostic
    {
        public float u;
        public Vector3 positionVisual;
        public Vector3 tangentVisual;
        public Vector3 normalVisual;
        public Vector3 sideVisual;
        public float bridgeNormal;
        public float bridgeSide;
        public float spatialWave;
        public bool futurePredictionAnchor;
        public bool targetProgressAnchor;
        public bool missileShadowAnchor;
        public bool terminalLandingAnchor;
    }

    // ================================================================
    // Integrated Future-Sample Collection
    // DATA ONLY: never moves Equalizer and never writes to physics authorities.
    // ================================================================

    public enum FutureSamplePhase
    {
        Idle,
        FirstStair,
        NextPlane
    }

    public enum FutureSampleCompletionKind
    {
        None,
        NextStairReached,
        TurnBoundaryReached
    }

    [Serializable]
    public struct FlattenedSamplePoint
    {
        public float time01;
        public float elapsedSeconds;
        public float s01;
        public float sMeters;

        public float equalizerN;
        public float equalizerB;
        public float equalizerVT;
        public float equalizerVN;
        public float equalizerVB;

        // First-stair-local normalized data used by the spatial template.
        // deltaN is measured from the first valid Stair entry of this run.
        public float equalizerDeltaNFromFirstStair;
        public float equalizerDnDs;

        public float ballN;
        public float ballB;
        public float ballVT;
        public float ballVN;
        public float ballVB;

        public float slopeProgress01;
        public bool onSlope;
        public int splineIndex;
        public int sectionIndex;
    }

    struct RawFutureSamplePoint
    {
        public float time;
        public Vector3 equalizerPosition;
        public Vector3 equalizerVelocity;
        public Vector3 ballPosition;
        public Vector3 ballVelocity;
        public Vector3 guideCenter;
        public Vector3 tangent;
        public Vector3 normal;
        public Vector3 side;
        public float slopeProgress01;
        public bool onSlope;
        public int splineIndex;
        public int sectionIndex;
    }

    [Serializable]
    struct SpatialTemplateBin
    {
        // One completed valid run contributes at most once to each bin.
        // meanRoughness is dimensionless and represents local change in dN/ds;
        // it is used to protect physically meaningful irregularity from knot thinning.
        public int runs;
        public float meanDeltaN;
        public float meanDnDs;
        public float meanRoughness;
        public float m2DeltaN;
    }

    struct PredictionArcLengthSample
    {
        public float u;
        public float distanceMeters;
    }

    [Header("Output / Ownership")]
    [Tooltip("VisualPlayerRoot/PhysicsSplineRoot. One child SplineContainer is generated per stair section.")]
    [SerializeField] Transform physicsSplineRoot;

    [Tooltip("Keep completed stair splines under PhysicsSplineRoot.")]
    [SerializeField] bool retainCompletedStairSplines = true;

    [Tooltip("Legacy freeze switch. Keep OFF during the realtime-emulator phase.")]
    [SerializeField] bool freezeOnSlopeEntry = false;

    [Header("Realtime Equalizer Future Emulation")]
    [Tooltip("Current development mode: rebuild the future spline continuously from the live Equalizer state.")]
    [SerializeField] bool continuousRealtimePrediction = true;

    [Tooltip("Rebuild the current stair prediction while already travelling on the slope.")]
    [SerializeField] bool rebuildWhileOnSlope = true;

    [Tooltip("Use BallVisualEqualizer position/velocity as the exact PF boundary while on the active stair.")]
    [SerializeField] bool useLiveEqualizerBoundaryOnSlope = true;

    [Tooltip("Minimum forward speed used when converting live Normal/Side velocity into spatial derivatives.")]
    [Min(0.05f)]
    [SerializeField] float minimumLiveTangentialSpeed = 0.50f;

    [Tooltip("Clamp only the acceleration model used for future distance integration. This never modifies Rigidbody motion.")]
    [Min(0f)]
    [SerializeField] float maximumPredictedTangentialAcceleration = 80f;

    [Range(0f, 1f)]
    [SerializeField] float tangentialAccelerationFilter01 = 0.25f;

    [Min(0.25f)]
    [SerializeField] float maximumPredictionTimeSeconds = 3.0f;

    [Header("Sparse Receding Prediction")]
    [Tooltip("Limits detailed model extrapolation to a short receding horizon. The Spline may retain semantic anchors beyond this range, but Equalizer-specific wave detail fades to the neutral backbone.")]
    [SerializeField] bool enableSparseRecedingPrediction = true;

    [Min(0.02f)]
    [SerializeField] float predictionLookaheadTimeSeconds = 0.18f;

    [Min(0.10f)]
    [SerializeField] float minimumPredictionLookaheadMeters = 1.20f;

    [Min(0.25f)]
    [SerializeField] float maximumPredictionLookaheadMeters = 4.00f;

    [Min(0.01f)]
    [SerializeField] float minimumPredictionRefreshSeconds = 0.06f;

    [Min(0.02f)]
    [SerializeField] float maximumPredictionRefreshSeconds = 0.15f;

    [Min(0f)]
    [SerializeField] float minimumTravelBeforePredictionRefreshMeters = 0.15f;

    [Min(0f)]
    [SerializeField] float minimumNormalChangeBeforePredictionRefreshMeters = 0.08f;

    [Min(0f)]
    [SerializeField] float minimumNormalVelocityChangeBeforePredictionRefresh = 0.75f;

    [Header("Future Prediction Error Measurement")]
    [Tooltip("Issue frozen diagnostic predictions at these horizons and compare them with the later real Equalizer position.")]
    [SerializeField] float[] predictionErrorHorizonsSeconds = { 0.05f, 0.10f, 0.20f, 0.40f };

    [Min(0.01f)]
    [SerializeField] float predictionProbeIssueIntervalSeconds = 0.05f;

    [Header("READ ONLY References")]
    [SerializeField] SlopeStickCore slopeCore;
    [SerializeField] NearestKnotDetector knotDetector;
    [SerializeField] CorrespondSubject correspondSubject;
    [SerializeField] BallVisualSlopeDrive ballVisualDrive;
    [SerializeField] BallVisualEqualizerSync equalizerSync;

    [Header("Integrated Future Sample Collection")]
    [Tooltip("Collect valid Plane -> Stair -> next Plane -> next Stair runs. A turn after Plane -> Stair is also a valid terminal boundary.")]
    [SerializeField] bool collectFutureSamples = true;

    [Tooltip("Plane history retained before the first Plane -> Stair boundary.")]
    [Min(0f)]
    [SerializeField] float samplePlanePreRollSeconds = 0.35f;

    [Tooltip("Minimum interval between stored sample points. 0 stores every FixedUpdate.")]
    [Min(0f)]
    [SerializeField] float minimumSampleIntervalSeconds = 0f;

    [Header("Integrated Sample Invalid Detection")]
    [Min(0.01f)]
    [SerializeField] float maximumSampleSingleStepPositionJumpMeters = 4.0f;

    [Min(0f)]
    [SerializeField] float sampleStuckSpeedThresholdMetersPerSecond = 0.05f;

    [Min(0f)]
    [SerializeField] float sampleStuckPositionDeltaThresholdMeters = 0.002f;

    [Min(0.05f)]
    [SerializeField] float sampleStuckDurationSeconds = 0.60f;

    [Header("Incident-derived Future Start PF")]
    [Tooltip("How far into the future the Plane-side measured state is projected before the stair is entered.")]
    [Min(0.01f)]
    [SerializeField] float incidentFutureLeadSeconds = 0.18f;

    [Tooltip("PF is never allowed to consume the entire PF->PT interval. Legacy safety while PT is still ahead.")]
    [Range(0.01f, 0.30f)]
    [SerializeField] float minimumPFToPTProgress01 = 0.08f;

    [Header("Incident Future Boundary")]
    [Tooltip("Use BallVisualSlopeDrive's READ ONLY Natural-Entry prediction for PF position/tangent.")]
    [SerializeField] bool useIncidentFutureBoundary = true;

    [Tooltip("Fallback neutral ball-center clearance in Equalizer radii. Used only until a stable Plane-side measurement window is accepted; transient flight height is never latched as the neutral baseline.")]
    [Min(0.25f)]
    [SerializeField] float baseCenterClearanceRadiusMultiplier = 4.0f;

    [Header("Neutral Center Clearance Estimator")]
    [Tooltip("Number of consecutive stable Plane measurements required before the neutral center clearance may be initialized or updated.")]
    [Min(2)]
    [SerializeField] int neutralClearanceRequiredStableFrames = 5;

    [Tooltip("Maximum absolute BallVisual velocity along the Plane normal while collecting a neutral-clearance window. Faster motion is treated as transient flight/roughness.")]
    [Min(0f)]
    [SerializeField] float neutralClearanceMaximumNormalSpeed = 0.50f;

    [Tooltip("Maximum max-min spread inside one stable Plane window, in Equalizer radii. A window that is still moving vertically is rejected before it can initialize or update the neutral baseline.")]
    [Min(0.01f)]
    [SerializeField] float neutralClearanceMaximumWindowSpreadR = 0.20f;

    [Tooltip("Accepted stable windows move the already-latched neutral baseline by only this fraction. The first trustworthy window initializes the latch directly.")]
    [Range(0.01f, 1f)]
    [SerializeField] float neutralClearanceLearningRate = 0.10f;

    [Tooltip("Maximum accepted difference, in Equalizer radii, between a stable Plane measurement window and the already-latched neutral baseline. This gate is intentionally not applied to the first trustworthy window.")]
    [Min(0.05f)]
    [SerializeField] float neutralClearanceMaximumInnovationR = 1.0f;

    [Tooltip("Current-state correction is allowed to influence only this finite route distance, then converges to zero with a quintic boundary residual.")]
    [Min(0.10f)]
    [SerializeField] float boundaryResidualLengthMeters = 1.50f;

    [Tooltip("Large live residuals are kept exact at PF but are localized more aggressively instead of being propagated deep into the FutureSpline.")]
    [Min(0.05f)]
    [SerializeField] float minimumBoundaryResidualLengthMeters = 0.35f;

    [Tooltip("Residual magnitude above which the residual influence length starts shrinking. This preserves the actual PF while preventing a large transient from inflating the whole Spline.")]
    [Min(0.05f)]
    [SerializeField] float largeResidualLocalizationThresholdMeters = 0.75f;

    [Header("Future Guide Transport for PM / PL")]
    [Tooltip("Use BallVisualDrive for timing/intent, but transport PM/PL along the future stair geometry instead of raw world-linear Y extrapolation.")]
    [SerializeField] bool useGuideTransportForPMPL = true;

    [Tooltip("0 = raw BallVisualDrive PM/PL, 1 = future Guide transported center trajectory.")]
    [Range(0f, 1f)]
    [SerializeField] float guideTransportBlend01 = 1f;

    [Tooltip("Preserve BallVisualDrive lateral intent while replacing only the guide-owned transport/height.")]
    [SerializeField] bool preserveDriveLateralIntent = true;

    [Header("4-ball analysis - READ ONLY")]
    [Tooltip("InSubject is the reference state; Subject is only checked as its mapped representation.")]
    [Range(0f, 1f)]
    [SerializeField] float inSubjectWeight = 0.50f;

    [Range(0f, 1f)]
    [SerializeField] float ballVisualWeight = 0.30f;

    [Range(0f, 1f)]
    [SerializeField] float equalizerWeight = 0.20f;

    [Header("Plane -> Stair Provisional Normal Bridge")]
    [Tooltip("Plane-side Incident state is still predictive, so only this fraction of its Normal position/slope residual is allowed into the next-Stair FutureSpline. Active-slope live Equalizer residuals remain full authority.")]
    [Range(0f, 1f)]
    [SerializeField] float planeNormalBridgeAuthority = 0.30f;

    [Header("Boundary Residual / Spatial Reference Model")]
    [Tooltip("Normal/side offset clamp for Incident and live-boundary diagnostics. Geometry only; never modifies physics.")]
    [Min(0.05f)]
    [SerializeField] float maximumInitialBridgeOffset = 2.0f;

    [Range(1, 8)]
    [SerializeField] int spatialWavesPerStair = 4;

    [Tooltip("Fallback absolute-stair-phase amplitude used only until the valid-sample template has enough coverage.")]
    [Min(0.01f)]
    [SerializeField] float fallbackAnalyticWaveAmplitudeMeters = 0.45f;

    [Tooltip("Spatial damping coefficient [1/m]. 0.22/m is approximately the old exp(-2.2*u) over a 10 m stair.")]
    [Min(0f)]
    [SerializeField] float spatialDecayPerMeter = 0.22f;

    [Header("Valid-Sample Spatial Template")]
    [Range(16, 128)]
    [SerializeField] int spatialTemplateBinCount = 64;

    [Min(1)]
    [SerializeField] int minimumTemplateRunsPerBin = 2;

    [Range(0f, 1f)]
    [SerializeField] float spatialTemplateBlend01 = 1.0f;

    [Header("Sparse Template Intake / Roughness Preservation")]
    [Tooltip("Each accepted run contributes only this many evenly distributed backbone samples from its first Stair.")]
    [Range(4, 16)]
    [SerializeField] int sparseTemplateBaseSamplesPerRun = 8;

    [Tooltip("Additional points may be retained when local dN/ds changes indicate meaningful Equalizer roughness.")]
    [Range(0, 12)]
    [SerializeField] int maximumRoughnessEventSamplesPerRun = 6;

    [Min(0f)]
    [SerializeField] float roughnessEventThreshold = 0.22f;

    [Range(0.005f, 0.25f)]
    [SerializeField] float minimumRoughnessEventProgressSeparation01 = 0.035f;

    [Tooltip("One run cannot pull a template bin by more than this Normal residual. This limits abnormal-run influence without flattening the whole reference wave.")]
    [Min(0.02f)]
    [SerializeField] float maximumTemplateNormalResidualPerRunMeters = 0.25f;

    [Tooltip("Maximum template adaptation fraction for one accepted run. The actual fraction is also limited by run count.")]
    [Range(0.01f, 0.5f)]
    [SerializeField] float maximumTemplateLearningRate = 0.08f;

    [Tooltip("Base limit applied only to the template-minus-analytic residual. Analytic backbone and exact PF are never globally clamped.")]
    [Min(0.05f)]
    [SerializeField] float templateResidualBaseSoftLimitMeters = 0.65f;

    [Tooltip("Bins with repeatable roughness are allowed additional template residual before soft limiting.")]
    [Min(0f)]
    [SerializeField] float templateRoughnessAllowanceMeters = 0.45f;

    [Header("Roughness-Protected Knot Layout")]
    [Range(0, 12)]
    [SerializeField] int maximumRoughnessProtectedKnots = 6;

    [Min(0f)]
    [SerializeField] float roughnessProtectedKnotThreshold = 0.20f;

    [Header("Upper Contact Protected Knot")]
    [Tooltip("Target spatial spacing of the internal Upper-gap search, in Equalizer radii. This refines candidate discovery only; it does not densify the final FutureSpline.")]
    [Min(0.05f)]
    [SerializeField] float upperContactSearchSpacingR = 0.25f;

    [Tooltip("Minimum internal samples used to search the receding Upper-contact horizon.")]
    [Range(8, 64)]
    [SerializeField] int upperContactMinimumSearchSamples = 16;

    [Tooltip("Maximum internal samples used to search the receding Upper-contact horizon.")]
    [Range(16, 96)]
    [SerializeField] int upperContactMaximumSearchSamples = 48;

    [Tooltip("Bisection iterations used after the first positive-to-nonpositive Upper-gap bracket is found.")]
    [Range(4, 20)]
    [SerializeField] int upperContactRootBisectionIterations = 12;

    [Tooltip("If no true gap root exists, a local minimum inside this distance from the Upper, in Equalizer radii, may be treated as a grazing-contact candidate.")]
    [Min(0f)]
    [SerializeField] float upperContactGrazingGapR = 0.15f;

    [Tooltip("A first-order TOI seed is only retained when its predicted gap is inside this validation distance, in Equalizer radii. This keeps TOI as a narrow-contact fallback rather than a broad proximity heuristic.")]
    [Min(0f)]
    [SerializeField] float upperContactToiValidationGapR = 0.25f;

    [Tooltip("World-distance half width around the accepted candidate. At most candidate-before / candidate / candidate-after knots are added.")]
    [Min(0.01f)]
    [SerializeField] float upperContactProtectedHalfWidthMeters = 0.12f;

    [Header("Upper Contact Event Quality Gate")]
    [Tooltip("A candidate at the very start of the FutureSpline is rejected only when the predicted Upper gap is already inside this neighborhood. This removes startup overlap from future-impact events without banning a genuine PF-immediate TOI.")]
    [Min(0f)]
    [SerializeField] float upperContactStartOverlapGapR = 0.05f;

    [Tooltip("Normalized Spline coordinate treated as the startup neighborhood for the overlap test.")]
    [Range(0f, 0.10f)]
    [SerializeField] float upperContactStartNeighborhoodU = 0.01f;

    [Tooltip("Minimum closing speed required for an Upper candidate to be treated as an impact event rather than resting / grazing contact.")]
    [Min(0f)]
    [SerializeField] float upperContactMinimumImpactSpeed = 0.10f;

    [Tooltip("Minimum Normal kinetic energy required for an Upper candidate to be treated as an impact event.")]
    [Min(0f)]
    [SerializeField] float upperContactMinimumImpactEnergyJ = 0.01f;

    [Tooltip("Immediately after the Virtual Upper is armed, a u-near-zero penetration is treated as startup overlap rather than a new impact event.")]
    [Min(0f)]
    [SerializeField] float upperContactRuntimeArmGraceSeconds = 0.06f;

    [Header("Mechanical Future-Time Integration")]
    [Tooltip("Measured non-gravity tangential acceleration is allowed to decay over this route distance. Gravity is recomputed from the future guide tangent every substep.")]
    [Min(0.10f)]
    [SerializeField] float residualTangentialAccelerationDecayMeters = 3.0f;

    [Range(4, 64)]
    [SerializeField] int mechanicalIntegrationSubsteps = 16;

    [Header("Mechanical Normal Prediction")]
    [Tooltip("ReferenceOnly keeps the existing Upper-gap model. ShadowOnly computes/grades the mechanical predictor without changing Upper candidates. BlendIntoUpperGap promotes the validated short-horizon blend into Upper-gap prediction.")]
    [SerializeField] MechanicalNormalPredictionMode mechanicalNormalPredictionMode =
        MechanicalNormalPredictionMode.ShadowOnly;

    [Tooltip("Stable FixedUpdates required after initialization/reset/emergency before live Normal state may seed the mechanical predictor.")]
    [Range(2, 12)]
    [SerializeField] int mechanicalNormalRequiredStableFrames = 4;

    [Tooltip("Additional quiet time after an invalid Normal state before the predictor may re-arm.")]
    [Min(0f)]
    [SerializeField] float mechanicalNormalRearmDelaySeconds = 0.08f;

    [Tooltip("Single-FixedUpdate Equalizer position jump above this distance invalidates the Normal state. This is deliberately well above ordinary maxGroundSpeed=24 travel.")]
    [Min(0.25f)]
    [SerializeField] float mechanicalNormalMaximumStepMeters = 4.0f;

    [Tooltip("Absolute Rigidbody speed above this value is treated as state contamination, not a trustworthy predictor seed.")]
    [Min(1f)]
    [SerializeField] float mechanicalNormalMaximumAbsoluteSpeed = 120f;

    [Tooltip("Absolute Stable-N speed above this value invalidates the mechanical seed. Kept high enough that maxGroundSpeed=24 high-energy tests remain measurable.")]
    [Min(1f)]
    [SerializeField] float mechanicalNormalMaximumAbsoluteNormalSpeed = 80f;

    [Tooltip("Clamp for measured Normal acceleration used only by active-slope shadow prediction.")]
    [Min(1f)]
    [SerializeField] float maximumPredictedNormalAcceleration = 250f;

    [Range(0.01f, 1f)]
    [SerializeField] float normalAccelerationFilter01 = 0.25f;

    [Tooltip("Active-slope measured non-gravity Normal acceleration decays over this short time. Plane->future-Stair prediction deliberately seeds zero residual acceleration.")]
    [Min(0.01f)]
    [SerializeField] float mechanicalNormalResidualAccelerationDecaySeconds = 0.12f;

    [Tooltip("Mechanical prediction has full authority inside this horizon when promoted. ShadowOnly still reports the same blend for comparison.")]
    [Min(0f)]
    [SerializeField] float mechanicalNormalFullAuthoritySeconds = 0.10f;

    [Tooltip("Mechanical authority reaches zero by this horizon and returns to the spatial Reference Wave.")]
    [Min(0.02f)]
    [SerializeField] float mechanicalNormalFadeEndSeconds = 0.28f;

    [Tooltip("Prediction is invalid rather than clamped when mechanical Normal displacement exceeds this many Equalizer radii.")]
    [Min(1f)]
    [SerializeField] float mechanicalNormalMaximumOffsetR = 8f;

    [Tooltip("Finite-difference carrier Normal acceleration is bounded to prevent a curved/degenerate baseline from becoming an artificial impulse.")]
    [Min(1f)]
    [SerializeField] float mechanicalNormalMaximumCarrierAcceleration = 200f;

    [Tooltip("Emit a throttled Shadow diagnostic so the next runtime log can compare Reference vs Mechanical without restoring per-FixedUpdate spam.")]
    [SerializeField] bool enableMechanicalNormalShadowLogs = true;

    [Tooltip("Only this horizon is written to the throttled Shadow log. All configured horizons still accumulate READ ONLY statistics.")]
    [Min(0.01f)]
    [SerializeField] float mechanicalNormalShadowLogHorizonSeconds = 0.10f;

    [Tooltip("Minimum interval between Mechanical Normal Shadow log lines.")]
    [Min(0.10f)]
    [SerializeField] float mechanicalNormalShadowLogIntervalSeconds = 0.50f;

    [Header("Mechanical Normal Oscillator Shadow")]
    [Tooltip("ON: upgrade the short-horizon Normal predictor from ballistic acceleration integration to a reduced-order spring/damper oscillator. ShadowOnly remains non-authoritative.")]
    [SerializeField] bool enableMechanicalNormalOscillator = true;

    [Tooltip("Reduced-order Stable-N spring strength [s^-2]. Default mirrors the current BallVisualEqualizerSync ride spring without reading its private runtime state.")]
    [Min(0f)]
    [SerializeField] float mechanicalNormalSpringStrength = 420f;

    [Tooltip("Reduced-order Stable-N damping gain [s^-1].")]
    [Min(0f)]
    [SerializeField] float mechanicalNormalDamper = 18f;

    [Tooltip("Fraction of Stable-N gravity cancelled by the reduced-order controller. 1 matches the current Equalizer default.")]
    [Range(0f, 1.5f)]
    [SerializeField] float mechanicalNormalGravityCompensation = 1.0f;

    [Tooltip("Feed-forward authority for the Reference Wave target Normal acceleration.")]
    [Range(0f, 1.5f)]
    [SerializeField] float mechanicalNormalReferenceAccelerationFeedForward = 1.0f;

    [Tooltip("Acceleration budget of the reduced-order Normal controller [m/s^2]. Kept independent of maxGroundSpeed.")]
    [Min(1f)]
    [SerializeField] float mechanicalNormalControllerAccelerationLimit = 450f;

    [Tooltip("Jerk budget used to move the reduced-order Normal controller toward its requested acceleration [m/s^3].")]
    [Min(1f)]
    [SerializeField] float mechanicalNormalControllerJerkLimit = 2500f;

    [Tooltip("Plane->future-Stair only: inherited Natural Connect observation interval before the reduced-order controller begins taking authority.")]
    [Min(0f)]
    [SerializeField] float mechanicalNormalFutureEntryDelaySeconds = 0.08f;

    [Tooltip("Plane->future-Stair only: C2 quintic authority blend after the delay. Active-Stair plans start at full authority.")]
    [Min(0.01f)]
    [SerializeField] float mechanicalNormalFutureEntryBlendSeconds = 0.12f;

    [Tooltip("Residual measured non-gravity Normal acceleration is already partly explained by spring/damper. Keep this share small to avoid double counting.")]
    [Range(0f, 1f)]
    [SerializeField] float mechanicalNormalResidualAccelerationAuthority01 = 0.10f;

    [Tooltip("ShadowOnly may simulate the first historical Upper reflection so 0.1-0.4s grading can include the return branch. BlendIntoUpperGap intentionally disables this so RootCrossing can see the pre-impact crossing.")]
    [SerializeField] bool mechanicalNormalEnableUpperReflectionInShadow = true;

    [Tooltip("Scale applied to the historical Upper restitution only inside the non-authoritative Shadow oscillator.")]
    [Range(0f, 1.25f)]
    [SerializeField] float mechanicalNormalShadowUpperRestitutionScale = 1.0f;

    [Tooltip("Positive->nonpositive Stable-N speed crossing threshold used to diagnose the predicted upper turning point.")]
    [Min(0f)]
    [SerializeField] float mechanicalNormalTurningSpeedEpsilon = 0.05f;

    [Header("Prediction Arc-Length Mapping")]
    [Range(24, 256)]
    [SerializeField] int predictionArcLengthSamples = 64;

    [Header("Prediction Layer Diagnostics")]
    [Tooltip("Diagnostic only: draw the center baseline without boundary bridge and without Equalizer wave.")]
    [SerializeField] bool baselineOnlyDiagnostic = false;

    [Tooltip("Allow the Equalizer spatial wave on top of the center baseline.")]
    [SerializeField] bool enableSpatialWave = true;

    [Header("Analytic Knot Layout")]
    [Tooltip("Quarter phase points: 4 => zero/rise/peak/fall/zero per wave. This is analytic phase placement, not frame sampling.")]
    [Range(2, 8)]
    [SerializeField] int phaseDivisionsPerWave = 4;

    [Tooltip("Bezier tangent strength relative to secant derivative. 1 means standard cubic derivative conversion.")]
    [Range(0.25f, 1.25f)]
    [SerializeField] float bezierTangentScale = 1.0f;

    [Header("Runtime Visualization (Spline remains authority)")]
    [SerializeField] bool createSplineExtrude = true;
    [SerializeField] Material splineMaterial;
    [Min(0.002f)] [SerializeField] float splineRadius = 0.025f;
    [Range(3, 12)] [SerializeField] int splineSides = 6;
    [Min(1f)] [SerializeField] float splineSegmentsPerUnit = 4f;

    [Header("Virtual Upper - Colliderless Authority")]
    [Tooltip("AnalysisOnly keeps all four bodies READ ONLY. ApplyImpulseToEqualizer is the collider-replacement test mode.")]
    [SerializeField] VirtualUpperRuntimeMode virtualUpperRuntimeMode = VirtualUpperRuntimeMode.AnalysisOnly;

    [SerializeField] bool virtualUpperEnabled = true;

    [Tooltip("Automatically arms the virtual Upper when the predicted stair spline is frozen at Plane -> Slope entry.")]
    [SerializeField] bool autoArmVirtualUpperOnSlopeEntry = true;

    [Tooltip("Optional safety reference. If assigned, this old physical Upper collider is forced OFF. The new solver does not require it.")]
    [SerializeField] Collider legacyUpperColliderSafety;

    [Header("Virtual Upper - Historical Upper-ON Teacher")]
    [Tooltip("Historical Upper impact restitution baseline. 0.92 -> energy retention 0.8464.")]
    [Range(0f, 1f)]
    [SerializeField] float historicalUpperRestitution01 = 0.92f;

    [Tooltip("Historical exp(-gamma*t) rate. Upper-ON logs around the mature 4R-Hn build used about 13.45/s.")]
    [Min(0f)]
    [SerializeField] float historicalDecayGammaPerSecond = 13.45f;

    [Tooltip("Old Envelope blend: q = lerp(1, epsilon*exp(-gamma*t), influence). 0.35 gives the late 0.65 ceiling ratio.")]
    [Range(0f, 1f)]
    [SerializeField] float historicalWaveTimeDecayInfluence = 0.35f;

    [Tooltip("Upper center travel ceiling in Equalizer radii by wave index. This is teacher data, not a physical Collider.")]
    [SerializeField] AnimationCurve historicalPresentationCeilingR =
        new AnimationCurve(
            new Keyframe(0f, 4.0f),
            new Keyframe(1f, 3.8f),
            new Keyframe(2f, 3.3f),
            new Keyframe(3f, 2.5f),
            new Keyframe(4f, 2.059f),
            new Keyframe(5f, 1.241f),
            new Keyframe(6f, 0.8f),
            new Keyframe(9f, 0.8f));

    [Tooltip("Historical envelope wave-index domain used by the old presentation ceiling curve. Independent from the new spatial wave count.")]
    [Range(1, 12)]
    [SerializeField] int historicalEnvelopeReferenceWaveCount = 8;

    [Tooltip("Fallback preferred Upper->Upper period used only for diagnostics until virtual impacts are observed.")]
    [Min(0.02f)]
    [SerializeField] float historicalPreferredUpperPeriodSeconds = 0.10f;

    [Header("Virtual Upper - Signed Gap / Contact")]
    [SerializeField] VirtualUpperContactModel virtualUpperContactModel = VirtualUpperContactModel.Hybrid;

    [Tooltip("Small numerical activation shell in Equalizer radii. Not rendered thickness.")]
    [Range(0f, 0.10f)]
    [SerializeField] float virtualUpperActivationShellR = 0.015f;

    [Tooltip("Approach-speed lead in FixedUpdate steps to reduce high-speed tunnelling.")]
    [Range(0f, 2f)]
    [SerializeField] float virtualUpperVelocityLeadSteps = 0.50f;

    [Tooltip("Gap beyond activation shell before a virtual impact is considered released.")]
    [Range(0f, 0.20f)]
    [SerializeField] float virtualUpperReleaseGapR = 0.04f;

    [Range(2f, 40f)]
    [SerializeField] float virtualUpperContactFrequencyHz = 14f;

    [SerializeField] bool deriveVirtualUpperDampingFromRestitution = true;

    [Range(0f, 2f)]
    [SerializeField] float explicitVirtualUpperDampingRatio = 0.08f;

    [Range(0f, 1f)]
    [SerializeField] float hybridImplicitShare = 0.70f;

    [Range(1f, 2.5f)]
    [SerializeField] float huntCrossleyExponent = 1.5f;

    [Range(0.01f, 0.50f)]
    [SerializeField] float huntReferenceCompressionR = 0.08f;

    [Header("Virtual Upper - Position-dependent Impedance")]
    [Range(0f, 0.10f)]
    [SerializeField] float impedanceStartR = 0.002f;

    [Range(0.005f, 0.50f)]
    [SerializeField] float impedanceFullR = 0.08f;

    [Header("Virtual Upper - Numerical Safety")]
    [Range(50f, 2500f)]
    [SerializeField] float maximumVirtualUpperAcceleration = 900f;

    [Range(100f, 100000f)]
    [SerializeField] float maximumVirtualUpperJerk = 30000f;

    [Range(0.03f, 1f)]
    [SerializeField] float adaptiveSubstepChiMax = 0.18f;

    [Range(1, 16)]
    [SerializeField] int maxInternalSubsteps = 8;

    [SerializeField] bool enforceRestitutionEnergyCap = true;

    [Header("Virtual Upper - Geometry Tracking")]
    [Range(1f, 200f)]
    [SerializeField] float maximumUpperGeometryTrackingSpeed = 80f;

    [Range(0f, 1f)]
    [SerializeField] float measuredUpperGeometryVelocityBlend01 = 0.70f;

    [Header("Virtual Upper - Debug")]
    [SerializeField] bool enableVirtualUpperImpactLogs = true;
    [SerializeField] bool drawVirtualUpperGizmos = true;
    [Min(0.01f)] [SerializeField] float virtualUpperGizmoRadius = 0.04f;

    [Header("Minimal Debug Logging")]
    [Tooltip("ON: 1階段につき、生成時とFreeze時だけ要点をログ出力します。毎Knot/毎FixedUpdateは出しません。")]
    [SerializeField] bool enableMinimalDebugLog = true;

    [Tooltip("ON: Build失敗理由が変化した時だけWarningを1回出します。通常はOFF推奨。")]
    [SerializeField] bool logBuildFailureReason = false;

    [Tooltip("PL終端一致誤差がこの値[m]を超えたら生成ログをWarningにします。")]
    [Min(0.0001f)]
    [SerializeField] float terminalValidationToleranceMeters = 0.02f;

    [Header("Diagnostics - READ ONLY")]
    [SerializeField] int activeSourceSplineIndex = -1;
    [SerializeField] int activeSourceSectionIndex = -1;
    [SerializeField] float pfSlopeProgress01;
    [SerializeField] float ptSlopeProgress01;
    [SerializeField] float predictedForwardSpeed;
    [SerializeField] float estimatedTimeToPT;
    [SerializeField] float estimatedTimeToSlopeExit;
    [SerializeField] float estimatedTimeToPL;
    [SerializeField] float initialNormalBridge;
    [SerializeField] float initialSideBridge;
    [SerializeField] float initialWaveAmplitude;
    [SerializeField] float subjectMappingResidualMeters;
    [SerializeField] float referenceCenterClearanceMeters;
    [SerializeField] float incidentBoundaryNormalSpeed;
    [SerializeField] float incidentBoundarySideSpeed;
    [SerializeField] float maxBuiltBridgeNormalMeters;
    [SerializeField] float maxBuiltWaveMeters;
    [SerializeField] float maxBuiltTotalNormalOffsetMeters;

    [Header("Integrated Sample Diagnostics - READ ONLY")]
    [SerializeField] FutureSamplePhase futureSamplePhase = FutureSamplePhase.Idle;
    [SerializeField] FutureSampleCompletionKind lastFutureSampleCompletionKind;
    [SerializeField] bool currentFutureSampleValid;
    [SerializeField] string lastFutureSampleInvalidReason;
    [SerializeField] int acceptedFutureSampleCount;
    [SerializeField] int rejectedFutureSampleCount;
    [SerializeField] int currentRawFutureSamplePointCount;
    [SerializeField] int latestFlattenedFutureSamplePointCount;
    [SerializeField] float currentFutureSampleStuckSeconds;
    [SerializeField] List<FlattenedSamplePoint> latestValidFlattenedSample =
        new List<FlattenedSamplePoint>(512);

    [Header("Spatial Template Diagnostics - READ ONLY")]
    [SerializeField] SpatialTemplateBin[] spatialTemplateBins = new SpatialTemplateBin[64];
    [SerializeField] int spatialTemplateAcceptedRuns;
    [SerializeField] int spatialTemplateFilledBins;
    [SerializeField] bool spatialTemplateReady;
    [SerializeField] float lastSpatialTemplateAuthority01;
    [SerializeField] float lastPredictionArcLengthMeters;
    [SerializeField] float currentPredictionLookaheadMeters;
    [SerializeField] float lastPredictionRefreshAgeSeconds;
    [SerializeField] int predictionRefreshCount;
    [SerializeField] int lastSparseTemplateContributionCount;
    [SerializeField] int lastRoughnessProtectedKnotCount;
    [SerializeField] int lastUpperContactProtectedKnotCount;
    [SerializeField] float lastUpperContactCandidateU = -1f;
    [SerializeField] float lastUpperContactCandidateGapMeters = float.PositiveInfinity;
    [SerializeField] float lastUpperContactCandidateClosingSpeed;
    [SerializeField] float lastUpperContactCandidateEnergyJ;
    [SerializeField] string lastUpperContactCandidateSource;
    [SerializeField] int lastUpperContactSearchSampleCount;
    [SerializeField] string lastUpperContactCandidateRejectReason;

    [Header("Virtual Upper Diagnostics - READ ONLY")]
    [SerializeField] bool virtualUpperArmed;
    [SerializeField] bool virtualUpperContactActive;
    [SerializeField] int virtualUpperImpactCount;
    [SerializeField] float currentVirtualUpperU;
    [SerializeField] float currentVirtualUpperGapMeters;
    [SerializeField] float currentVirtualUpperPenetrationMeters;
    [SerializeField] float currentVirtualUpperRelativeNormalSpeed;
    [SerializeField] float currentVirtualUpperForceNewton;
    [SerializeField] float currentVirtualUpperImpedance01;
    [SerializeField] float currentVirtualUpperSpanR;
    [SerializeField] float currentVirtualUpperSpanMeters;
    [SerializeField] float observedVirtualUpperPeriodSeconds;
    [SerializeField] float lastVirtualUpperIncomingSpeed;
    [SerializeField] float lastVirtualUpperOutgoingSpeed;
    [SerializeField] float lastVirtualUpperEnergyRetention;
    [SerializeField] int rejectedVirtualUpperImpactCount;
    [SerializeField] string lastVirtualUpperRejectedImpactReason;
    [SerializeField] VirtualUpperDiagnostic currentVirtualUpperDiagnostic;

    [SerializeField] List<KnotDiagnostic> lastBuiltKnots = new List<KnotDiagnostic>();

    [Serializable]
    public struct PredictionErrorDiagnostic
    {
        public float horizonSeconds;
        public int samples;
        public float latestPositionErrorMeters;
        public float latestTangentErrorMeters;
        public float latestNormalErrorMeters;
        public float latestSideErrorMeters;
        public float meanPositionErrorMeters;
        public float meanAbsTangentErrorMeters;
        public float meanAbsNormalErrorMeters;
        public float meanAbsSideErrorMeters;
    }

    [Serializable]
    public struct MechanicalNormalPredictionDiagnostic
    {
        public float maxGroundSpeedCondition;
        public float horizonSeconds;
        public int samples;
        public float latestActualNormalMeters;
        public float latestReferenceNormalMeters;
        public float latestMechanicalNormalMeters;
        public float latestBlendedNormalMeters;
        public float latestAbsReferenceErrorMeters;
        public float latestAbsMechanicalErrorMeters;
        public float latestAbsBlendedErrorMeters;
        public float meanAbsReferenceErrorMeters;
        public float meanAbsMechanicalErrorMeters;
        public float meanAbsBlendedErrorMeters;
        public float mechanicalWinRate01;

        public float latestActualNormalSpeed;
        public float latestMechanicalNormalSpeed;
        public float latestAbsMechanicalSpeedError;
        public float meanAbsMechanicalSpeedError;
        public bool latestPositionSignMatch;
        public bool latestVelocitySignMatch;
        public float positionSignMatchRate01;
        public float velocitySignMatchRate01;
        public bool latestPredictedUpperTurning;
        public bool latestObservedUpperTurningByHorizon;
        public float latestPredictedTurningTimeSeconds;
        public float latestObservedTurningTimeSeconds;
        public float latestTurningTimeErrorSeconds;
        public int turningTimeSamples;
        public float meanTurningTimeErrorSeconds;
        public float upperTurningAgreementRate01;
        public bool latestPredictedUpperImpact;
        public bool latestObservedUpperImpactByHorizon;
        public float latestPredictedUpperImpactTimeSeconds;
        public float latestObservedUpperImpactTimeSeconds;
        public float latestUpperImpactTimeErrorSeconds;
        public int upperImpactTimeSamples;
        public float meanUpperImpactTimeErrorSeconds;
        public float upperImpactAgreementRate01;
    }

    [Header("Realtime Boundary Diagnostics - READ ONLY")]
    [SerializeField] float liveEqualizerTangentialSpeed;
    [SerializeField] float liveEqualizerNormalSpeed;
    [SerializeField] float liveEqualizerSideSpeed;
    [SerializeField] float liveEqualizerTangentialAcceleration;
    [SerializeField] float liveEqualizerResidualTangentialAcceleration;
    [SerializeField] float liveEqualizerNormalAcceleration;
    [SerializeField] float liveEqualizerResidualNormalAcceleration;
    [SerializeField] float liveEqualizerNormalOffsetMeters;
    [SerializeField] float liveEqualizerSideOffsetMeters;
    [SerializeField] bool mechanicalNormalStateValid;
    [SerializeField] int mechanicalNormalStableFrames;
    [SerializeField] float mechanicalNormalLastStepMeters;
    [SerializeField] string mechanicalNormalInvalidReason;
    [SerializeField] bool lastMechanicalPlanValid;
    [SerializeField] float lastMechanicalInitialRelativeNormalSpeed;
    [SerializeField] float lastMechanicalResidualNormalAcceleration;
    [SerializeField] List<PredictionErrorDiagnostic> predictionErrors =
        new List<PredictionErrorDiagnostic>();
    [SerializeField] List<MechanicalNormalPredictionDiagnostic> mechanicalNormalPredictionErrors =
        new List<MechanicalNormalPredictionDiagnostic>();

    struct MechanicalNormalSolveResult
    {
        public bool valid;
        public float normalOffsetMeters;
        public float relativeNormalSpeed;
        public bool predictedUpperTurning;
        public float firstUpperTurningTimeSeconds;
        public bool predictedUpperImpact;
        public float firstUpperImpactTimeSeconds;
        public int upperTurningCount;
    }

    struct PredictionProbe
    {
        public float dueTime;
        public float horizonSeconds;
        public Vector3 predictedPosition;
        public Vector3 tangent;
        public Vector3 normal;
        public Vector3 side;

        public bool mechanicalNormalValid;
        public int sourceSplineIndex;
        public int sourceSectionIndex;
        public float maxGroundSpeedCondition;
        public int realUpperReflectionCountAtIssue;
        public float issueTime;
        public Vector3 observationNormalAtIssue;
        public float carrierBridgeNormalMeters;
        public float referenceNormalMeters;
        public float mechanicalNormalMeters;
        public float blendedNormalMeters;

        public float previousObservedRelativeNormalSpeed;
        public bool observedUpperTurning;
        public float observedUpperTurningTimeSeconds;
        public bool observedUpperImpact;
        public float observedUpperImpactTimeSeconds;
        public bool observationInvalidated;
        public string observationInvalidReason;

        public float predictedMechanicalNormalSpeed;
        public bool predictedUpperTurning;
        public float predictedUpperTurningTimeSeconds;
        public bool predictedUpperImpact;
        public float predictedUpperImpactTimeSeconds;
    }

    [Serializable]
    struct VirtualUpperKnotSample
    {
        public float u;
        public Vector3 carrierCenterLocal;
        public Vector3 tangentLocal;
        public Vector3 normalLocal;
        public Vector3 sideLocal;
        public float ceilingR;
        public float retention01;
        public float spanR;
        public float spanMeters;
        public Vector3 upperCenterLocal;
    }

    struct VirtualUpperFrame
    {
        public bool valid;
        public long key;
        public float u;
        public Vector3 carrierCenterVisual;
        public Vector3 upperCenterVisual;
        public Vector3 tangentVisual;
        public Vector3 normalVisual;
        public Vector3 sideVisual;
        public float ceilingR;
        public float retention01;
        public float spanR;
        public float spanMeters;
    }

    sealed class GeneratedStairSpline
    {
        public long key;
        public int sourceSplineIndex;
        public int sourceSectionIndex;
        public GameObject gameObject;
        public SplineContainer container;
        public SplineExtrude extrude;
        public bool frozen;

        // Minimal debug snapshot. Updated on each analytic rebuild, logged only
        // at first successful creation and at Plane -> Slope freeze.
        public bool creationLogged;

        // SplineContainer output stops at uSlopeExit, while the internal analytic
        // model may keep PM/PL through u=1 for validation and future probes.
        public int lastKnotCount;
        public int lastInternalKnotCount;
        public float lastRenderedEndU;
        public float lastTerminalErrorMeters;
        public float lastUT;
        public float lastUM;
        public Vector3 lastPF;
        public Vector3 lastPFGuideCenter;
        public Vector3 lastPT;
        public Vector3 lastPTSurface;
        public Vector3 lastPTCenterBase;
        public Vector3 lastPTFinal;
        public Vector3 lastPM;
        public Vector3 lastPL;
        public float lastReferenceCenterClearance;
        public float lastIncidentNormalSpeed;
        public float lastIncidentSideSpeed;
        public float lastMaxBridgeNormal;
        public float lastMaxWave;
        public float lastMaxTotalNormalOffset;
        public float lastPfProgress01;
        public float lastPtProgress01;
        public float lastPredictedSpeed;
        public float lastWaveAmplitude;
        public float lastTimeToPT;
        public float lastTimeToPL;

        // FutureSpline-derived Virtual Upper field. This is NOT another Spline authority.
        public readonly List<VirtualUpperKnotSample> virtualUpperSamples =
            new List<VirtualUpperKnotSample>(64);
    }

    struct FourBallSnapshot
    {
        public Vector3 inPosition;
        public Vector3 inVelocity;
        public Vector3 subjectPosition;
        public Vector3 subjectVelocity;
        public Vector3 ballPosition;
        public Vector3 ballVelocity;
        public Vector3 equalizerPosition;
        public Vector3 equalizerVelocity;
    }

    struct CurvePlan
    {
        // Baseline anchors are neutral ball-center trajectory positions. Current
        // Equalizer state is carried only by a finite boundary residual.
        public Vector3 pfBase;
        public Vector3 pfGuideCenter;
        public Vector3 pt;
        public Vector3 ptSurface;
        public Vector3 ptCenterBase;
        public Vector3 pm;
        public Vector3 pl;
        public Vector3 incidentVelocityPF;

        // Baseline Hermite tangents. tangentPF is route-only; exactFinalDerivativePF
        // restores the measured/predicted T/N/B first derivative at u=0.
        public Vector3 tangentPF;
        public Vector3 tangentPT;
        public Vector3 tangentPM;
        public Vector3 tangentPL;
        public Vector3 exactFinalDerivativePF;

        public float uT;
        public float uM;
        public float uSlopeExit;
        public float totalChordLength;
        public float totalDuration;
        public float timeToPT;
        public float timeToPM;

        public int sourceSplineIndex;
        public int sourceSectionIndex;

        public bool mechanicalNormalValid;
        public bool mechanicalStartsOnActiveSlope;
        public float mechanicalInitialRelativeNormalSpeed;
        public float mechanicalResidualNormalAcceleration0;

        public float startSlopeProgress01;
        public float sectionLength;
        public float modelLookaheadMeters;
        public float referenceTemplateAuthority01;
        public float referenceAmplitudeDiagnostic;

        public float residualNormal0;
        public float residualNormalSlopePerMeter0;
        public float residualSide0;
        public float residualSideSlopePerMeter0;

        public Vector3 initialNormal;
    }

    readonly Dictionary<long, GeneratedStairSpline> generated =
        new Dictionary<long, GeneratedStairSpline>();

    // Internal analytic parameters may continue to PL (u=1).
    readonly List<float> knotParameters = new List<float>(64);

    // Only these indices are emitted to the visible/generated SplineContainer.
    // They end at uSlopeExit.
    readonly List<int> renderedKnotIndices = new List<int>(64);

    readonly List<Vector3> baselinePoints = new List<Vector3>(64);
    readonly List<Vector3> baselineTangents = new List<Vector3>(64);
    readonly List<Vector3> normals = new List<Vector3>(64);
    readonly List<Vector3> sides = new List<Vector3>(64);
    readonly List<Vector3> finalPoints = new List<Vector3>(64);
    readonly List<Vector3> finalDerivatives = new List<Vector3>(64);
    readonly List<PredictionArcLengthSample> predictionArcLengthTable =
        new List<PredictionArcLengthSample>(128);

    Rigidbody ballVisualBody;
    Rigidbody equalizerBody;
    SphereCollider equalizerCollider;
    bool neutralCenterClearanceLatched;
    float latchedNeutralCenterClearanceMeters;
    int neutralClearanceStableFrames;
    float neutralClearanceStableSumMeters;
    float neutralClearanceStableMinimumMeters = float.PositiveInfinity;
    float neutralClearanceStableMaximumMeters = float.NegativeInfinity;

    bool previousUpperCenterValid;
    Vector3 previousUpperCenter;
    Vector3 currentUpperCenterVelocity;
    float virtualUpperNormalAccelerationState;
    float virtualUpperImpactIncomingSpeed;
    float virtualUpperAllowedOutgoingSpeed;
    float virtualUpperLastImpactTime = -1f;
    float virtualUpperArmedTime = -1f;
    bool virtualUpperRejectedContactLatched;
    long virtualUpperActiveKey = long.MinValue;

    readonly List<RawFutureSamplePoint> samplePlanePreRoll =
        new List<RawFutureSamplePoint>(64);
    readonly List<RawFutureSamplePoint> currentRawFutureSample =
        new List<RawFutureSamplePoint>(512);

    bool samplePreviousGuideValid;
    bool samplePreviousWasSlope;
    bool samplePreviousTurnActive;
    bool manualTurnBoundaryPending;
    bool previousSampleEqualizerPositionValid;
    Vector3 previousSampleEqualizerPosition;

    public IReadOnlyList<FlattenedSamplePoint> LatestValidFlattenedSample =>
        latestValidFlattenedSample;
    public FutureSamplePhase CurrentFutureSamplePhase => futureSamplePhase;
    public bool IsCollectingFutureSample => futureSamplePhase != FutureSamplePhase.Idle;
    public bool CurrentFutureSampleValid => currentFutureSampleValid;

    bool wasSlope;
    long lastPredictedKey = long.MinValue;
    string lastBuildFailureCode;

    bool liveKinematicsValid;
    Vector3 previousLiveVelocity;
    float previousLiveKinematicsTime;

    bool mechanicalNormalPreviousPositionValid;
    Vector3 previousMechanicalNormalPosition;
    float mechanicalNormalLastInvalidTime = -1000f;
    float lastMechanicalNormalShadowLogTime = -1000f;

    float lastPredictionProbeIssueTime = -1000f;
    readonly List<PredictionProbe> pendingPredictionProbes =
        new List<PredictionProbe>(96);

    bool predictionRefreshStateValid;
    float lastPredictionRefreshTime = -1000f;
    Vector3 lastPredictionRefreshPosition;
    float lastPredictionRefreshNormalOffset;
    float lastPredictionRefreshNormalVelocity;
    bool lastPredictionRefreshWasSlope;
    int lastPredictionRefreshSplineIndex = int.MinValue;
    int lastPredictionRefreshSectionIndex = int.MinValue;

    readonly List<int> sparseTemplateSelectedIndices = new List<int>(32);
    readonly List<int> sparseTemplateCandidateIndices = new List<int>(128);
    readonly List<float> sparseTemplateCandidateScores = new List<float>(128);
    readonly List<float> roughnessProtectedProgress = new List<float>(16);
    readonly List<float> roughnessProtectedScore = new List<float>(16);

    void Awake()
    {
        EnsureSpatialTemplateStorage();
        ResolveReferences();

        if (enableMinimalDebugLog)
        {
            Debug.Log(
                $"[FUTURE SPLINE BUILD] {RuntimeBuildId}",
                this);
        }
    }

    void OnEnable()
    {
        EnsureSpatialTemplateStorage();
        ResolveReferences();
    }

    void OnDisable()
    {
        DisarmVirtualUpper("ComponentDisabled");
        ResetFutureSampleCandidateState(clearLatestValidSample: false);
        ResetFutureSampleTrackingState();
        predictionRefreshStateValid = false;
        mechanicalNormalStateValid = false;
        mechanicalNormalStableFrames = 0;
        mechanicalNormalPreviousPositionValid = false;
        mechanicalNormalInvalidReason = "ComponentDisabled";
    }

    void FixedUpdate()
    {
        if (!ResolveReferences())
            return;

        NearestKnotDetector.GuideFrame guide = knotDetector.CurrentGuide;
        if (!guide.valid)
        {
            if (collectFutureSamples && IsCollectingFutureSample)
                RejectCurrentFutureSample("GuideInvalid");
            liveKinematicsValid = false;
            pendingPredictionProbes.Clear();
            InvalidateMechanicalNormalState("GuideInvalid", clearPreviousPosition: false);
            return;
        }

        UpdateLiveEqualizerKinematics(guide);
        UpdateMechanicalNormalStateValidity(guide);
        UpdatePredictionProbeObservations();
        EvaluateDuePredictionProbes();

        // Do not create a fake pre-play prediction while BallVisualDrive is still
        // Waiting/Synchronized at the initial scene pose.
        if (!slopeCore.BeginCommandOnTouch)
        {
            DisarmVirtualUpper("PrePlay");
            ResetFutureSampleCandidateState(clearLatestValidSample: false);
            ResetFutureSampleTrackingState();
            wasSlope = guide.isSlope;
            return;
        }

        if (collectFutureSamples)
            UpdateIntegratedFutureSampleCollection(guide);

        bool predictionRefreshDue = ShouldRefreshPrediction(guide);
        bool predictionRebuilt = false;

        if (predictionRefreshDue && !guide.isSlope && guide.nextIsSlope)
        {
            predictionRebuilt = TryBuildOrRefreshNextStairSpline(guide);
        }
        else if (predictionRefreshDue &&
                 continuousRealtimePrediction &&
                 rebuildWhileOnSlope &&
                 guide.isSlope)
        {
            predictionRebuilt = TryBuildOrRefreshCurrentStairSpline(guide);
        }

        if (predictionRebuilt)
            MarkPredictionRefresh(guide);

        // Freeze is a later/event-driven mode. During the current realtime-emulator
        // phase the same stair remains live and is rebuilt from the latest state.
        if (!continuousRealtimePrediction &&
            !wasSlope &&
            guide.isSlope &&
            freezeOnSlopeEntry)
        {
            FreezeEnteredStair(guide);
        }

        StepIntegratedVirtualUpper(guide);

        if (!retainCompletedStairSplines)
        {
            RemoveInactiveSplines(guide);
        }

        if (wasSlope && !guide.isSlope)
            DisarmVirtualUpper("SlopeExit");

        wasSlope = guide.isSlope;
    }

    bool ShouldRefreshPrediction(NearestKnotDetector.GuideFrame guide)
    {
        if (!enableSparseRecedingPrediction)
            return true;

        if (!predictionRefreshStateValid || !equalizerBody)
            return true;

        // Topology/phase transitions are prediction events and bypass the cadence gate.
        if (guide.isSlope != lastPredictionRefreshWasSlope ||
            guide.splineIndex != lastPredictionRefreshSplineIndex ||
            guide.sectionIndex != lastPredictionRefreshSectionIndex)
        {
            return true;
        }

        float now = Time.fixedTime;
        float age = Mathf.Max(0f, now - lastPredictionRefreshTime);
        lastPredictionRefreshAgeSeconds = age;

        if (age >= Mathf.Max(
                minimumPredictionRefreshSeconds,
                maximumPredictionRefreshSeconds))
        {
            return true;
        }

        if (age < Mathf.Max(0.01f, minimumPredictionRefreshSeconds))
            return false;

        float travel = Vector3.Distance(
            equalizerBody.position,
            lastPredictionRefreshPosition);
        if (travel >= Mathf.Max(0f, minimumTravelBeforePredictionRefreshMeters))
            return true;

        Vector3 tangent = NormalizeSafe(
            correspondSubject.MapDirection(guide.tangent),
            Vector3.forward);
        Vector3 normal = BuildInitialNormal(
            tangent,
            correspondSubject.MapDirection(guide.normal));
        Vector3 guidePoint = correspondSubject.MapPoint(guide.point);
        float normalOffset = Vector3.Dot(equalizerBody.position - guidePoint, normal);

        if (Mathf.Abs(normalOffset - lastPredictionRefreshNormalOffset) >=
            Mathf.Max(0f, minimumNormalChangeBeforePredictionRefreshMeters))
        {
            return true;
        }

        float normalVelocity = Vector3.Dot(equalizerBody.velocity, normal);
        return Mathf.Abs(normalVelocity - lastPredictionRefreshNormalVelocity) >=
            Mathf.Max(0f, minimumNormalVelocityChangeBeforePredictionRefresh);
    }

    void MarkPredictionRefresh(NearestKnotDetector.GuideFrame guide)
    {
        if (!equalizerBody)
            return;

        Vector3 tangent = NormalizeSafe(
            correspondSubject.MapDirection(guide.tangent),
            Vector3.forward);
        Vector3 normal = BuildInitialNormal(
            tangent,
            correspondSubject.MapDirection(guide.normal));
        Vector3 guidePoint = correspondSubject.MapPoint(guide.point);

        predictionRefreshStateValid = true;
        lastPredictionRefreshTime = Time.fixedTime;
        lastPredictionRefreshPosition = equalizerBody.position;
        lastPredictionRefreshNormalOffset =
            Vector3.Dot(equalizerBody.position - guidePoint, normal);
        lastPredictionRefreshNormalVelocity =
            Vector3.Dot(equalizerBody.velocity, normal);
        lastPredictionRefreshWasSlope = guide.isSlope;
        lastPredictionRefreshSplineIndex = guide.splineIndex;
        lastPredictionRefreshSectionIndex = guide.sectionIndex;
        lastPredictionRefreshAgeSeconds = 0f;
        predictionRefreshCount++;
    }

    float ResolvePredictionLookaheadMeters(
        float tangentialSpeed,
        float remainingSlopeDistance)
    {
        float remaining = Mathf.Max(0f, remainingSlopeDistance);
        if (!enableSparseRecedingPrediction)
            return remaining;

        float dynamicLookahead =
            Mathf.Abs(tangentialSpeed) *
            Mathf.Max(0.02f, predictionLookaheadTimeSeconds);

        float lower = Mathf.Max(0.10f, minimumPredictionLookaheadMeters);
        float upper = Mathf.Max(lower, maximumPredictionLookaheadMeters);
        float lookahead = Mathf.Clamp(dynamicLookahead, lower, upper);
        return remaining > Eps ? Mathf.Min(lookahead, remaining) : lookahead;
    }

    bool ResolveReferences()
    {
        if (!physicsSplineRoot)
        {
            GameObject root = GameObject.Find("VisualPlayerRoot/PhysicsSplineRoot");
            if (root)
                physicsSplineRoot = root.transform;
        }

        if (!slopeCore)
            slopeCore = FindObjectOfType<SlopeStickCore>();

        if (!knotDetector)
            knotDetector = FindObjectOfType<NearestKnotDetector>();

        if (!correspondSubject)
            correspondSubject = FindObjectOfType<CorrespondSubject>();

        if (!ballVisualDrive)
            ballVisualDrive = FindObjectOfType<BallVisualSlopeDrive>();

        if (!equalizerSync)
            equalizerSync = FindObjectOfType<BallVisualEqualizerSync>();

        if (!ballVisualBody && ballVisualDrive)
            ballVisualBody = ballVisualDrive.GetComponent<Rigidbody>();

        if (!equalizerBody && equalizerSync)
            equalizerBody = equalizerSync.Body;

        if (!equalizerCollider && equalizerBody)
            equalizerCollider = equalizerBody.GetComponent<SphereCollider>();

        // AnalysisOnly must be observational: never disable the real physical Upper.
        // The legacy collider is disabled only when the explicit virtual-contact
        // replacement experiment is selected.
        if (legacyUpperColliderSafety &&
            virtualUpperRuntimeMode == VirtualUpperRuntimeMode.ApplyImpulseToEqualizer &&
            legacyUpperColliderSafety.enabled)
        {
            legacyUpperColliderSafety.enabled = false;
        }

        return physicsSplineRoot &&
               slopeCore &&
               knotDetector &&
               correspondSubject &&
               correspondSubject.InSubjectBody &&
               correspondSubject.SubjectBody &&
               ballVisualDrive &&
               ballVisualBody &&
               equalizerSync &&
               equalizerBody;
    }

    bool TryBuildOrRefreshNextStairSpline(
        NearestKnotDetector.GuideFrame flatGuide)
    {
        if (!TryBuildCurvePlan(flatGuide, out CurvePlan plan,
                out int sourceSplineIndex,
                out int sourceSectionIndex))
        {
            LogBuildFailureOnce("CurvePlanUnavailable");
            return false;
        }

        long key = MakeKey(sourceSplineIndex, sourceSectionIndex);
        GeneratedStairSpline target = GetOrCreateGeneratedSpline(
            key,
            sourceSplineIndex,
            sourceSectionIndex);

        if (target.frozen)
            return true;

        BuildSplineFromPlan(target, plan);
        LogSplineCreatedOnce(target);
        lastBuildFailureCode = null;

        lastPredictedKey = key;
        activeSourceSplineIndex = sourceSplineIndex;
        activeSourceSectionIndex = sourceSectionIndex;
        return true;
    }

    bool TryBuildOrRefreshCurrentStairSpline(
        NearestKnotDetector.GuideFrame slopeGuide)
    {
        if (!TryBuildCurvePlanFromActiveSlope(
                slopeGuide,
                out CurvePlan plan,
                out int sourceSplineIndex,
                out int sourceSectionIndex))
        {
            LogBuildFailureOnce("ActiveSlopeCurvePlanUnavailable");
            return false;
        }

        long key = MakeKey(sourceSplineIndex, sourceSectionIndex);
        GeneratedStairSpline target = GetOrCreateGeneratedSpline(
            key,
            sourceSplineIndex,
            sourceSectionIndex);

        // Realtime-emulator authority: a formerly frozen prediction becomes live again.
        if (continuousRealtimePrediction)
            target.frozen = false;

        if (target.frozen)
            return true;

        BuildSplineFromPlan(target, plan);
        LogSplineCreatedOnce(target);
        lastBuildFailureCode = null;

        lastPredictedKey = key;
        activeSourceSplineIndex = sourceSplineIndex;
        activeSourceSectionIndex = sourceSectionIndex;
        return true;
    }

    bool TryBuildCurvePlan(
        NearestKnotDetector.GuideFrame flatGuide,
        out CurvePlan plan,
        out int sourceSplineIndex,
        out int sourceSectionIndex)
    {
        plan = default;
        sourceSplineIndex = -1;
        sourceSectionIndex = -1;

        if (!flatGuide.valid || flatGuide.isSlope || !flatGuide.nextIsSlope)
            return false;

        float targetProgress =
            Mathf.Clamp01(slopeCore.TargetSlopeProgress01ReadOnly);
        ptSlopeProgress01 = targetProgress;

        if (!knotDetector.TryEvaluateForwardSlopeSection(
                flatGuide,
                0f,
                out NearestKnotDetector.GuideSample entrySample) ||
            !knotDetector.TryEvaluateForwardSlopeSection(
                flatGuide,
                targetProgress,
                out NearestKnotDetector.GuideSample targetSample) ||
            !knotDetector.TryEvaluateForwardSlopeSection(
                flatGuide,
                1f,
                out NearestKnotDetector.GuideSample exitSample))
        {
            return false;
        }

        sourceSplineIndex = entrySample.splineIndex;
        sourceSectionIndex = entrySample.sectionIndex;

        float sectionLength = Mathf.Max(Eps, entrySample.sectionLength);
        FourBallSnapshot snapshot = ReadFourBallSnapshot();

        Vector3 entryTangentVisual = NormalizeSafe(
            correspondSubject.MapDirection(entrySample.tangent),
            Vector3.forward);

        float measuredForwardSpeed = ResolveMeasuredForwardSpeed(
            snapshot,
            entryTangentVisual);

        // maxGroundSpeed is a future speed cap, not an initial-speed estimator.
        // Preserve the measured state and let the mechanical integrator approach the cap.
        predictedForwardSpeed = Mathf.Max(0.10f, measuredForwardSpeed);

        float distanceToEntry = Mathf.Max(0f, flatGuide.distanceToNextSlope);
        float predictedDistanceAtPF = PredictTravelDistanceMechanical(
            flatGuide,
            Mathf.Max(0f, incidentFutureLeadSeconds),
            predictedForwardSpeed,
            liveEqualizerResidualTangentialAcceleration);

        float distanceInsideSlopeAtPF =
            Mathf.Max(0f, predictedDistanceAtPF - distanceToEntry);

        float startProgress = Mathf.Clamp01(
            distanceInsideSlopeAtPF / sectionLength);

        startProgress = Mathf.Min(
            startProgress,
            Mathf.Max(0f, targetProgress - minimumPFToPTProgress01));
        pfSlopeProgress01 = startProgress;

        if (!knotDetector.TryEvaluateForwardSlopeSection(
                flatGuide,
                startProgress,
                out NearestKnotDetector.GuideSample startSample))
        {
            return false;
        }

        Vector3 startSurfaceVisual = correspondSubject.MapPoint(startSample.point);
        Vector3 targetSurfaceVisual = correspondSubject.MapPoint(targetSample.point);

        Vector3 startTangentVisual = NormalizeSafe(
            correspondSubject.MapDirection(startSample.tangent),
            entryTangentVisual);
        Vector3 targetTangentVisual = NormalizeSafe(
            correspondSubject.MapDirection(targetSample.tangent),
            startTangentVisual);

        Vector3 startNormalVisual = BuildInitialNormal(
            startTangentVisual,
            correspondSubject.MapDirection(startSample.normal));
        Vector3 startSideVisual = NormalizeSafe(
            Vector3.Cross(startNormalVisual, startTangentVisual),
            Vector3.right);
        startNormalVisual = NormalizeSafe(
            Vector3.Cross(startTangentVisual, startSideVisual),
            startNormalVisual);

        Vector3 targetNormalVisual = BuildInitialNormal(
            targetTangentVisual,
            correspondSubject.MapDirection(targetSample.normal));

        referenceCenterClearanceMeters = ResolveNeutralCenterClearance(flatGuide, snapshot, true);

        Vector3 pfGuideCenter =
            startSurfaceVisual +
            startNormalVisual * referenceCenterClearanceMeters;
        Vector3 ptCenterBase =
            targetSurfaceVisual +
            targetNormalVisual * referenceCenterClearanceMeters;

        float distanceToPT = distanceToEntry + targetProgress * sectionLength;
        float distanceToExit = distanceToEntry + sectionLength;

        estimatedTimeToPT = PredictTimeForDistanceMechanical(
            flatGuide,
            distanceToPT,
            predictedForwardSpeed,
            liveEqualizerResidualTangentialAcceleration);
        estimatedTimeToSlopeExit = PredictTimeForDistanceMechanical(
            flatGuide,
            distanceToExit,
            predictedForwardSpeed,
            liveEqualizerResidualTangentialAcceleration);
        float estimatedTimeToEntry = PredictTimeForDistanceMechanical(
            flatGuide,
            distanceToEntry,
            predictedForwardSpeed,
            liveEqualizerResidualTangentialAcceleration);

        Vector3 incidentVelocityPF = startTangentVisual * predictedForwardSpeed;
        float residualN0 = 0f;
        float residualB0 = 0f;

        if (useIncidentFutureBoundary)
        {
            if (!ballVisualDrive.TryGetFutureSplineIncidentBoundary(
                    Mathf.Max(0f, incidentFutureLeadSeconds),
                    Mathf.Max(0f, estimatedTimeToEntry),
                    out BallVisualSlopeDrive.FutureSplineIntent incidentIntent))
            {
                return false;
            }

            Vector3 incidentDelta = incidentIntent.positionVisual - pfGuideCenter;
            float rawResidualN0 = Mathf.Clamp(
                Vector3.Dot(incidentDelta, startNormalVisual),
                -maximumInitialBridgeOffset,
                maximumInitialBridgeOffset);

            // Plane-side Incident is a forecast, not a measured active-slope state.
            // Reduce only Normal authority here. Side intent is left unchanged.
            residualN0 =
                rawResidualN0 *
                Mathf.Clamp01(planeNormalBridgeAuthority);

            residualB0 = Mathf.Clamp(
                Vector3.Dot(incidentDelta, startSideVisual),
                -maximumInitialBridgeOffset,
                maximumInitialBridgeOffset);
            incidentVelocityPF = incidentIntent.velocityVisual;
        }

        Vector3 pt = ptCenterBase;

        if (!ballVisualDrive.TryGetFutureSplineMissileShadow(
                estimatedTimeToPT,
                out BallVisualSlopeDrive.FutureSplineIntent missileIntent) ||
            !ballVisualDrive.TryGetFutureSplineTerminalLanding(
                estimatedTimeToSlopeExit,
                out BallVisualSlopeDrive.FutureSplineIntent terminalIntent))
        {
            return false;
        }

        Vector3 pm = missileIntent.positionVisual;
        Vector3 pl = terminalIntent.positionVisual;
        Vector3 missileTangent = NormalizeSafe(
            missileIntent.velocityVisual,
            pl - pt);
        Vector3 terminalTangent = NormalizeSafe(
            terminalIntent.velocityVisual,
            pl - pm);

        if (useGuideTransportForPMPL)
        {
            if (TryEvaluateFutureGuideTransport(
                    flatGuide,
                    exitSample,
                    sectionLength,
                    distanceToEntry,
                    referenceCenterClearanceMeters,
                    missileIntent.timeFromNowSeconds,
                    predictedForwardSpeed,
                    missileIntent.positionVisual,
                    out Vector3 guidePM,
                    out Vector3 guidePMTangent))
            {
                pm = Vector3.Lerp(
                    missileIntent.positionVisual,
                    guidePM,
                    Mathf.Clamp01(guideTransportBlend01));
                missileTangent = NormalizeSafe(
                    Vector3.Lerp(
                        missileIntent.velocityVisual.normalized,
                        guidePMTangent,
                        Mathf.Clamp01(guideTransportBlend01)),
                    guidePMTangent);
            }

            if (TryEvaluateFutureGuideTransport(
                    flatGuide,
                    exitSample,
                    sectionLength,
                    distanceToEntry,
                    referenceCenterClearanceMeters,
                    terminalIntent.timeFromNowSeconds,
                    predictedForwardSpeed,
                    terminalIntent.positionVisual,
                    out Vector3 guidePL,
                    out Vector3 guidePLTangent))
            {
                pl = Vector3.Lerp(
                    terminalIntent.positionVisual,
                    guidePL,
                    Mathf.Clamp01(guideTransportBlend01));
                terminalTangent = NormalizeSafe(
                    Vector3.Lerp(
                        terminalIntent.velocityVisual.normalized,
                        guidePLTangent,
                        Mathf.Clamp01(guideTransportBlend01)),
                    guidePLTangent);
            }
        }

        estimatedTimeToPL = terminalIntent.timeFromNowSeconds;

        if (!IsFinite(pfGuideCenter) || !IsFinite(pt) || !IsFinite(pm) || !IsFinite(pl))
            return false;

        BuildChordParameters(
            pfGuideCenter,
            pt,
            pm,
            pl,
            out float uT,
            out float uM,
            out float totalChordLength);

        float totalDuration = Mathf.Max(
            0.05f,
            terminalIntent.timeFromNowSeconds - Mathf.Max(0f, incidentFutureLeadSeconds));

        float incidentVT = Mathf.Max(
            minimumLiveTangentialSpeed,
            Vector3.Dot(incidentVelocityPF, startTangentVisual));
        float incidentVN = Vector3.Dot(incidentVelocityPF, startNormalVisual);
        float incidentVB = Vector3.Dot(incidentVelocityPF, startSideVisual);
        float dnDs = Mathf.Clamp(incidentVN / incidentVT, -4f, 4f);
        float dbDs = Mathf.Clamp(incidentVB / incidentVT, -4f, 4f);

        float predictedTravelToPL = Mathf.Max(
            (1f - startProgress) * sectionLength,
            PredictTravelDistanceMechanical(
                flatGuide,
                terminalIntent.timeFromNowSeconds,
                predictedForwardSpeed,
                liveEqualizerResidualTangentialAcceleration) - predictedDistanceAtPF);
        float distanceFromPFToSlopeExit = (1f - startProgress) * sectionLength;
        float uSlopeExit = Mathf.Clamp01(
            distanceFromPFToSlopeExit /
            Mathf.Max(Eps, predictedTravelToPL));
        uSlopeExit = Mathf.Max(0.05f, uSlopeExit);

        float templateAuthority = ResolveSpatialTemplateAuthority(startProgress);
        // The analytic fallback amplitude is never inflated by template extrema.
        // Learned shape enters only through the bounded template residual.
        float referenceAmplitude = fallbackAnalyticWaveAmplitudeMeters;
        float modelLookahead = ResolvePredictionLookaheadMeters(
            predictedForwardSpeed,
            (1f - startProgress) * sectionLength);
        currentPredictionLookaheadMeters = modelLookahead;

        CurvePlan provisional = new CurvePlan
        {
            startSlopeProgress01 = startProgress,
            sectionLength = sectionLength,
            modelLookaheadMeters = modelLookahead,
            uSlopeExit = uSlopeExit,
            referenceTemplateAuthority01 = templateAuthority,
            referenceAmplitudeDiagnostic = referenceAmplitude
        };

        float referenceSlopeAtStart = EvaluateReferenceNormalSlopeAtStart(provisional);

        // Do not let a Plane-side predicted Normal velocity re-inflate the Bridge
        // after residualN0 has already been weakened. Keep the reference-wave slope
        // as the neutral component and admit only the provisional residual fraction.
        float planeNormalAuthority = Mathf.Clamp01(planeNormalBridgeAuthority);
        float rawResidualDnDs = dnDs - referenceSlopeAtStart;
        float residualDnDs = rawResidualDnDs * planeNormalAuthority;
        float effectiveDnDs =
            referenceSlopeAtStart +
            residualDnDs;

        incidentBoundaryNormalSpeed = incidentVN;
        incidentBoundarySideSpeed = incidentVB;
        initialNormalBridge = residualN0;
        initialSideBridge = residualB0;
        initialWaveAmplitude = referenceAmplitude;

        float tangentMagnitude = Mathf.Max(0.25f, totalChordLength);
        Vector3 exactFinalDerivativePF =
            (startTangentVisual +
             startNormalVisual * effectiveDnDs +
             startSideVisual * dbDs) * tangentMagnitude;

        plan = new CurvePlan
        {
            pfBase = pfGuideCenter,
            pfGuideCenter = pfGuideCenter,
            pt = pt,
            ptSurface = targetSurfaceVisual,
            ptCenterBase = ptCenterBase,
            pm = pm,
            pl = pl,
            incidentVelocityPF = incidentVelocityPF,

            tangentPF = startTangentVisual * tangentMagnitude,
            tangentPT = targetTangentVisual * tangentMagnitude,
            tangentPM = missileTangent * tangentMagnitude,
            tangentPL = terminalTangent * tangentMagnitude,
            exactFinalDerivativePF = exactFinalDerivativePF,

            uT = uT,
            uM = uM,
            uSlopeExit = uSlopeExit,
            totalChordLength = totalChordLength,
            totalDuration = totalDuration,
            timeToPT = Mathf.Clamp(
                estimatedTimeToPT - Mathf.Max(0f, incidentFutureLeadSeconds),
                0f,
                totalDuration),
            timeToPM = Mathf.Clamp(
                missileIntent.timeFromNowSeconds - Mathf.Max(0f, incidentFutureLeadSeconds),
                0f,
                totalDuration),

            sourceSplineIndex = sourceSplineIndex,
            sourceSectionIndex = sourceSectionIndex,

            startSlopeProgress01 = startProgress,
            sectionLength = sectionLength,
            modelLookaheadMeters = modelLookahead,
            referenceTemplateAuthority01 = templateAuthority,
            referenceAmplitudeDiagnostic = referenceAmplitude,

            residualNormal0 = residualN0,
            residualNormalSlopePerMeter0 = residualDnDs,
            residualSide0 = residualB0,
            residualSideSlopePerMeter0 = dbDs,

            initialNormal = startNormalVisual
        };

        FinalizeMechanicalNormalPlan(
            ref plan,
            allowLiveResidualNormalAcceleration: false);

        return true;
    }

    bool TryBuildCurvePlanFromActiveSlope(
        NearestKnotDetector.GuideFrame slopeGuide,
        out CurvePlan plan,
        out int sourceSplineIndex,
        out int sourceSectionIndex)
    {
        plan = default;
        sourceSplineIndex = -1;
        sourceSectionIndex = -1;

        if (!slopeGuide.valid || !slopeGuide.isSlope || slopeGuide.sectionIndex < 0)
            return false;

        float currentProgress = Mathf.Clamp01(slopeGuide.sectionProgress01);
        float sectionLength = Mathf.Max(Eps, slopeGuide.sectionLength);

        if (!knotDetector.TryEvaluateSameSection(
                slopeGuide,
                currentProgress,
                out NearestKnotDetector.GuideSample currentSample) ||
            !knotDetector.TryEvaluateSameSection(
                slopeGuide,
                1f,
                out NearestKnotDetector.GuideSample exitSample))
        {
            return false;
        }

        sourceSplineIndex = currentSample.splineIndex;
        sourceSectionIndex = currentSample.sectionIndex;

        float configuredTarget = Mathf.Clamp01(slopeCore.TargetSlopeProgress01ReadOnly);
        float minimumLead = Mathf.Max(0.025f, minimumPFToPTProgress01);
        float targetProgress = configuredTarget;
        if (targetProgress <= currentProgress + minimumLead)
            targetProgress = Mathf.Lerp(currentProgress, 1f, 0.50f);
        targetProgress = Mathf.Clamp(
            targetProgress,
            Mathf.Min(1f, currentProgress + minimumLead),
            1f);

        if (!knotDetector.TryEvaluateSameSection(
                slopeGuide,
                targetProgress,
                out NearestKnotDetector.GuideSample targetSample))
        {
            return false;
        }

        FourBallSnapshot snapshot = ReadFourBallSnapshot();
        if (!IsFinite(snapshot.equalizerPosition) || !IsFinite(snapshot.equalizerVelocity))
            return false;

        Vector3 currentSurfaceVisual = correspondSubject.MapPoint(currentSample.point);
        Vector3 currentTangent = NormalizeSafe(
            correspondSubject.MapDirection(currentSample.tangent),
            Vector3.forward);
        Vector3 currentNormal = BuildInitialNormal(
            currentTangent,
            correspondSubject.MapDirection(currentSample.normal));
        Vector3 currentSide = NormalizeSafe(
            Vector3.Cross(currentNormal, currentTangent),
            Vector3.right);
        currentNormal = NormalizeSafe(
            Vector3.Cross(currentTangent, currentSide),
            currentNormal);

        Vector3 targetSurfaceVisual = correspondSubject.MapPoint(targetSample.point);
        Vector3 targetTangent = NormalizeSafe(
            correspondSubject.MapDirection(targetSample.tangent),
            currentTangent);
        Vector3 targetNormal = BuildInitialNormal(
            targetTangent,
            correspondSubject.MapDirection(targetSample.normal));

        referenceCenterClearanceMeters = ResolveNeutralCenterClearance(slopeGuide, snapshot, false);
        Vector3 pfGuideCenter =
            currentSurfaceVisual + currentNormal * referenceCenterClearanceMeters;
        Vector3 ptCenterBase =
            targetSurfaceVisual + targetNormal * referenceCenterClearanceMeters;

        Vector3 incidentVelocityPF = useLiveEqualizerBoundaryOnSlope
            ? snapshot.equalizerVelocity
            : currentTangent * Mathf.Max(
                minimumLiveTangentialSpeed,
                Vector3.Dot(snapshot.equalizerVelocity, currentTangent));

        float vT = Mathf.Max(
            minimumLiveTangentialSpeed,
            Vector3.Dot(incidentVelocityPF, currentTangent));
        float vN = Vector3.Dot(incidentVelocityPF, currentNormal);
        float vB = Vector3.Dot(incidentVelocityPF, currentSide);
        float dnDs = Mathf.Clamp(vN / vT, -4f, 4f);
        float dbDs = Mathf.Clamp(vB / vT, -4f, 4f);

        Vector3 equalizerFromGuide = snapshot.equalizerPosition - pfGuideCenter;
        float residualT0 = useLiveEqualizerBoundaryOnSlope
            ? Mathf.Clamp(
                Vector3.Dot(equalizerFromGuide, currentTangent),
                -maximumInitialBridgeOffset,
                maximumInitialBridgeOffset)
            : 0f;
        Vector3 pfBase = pfGuideCenter + currentTangent * residualT0;

        float residualN0 = useLiveEqualizerBoundaryOnSlope
            ? Mathf.Clamp(
                Vector3.Dot(equalizerFromGuide, currentNormal),
                -maximumInitialBridgeOffset,
                maximumInitialBridgeOffset)
            : 0f;
        float residualB0 = useLiveEqualizerBoundaryOnSlope
            ? Mathf.Clamp(
                Vector3.Dot(snapshot.equalizerPosition - pfGuideCenter, currentSide),
                -maximumInitialBridgeOffset,
                maximumInitialBridgeOffset)
            : 0f;

        liveEqualizerTangentialSpeed = vT;
        liveEqualizerNormalSpeed = vN;
        liveEqualizerSideSpeed = vB;
        liveEqualizerNormalOffsetMeters = residualN0;
        liveEqualizerSideOffsetMeters = residualB0;

        predictedForwardSpeed = Mathf.Max(0.10f, vT);

        float distanceToPT = Mathf.Max(
            0f,
            (targetProgress - currentProgress) * sectionLength);
        float distanceToExit = Mathf.Max(
            0f,
            (1f - currentProgress) * sectionLength);

        estimatedTimeToPT = PredictTimeForDistanceMechanical(
            slopeGuide,
            distanceToPT,
            vT,
            liveEqualizerResidualTangentialAcceleration);
        estimatedTimeToSlopeExit = PredictTimeForDistanceMechanical(
            slopeGuide,
            distanceToExit,
            vT,
            liveEqualizerResidualTangentialAcceleration);

        Vector3 pt = ptCenterBase;

        if (!ballVisualDrive.TryGetFutureSplineMissileShadow(
                estimatedTimeToPT,
                out BallVisualSlopeDrive.FutureSplineIntent missileIntent) ||
            !ballVisualDrive.TryGetFutureSplineTerminalLanding(
                estimatedTimeToSlopeExit,
                out BallVisualSlopeDrive.FutureSplineIntent terminalIntent))
        {
            return false;
        }

        Vector3 pm = missileIntent.positionVisual;
        Vector3 pl = terminalIntent.positionVisual;
        Vector3 missileTangent = NormalizeSafe(
            missileIntent.velocityVisual,
            targetTangent);
        Vector3 terminalTangent = NormalizeSafe(
            terminalIntent.velocityVisual,
            missileTangent);

        if (useGuideTransportForPMPL)
        {
            if (TryEvaluateFutureGuideTransportFromActiveSlope(
                    slopeGuide,
                    exitSample,
                    sectionLength,
                    currentProgress,
                    referenceCenterClearanceMeters,
                    missileIntent.timeFromNowSeconds,
                    vT,
                    missileIntent.positionVisual,
                    out Vector3 guidePM,
                    out Vector3 guidePMTangent))
            {
                pm = Vector3.Lerp(
                    missileIntent.positionVisual,
                    guidePM,
                    Mathf.Clamp01(guideTransportBlend01));
                missileTangent = NormalizeSafe(
                    Vector3.Lerp(
                        missileIntent.velocityVisual.normalized,
                        guidePMTangent,
                        Mathf.Clamp01(guideTransportBlend01)),
                    guidePMTangent);
            }

            if (TryEvaluateFutureGuideTransportFromActiveSlope(
                    slopeGuide,
                    exitSample,
                    sectionLength,
                    currentProgress,
                    referenceCenterClearanceMeters,
                    terminalIntent.timeFromNowSeconds,
                    vT,
                    terminalIntent.positionVisual,
                    out Vector3 guidePL,
                    out Vector3 guidePLTangent))
            {
                pl = Vector3.Lerp(
                    terminalIntent.positionVisual,
                    guidePL,
                    Mathf.Clamp01(guideTransportBlend01));
                terminalTangent = NormalizeSafe(
                    Vector3.Lerp(
                        terminalIntent.velocityVisual.normalized,
                        guidePLTangent,
                        Mathf.Clamp01(guideTransportBlend01)),
                    guidePLTangent);
            }
        }

        estimatedTimeToPL = terminalIntent.timeFromNowSeconds;
        if (!IsFinite(pfGuideCenter) || !IsFinite(pt) || !IsFinite(pm) || !IsFinite(pl))
            return false;

        BuildChordParameters(
            pfBase,
            pt,
            pm,
            pl,
            out float uT,
            out float uM,
            out float totalChordLength);

        float predictedTravelToPL = Mathf.Max(
            distanceToExit,
            PredictTravelDistanceMechanical(
                slopeGuide,
                terminalIntent.timeFromNowSeconds,
                vT,
                liveEqualizerResidualTangentialAcceleration));
        float uSlopeExit = Mathf.Clamp01(
            distanceToExit / Mathf.Max(Eps, predictedTravelToPL));
        uSlopeExit = Mathf.Max(0.05f, uSlopeExit);

        float templateAuthority = ResolveSpatialTemplateAuthority(currentProgress);
        // Keep the fallback backbone conservative; template detail is residual-only.
        float referenceAmplitude = fallbackAnalyticWaveAmplitudeMeters;
        float modelLookahead = ResolvePredictionLookaheadMeters(
            vT,
            (1f - currentProgress) * sectionLength);
        currentPredictionLookaheadMeters = modelLookahead;

        CurvePlan provisional = new CurvePlan
        {
            startSlopeProgress01 = currentProgress,
            sectionLength = sectionLength,
            modelLookaheadMeters = modelLookahead,
            uSlopeExit = uSlopeExit,
            referenceTemplateAuthority01 = templateAuthority,
            referenceAmplitudeDiagnostic = referenceAmplitude
        };

        float referenceSlopeAtStart = EvaluateReferenceNormalSlopeAtStart(provisional);
        float residualDnDs = dnDs - referenceSlopeAtStart;

        float totalDuration = Mathf.Max(0.05f, terminalIntent.timeFromNowSeconds);
        float derivativeScale = Mathf.Max(0.25f, totalChordLength);

        initialNormalBridge = residualN0;
        initialSideBridge = residualB0;
        incidentBoundaryNormalSpeed = vN;
        incidentBoundarySideSpeed = vB;
        initialWaveAmplitude = referenceAmplitude;
        pfSlopeProgress01 = currentProgress;
        ptSlopeProgress01 = targetProgress;

        Vector3 exactFinalDerivativePF =
            (currentTangent +
             currentNormal * dnDs +
             currentSide * dbDs) * derivativeScale;

        plan = new CurvePlan
        {
            pfBase = pfBase,
            pfGuideCenter = pfGuideCenter,
            pt = pt,
            ptSurface = targetSurfaceVisual,
            ptCenterBase = ptCenterBase,
            pm = pm,
            pl = pl,
            incidentVelocityPF = incidentVelocityPF,

            tangentPF = currentTangent * derivativeScale,
            tangentPT = targetTangent * derivativeScale,
            tangentPM = missileTangent * derivativeScale,
            tangentPL = terminalTangent * derivativeScale,
            exactFinalDerivativePF = exactFinalDerivativePF,

            uT = uT,
            uM = uM,
            uSlopeExit = uSlopeExit,
            totalChordLength = totalChordLength,
            totalDuration = totalDuration,
            timeToPT = Mathf.Clamp(
                estimatedTimeToPT,
                0f,
                totalDuration),
            timeToPM = Mathf.Clamp(
                missileIntent.timeFromNowSeconds,
                0f,
                totalDuration),

            sourceSplineIndex = sourceSplineIndex,
            sourceSectionIndex = sourceSectionIndex,

            startSlopeProgress01 = currentProgress,
            sectionLength = sectionLength,
            modelLookaheadMeters = modelLookahead,
            referenceTemplateAuthority01 = templateAuthority,
            referenceAmplitudeDiagnostic = referenceAmplitude,

            residualNormal0 = residualN0,
            residualNormalSlopePerMeter0 = residualDnDs,
            residualSide0 = residualB0,
            residualSideSlopePerMeter0 = dbDs,

            initialNormal = currentNormal
        };

        FinalizeMechanicalNormalPlan(
            ref plan,
            allowLiveResidualNormalAcceleration: true);

        return true;
    }

    float ResolveNeutralCenterClearance(
        NearestKnotDetector.GuideFrame guide,
        FourBallSnapshot snapshot,
        bool allowPlaneMeasurement)
    {
        float radius = ResolveEqualizerWorldRadius();
        float fallback = Mathf.Max(
            0.0001f,
            radius * Mathf.Max(0.25f, baseCenterClearanceRadiusMultiplier));

        float currentNeutral =
            neutralCenterClearanceLatched &&
            IsFiniteScalar(latchedNeutralCenterClearanceMeters) &&
            latchedNeutralCenterClearanceMeters > 0f
                ? latchedNeutralCenterClearanceMeters
                : fallback;

        // Stair flight and non-measurement calls may use the current neutral value,
        // but they must never contribute samples to the Plane-side estimator.
        if (!allowPlaneMeasurement || !guide.valid || guide.isSlope)
        {
            ResetNeutralCenterClearanceCandidate();
            return currentNeutral;
        }

        Vector3 guidePoint = correspondSubject.MapPoint(guide.point);
        Vector3 tangent = NormalizeSafe(
            correspondSubject.MapDirection(guide.tangent),
            Vector3.forward);
        Vector3 normal = BuildInitialNormal(
            tangent,
            correspondSubject.MapDirection(guide.normal));

        float measured = Vector3.Dot(
            snapshot.ballPosition - guidePoint,
            normal);
        float normalSpeed = Mathf.Abs(
            Vector3.Dot(snapshot.ballVelocity, normal));

        float minimum = Mathf.Max(0.25f * radius, 0.001f);
        bool emergencyRecoveryActive =
            equalizerSync && equalizerSync.IsEmergencyVisualRecoveryActive;

        bool stablePlaneMeasurement =
            IsFiniteScalar(measured) &&
            IsFiniteScalar(normalSpeed) &&
            measured >= minimum &&
            normalSpeed <= Mathf.Max(0f, neutralClearanceMaximumNormalSpeed) &&
            !emergencyRecoveryActive;

        if (!stablePlaneMeasurement)
        {
            ResetNeutralCenterClearanceCandidate();
            return currentNeutral;
        }

        neutralClearanceStableFrames++;
        neutralClearanceStableSumMeters += measured;
        neutralClearanceStableMinimumMeters = Mathf.Min(
            neutralClearanceStableMinimumMeters,
            measured);
        neutralClearanceStableMaximumMeters = Mathf.Max(
            neutralClearanceStableMaximumMeters,
            measured);

        int requiredFrames = Mathf.Max(2, neutralClearanceRequiredStableFrames);
        if (neutralClearanceStableFrames < requiredFrames)
            return currentNeutral;

        float stableMean =
            neutralClearanceStableSumMeters /
            Mathf.Max(1, neutralClearanceStableFrames);
        float stableSpread =
            neutralClearanceStableMaximumMeters -
            neutralClearanceStableMinimumMeters;

        // Use finite, non-overlapping windows so one long Plane interval does not
        // dominate merely by frame count. Capture mean/spread before resetting.
        ResetNeutralCenterClearanceCandidate();

        if (!IsFiniteScalar(stableMean) || !IsFiniteScalar(stableSpread))
            return currentNeutral;

        float maximumSpread = Mathf.Max(
            0.01f,
            radius * Mathf.Max(0.01f, neutralClearanceMaximumWindowSpreadR));
        if (stableSpread > maximumSpread)
            return currentNeutral;

        // First latch: the fallback is only a startup placeholder. Comparing the
        // first trustworthy Plane window against that arbitrary value prevented
        // the estimator from ever leaving 4R, so no innovation gate is used here.
        if (!neutralCenterClearanceLatched)
        {
            latchedNeutralCenterClearanceMeters = stableMean;
            neutralCenterClearanceLatched = true;
            return latchedNeutralCenterClearanceMeters;
        }

        // Subsequent windows must stay near the already-established neutral
        // baseline. Large offsets remain transient Incident/Bridge residuals.
        float maximumInnovation = Mathf.Max(
            0.05f,
            radius * Mathf.Max(0.05f, neutralClearanceMaximumInnovationR));
        float innovation =
            stableMean - latchedNeutralCenterClearanceMeters;
        if (Mathf.Abs(innovation) > maximumInnovation)
            return latchedNeutralCenterClearanceMeters;

        latchedNeutralCenterClearanceMeters = Mathf.Lerp(
            latchedNeutralCenterClearanceMeters,
            stableMean,
            Mathf.Clamp01(neutralClearanceLearningRate));

        return latchedNeutralCenterClearanceMeters;
    }

    void ResetNeutralCenterClearanceCandidate()
    {
        neutralClearanceStableFrames = 0;
        neutralClearanceStableSumMeters = 0f;
        neutralClearanceStableMinimumMeters = float.PositiveInfinity;
        neutralClearanceStableMaximumMeters = float.NegativeInfinity;
    }

    void EnsureSpatialTemplateStorage()
    {
        int count = Mathf.Clamp(spatialTemplateBinCount, 16, 128);
        if (spatialTemplateBins != null && spatialTemplateBins.Length == count)
            return;

        SpatialTemplateBin[] old = spatialTemplateBins;
        spatialTemplateBins = new SpatialTemplateBin[count];
        if (old == null || old.Length == 0)
            return;

        int copy = Mathf.Min(old.Length, spatialTemplateBins.Length);
        for (int i = 0; i < copy; i++)
            spatialTemplateBins[i] = old[i];
    }

    float ResolveSpatialTemplateAuthority(float startProgress01)
    {
        EnsureSpatialTemplateStorage();
        UpdateSpatialTemplateDiagnostics();

        if (!spatialTemplateReady || spatialTemplateBlend01 <= 0f)
            return 0f;

        // Require coverage across the remaining Stair domain. This avoids switching
        // template authority on/off from knot to knot and keeps the model analyzable.
        const int CoverageChecks = 9;
        for (int i = 0; i < CoverageChecks; i++)
        {
            float t = i / (float)(CoverageChecks - 1);
            float p = Mathf.Lerp(Mathf.Clamp01(startProgress01), 1f, t);
            if (!TryEvaluateSpatialTemplate(p, out _, out _, out _, out _))
                return 0f;
        }

        return Mathf.Clamp01(spatialTemplateBlend01);
    }

    bool TryEvaluateSpatialTemplate(
        float progress01,
        out float meanDeltaN,
        out float meanDnDs,
        out float meanRoughness,
        out float confidence01)
    {
        meanDeltaN = 0f;
        meanDnDs = 0f;
        meanRoughness = 0f;
        confidence01 = 0f;

        EnsureSpatialTemplateStorage();
        if (spatialTemplateBins == null || spatialTemplateBins.Length < 2)
            return false;

        float f = Mathf.Clamp01(progress01) * (spatialTemplateBins.Length - 1);
        int center = Mathf.Clamp(Mathf.RoundToInt(f), 0, spatialTemplateBins.Length - 1);
        int required = Mathf.Max(1, minimumTemplateRunsPerBin);

        int left = -1;
        int right = -1;
        for (int i = center; i >= 0; i--)
        {
            if (spatialTemplateBins[i].runs >= required)
            {
                left = i;
                break;
            }
        }
        for (int i = center; i < spatialTemplateBins.Length; i++)
        {
            if (spatialTemplateBins[i].runs >= required)
            {
                right = i;
                break;
            }
        }

        if (left < 0 && right < 0)
            return false;

        // Sparse intake intentionally leaves gaps. Permit interpolation across a
        // bounded neighborhood, but never extrapolate over a large unsupported span.
        int maxGapBins = Mathf.Max(
            4,
            Mathf.CeilToInt(
                spatialTemplateBins.Length /
                (float)Mathf.Max(4, sparseTemplateBaseSamplesPerRun)) + 1);

        if (left < 0)
        {
            if (right - center > maxGapBins)
                return false;
            SpatialTemplateBin b = spatialTemplateBins[right];
            meanDeltaN = b.meanDeltaN;
            meanDnDs = b.meanDnDs;
            meanRoughness = b.meanRoughness;
            confidence01 = Mathf.Clamp01(b.runs / (float)required);
            return true;
        }
        if (right < 0)
        {
            if (center - left > maxGapBins)
                return false;
            SpatialTemplateBin b = spatialTemplateBins[left];
            meanDeltaN = b.meanDeltaN;
            meanDnDs = b.meanDnDs;
            meanRoughness = b.meanRoughness;
            confidence01 = Mathf.Clamp01(b.runs / (float)required);
            return true;
        }

        if (right - left > maxGapBins * 2)
            return false;

        if (left == right)
        {
            SpatialTemplateBin b = spatialTemplateBins[left];
            meanDeltaN = b.meanDeltaN;
            meanDnDs = b.meanDnDs;
            meanRoughness = b.meanRoughness;
            confidence01 = Mathf.Clamp01(b.runs / (float)required);
            return true;
        }

        float x = Mathf.InverseLerp(left, right, f);
        SpatialTemplateBin aBin = spatialTemplateBins[left];
        SpatialTemplateBin bBin = spatialTemplateBins[right];
        meanDeltaN = Mathf.Lerp(aBin.meanDeltaN, bBin.meanDeltaN, x);
        meanDnDs = Mathf.Lerp(aBin.meanDnDs, bBin.meanDnDs, x);
        meanRoughness = Mathf.Lerp(aBin.meanRoughness, bBin.meanRoughness, x);
        confidence01 = Mathf.Min(
            1f,
            Mathf.Min(aBin.runs, bBin.runs) / (float)required);
        return true;
    }

    void UpdateSpatialTemplateDiagnostics()
    {
        EnsureSpatialTemplateStorage();
        int required = Mathf.Max(1, minimumTemplateRunsPerBin);
        int filled = 0;
        for (int i = 0; i < spatialTemplateBins.Length; i++)
        {
            if (spatialTemplateBins[i].runs >= required)
                filled++;
        }
        spatialTemplateFilledBins = filled;
        int sparseCoverageRequirement = Mathf.Clamp(
            Mathf.Max(4, sparseTemplateBaseSamplesPerRun - 2),
            4,
            spatialTemplateBins.Length);
        spatialTemplateReady =
            spatialTemplateAcceptedRuns >= required &&
            filled >= sparseCoverageRequirement;
    }

    void EvaluateAnalyticAbsoluteWave(
        CurvePlan plan,
        float progress01,
        out float normalOffset,
        out float dnDs)
    {
        float length = Mathf.Max(Eps, plan.sectionLength);
        float sMeters = Mathf.Clamp01(progress01) * length;
        float k = 2f * Mathf.PI * Mathf.Max(1, spatialWavesPerStair) / length;
        float phase = k * sMeters;
        float carrier = 0.5f * (1f - Mathf.Cos(phase));
        float decayRate = Mathf.Max(0f, spatialDecayPerMeter);
        float decay = Mathf.Exp(-decayRate * sMeters);
        float amplitude = Mathf.Max(0f, plan.referenceAmplitudeDiagnostic);

        normalOffset = amplitude * decay * carrier;
        dnDs = amplitude * decay *
            (0.5f * k * Mathf.Sin(phase) - decayRate * carrier);
    }

    void EvaluateReferenceRaw(
        CurvePlan plan,
        float progress01,
        out float normalOffset,
        out float dnDs)
    {
        EvaluateAnalyticAbsoluteWave(
            plan,
            progress01,
            out float analyticN,
            out float analyticDnDs);

        float authority = Mathf.Clamp01(plan.referenceTemplateAuthority01);
        if (authority > 0f &&
            TryEvaluateSpatialTemplate(
                progress01,
                out float templateN,
                out float templateDnDs,
                out float templateRoughness,
                out float confidence))
        {
            float blend = authority * confidence;

            // Limit only the learned residual relative to the analytic backbone.
            // Repeatable roughness earns additional allowance; the backbone itself
            // and the exact PF boundary are never globally flattened.
            float roughness01 = Mathf.Clamp01(
                templateRoughness /
                Mathf.Max(0.0001f, roughnessEventThreshold));
            float residualLimit = Mathf.Max(
                0.05f,
                templateResidualBaseSoftLimitMeters +
                templateRoughnessAllowanceMeters * roughness01);
            float rawResidual = templateN - analyticN;
            float limitedResidual = SoftLimitSigned(rawResidual, residualLimit);

            normalOffset = Mathf.Lerp(
                analyticN,
                analyticN + limitedResidual,
                blend);
            dnDs = Mathf.Lerp(analyticDnDs, templateDnDs, blend);
            return;
        }

        normalOffset = analyticN;
        dnDs = analyticDnDs;
    }

    float EvaluateReferenceNormalOffset(CurvePlan plan, float u)
    {
        if (!enableSpatialWave)
            return 0f;

        float exitU = Mathf.Clamp(plan.uSlopeExit, 0.05f, 1f);
        if (u >= exitU)
            return 0f;

        float x = Mathf.Clamp01(u / exitU);
        float progress = Mathf.Lerp(
            plan.startSlopeProgress01,
            1f,
            x);

        EvaluateReferenceRaw(plan, plan.startSlopeProgress01, out float startN, out _);
        EvaluateReferenceRaw(plan, progress, out float currentN, out _);

        float distanceFromPF =
            Mathf.Clamp01(u) * Mathf.Max(Eps, plan.totalChordLength);
        float modelLookahead = Mathf.Max(
            0.10f,
            Mathf.Min(
                Mathf.Max(0.10f, plan.modelLookaheadMeters),
                Mathf.Max(0.10f, plan.totalChordLength)));

        // Preserve full detail over most of the receding horizon, then return to the
        // neutral backbone smoothly. This limits extrapolation, not local roughness.
        float lookaheadFadeStart = modelLookahead * 0.78f;
        float lookaheadWindow = 1f;
        if (distanceFromPF > lookaheadFadeStart)
        {
            float fade01 = Mathf.InverseLerp(
                lookaheadFadeStart,
                modelLookahead,
                distanceFromPF);
            float smoothFade = fade01 * fade01 * (3f - 2f * fade01);
            lookaheadWindow = 1f - Mathf.Clamp01(smoothFade);
        }

        if (distanceFromPF >= modelLookahead)
            lookaheadWindow = 0f;

        float smoother = x * x * x * (x * (x * 6f - 15f) + 10f);
        float terminalWindow = 1f - smoother;
        return (currentN - startN) * terminalWindow * lookaheadWindow;
    }

    float EvaluateReferenceNormalSlopeAtStart(CurvePlan plan)
    {
        if (!enableSpatialWave)
            return 0f;

        EvaluateReferenceRaw(
            plan,
            plan.startSlopeProgress01,
            out _,
            out float dnDs);
        return dnDs;
    }

    float EvaluateBoundaryResidual(
        CurvePlan plan,
        float u,
        float value0,
        float slopePerMeter0)
    {
        float maximumLength = Mathf.Max(
            0.10f,
            Mathf.Min(boundaryResidualLengthMeters, Mathf.Max(0.10f, plan.totalChordLength)));
        float minimumLength = Mathf.Clamp(
            minimumBoundaryResidualLengthMeters,
            0.05f,
            maximumLength);

        float threshold = Mathf.Max(0.05f, largeResidualLocalizationThresholdMeters);
        float excess = Mathf.Max(0f, Mathf.Abs(value0) - threshold);
        float localization = 1f / (1f + excess / threshold);
        float length = Mathf.Lerp(minimumLength, maximumLength, localization);

        float distance = Mathf.Clamp01(u) * Mathf.Max(Eps, plan.totalChordLength);
        if (distance >= length)
            return 0f;

        float x = Mathf.Clamp01(distance / length);
        return EvaluateQuinticBridge(
            x,
            value0,
            slopePerMeter0 * length,
            0f);
    }

    static float SoftLimitSigned(float value, float limit)
    {
        float safeLimit = Mathf.Max(0.0001f, limit);
        return safeLimit * (float)Math.Tanh(value / safeLimit);
    }


    bool TryEvaluateFutureGuideTransportFromActiveSlope(
        NearestKnotDetector.GuideFrame slopeGuide,
        NearestKnotDetector.GuideSample exitSample,
        float sectionLength,
        float currentProgress,
        float centerClearance,
        float timeFromNowSeconds,
        float forwardSpeed,
        Vector3 rawDriveIntentPosition,
        out Vector3 centerPositionVisual,
        out Vector3 tangentVisual)
    {
        centerPositionVisual = Vector3.zero;
        tangentVisual = Vector3.forward;

        float travelDistance = PredictTravelDistanceMechanical(
            slopeGuide,
            timeFromNowSeconds,
            forwardSpeed,
            liveEqualizerResidualTangentialAcceleration);

        float absoluteSectionDistance =
            Mathf.Clamp01(currentProgress) * sectionLength + travelDistance;

        Vector3 normalVisual;
        Vector3 sideVisual;

        if (absoluteSectionDistance <= sectionLength + Eps)
        {
            float progress = Mathf.Clamp01(
                absoluteSectionDistance / Mathf.Max(Eps, sectionLength));

            if (!knotDetector.TryEvaluateSameSection(
                    slopeGuide,
                    progress,
                    out NearestKnotDetector.GuideSample sample))
            {
                return false;
            }

            tangentVisual = NormalizeSafe(
                correspondSubject.MapDirection(sample.tangent),
                Vector3.forward);
            normalVisual = BuildInitialNormal(
                tangentVisual,
                correspondSubject.MapDirection(sample.normal));
            sideVisual = NormalizeSafe(
                Vector3.Cross(normalVisual, tangentVisual),
                Vector3.right);

            centerPositionVisual =
                correspondSubject.MapPoint(sample.point) +
                normalVisual * centerClearance;
        }
        else
        {
            Vector3 exitSurface = correspondSubject.MapPoint(exitSample.point);
            Vector3 exitTangent = NormalizeSafe(
                correspondSubject.MapDirection(exitSample.tangent),
                Vector3.forward);
            Vector3 exitNormal = BuildInitialNormal(
                exitTangent,
                correspondSubject.MapDirection(exitSample.normal));

            Vector3 postSlopeForward = NormalizeSafe(
                Vector3.ProjectOnPlane(exitTangent, Vector3.up),
                exitTangent);

            tangentVisual = postSlopeForward;
            normalVisual = BuildInitialNormal(tangentVisual, Vector3.up);
            sideVisual = NormalizeSafe(
                Vector3.Cross(normalVisual, tangentVisual),
                Vector3.right);

            float extraDistance = Mathf.Max(0f, absoluteSectionDistance - sectionLength);
            Vector3 exitCenter = exitSurface + exitNormal * centerClearance;
            centerPositionVisual = exitCenter + postSlopeForward * extraDistance;
        }

        if (preserveDriveLateralIntent && IsFinite(rawDriveIntentPosition))
        {
            float lateral = Mathf.Clamp(
                Vector3.Dot(rawDriveIntentPosition - centerPositionVisual, sideVisual),
                -maximumInitialBridgeOffset,
                maximumInitialBridgeOffset);
            centerPositionVisual += sideVisual * lateral;
        }

        return IsFinite(centerPositionVisual) && IsFinite(tangentVisual);
    }

    bool TryEvaluateFutureGuideTransport(
        NearestKnotDetector.GuideFrame flatGuide,
        NearestKnotDetector.GuideSample exitSample,
        float sectionLength,
        float distanceToEntry,
        float centerClearance,
        float timeFromNowSeconds,
        float forwardSpeed,
        Vector3 rawDriveIntentPosition,
        out Vector3 centerPositionVisual,
        out Vector3 tangentVisual)
    {
        centerPositionVisual = Vector3.zero;
        tangentVisual = Vector3.forward;

        float travelDistance = PredictTravelDistanceMechanical(
            flatGuide,
            timeFromNowSeconds,
            Mathf.Max(0.01f, forwardSpeed),
            liveEqualizerResidualTangentialAcceleration);

        float distanceInsideOrBeyond =
            Mathf.Max(0f, travelDistance - Mathf.Max(0f, distanceToEntry));

        Vector3 normalVisual;
        Vector3 sideVisual;

        if (distanceInsideOrBeyond <= sectionLength + Eps)
        {
            float progress = Mathf.Clamp01(
                distanceInsideOrBeyond / Mathf.Max(Eps, sectionLength));

            if (!knotDetector.TryEvaluateForwardSlopeSection(
                    flatGuide,
                    progress,
                    out NearestKnotDetector.GuideSample sample))
            {
                return false;
            }

            tangentVisual = NormalizeSafe(
                correspondSubject.MapDirection(sample.tangent),
                Vector3.forward);

            normalVisual = BuildInitialNormal(
                tangentVisual,
                correspondSubject.MapDirection(sample.normal));

            sideVisual = NormalizeSafe(
                Vector3.Cross(normalVisual, tangentVisual),
                Vector3.right);

            centerPositionVisual =
                correspondSubject.MapPoint(sample.point) +
                normalVisual * centerClearance;
        }
        else
        {
            Vector3 exitSurface =
                correspondSubject.MapPoint(exitSample.point);

            Vector3 exitTangent = NormalizeSafe(
                correspondSubject.MapDirection(exitSample.tangent),
                Vector3.forward);

            Vector3 exitNormal = BuildInitialNormal(
                exitTangent,
                correspondSubject.MapDirection(exitSample.normal));

            // After the stair, use its XZ heading as the next-flat transport direction.
            // This deliberately removes the stale downward Y component that caused PM/PL
            // to remain above or below the actual post-stair trajectory.
            Vector3 postSlopeForward = Vector3.ProjectOnPlane(
                exitTangent,
                Vector3.up);

            postSlopeForward = NormalizeSafe(
                postSlopeForward,
                exitTangent);

            tangentVisual = postSlopeForward;
            normalVisual = BuildInitialNormal(
                tangentVisual,
                Vector3.up);

            sideVisual = NormalizeSafe(
                Vector3.Cross(normalVisual, tangentVisual),
                Vector3.right);

            float extraDistance =
                Mathf.Max(0f, distanceInsideOrBeyond - sectionLength);

            Vector3 exitCenter =
                exitSurface + exitNormal * centerClearance;

            centerPositionVisual =
                exitCenter + postSlopeForward * extraDistance;
        }

        if (preserveDriveLateralIntent && IsFinite(rawDriveIntentPosition))
        {
            float lateral = Mathf.Clamp(
                Vector3.Dot(
                    rawDriveIntentPosition - centerPositionVisual,
                    sideVisual),
                -maximumInitialBridgeOffset,
                maximumInitialBridgeOffset);

            centerPositionVisual += sideVisual * lateral;
        }

        return IsFinite(centerPositionVisual) &&
               IsFinite(tangentVisual);
    }

    FourBallSnapshot ReadFourBallSnapshot()
    {
        Rigidbody inBody = correspondSubject.InSubjectBody;
        Rigidbody subjectBody = correspondSubject.SubjectBody;

        FourBallSnapshot snapshot = new FourBallSnapshot
        {
            inPosition = correspondSubject.MapPoint(inBody.position),
            inVelocity = correspondSubject.MapDirection(inBody.velocity),
            subjectPosition = subjectBody.position,
            subjectVelocity = correspondSubject.MappedVelocity,
            ballPosition = ballVisualBody.position,
            ballVelocity = ballVisualBody.velocity,
            equalizerPosition = equalizerBody.position,
            equalizerVelocity = equalizerBody.velocity
        };

        subjectMappingResidualMeters =
            Vector3.Distance(
                snapshot.inPosition,
                snapshot.subjectPosition);

        return snapshot;
    }



    float ResolveMeasuredForwardSpeed(
        FourBallSnapshot snapshot,
        Vector3 tangent)
    {
        float totalWeight = Mathf.Max(
            Eps,
            inSubjectWeight +
            ballVisualWeight +
            equalizerWeight);

        float inSpeed = Mathf.Max(
            0f,
            Vector3.Dot(snapshot.inVelocity, tangent));

        float ballSpeed = Mathf.Max(
            0f,
            Vector3.Dot(snapshot.ballVelocity, tangent));

        float equalizerSpeed = Mathf.Max(
            0f,
            Vector3.Dot(snapshot.equalizerVelocity, tangent));

        return Mathf.Max(
            0.10f,
            (inSpeed * inSubjectWeight +
             ballSpeed * ballVisualWeight +
             equalizerSpeed * equalizerWeight) /
            totalWeight);
    }



    void BuildSplineFromPlan(
        GeneratedStairSpline target,
        CurvePlan plan)
    {
        BuildAnalyticKnotParameters(plan);
        BuildBaselineAndRotationMinimizingFrame(plan);
        BuildFinalAnalyticPoints(plan);
        BuildBezierDerivatives(plan);

        Spline spline = target.container.Spline;
        if (spline == null)
        {
            spline = new Spline();
            target.container.Spline = spline;
        }

        spline.Clear();
        spline.Closed = false;
        lastBuiltKnots.Clear();
        renderedKnotIndices.Clear();

        // Keep PL and other post-slope anchors in the INTERNAL analytic plan, but
        // emit only the current slope portion to the SplineContainer. This removes
        // the long terminal tail without deleting terminal prediction data.
        float renderedEndU = Mathf.Clamp(plan.uSlopeExit, 0.05f, 1f);

        for (int i = 0; i < knotParameters.Count; i++)
        {
            if (knotParameters[i] <= renderedEndU + 0.0001f)
                renderedKnotIndices.Add(i);
        }

        for (int visibleIndex = 0;
             visibleIndex < renderedKnotIndices.Count;
             visibleIndex++)
        {
            int i = renderedKnotIndices[visibleIndex];
            float u = knotParameters[i];
            Vector3 worldPoint = finalPoints[i];
            Vector3 worldDerivative = finalDerivatives[i];

            bool hasPrevious = visibleIndex > 0;
            bool hasNext = visibleIndex < renderedKnotIndices.Count - 1;

            float previousDu = 0f;
            if (hasPrevious)
            {
                int previousIndex = renderedKnotIndices[visibleIndex - 1];
                previousDu = Mathf.Max(
                    Eps,
                    u - knotParameters[previousIndex]);
            }

            float nextDu = 0f;
            if (hasNext)
            {
                int nextIndex = renderedKnotIndices[visibleIndex + 1];
                nextDu = Mathf.Max(
                    Eps,
                    knotParameters[nextIndex] - u);
            }

            Vector3 tangentInWorld =
                hasPrevious
                    ? -worldDerivative * previousDu / 3f * bezierTangentScale
                    : Vector3.zero;

            // No outgoing handle is allowed past SlopeExit. The old oversized
            // terminal handle was one of the visible causes of the long tail.
            Vector3 tangentOutWorld =
                hasNext
                    ? worldDerivative * nextDu / 3f * bezierTangentScale
                    : Vector3.zero;

            Vector3 localPoint =
                physicsSplineRoot.InverseTransformPoint(worldPoint);

            Vector3 localTangentIn =
                physicsSplineRoot.InverseTransformVector(tangentInWorld);

            Vector3 localTangentOut =
                physicsSplineRoot.InverseTransformVector(tangentOutWorld);

            BezierKnot knot = new BezierKnot(
                ToFloat3(localPoint),
                ToFloat3(localTangentIn),
                ToFloat3(localTangentOut));

            spline.Add(knot, TangentMode.Broken);

            float bridgeN = baselineOnlyDiagnostic
                ? 0f
                : EvaluateBoundaryResidual(
                    plan,
                    u,
                    plan.residualNormal0,
                    plan.residualNormalSlopePerMeter0);

            float bridgeB = baselineOnlyDiagnostic
                ? 0f
                : EvaluateBoundaryResidual(
                    plan,
                    u,
                    plan.residualSide0,
                    plan.residualSideSlopePerMeter0);

            float wave =
                baselineOnlyDiagnostic
                    ? 0f
                    : EvaluateReferenceNormalOffset(plan, u);

            lastBuiltKnots.Add(new KnotDiagnostic
            {
                u = u,
                positionVisual = worldPoint,
                tangentVisual = NormalizeSafe(worldDerivative, Vector3.forward),
                normalVisual = normals[i],
                sideVisual = sides[i],
                bridgeNormal = bridgeN,
                bridgeSide = bridgeB,
                spatialWave = wave,
                futurePredictionAnchor = Approximately(u, 0f),
                targetProgressAnchor = Approximately(u, plan.uT),
                missileShadowAnchor = Approximately(u, plan.uM),
                terminalLandingAnchor =
                    renderedEndU >= 1f - 0.0001f && Approximately(u, 1f)
            });
        }

        BuildPredictionArcLengthTable(plan);

        int ptIndex = FindParameterIndex(plan.uT);
        Vector3 ptFinal =
            ptIndex >= 0 && ptIndex < finalPoints.Count
                ? finalPoints[ptIndex]
                : plan.pt;

        maxBuiltBridgeNormalMeters = 0f;
        maxBuiltWaveMeters = 0f;
        maxBuiltTotalNormalOffsetMeters = 0f;

        for (int i = 0; i < lastBuiltKnots.Count; i++)
        {
            KnotDiagnostic d = lastBuiltKnots[i];
            maxBuiltBridgeNormalMeters = Mathf.Max(
                maxBuiltBridgeNormalMeters,
                Mathf.Abs(d.bridgeNormal));
            maxBuiltWaveMeters = Mathf.Max(
                maxBuiltWaveMeters,
                Mathf.Abs(d.spatialWave));
            maxBuiltTotalNormalOffsetMeters = Mathf.Max(
                maxBuiltTotalNormalOffsetMeters,
                Mathf.Abs(d.bridgeNormal + d.spatialWave));
        }

        target.lastKnotCount = spline.Count;
        target.lastInternalKnotCount = knotParameters.Count;
        target.lastRenderedEndU = renderedEndU;
        target.lastUT = plan.uT;
        target.lastUM = plan.uM;
        target.lastPF = finalPoints.Count > 0 ? finalPoints[0] : plan.pfBase;
        target.lastPFGuideCenter = plan.pfGuideCenter;
        target.lastPT = plan.pt;
        target.lastPTSurface = plan.ptSurface;
        target.lastPTCenterBase = plan.ptCenterBase;
        target.lastPTFinal = ptFinal;
        target.lastPM = plan.pm;
        target.lastPL = plan.pl;
        target.lastReferenceCenterClearance = referenceCenterClearanceMeters;
        target.lastIncidentNormalSpeed = incidentBoundaryNormalSpeed;
        target.lastIncidentSideSpeed = incidentBoundarySideSpeed;
        target.lastMaxBridgeNormal = maxBuiltBridgeNormalMeters;
        target.lastMaxWave = maxBuiltWaveMeters;
        target.lastMaxTotalNormalOffset = maxBuiltTotalNormalOffsetMeters;
        target.lastPfProgress01 = pfSlopeProgress01;
        target.lastPtProgress01 = ptSlopeProgress01;
        target.lastPredictedSpeed = predictedForwardSpeed;
        target.lastWaveAmplitude = plan.referenceAmplitudeDiagnostic;
        target.lastTimeToPT = estimatedTimeToPT;
        target.lastTimeToPL = estimatedTimeToPL;
        target.lastTerminalErrorMeters =
            finalPoints.Count > 0
                ? Vector3.Distance(finalPoints[finalPoints.Count - 1], plan.pl)
                : float.PositiveInfinity;

        lastSpatialTemplateAuthority01 = plan.referenceTemplateAuthority01;
        BuildVirtualUpperField(target, plan);
        RefreshExtrude(target);
        SchedulePredictionProbes(plan);
    }

    void BuildAnalyticKnotParameters(CurvePlan plan)
    {
        knotParameters.Clear();
        AddUniqueParameter(0f);
        AddUniqueParameter(plan.uT);
        AddUniqueParameter(plan.uM);
        AddUniqueParameter(Mathf.Clamp01(plan.uSlopeExit));
        AddUniqueParameter(1f);

        int waves = Mathf.Max(1, spatialWavesPerStair);
        int divisions = Mathf.Max(2, phaseDivisionsPerWave);
        int count = waves * divisions;
        float start = Mathf.Clamp01(plan.startSlopeProgress01);
        float exitU = Mathf.Clamp(plan.uSlopeExit, 0.05f, 1f);
        float lookahead = Mathf.Max(0.10f, plan.modelLookaheadMeters);
        float maxProgressFromLookahead = Mathf.Clamp01(
            start + lookahead / Mathf.Max(Eps, plan.sectionLength));

        // Keep analytic phase extrema only inside the receding model horizon.
        // Beyond it, semantic anchors remain but no dense Equalizer-specific knots are added.
        for (int k = 0; k <= count; k++)
        {
            float absoluteProgress = k / (float)count;
            if (absoluteProgress + 0.0001f < start ||
                absoluteProgress - 0.0001f > maxProgressFromLookahead)
            {
                continue;
            }

            float localProgress =
                (absoluteProgress - start) /
                Mathf.Max(Eps, 1f - start);
            AddUniqueParameter(Mathf.Clamp01(localProgress) * exitU);
        }

        AddRoughnessProtectedKnotParameters(plan, start, maxProgressFromLookahead, exitU);
        AddUpperContactProtectedKnotParameters(plan);
        knotParameters.Sort();
    }

    void AddRoughnessProtectedKnotParameters(
        CurvePlan plan,
        float startProgress,
        float endProgress,
        float exitU)
    {
        lastRoughnessProtectedKnotCount = 0;
        if (maximumRoughnessProtectedKnots <= 0 || !spatialTemplateReady)
            return;

        roughnessProtectedProgress.Clear();
        roughnessProtectedScore.Clear();
        int required = Mathf.Max(1, minimumTemplateRunsPerBin);
        float threshold = Mathf.Max(0f, roughnessProtectedKnotThreshold);

        for (int i = 0; i < spatialTemplateBins.Length; i++)
        {
            SpatialTemplateBin b = spatialTemplateBins[i];
            if (b.runs < required || b.meanRoughness < threshold)
                continue;

            float progress = i / (float)Mathf.Max(1, spatialTemplateBins.Length - 1);
            if (progress + 0.0001f < startProgress ||
                progress - 0.0001f > endProgress)
            {
                continue;
            }

            int insert = roughnessProtectedScore.Count;
            for (int j = 0; j < roughnessProtectedScore.Count; j++)
            {
                if (b.meanRoughness > roughnessProtectedScore[j])
                {
                    insert = j;
                    break;
                }
            }
            roughnessProtectedScore.Insert(insert, b.meanRoughness);
            roughnessProtectedProgress.Insert(insert, progress);
        }

        int take = Mathf.Min(
            maximumRoughnessProtectedKnots,
            roughnessProtectedProgress.Count);
        for (int i = 0; i < take; i++)
        {
            float progress = roughnessProtectedProgress[i];
            float localProgress =
                (progress - startProgress) /
                Mathf.Max(Eps, 1f - startProgress);
            AddUniqueParameter(Mathf.Clamp01(localProgress) * exitU);
            lastRoughnessProtectedKnotCount++;
        }
    }

    void AddUpperContactProtectedKnotParameters(CurvePlan plan)
    {
        lastUpperContactProtectedKnotCount = 0;
        lastUpperContactCandidateU = -1f;
        lastUpperContactCandidateGapMeters = float.PositiveInfinity;
        lastUpperContactCandidateClosingSpeed = 0f;
        lastUpperContactCandidateEnergyJ = 0f;
        lastUpperContactCandidateSource = string.Empty;
        lastUpperContactSearchSampleCount = 0;
        lastUpperContactCandidateRejectReason = string.Empty;

        float radius = ResolveEqualizerWorldRadius();
        if (radius <= Eps || plan.totalChordLength <= Eps)
            return;

        float exitU = Mathf.Clamp(plan.uSlopeExit, 0.05f, 1f);
        float lookaheadU = Mathf.Clamp01(
            Mathf.Max(0.10f, plan.modelLookaheadMeters) /
            Mathf.Max(Eps, plan.totalChordLength));
        float maxU = Mathf.Min(exitU, lookaheadU);

        if (maxU <= 0.001f || plan.totalDuration <= Eps)
            return;

        float gapAtStart = EvaluatePredictedUpperGap(plan, 0f, radius);
        if (!IsFiniteScalar(gapAtStart))
        {
            lastUpperContactCandidateRejectReason = "InvalidStartGap";
            return;
        }

        // Search resolution is spatial rather than an arbitrary fixed count.
        // The internal scan may be dense, but only the accepted contact
        // neighborhood is converted into final FutureSpline knots.
        int minimumSamples = Mathf.Clamp(
            upperContactMinimumSearchSamples,
            8,
            64);
        int maximumSamples = Mathf.Clamp(
            upperContactMaximumSearchSamples,
            Mathf.Max(16, minimumSamples),
            96);
        float searchSpacingMeters =
            Mathf.Max(
                0.05f,
                radius * Mathf.Max(0.05f, upperContactSearchSpacingR));
        float searchDistanceMeters =
            maxU * Mathf.Max(Eps, plan.totalChordLength);

        int searchSamples = Mathf.Clamp(
            Mathf.CeilToInt(
                searchDistanceMeters /
                searchSpacingMeters),
            minimumSamples,
            maximumSamples);

        lastUpperContactSearchSampleCount = searchSamples;

        // g(u) = Upper center span - predicted Equalizer Normal offset.
        // A real center-locus contact is therefore g(u)=0.
        //
        // Candidate priority:
        //   1) earliest positive -> nonpositive root,
        //   2) earliest near-contact local minimum,
        //   3) validated first-order TOI seed.
        bool rootCandidate = false;
        float rootU = -1f;
        float rootGap = float.PositiveInfinity;

        bool localMinimumCandidate = false;
        float localMinimumU = -1f;
        float localMinimumGap = float.PositiveInfinity;

        float grazingGapMeters =
            Mathf.Max(0f, upperContactGrazingGapR) * radius;

        float previousU = 0f;
        float previousGap = gapAtStart;
        bool previousFinite = true;

        float leftU = 0f;
        float leftGap = gapAtStart;
        float middleU = 0f;
        float middleGap = gapAtStart;
        bool haveMiddle = false;

        // If prediction begins already at/inside the Upper, preserve u=0 as
        // a candidate so the existing StartOverlap quality gate can reject it
        // explicitly instead of hiding the diagnostic.
        if (gapAtStart <= 0f)
        {
            rootCandidate = true;
            rootU = 0f;
            rootGap = gapAtStart;
        }

        for (int i = 1; i <= searchSamples; i++)
        {
            float currentU =
                maxU * i /
                Mathf.Max(1f, searchSamples);
            float currentGap =
                EvaluatePredictedUpperGap(
                    plan,
                    currentU,
                    radius);

            bool currentFinite =
                IsFiniteScalar(currentGap);

            if (!currentFinite)
            {
                previousFinite = false;
                haveMiddle = false;
                continue;
            }

            if (previousFinite)
            {
                if (!rootCandidate &&
                    previousGap > 0f &&
                    currentGap <= 0f)
                {
                    rootU = RefineUpperContactRootBisection(
                        plan,
                        previousU,
                        previousGap,
                        currentU,
                        currentGap,
                        radius,
                        out rootGap);
                    rootCandidate = true;
                }

                if (!localMinimumCandidate)
                {
                    if (!haveMiddle)
                    {
                        leftU = previousU;
                        leftGap = previousGap;
                        middleU = currentU;
                        middleGap = currentGap;
                        haveMiddle = true;
                    }
                    else
                    {
                        bool isLocalMinimum =
                            middleGap <= leftGap &&
                            middleGap <= currentGap;

                        if (isLocalMinimum)
                        {
                            float refinedU =
                                RefineUpperContactLocalMinimum(
                                    plan,
                                    leftU,
                                    currentU,
                                    radius,
                                    out float refinedGap);

                            if (IsFiniteScalar(refinedGap) &&
                                refinedGap <= grazingGapMeters)
                            {
                                localMinimumCandidate = true;
                                localMinimumU = refinedU;
                                localMinimumGap = refinedGap;
                            }
                        }

                        leftU = middleU;
                        leftGap = middleGap;
                        middleU = currentU;
                        middleGap = currentGap;
                    }
                }
            }
            else
            {
                // Restart local-neighborhood tracking after an invalid gap
                // instead of constructing a bracket across missing data.
                leftU = currentU;
                leftGap = currentGap;
                haveMiddle = false;
            }

            previousU = currentU;
            previousGap = currentGap;
            previousFinite = true;
        }

        float incomingVN =
            Vector3.Dot(
                plan.incidentVelocityPF,
                plan.initialNormal);

        // TOI is now based on the actual remaining gap g(0), not the full
        // historical Upper span. It remains only a fallback seed for a narrow
        // contact that the bracket scan may have stepped across.
        bool toiCandidate = false;
        float toiU = -1f;
        float toiGap = float.PositiveInfinity;
        float initialClosingSpeed = Mathf.Max(
            EstimatePredictedUpperClosingSpeed(
                plan,
                0f,
                maxU,
                radius),
            Mathf.Max(0f, incomingVN));

        if (!rootCandidate &&
            !localMinimumCandidate &&
            gapAtStart > 0f &&
            initialClosingSpeed >
                Mathf.Max(Eps, upperContactMinimumImpactSpeed))
        {
            float timeToUpper =
                gapAtStart /
                Mathf.Max(Eps, initialClosingSpeed);

            toiU =
                timeToUpper /
                Mathf.Max(Eps, plan.totalDuration);

            if (toiU >= 0f &&
                toiU <= maxU + 0.0001f)
            {
                toiU = Mathf.Clamp(toiU, 0f, maxU);
                toiGap =
                    EvaluatePredictedUpperGap(
                        plan,
                        toiU,
                        radius);

                float maximumToiGapMeters =
                    Mathf.Max(
                        0f,
                        upperContactToiValidationGapR) *
                    radius;

                toiCandidate =
                    IsFiniteScalar(toiGap) &&
                    toiGap <= maximumToiGapMeters;
            }
        }

        float candidateU;
        float candidateGap;
        string candidateSource;

        if (rootCandidate)
        {
            candidateU = rootU;
            candidateGap = rootGap;
            candidateSource = "RootCrossing";
        }
        else if (localMinimumCandidate)
        {
            candidateU = localMinimumU;
            candidateGap = localMinimumGap;
            candidateSource = "LocalMinimum";
        }
        else if (toiCandidate)
        {
            candidateU = toiU;
            candidateGap = toiGap;
            candidateSource = "TOISeed";
        }
        else
        {
            lastUpperContactCandidateRejectReason = "NoContactCandidate";
            return;
        }

        candidateU =
            Mathf.Clamp(
                candidateU,
                0f,
                maxU);

        float predictedClosingSpeed =
            EstimatePredictedUpperClosingSpeed(
                plan,
                candidateU,
                maxU,
                radius);

        // TOI comes directly from the causal PF closing state, so preserve that
        // evidence when the local finite-difference derivative is weaker.
        if (candidateSource == "TOISeed")
        {
            predictedClosingSpeed = Mathf.Max(
                predictedClosingSpeed,
                initialClosingSpeed);
        }

        float predictedEnergy =
            ComputeNormalImpactEnergy(
                predictedClosingSpeed);

        lastUpperContactCandidateU = candidateU;
        lastUpperContactCandidateGapMeters = candidateGap;
        lastUpperContactCandidateClosingSpeed = predictedClosingSpeed;
        lastUpperContactCandidateEnergyJ = predictedEnergy;
        lastUpperContactCandidateSource = candidateSource;

        // Keep the previously validated quality gate. Search recall is improved
        // before this point; startup overlap and zero-energy events remain rejected.
        if (candidateU <= Mathf.Clamp01(upperContactStartNeighborhoodU) &&
            gapAtStart <=
                Mathf.Max(0f, upperContactStartOverlapGapR) *
                radius)
        {
            lastUpperContactCandidateRejectReason = "StartOverlap";
            return;
        }

        if (!PassesUpperImpactEnergyGate(
                predictedClosingSpeed,
                predictedEnergy,
                out string candidateRejectReason))
        {
            lastUpperContactCandidateRejectReason =
                candidateRejectReason;
            return;
        }

        float halfWidthU =
            Mathf.Max(
                0.01f,
                upperContactProtectedHalfWidthMeters) /
            Mathf.Max(
                Eps,
                plan.totalChordLength);

        int beforeCount =
            knotParameters.Count;

        AddUniqueParameter(
            Mathf.Clamp(
                candidateU - halfWidthU,
                0f,
                maxU));

        AddUniqueParameter(candidateU);

        AddUniqueParameter(
            Mathf.Clamp(
                candidateU + halfWidthU,
                0f,
                maxU));

        lastUpperContactProtectedKnotCount =
            Mathf.Max(
                0,
                knotParameters.Count - beforeCount);

        lastUpperContactCandidateRejectReason =
            string.Empty;
    }

    float RefineUpperContactRootBisection(
        CurvePlan plan,
        float leftU,
        float leftGap,
        float rightU,
        float rightGap,
        float radius,
        out float refinedGap)
    {
        float a = Mathf.Min(leftU, rightU);
        float b = Mathf.Max(leftU, rightU);
        float ga = leftU <= rightU ? leftGap : rightGap;
        float gb = leftU <= rightU ? rightGap : leftGap;

        if (!IsFiniteScalar(ga) ||
            !IsFiniteScalar(gb) ||
            ga <= 0f ||
            gb > 0f)
        {
            refinedGap = gb;
            return b;
        }

        int iterations =
            Mathf.Clamp(
                upperContactRootBisectionIterations,
                4,
                20);

        for (int i = 0; i < iterations; i++)
        {
            float mid =
                0.5f * (a + b);
            float gm =
                EvaluatePredictedUpperGap(
                    plan,
                    mid,
                    radius);

            if (!IsFiniteScalar(gm))
                break;

            if (gm > 0f)
            {
                a = mid;
                ga = gm;
            }
            else
            {
                b = mid;
                gb = gm;
            }
        }

        refinedGap = gb;
        return b;
    }

    float RefineUpperContactLocalMinimum(
        CurvePlan plan,
        float leftU,
        float rightU,
        float radius,
        out float refinedGap)
    {
        float a = Mathf.Min(leftU, rightU);
        float b = Mathf.Max(leftU, rightU);

        // Small fixed ternary refinement is enough because this is only a
        // candidate locator; the final Event Quality Gate still decides whether
        // the neighborhood is physically meaningful.
        const int iterations = 8;
        for (int i = 0; i < iterations; i++)
        {
            float third =
                (b - a) / 3f;
            float u1 = a + third;
            float u2 = b - third;

            float g1 =
                EvaluatePredictedUpperGap(
                    plan,
                    u1,
                    radius);
            float g2 =
                EvaluatePredictedUpperGap(
                    plan,
                    u2,
                    radius);

            if (!IsFiniteScalar(g1) ||
                !IsFiniteScalar(g2))
            {
                break;
            }

            if (g1 <= g2)
                b = u2;
            else
                a = u1;
        }

        float u =
            0.5f * (a + b);
        refinedGap =
            EvaluatePredictedUpperGap(
                plan,
                u,
                radius);

        return u;
    }

    float EstimatePredictedUpperClosingSpeed(
        CurvePlan plan,
        float u,
        float maxU,
        float radius)
    {
        if (plan.totalDuration <= Eps || maxU <= Eps)
            return 0f;

        u = Mathf.Clamp(u, 0f, maxU);

        // Central finite difference where possible, one-sided at the horizon edges.
        // g(u) is Upper-center gap; -dg/dt is the predicted closing speed.
        float du = Mathf.Clamp(
            0.01f,
            0.001f,
            Mathf.Max(0.001f, 0.25f * maxU));
        float u0 = Mathf.Max(0f, u - du);
        float u1 = Mathf.Min(maxU, u + du);

        if (u1 - u0 <= Eps)
            return 0f;

        float g0 = EvaluatePredictedUpperGap(plan, u0, radius);
        float g1 = EvaluatePredictedUpperGap(plan, u1, radius);
        if (!IsFiniteScalar(g0) || !IsFiniteScalar(g1))
            return 0f;

        float dgDu = (g1 - g0) / Mathf.Max(Eps, u1 - u0);
        float dgDt = dgDu / Mathf.Max(Eps, plan.totalDuration);
        return Mathf.Max(0f, -dgDt);
    }

    float ComputeNormalImpactEnergy(float closingSpeed)
    {
        float mass = equalizerBody
            ? Mathf.Max(Eps, equalizerBody.mass)
            : 1f;
        float speed = Mathf.Max(0f, closingSpeed);
        return 0.5f * mass * speed * speed;
    }

    bool PassesUpperImpactEnergyGate(
        float closingSpeed,
        float energyJ,
        out string rejectReason)
    {
        rejectReason = string.Empty;

        if (!IsFiniteScalar(closingSpeed) ||
            closingSpeed < Mathf.Max(0f, upperContactMinimumImpactSpeed))
        {
            rejectReason = "LowClosingSpeed";
            return false;
        }

        if (!IsFiniteScalar(energyJ) ||
            energyJ < Mathf.Max(0f, upperContactMinimumImpactEnergyJ))
        {
            rejectReason = "LowImpactEnergy";
            return false;
        }

        return true;
    }

    float EvaluatePredictedUpperGap(
        CurvePlan plan,
        float u,
        float radius)
    {
        u = Mathf.Clamp01(u);

        // Carrier and Upper share the same finite boundary bridge, so that bridge
        // cancels from the center-to-Upper gap. What matters here is the Equalizer
        // motion relative to the carrier.
        float referenceNormal =
            EvaluateReferenceNormalOffset(
                plan,
                u);

        float predictedNormal =
            referenceNormal;

        if (mechanicalNormalPredictionMode ==
            MechanicalNormalPredictionMode.BlendIntoUpperGap)
        {
            bool mechanicalValid =
                TryEvaluatePredictedNormalLayers(
                    plan,
                    u,
                    out _,
                    out _,
                    out float blendedNormal);

            if (mechanicalValid)
            {
                predictedNormal =
                    blendedNormal;
            }
        }

        float elapsed =
            mechanicalNormalPredictionMode ==
                MechanicalNormalPredictionMode.BlendIntoUpperGap
                ? ResolvePredictionElapsedTimeAtU(plan, u)
                : u * Mathf.Max(0.001f, plan.totalDuration);

        ResolveHistoricalUpperSpanAtElapsedTime(
            u,
            elapsed,
            plan.totalDuration,
            radius,
            out _,
            out _,
            out _,
            out float upperSpanMeters);

        return
            upperSpanMeters -
            predictedNormal;
    }

    void AddUniqueParameter(float u)
    {
        u = Mathf.Clamp01(u);

        for (int i = 0; i < knotParameters.Count; i++)
        {
            if (Mathf.Abs(knotParameters[i] - u) <= 0.0001f)
                return;
        }

        knotParameters.Add(u);
    }

    int FindParameterIndex(float u)
    {
        for (int i = 0; i < knotParameters.Count; i++)
        {
            if (Mathf.Abs(knotParameters[i] - u) <= 0.0001f)
                return i;
        }

        return -1;
    }

    void BuildBaselineAndRotationMinimizingFrame(CurvePlan plan)
    {
        baselinePoints.Clear();
        baselineTangents.Clear();
        normals.Clear();
        sides.Clear();

        Vector3 previousTangent = Vector3.zero;
        Vector3 previousNormal = plan.initialNormal;

        for (int i = 0; i < knotParameters.Count; i++)
        {
            float u = knotParameters[i];
            EvaluateBaseline(
                plan,
                u,
                out Vector3 point,
                out Vector3 derivative);

            Vector3 tangent = NormalizeSafe(
                derivative,
                i > 0 ? previousTangent : plan.tangentPF);

            Vector3 normal;

            if (i == 0)
            {
                normal = BuildInitialNormal(tangent, plan.initialNormal);
            }
            else
            {
                Quaternion transport =
                    Quaternion.FromToRotation(previousTangent, tangent);

                normal = transport * previousNormal;
                normal = Vector3.ProjectOnPlane(normal, tangent);

                if (normal.sqrMagnitude <= Eps)
                    normal = BuildInitialNormal(tangent, Vector3.up);
                else
                    normal.Normalize();
            }

            Vector3 side =
                NormalizeSafe(
                    Vector3.Cross(normal, tangent),
                    Vector3.right);

            normal =
                NormalizeSafe(
                    Vector3.Cross(tangent, side),
                    normal);

            baselinePoints.Add(point);
            baselineTangents.Add(tangent);
            normals.Add(normal);
            sides.Add(side);

            previousTangent = tangent;
            previousNormal = normal;
        }
    }

    void BuildFinalAnalyticPoints(CurvePlan plan)
    {
        finalPoints.Clear();

        for (int i = 0; i < knotParameters.Count; i++)
        {
            float u = knotParameters[i];
            float bridgeN = baselineOnlyDiagnostic
                ? 0f
                : EvaluateBoundaryResidual(
                    plan,
                    u,
                    plan.residualNormal0,
                    plan.residualNormalSlopePerMeter0);
            float bridgeB = baselineOnlyDiagnostic
                ? 0f
                : EvaluateBoundaryResidual(
                    plan,
                    u,
                    plan.residualSide0,
                    plan.residualSideSlopePerMeter0);
            float referenceN = baselineOnlyDiagnostic
                ? 0f
                : EvaluateReferenceNormalOffset(plan, u);

            finalPoints.Add(
                baselinePoints[i] +
                normals[i] * (bridgeN + referenceN) +
                sides[i] * bridgeB);
        }
    }

    void BuildBezierDerivatives(CurvePlan plan)
    {
        finalDerivatives.Clear();

        int count = finalPoints.Count;
        for (int i = 0; i < count; i++)
        {
            if (count <= 1)
            {
                finalDerivatives.Add(Vector3.forward);
                continue;
            }

            if (i == 0 && IsFinite(plan.exactFinalDerivativePF))
            {
                finalDerivatives.Add(plan.exactFinalDerivativePF);
                continue;
            }

            if (i == count - 1)
            {
                finalDerivatives.Add(plan.tangentPL);
                continue;
            }

            float leftSpan = Mathf.Max(Eps, knotParameters[i] - knotParameters[i - 1]);
            float rightSpan = Mathf.Max(Eps, knotParameters[i + 1] - knotParameters[i]);
            float du = Mathf.Max(0.0005f, Mathf.Min(leftSpan, rightSpan) * 0.25f);
            float u0 = Mathf.Clamp01(knotParameters[i] - du);
            float u1 = Mathf.Clamp01(knotParameters[i] + du);

            if (!TryEvaluateFinalAnalyticPoint(plan, u0, out Vector3 p0) ||
                !TryEvaluateFinalAnalyticPoint(plan, u1, out Vector3 p1))
            {
                float span = Mathf.Max(Eps, knotParameters[i + 1] - knotParameters[i - 1]);
                finalDerivatives.Add(
                    (finalPoints[i + 1] - finalPoints[i - 1]) / span);
                continue;
            }

            finalDerivatives.Add((p1 - p0) / Mathf.Max(Eps, u1 - u0));
        }
    }

    bool TryEvaluateTransportFrameAtU(
        float u,
        Vector3 tangent,
        out Vector3 normal,
        out Vector3 side)
    {
        normal = Vector3.up;
        side = Vector3.right;
        if (knotParameters.Count == 0 || normals.Count != knotParameters.Count)
            return false;

        u = Mathf.Clamp01(u);
        int upper = 0;
        while (upper < knotParameters.Count && knotParameters[upper] < u)
            upper++;

        if (upper <= 0)
        {
            normal = normals[0];
            side = sides[0];
        }
        else if (upper >= knotParameters.Count)
        {
            normal = normals[normals.Count - 1];
            side = sides[sides.Count - 1];
        }
        else
        {
            int lower = upper - 1;
            float t = Mathf.InverseLerp(
                knotParameters[lower],
                knotParameters[upper],
                u);
            normal = NormalizeSafe(
                Vector3.Lerp(normals[lower], normals[upper], t),
                normals[lower]);
            side = NormalizeSafe(
                Vector3.Lerp(sides[lower], sides[upper], t),
                sides[lower]);
        }

        tangent = NormalizeSafe(tangent, Vector3.forward);
        normal = NormalizeSafe(Vector3.ProjectOnPlane(normal, tangent), normal);
        side = NormalizeSafe(Vector3.Cross(normal, tangent), side);
        normal = NormalizeSafe(Vector3.Cross(tangent, side), normal);
        return true;
    }

    bool TryEvaluateFinalAnalyticPoint(
        CurvePlan plan,
        float u,
        out Vector3 position)
    {
        position = Vector3.zero;
        EvaluateBaseline(plan, u, out Vector3 baseline, out Vector3 derivative);
        if (!TryEvaluateTransportFrameAtU(
                u,
                derivative,
                out Vector3 normal,
                out Vector3 side))
        {
            return false;
        }

        float residualN = baselineOnlyDiagnostic
            ? 0f
            : EvaluateBoundaryResidual(
                plan,
                u,
                plan.residualNormal0,
                plan.residualNormalSlopePerMeter0);
        float residualB = baselineOnlyDiagnostic
            ? 0f
            : EvaluateBoundaryResidual(
                plan,
                u,
                plan.residualSide0,
                plan.residualSideSlopePerMeter0);
        float referenceN = baselineOnlyDiagnostic
            ? 0f
            : EvaluateReferenceNormalOffset(plan, u);

        position =
            baseline +
            normal * (residualN + referenceN) +
            side * residualB;
        return IsFinite(position);
    }

    bool TryEvaluateFinalAnalyticDiagnostic(
        CurvePlan plan,
        float u,
        out Vector3 position,
        out Vector3 tangent,
        out Vector3 normal,
        out Vector3 side)
    {
        position = Vector3.zero;
        tangent = Vector3.forward;
        normal = Vector3.up;
        side = Vector3.right;

        if (!TryEvaluateFinalAnalyticPoint(plan, u, out position))
            return false;

        float du = 0.0015f;
        float u0 = Mathf.Clamp01(u - du);
        float u1 = Mathf.Clamp01(u + du);
        if (!TryEvaluateFinalAnalyticPoint(plan, u0, out Vector3 p0) ||
            !TryEvaluateFinalAnalyticPoint(plan, u1, out Vector3 p1))
        {
            return false;
        }

        EvaluateBaseline(plan, u, out _, out Vector3 baselineDerivative);
        tangent = NormalizeSafe(p1 - p0, baselineDerivative);
        return TryEvaluateTransportFrameAtU(u, tangent, out normal, out side);
    }


    void EvaluateBaseline(
        CurvePlan plan,
        float u,
        out Vector3 position,
        out Vector3 derivativeGlobalU)
    {
        u = Mathf.Clamp01(u);

        if (u <= plan.uT)
        {
            EvaluateHermiteSegment(
                plan.pfBase,
                plan.tangentPF,
                plan.pt,
                plan.tangentPT,
                0f,
                plan.uT,
                u,
                out position,
                out derivativeGlobalU);
            return;
        }

        if (u <= plan.uM)
        {
            EvaluateHermiteSegment(
                plan.pt,
                plan.tangentPT,
                plan.pm,
                plan.tangentPM,
                plan.uT,
                plan.uM,
                u,
                out position,
                out derivativeGlobalU);
            return;
        }

        EvaluateHermiteSegment(
            plan.pm,
            plan.tangentPM,
            plan.pl,
            plan.tangentPL,
            plan.uM,
            1f,
            u,
            out position,
            out derivativeGlobalU);
    }

    static void EvaluateHermiteSegment(
        Vector3 p0,
        Vector3 m0Global,
        Vector3 p1,
        Vector3 m1Global,
        float u0,
        float u1,
        float u,
        out Vector3 position,
        out Vector3 derivativeGlobalU)
    {
        float span = Mathf.Max(Eps, u1 - u0);
        float t = Mathf.Clamp01((u - u0) / span);
        float t2 = t * t;
        float t3 = t2 * t;

        float h00 = 2f * t3 - 3f * t2 + 1f;
        float h10 = t3 - 2f * t2 + t;
        float h01 = -2f * t3 + 3f * t2;
        float h11 = t3 - t2;

        Vector3 m0LocalT = m0Global * span;
        Vector3 m1LocalT = m1Global * span;

        position =
            h00 * p0 +
            h10 * m0LocalT +
            h01 * p1 +
            h11 * m1LocalT;

        float dh00 = 6f * t2 - 6f * t;
        float dh10 = 3f * t2 - 4f * t + 1f;
        float dh01 = -6f * t2 + 6f * t;
        float dh11 = 3f * t2 - 2f * t;

        Vector3 derivativeLocalT =
            dh00 * p0 +
            dh10 * m0LocalT +
            dh01 * p1 +
            dh11 * m1LocalT;

        derivativeGlobalU = derivativeLocalT / span;
    }

    static float EvaluateQuinticBridge(
        float u,
        float d0,
        float d1,
        float d2)
    {
        u = Mathf.Clamp01(u);

        float a0 = d0;
        float a1 = d1;
        float a2 = 0.5f * d2;
        float a3 = -10f * d0 - 6f * d1 - 1.5f * d2;
        float a4 = 15f * d0 + 8f * d1 + 1.5f * d2;
        float a5 = -6f * d0 - 3f * d1 - 0.5f * d2;

        float u2 = u * u;
        float u3 = u2 * u;
        float u4 = u3 * u;
        float u5 = u4 * u;

        return
            a0 +
            a1 * u +
            a2 * u2 +
            a3 * u3 +
            a4 * u4 +
            a5 * u5;
    }



    static void BuildChordParameters(
        Vector3 pf,
        Vector3 pt,
        Vector3 pm,
        Vector3 pl,
        out float uT,
        out float uM,
        out float total)
    {
        float a = Mathf.Max(0.001f, Vector3.Distance(pf, pt));
        float b = Mathf.Max(0.001f, Vector3.Distance(pt, pm));
        float c = Mathf.Max(0.001f, Vector3.Distance(pm, pl));

        total = Mathf.Max(0.003f, a + b + c);
        uT = Mathf.Clamp(a / total, 0.05f, 0.90f);
        uM = Mathf.Clamp((a + b) / total, uT + 0.025f, 0.975f);
    }

    // ================================================================
    // Integrated Virtual Upper
    // FutureSpline geometry + historical Upper-ON teacher + colliderless contact.
    // ================================================================

    void BuildVirtualUpperField(
        GeneratedStairSpline target,
        CurvePlan plan)
    {
        target.virtualUpperSamples.Clear();

        float radius = ResolveEqualizerWorldRadius();
        if (radius <= Eps || knotParameters.Count == 0)
            return;

        float maximumActiveU = Mathf.Clamp(plan.uSlopeExit, 0.05f, 1f);

        for (int i = 0; i < knotParameters.Count; i++)
        {
            float u = knotParameters[i];

            // Virtual Upper is disarmed at SlopeExit; post-slope samples had no
            // runtime effect and only increased work and gizmo tail length.
            if (u > maximumActiveU + 0.0001f)
                break;

            float bridgeN = baselineOnlyDiagnostic
                ? 0f
                : EvaluateBoundaryResidual(
                    plan,
                    u,
                    plan.residualNormal0,
                    plan.residualNormalSlopePerMeter0);

            float bridgeB = baselineOnlyDiagnostic
                ? 0f
                : EvaluateBoundaryResidual(
                    plan,
                    u,
                    plan.residualSide0,
                    plan.residualSideSlopePerMeter0);

            // Carrier base excludes the Equalizer display wave. The Virtual Upper
            // lives above this carrier using historical 4R-Hn/decay teacher data.
            Vector3 carrierCenter =
                baselinePoints[i] +
                normals[i] * bridgeN +
                sides[i] * bridgeB;

            float upperElapsed =
                mechanicalNormalPredictionMode ==
                    MechanicalNormalPredictionMode.BlendIntoUpperGap
                    ? ResolvePredictionElapsedTimeAtU(plan, u)
                    : u * Mathf.Max(0.001f, plan.totalDuration);

            ResolveHistoricalUpperSpanAtElapsedTime(
                u,
                upperElapsed,
                plan.totalDuration,
                radius,
                out float ceilingR,
                out float retention01,
                out float spanR,
                out float spanMeters);

            Vector3 upperCenter =
                carrierCenter +
                normals[i] * spanMeters;

            target.virtualUpperSamples.Add(
                new VirtualUpperKnotSample
                {
                    u = u,
                    carrierCenterLocal =
                        physicsSplineRoot.InverseTransformPoint(carrierCenter),
                    tangentLocal =
                        physicsSplineRoot.InverseTransformDirection(
                            baselineTangents[i]),
                    normalLocal =
                        physicsSplineRoot.InverseTransformDirection(
                            normals[i]),
                    sideLocal =
                        physicsSplineRoot.InverseTransformDirection(
                            sides[i]),
                    ceilingR = ceilingR,
                    retention01 = retention01,
                    spanR = spanR,
                    spanMeters = spanMeters,
                    upperCenterLocal =
                        physicsSplineRoot.InverseTransformPoint(upperCenter)
                });
        }
    }

    void ResolveHistoricalUpperSpanAtElapsedTime(
        float u,
        float elapsedSeconds,
        float totalDuration,
        float radius,
        out float ceilingR,
        out float retention01,
        out float spanR,
        out float spanMeters)
    {
        u = Mathf.Clamp01(u);

        int referenceWaves =
            Mathf.Max(
                1,
                historicalEnvelopeReferenceWaveCount);

        float waveCoordinate =
            u * referenceWaves;

        int waveIndex =
            Mathf.Clamp(
                Mathf.FloorToInt(waveCoordinate),
                0,
                referenceWaves);

        ceilingR =
            historicalPresentationCeilingR != null &&
            historicalPresentationCeilingR.length > 0
                ? Mathf.Max(
                    0f,
                    historicalPresentationCeilingR.Evaluate(waveIndex))
                : 4f;

        float elapsed =
            Mathf.Clamp(
                elapsedSeconds,
                0f,
                Mathf.Max(0.001f, totalDuration));

        float epsilon =
            Mathf.Clamp01(historicalUpperRestitution01) *
            Mathf.Clamp01(historicalUpperRestitution01);

        float timeRetention =
            Mathf.Exp(
                -Mathf.Max(
                    0f,
                    historicalDecayGammaPerSecond) *
                elapsed);

        float rawRetention =
            Mathf.Clamp01(
                epsilon *
                timeRetention);

        // The release endpoint historically began at the full first ceiling.
        // Afterwards preserve the mature old Envelope blend:
        // q = lerp(1, epsilon*exp(-gamma*t), waveTimeDecayInfluence).
        retention01 =
            u <= 0.0001f
                ? 1f
                : Mathf.Lerp(
                    1f,
                    rawRetention,
                    Mathf.Clamp01(
                        historicalWaveTimeDecayInfluence));

        spanR =
            Mathf.Max(
                0f,
                ceilingR * retention01);

        spanMeters =
            spanR *
            Mathf.Max(
                0.0001f,
                radius);
    }

    void StepIntegratedVirtualUpper(
        NearestKnotDetector.GuideFrame guide)
    {
        if (!virtualUpperEnabled || !equalizerBody)
        {
            ResetVirtualUpperLiveDiagnostics();
            return;
        }

        // AnalysisOnly must be observational: never disable the real physical Upper.
        // The legacy collider is disabled only when the explicit virtual-contact
        // replacement experiment is selected.
        if (legacyUpperColliderSafety &&
            virtualUpperRuntimeMode == VirtualUpperRuntimeMode.ApplyImpulseToEqualizer &&
            legacyUpperColliderSafety.enabled)
        {
            legacyUpperColliderSafety.enabled = false;
        }

        if (autoArmVirtualUpperOnSlopeEntry &&
            guide.isSlope &&
            !virtualUpperArmed)
        {
            long exactKey = MakeKey(guide.splineIndex, guide.sectionIndex);
            if (generated.ContainsKey(exactKey))
                ArmVirtualUpper(exactKey, "SlopeGuideExact");
            else if (lastPredictedKey != long.MinValue &&
                     generated.ContainsKey(lastPredictedKey))
                ArmVirtualUpper(lastPredictedKey, "SlopeGuideFallback");
        }

        if (!virtualUpperArmed ||
            equalizerBody.isKinematic ||
            (equalizerSync && equalizerSync.IsSynchronized))
        {
            ResetVirtualUpperLiveDiagnostics();
            return;
        }

        if (!TryEvaluateVirtualUpperFrame(
                virtualUpperActiveKey,
                equalizerBody.position,
                out VirtualUpperFrame frame))
        {
            ResetVirtualUpperLiveDiagnostics();
            return;
        }

        Vector3 normal = NormalizeSafe(frame.normalVisual, Vector3.up);
        float radius = ResolveEqualizerWorldRadius();
        float dt = Mathf.Max(Time.fixedDeltaTime, Eps);

        Vector3 upperVelocity = EstimateVirtualUpperCenterVelocity(
            frame.upperCenterVisual,
            dt);

        Vector3 relativeVelocity =
            equalizerBody.velocity - upperVelocity;

        // N is positive from carrier/lower toward Upper.
        float relativeNormalSpeed =
            Vector3.Dot(relativeVelocity, normal);

        // Upper center-locus is a BALL-CENTER contact locus, so radius is not
        // subtracted from the geometric gap here.
        float exactGap = Vector3.Dot(
            frame.upperCenterVisual - equalizerBody.position,
            normal);

        float activationHeight =
            Mathf.Max(0f, virtualUpperActivationShellR) * radius +
            Mathf.Max(0f, relativeNormalSpeed) *
            dt * Mathf.Max(0f, virtualUpperVelocityLeadSteps);

        float constraintCoordinate =
            exactGap - activationHeight;

        float penetration =
            Mathf.Max(0f, -constraintCoordinate);

        currentVirtualUpperU = frame.u;
        currentVirtualUpperGapMeters = exactGap;
        currentVirtualUpperPenetrationMeters = penetration;
        currentVirtualUpperRelativeNormalSpeed = relativeNormalSpeed;
        currentVirtualUpperSpanR = frame.spanR;
        currentVirtualUpperSpanMeters = frame.spanMeters;

        if (penetration <= Eps)
        {
            // Separation re-arms the event detector. A rejected overlap/contact is
            // allowed to become a new event only after it has actually separated.
            virtualUpperRejectedContactLatched = false;
            currentVirtualUpperForceNewton = 0f;
            currentVirtualUpperImpedance01 = 0f;
            virtualUpperNormalAccelerationState = Mathf.MoveTowards(
                virtualUpperNormalAccelerationState,
                0f,
                Mathf.Max(0f, maximumVirtualUpperJerk) * dt);

            TryEndVirtualUpperContact(
                exactGap,
                activationHeight,
                relativeNormalSpeed,
                radius);

            currentVirtualUpperDiagnostic = BuildVirtualUpperDiagnostic(
                frame,
                exactGap,
                penetration,
                relativeNormalSpeed,
                0f,
                0f);
            return;
        }

        if (!virtualUpperContactActive &&
            !virtualUpperRejectedContactLatched &&
            relativeNormalSpeed > 0f)
        {
            float impactEnergy = ComputeNormalImpactEnergy(relativeNormalSpeed);

            if (PassesRuntimeVirtualUpperImpactGate(
                    relativeNormalSpeed,
                    impactEnergy,
                    frame,
                    out string rejectReason))
            {
                BeginVirtualUpperImpact(relativeNormalSpeed, frame, impactEnergy);
            }
            else
            {
                RejectVirtualUpperImpactCandidate(rejectReason, frame, relativeNormalSpeed, impactEnergy);
            }
        }

        float impulseMagnitude = SolveVirtualUpperImpulse(
            constraintCoordinate,
            -relativeNormalSpeed,
            Mathf.Max(Eps, equalizerBody.mass),
            dt,
            radius,
            out float impedance01,
            out int substeps);

        float maxImpulse =
            Mathf.Max(Eps, equalizerBody.mass) *
            Mathf.Max(0f, maximumVirtualUpperAcceleration) *
            dt;

        impulseMagnitude = Mathf.Clamp(
            impulseMagnitude,
            0f,
            maxImpulse);

        if (enforceRestitutionEnergyCap &&
            virtualUpperContactActive &&
            virtualUpperImpactIncomingSpeed > Eps)
        {
            float currentGapVelocity = -relativeNormalSpeed;
            float proposedGapVelocity =
                currentGapVelocity +
                impulseMagnitude / Mathf.Max(Eps, equalizerBody.mass);

            if (proposedGapVelocity > virtualUpperAllowedOutgoingSpeed)
            {
                impulseMagnitude = Mathf.Max(
                    0f,
                    (virtualUpperAllowedOutgoingSpeed - currentGapVelocity) *
                    equalizerBody.mass);
            }
        }

        currentVirtualUpperImpedance01 = impedance01;
        currentVirtualUpperForceNewton = impulseMagnitude / dt;

        currentVirtualUpperDiagnostic = BuildVirtualUpperDiagnostic(
            frame,
            exactGap,
            penetration,
            relativeNormalSpeed,
            impedance01,
            impulseMagnitude);

        if (virtualUpperRuntimeMode ==
                VirtualUpperRuntimeMode.ApplyImpulseToEqualizer &&
            impulseMagnitude > Eps)
        {
            // Upper is unilateral: it may only push Equalizer away from Upper.
            equalizerBody.AddForce(
                -normal * impulseMagnitude,
                ForceMode.Impulse);
        }

        TryEndVirtualUpperContact(
            exactGap,
            activationHeight,
            relativeNormalSpeed,
            radius);
    }

    VirtualUpperDiagnostic BuildVirtualUpperDiagnostic(
        VirtualUpperFrame frame,
        float gap,
        float penetration,
        float relativeNormalSpeed,
        float impedance01,
        float impulseMagnitude)
    {
        return new VirtualUpperDiagnostic
        {
            valid = frame.valid,
            u = frame.u,
            carrierCenterVisual = frame.carrierCenterVisual,
            upperCenterVisual = frame.upperCenterVisual,
            tangentVisual = frame.tangentVisual,
            normalVisual = frame.normalVisual,
            sideVisual = frame.sideVisual,
            ceilingR = frame.ceilingR,
            retention01 = frame.retention01,
            spanR = frame.spanR,
            spanMeters = frame.spanMeters,
            signedGapMeters = gap,
            penetrationMeters = penetration,
            relativeNormalSpeed = relativeNormalSpeed,
            impedance01 = impedance01,
            predictedImpulseNs = impulseMagnitude
        };
    }

    bool TryEvaluateVirtualUpperFrame(
        long key,
        Vector3 probePositionVisual,
        out VirtualUpperFrame frame)
    {
        frame = default;

        if (key == long.MinValue ||
            !generated.TryGetValue(key, out GeneratedStairSpline target) ||
            target == null ||
            target.virtualUpperSamples == null ||
            target.virtualUpperSamples.Count < 2 ||
            !physicsSplineRoot)
        {
            return false;
        }

        int bestSegment = -1;
        float bestT = 0f;
        float bestDistanceSqr = float.PositiveInfinity;

        for (int i = 0; i < target.virtualUpperSamples.Count - 1; i++)
        {
            VirtualUpperKnotSample a = target.virtualUpperSamples[i];
            VirtualUpperKnotSample b = target.virtualUpperSamples[i + 1];

            Vector3 pa = physicsSplineRoot.TransformPoint(a.carrierCenterLocal);
            Vector3 pb = physicsSplineRoot.TransformPoint(b.carrierCenterLocal);
            Vector3 ab = pb - pa;

            float t = ab.sqrMagnitude > Eps
                ? Mathf.Clamp01(
                    Vector3.Dot(probePositionVisual - pa, ab) /
                    ab.sqrMagnitude)
                : 0f;

            Vector3 closest = Vector3.Lerp(pa, pb, t);
            float distanceSqr =
                (probePositionVisual - closest).sqrMagnitude;

            if (distanceSqr < bestDistanceSqr)
            {
                bestDistanceSqr = distanceSqr;
                bestSegment = i;
                bestT = t;
            }
        }

        if (bestSegment < 0)
            return false;

        VirtualUpperKnotSample s0 =
            target.virtualUpperSamples[bestSegment];
        VirtualUpperKnotSample s1 =
            target.virtualUpperSamples[bestSegment + 1];

        Vector3 carrier0 =
            physicsSplineRoot.TransformPoint(s0.carrierCenterLocal);
        Vector3 carrier1 =
            physicsSplineRoot.TransformPoint(s1.carrierCenterLocal);
        Vector3 upper0 =
            physicsSplineRoot.TransformPoint(s0.upperCenterLocal);
        Vector3 upper1 =
            physicsSplineRoot.TransformPoint(s1.upperCenterLocal);

        Vector3 tangent = NormalizeSafe(
            Vector3.Lerp(
                physicsSplineRoot.TransformDirection(s0.tangentLocal),
                physicsSplineRoot.TransformDirection(s1.tangentLocal),
                bestT),
            carrier1 - carrier0);

        Vector3 normal = NormalizeSafe(
            Vector3.Lerp(
                physicsSplineRoot.TransformDirection(s0.normalLocal),
                physicsSplineRoot.TransformDirection(s1.normalLocal),
                bestT),
            Vector3.up);

        normal = Vector3.ProjectOnPlane(normal, tangent);
        normal = NormalizeSafe(normal, Vector3.up);

        Vector3 side = NormalizeSafe(
            Vector3.Cross(normal, tangent),
            Vector3.right);
        normal = NormalizeSafe(
            Vector3.Cross(tangent, side),
            normal);

        frame = new VirtualUpperFrame
        {
            valid = true,
            key = key,
            u = Mathf.Lerp(s0.u, s1.u, bestT),
            carrierCenterVisual = Vector3.Lerp(carrier0, carrier1, bestT),
            upperCenterVisual = Vector3.Lerp(upper0, upper1, bestT),
            tangentVisual = tangent,
            normalVisual = normal,
            sideVisual = side,
            ceilingR = Mathf.Lerp(s0.ceilingR, s1.ceilingR, bestT),
            retention01 = Mathf.Lerp(s0.retention01, s1.retention01, bestT),
            spanR = Mathf.Lerp(s0.spanR, s1.spanR, bestT),
            spanMeters = Mathf.Lerp(s0.spanMeters, s1.spanMeters, bestT)
        };

        return IsFinite(frame.carrierCenterVisual) &&
               IsFinite(frame.upperCenterVisual) &&
               IsFinite(frame.normalVisual);
    }

    void ArmVirtualUpper(long key, string reason)
    {
        if (!virtualUpperEnabled ||
            key == long.MinValue ||
            !generated.ContainsKey(key))
        {
            return;
        }

        virtualUpperActiveKey = key;
        virtualUpperArmed = true;
        virtualUpperContactActive = false;
        previousUpperCenterValid = false;
        currentUpperCenterVelocity = Vector3.zero;
        virtualUpperNormalAccelerationState = 0f;
        virtualUpperImpactIncomingSpeed = 0f;
        virtualUpperAllowedOutgoingSpeed = 0f;
        virtualUpperArmedTime = Time.fixedTime;
        virtualUpperRejectedContactLatched = false;
        ResetVirtualUpperLiveDiagnostics();

        if (enableVirtualUpperImpactLogs)
        {
            Debug.Log(
                $"[FUTURE SPLINE VIRTUAL UPPER ARMED] " +
                $"mode={virtualUpperRuntimeMode} key={key} reason={reason}",
                this);
        }
    }

    void DisarmVirtualUpper(string reason)
    {
        if (!virtualUpperArmed && virtualUpperActiveKey == long.MinValue)
            return;

        virtualUpperArmed = false;
        virtualUpperContactActive = false;
        virtualUpperActiveKey = long.MinValue;
        previousUpperCenterValid = false;
        currentUpperCenterVelocity = Vector3.zero;
        virtualUpperNormalAccelerationState = 0f;
        virtualUpperArmedTime = -1f;
        virtualUpperRejectedContactLatched = false;
        ResetVirtualUpperLiveDiagnostics();

        if (enableVirtualUpperImpactLogs)
        {
            Debug.Log(
                $"[FUTURE SPLINE VIRTUAL UPPER DISARMED] reason={reason}",
                this);
        }
    }

    bool PassesRuntimeVirtualUpperImpactGate(
        float incomingNormalSpeed,
        float impactEnergyJ,
        VirtualUpperFrame frame,
        out string rejectReason)
    {
        rejectReason = string.Empty;

        if (equalizerSync && equalizerSync.IsEmergencyVisualRecoveryActive)
        {
            rejectReason = "EmergencyRecovery";
            return false;
        }

        bool insideArmGrace =
            virtualUpperArmedTime >= 0f &&
            Time.fixedTime - virtualUpperArmedTime <=
                Mathf.Max(0f, upperContactRuntimeArmGraceSeconds);

        if (insideArmGrace &&
            frame.u <= Mathf.Clamp01(upperContactStartNeighborhoodU))
        {
            rejectReason = "StartupOverlap";
            return false;
        }

        return PassesUpperImpactEnergyGate(
            incomingNormalSpeed,
            impactEnergyJ,
            out rejectReason);
    }

    void RejectVirtualUpperImpactCandidate(
        string reason,
        VirtualUpperFrame frame,
        float incomingNormalSpeed,
        float impactEnergyJ)
    {
        virtualUpperRejectedContactLatched = true;
        rejectedVirtualUpperImpactCount++;
        lastVirtualUpperRejectedImpactReason =
            string.IsNullOrEmpty(reason) ? "QualityGate" : reason;

        if (enableVirtualUpperImpactLogs)
        {
            Debug.Log(
                $"[FUTURE SPLINE VIRTUAL UPPER REJECTED] " +
                $"reason={lastVirtualUpperRejectedImpactReason} " +
                $"u={frame.u:F3} incomingVN={Mathf.Max(0f, incomingNormalSpeed):F4}m/s " +
                $"Ein={Mathf.Max(0f, impactEnergyJ):F4}J",
                this);
        }
    }

    void BeginVirtualUpperImpact(
        float incomingNormalSpeed,
        VirtualUpperFrame frame,
        float impactEnergyJ)
    {
        virtualUpperContactActive = true;
        lastVirtualUpperRejectedImpactReason = string.Empty;
        virtualUpperImpactCount++;

        virtualUpperImpactIncomingSpeed =
            Mathf.Max(0f, incomingNormalSpeed);

        virtualUpperAllowedOutgoingSpeed =
            Mathf.Clamp01(historicalUpperRestitution01) *
            virtualUpperImpactIncomingSpeed;

        if (virtualUpperLastImpactTime >= 0f)
        {
            observedVirtualUpperPeriodSeconds =
                Mathf.Max(0f, Time.fixedTime - virtualUpperLastImpactTime);
        }
        else
        {
            observedVirtualUpperPeriodSeconds =
                Mathf.Max(0f, historicalPreferredUpperPeriodSeconds);
        }

        virtualUpperLastImpactTime = Time.fixedTime;
        lastVirtualUpperIncomingSpeed = virtualUpperImpactIncomingSpeed;

        if (enableVirtualUpperImpactLogs)
        {
            Debug.Log(
                $"[FUTURE SPLINE VIRTUAL UPPER IMPACT] " +
                $"mode={virtualUpperRuntimeMode} " +
                $"index={virtualUpperImpactCount} u={frame.u:F3} " +
                $"incomingVN={virtualUpperImpactIncomingSpeed:F4}m/s " +
                $"Ein={Mathf.Max(0f, impactEnergyJ):F4}J span={frame.spanR:F3}R " +
                $"period={observedVirtualUpperPeriodSeconds:F4}s",
                this);
        }
    }

    void TryEndVirtualUpperContact(
        float exactGap,
        float activationHeight,
        float relativeNormalSpeed,
        float radius)
    {
        if (!virtualUpperContactActive)
            return;

        float releaseGap =
            activationHeight +
            Mathf.Max(0f, virtualUpperReleaseGapR) * radius;

        if (exactGap < releaseGap || relativeNormalSpeed > 0f)
            return;

        virtualUpperContactActive = false;
        lastVirtualUpperOutgoingSpeed =
            Mathf.Max(0f, -relativeNormalSpeed);

        virtualUpperNormalAccelerationState = 0f;

        lastVirtualUpperEnergyRetention =
            virtualUpperImpactIncomingSpeed > Eps
                ? Mathf.Clamp01(
                    (lastVirtualUpperOutgoingSpeed *
                     lastVirtualUpperOutgoingSpeed) /
                    (virtualUpperImpactIncomingSpeed *
                     virtualUpperImpactIncomingSpeed))
                : 0f;

        if (enableVirtualUpperImpactLogs)
        {
            Debug.Log(
                $"[FUTURE SPLINE VIRTUAL UPPER RELEASE] " +
                $"outgoingVN={lastVirtualUpperOutgoingSpeed:F4}m/s " +
                $"energyRetention={lastVirtualUpperEnergyRetention:F4} " +
                $"teacherRetention={(historicalUpperRestitution01 * historicalUpperRestitution01):F4} " +
                $"gap={exactGap:F5}m",
                this);
        }
    }

    float SolveVirtualUpperImpulse(
        float initialConstraintCoordinate,
        float initialGapVelocity,
        float mass,
        float dt,
        float radius,
        out float impedance01,
        out int substeps)
    {
        impedance01 = 0f;
        substeps = 1;

        if (mass <= Eps || dt <= Eps || radius <= Eps)
            return 0f;

        float chi =
            Mathf.Abs(initialGapVelocity) * dt / radius;

        float chiMax =
            Mathf.Max(0.01f, adaptiveSubstepChiMax);

        substeps = Mathf.Clamp(
            Mathf.CeilToInt(Mathf.Max(1f, chi / chiMax)),
            1,
            Mathf.Max(1, maxInternalSubsteps));

        float subDt = dt / substeps;
        float x = initialConstraintCoordinate;
        float v = initialGapVelocity;
        float accumulatedImpulse = 0f;
        float accelerationState = virtualUpperNormalAccelerationState;

        float dampingRatio = ResolveVirtualUpperDampingRatio();
        float huntDissipation = ResolveHuntCrossleyDissipation();

        for (int i = 0; i < substeps; i++)
        {
            float penetration = Mathf.Max(0f, -x);
            float impedance = ComputeVirtualUpperImpedance01(
                penetration,
                radius);

            impedance01 = Mathf.Max(impedance01, impedance);

            if (penetration <= Eps || impedance <= Eps)
            {
                x += v * subDt;
                continue;
            }

            float implicitForce = VirtualUpperSoftConstraintForce(
                x,
                v,
                mass,
                subDt,
                virtualUpperContactFrequencyHz,
                dampingRatio);

            float huntForce = VirtualUpperHuntCrossleyForce(
                penetration,
                Mathf.Max(0f, -v),
                mass,
                radius,
                virtualUpperContactFrequencyHz,
                huntCrossleyExponent,
                huntDissipation,
                huntReferenceCompressionR);

            float rawForce;
            switch (virtualUpperContactModel)
            {
                case VirtualUpperContactModel.ImplicitSoftConstraint:
                    rawForce = implicitForce;
                    break;

                case VirtualUpperContactModel.HuntCrossley:
                    rawForce = huntForce;
                    break;

                default:
                    rawForce = Mathf.Lerp(
                        huntForce,
                        implicitForce,
                        Mathf.Clamp01(hybridImplicitShare));
                    break;
            }

            rawForce *= impedance;

            float requestedAcceleration = Mathf.Clamp(
                rawForce / mass,
                0f,
                Mathf.Max(0f, maximumVirtualUpperAcceleration));

            accelerationState = ApplyVirtualUpperJerkLimit(
                accelerationState,
                requestedAcceleration,
                maximumVirtualUpperJerk,
                subDt);

            float vRequested =
                v + accelerationState * subDt;

            if (enforceRestitutionEnergyCap &&
                virtualUpperContactActive &&
                virtualUpperImpactIncomingSpeed > Eps)
            {
                vRequested = Mathf.Min(
                    vRequested,
                    Mathf.Max(0f, virtualUpperAllowedOutgoingSpeed));
            }

            float deltaV = Mathf.Max(0f, vRequested - v);
            accumulatedImpulse += mass * deltaV;
            v += deltaV;
            x += v * subDt;
        }

        virtualUpperNormalAccelerationState =
            Mathf.Max(0f, accelerationState);

        return Mathf.Max(0f, accumulatedImpulse);
    }

    float ResolveVirtualUpperDampingRatio()
    {
        if (!deriveVirtualUpperDampingFromRestitution)
            return Mathf.Max(0f, explicitVirtualUpperDampingRatio);

        float e = Mathf.Clamp(
            historicalUpperRestitution01,
            0.001f,
            0.9999f);

        float lnE = Mathf.Log(e);

        return Mathf.Max(
            0f,
            -lnE /
            Mathf.Sqrt(Mathf.PI * Mathf.PI + lnE * lnE));
    }

    float ResolveHuntCrossleyDissipation()
    {
        float incoming =
            Mathf.Max(0.10f, virtualUpperImpactIncomingSpeed);

        float e = Mathf.Clamp01(historicalUpperRestitution01);

        // Low-speed Hunt-Crossley relation: e ~= 1 - c*v.
        return Mathf.Max(0f, (1f - e) / incoming);
    }

    static float VirtualUpperSoftConstraintForce(
        float x,
        float gapVelocity,
        float mass,
        float dt,
        float frequencyHz,
        float dampingRatio)
    {
        if (x >= 0f || mass <= Eps || dt <= Eps)
            return 0f;

        float omega =
            2f * Mathf.PI * Mathf.Max(0.01f, frequencyHz);

        float k = mass * omega * omega;
        float c =
            2f * Mathf.Max(0f, dampingRatio) * mass * omega;

        float denominator =
            mass / dt + k * dt + c;

        if (denominator <= Eps)
            return 0f;

        float vNext =
            (mass * gapVelocity / dt - k * x) /
            denominator;

        float impulse =
            mass * (vNext - gapVelocity);

        return Mathf.Max(0f, impulse / dt);
    }

    static float VirtualUpperHuntCrossleyForce(
        float penetration,
        float compressionSpeed,
        float mass,
        float radius,
        float frequencyHz,
        float exponent,
        float dissipation,
        float referenceCompressionR)
    {
        if (penetration <= 0f || mass <= Eps)
            return 0f;

        float p = Mathf.Clamp(exponent, 1f, 2.5f);
        float omega =
            2f * Mathf.PI * Mathf.Max(0.01f, frequencyHz);
        float linearK = mass * omega * omega;

        float referenceCompression = Mathf.Max(
            0.0001f,
            radius * Mathf.Max(0.01f, referenceCompressionR));

        float nonlinearK =
            linearK /
            Mathf.Pow(referenceCompression, p - 1f);

        float elastic =
            nonlinearK * Mathf.Pow(penetration, p);

        float dampingFactor = Mathf.Max(
            0f,
            1f +
            1.5f * Mathf.Max(0f, dissipation) * compressionSpeed);

        return Mathf.Max(0f, elastic * dampingFactor);
    }

    float ComputeVirtualUpperImpedance01(
        float penetrationMeters,
        float radius)
    {
        float start =
            Mathf.Max(0f, impedanceStartR) * radius;

        float full = Mathf.Max(
            start + 0.000001f,
            Mathf.Max(0f, impedanceFullR) * radius);

        float u = Mathf.InverseLerp(
            start,
            full,
            Mathf.Max(0f, penetrationMeters));

        return SmootherStep01(u);
    }

    static float ApplyVirtualUpperJerkLimit(
        float previousAcceleration,
        float requestedAcceleration,
        float maximumJerk,
        float dt)
    {
        float maxDelta =
            Mathf.Max(0f, maximumJerk) *
            Mathf.Max(0f, dt);

        return previousAcceleration + Mathf.Clamp(
            requestedAcceleration - previousAcceleration,
            -maxDelta,
            maxDelta);
    }

    Vector3 EstimateVirtualUpperCenterVelocity(
        Vector3 upperCenter,
        float dt)
    {
        Vector3 subjectVelocity =
            correspondSubject
                ? correspondSubject.MappedVelocity
                : Vector3.zero;

        if (!previousUpperCenterValid)
        {
            previousUpperCenterValid = true;
            previousUpperCenter = upperCenter;
            currentUpperCenterVelocity = subjectVelocity;
            return currentUpperCenterVelocity;
        }

        Vector3 measured =
            (upperCenter - previousUpperCenter) /
            Mathf.Max(Eps, dt);

        previousUpperCenter = upperCenter;

        if (!IsFinite(measured) ||
            measured.magnitude >
            Mathf.Max(1f, maximumUpperGeometryTrackingSpeed))
        {
            currentUpperCenterVelocity = subjectVelocity;
            return currentUpperCenterVelocity;
        }

        currentUpperCenterVelocity = Vector3.Lerp(
            subjectVelocity,
            measured,
            Mathf.Clamp01(measuredUpperGeometryVelocityBlend01));

        return currentUpperCenterVelocity;
    }

    float ResolveEqualizerWorldRadius()
    {
        if (!equalizerCollider)
            return 0.5f;

        Vector3 scale = equalizerCollider.transform.lossyScale;
        float maxScale = Mathf.Max(
            Mathf.Abs(scale.x),
            Mathf.Abs(scale.y),
            Mathf.Abs(scale.z));

        return Mathf.Max(
            0.0001f,
            equalizerCollider.radius * maxScale);
    }

    void ResetVirtualUpperLiveDiagnostics()
    {
        currentVirtualUpperGapMeters = 0f;
        currentVirtualUpperPenetrationMeters = 0f;
        currentVirtualUpperRelativeNormalSpeed = 0f;
        currentVirtualUpperForceNewton = 0f;
        currentVirtualUpperImpedance01 = 0f;
        currentVirtualUpperSpanR = 0f;
        currentVirtualUpperSpanMeters = 0f;
        currentVirtualUpperDiagnostic = default;
    }

    static float SmootherStep01(float t)
    {
        t = Mathf.Clamp01(t);
        return t * t * t *
               (t * (t * 6f - 15f) + 10f);
    }

    void OnDrawGizmosSelected()
    {
        if (!drawVirtualUpperGizmos ||
            !Application.isPlaying ||
            !physicsSplineRoot)
        {
            return;
        }

        foreach (KeyValuePair<long, GeneratedStairSpline> pair in generated)
        {
            GeneratedStairSpline stair = pair.Value;
            if (stair == null || stair.virtualUpperSamples.Count == 0)
                continue;

            Vector3 previous = Vector3.zero;
            bool hasPrevious = false;

            for (int i = 0; i < stair.virtualUpperSamples.Count; i++)
            {
                Vector3 point = physicsSplineRoot.TransformPoint(
                    stair.virtualUpperSamples[i].upperCenterLocal);

                Gizmos.DrawSphere(
                    point,
                    Mathf.Max(0.005f, virtualUpperGizmoRadius));

                if (hasPrevious)
                    Gizmos.DrawLine(previous, point);

                previous = point;
                hasPrevious = true;
            }
        }
    }

    GeneratedStairSpline GetOrCreateGeneratedSpline(
        long key,
        int sourceSplineIndex,
        int sourceSectionIndex)
    {
        if (generated.TryGetValue(key, out GeneratedStairSpline existing) &&
            existing != null &&
            existing.container)
        {
            return existing;
        }

        GameObject go = new GameObject(
            $"BallVisualEqualizerFutureSpline_S{sourceSplineIndex:00}_Sec{sourceSectionIndex:00}");

        go.transform.SetParent(physicsSplineRoot, false);
        go.transform.localPosition = Vector3.zero;
        go.transform.localRotation = Quaternion.identity;
        go.transform.localScale = Vector3.one;

        SplineContainer container = go.AddComponent<SplineContainer>();
        container.Spline = new Spline();

        SplineExtrude extrude = null;

        if (createSplineExtrude)
        {
            extrude = go.AddComponent<SplineExtrude>();
            extrude.Container = container;
            extrude.Radius = Mathf.Max(0.002f, splineRadius);
            extrude.Sides = Mathf.Max(3, splineSides);
            extrude.SegmentsPerUnit = Mathf.Max(1f, splineSegmentsPerUnit);
            extrude.Capped = true;
            extrude.RebuildOnSplineChange = true;
            extrude.RebuildFrequency = 30;

            MeshRenderer renderer = go.GetComponent<MeshRenderer>();
            if (renderer && splineMaterial)
                renderer.sharedMaterial = splineMaterial;
        }

        // No Collider and no Rigidbody are ever added to this generated object.

        GeneratedStairSpline created = new GeneratedStairSpline
        {
            key = key,
            sourceSplineIndex = sourceSplineIndex,
            sourceSectionIndex = sourceSectionIndex,
            gameObject = go,
            container = container,
            extrude = extrude,
            frozen = false,
            creationLogged = false
        };

        generated[key] = created;
        return created;
    }

    void RefreshExtrude(GeneratedStairSpline target)
    {
        if (!target.extrude)
            return;

        target.extrude.Radius = Mathf.Max(0.002f, splineRadius);
        target.extrude.Sides = Mathf.Max(3, splineSides);
        target.extrude.SegmentsPerUnit = Mathf.Max(1f, splineSegmentsPerUnit);

        MeshRenderer renderer = target.gameObject.GetComponent<MeshRenderer>();
        if (renderer && splineMaterial)
            renderer.sharedMaterial = splineMaterial;

        target.extrude.Rebuild();
    }

    void UpdateLiveEqualizerKinematics(
        NearestKnotDetector.GuideFrame guide)
    {
        if (!equalizerBody || !guide.valid || !IsFinite(equalizerBody.velocity))
        {
            liveKinematicsValid = false;
            return;
        }

        Vector3 tangent = NormalizeSafe(
            correspondSubject.MapDirection(guide.tangent),
            Vector3.forward);
        Vector3 normal = BuildInitialNormal(
            tangent,
            correspondSubject.MapDirection(guide.normal));
        Vector3 side = NormalizeSafe(
            Vector3.Cross(normal, tangent),
            Vector3.right);

        Vector3 velocity = equalizerBody.velocity;
        float currentVT = Vector3.Dot(velocity, tangent);
        float currentVN = Vector3.Dot(velocity, normal);
        float currentVB = Vector3.Dot(velocity, side);

        float now = Time.fixedTime;
        if (liveKinematicsValid)
        {
            float dt = Mathf.Max(Time.fixedDeltaTime, now - previousLiveKinematicsTime);

            // Project world acceleration, not d(v dot T)/dt. This avoids treating
            // rotation of the moving T/N/B frame itself as a physical tangential force.
            Vector3 worldAcceleration =
                (velocity - previousLiveVelocity) / Mathf.Max(Eps, dt);
            float rawAT = Vector3.Dot(worldAcceleration, tangent);
            rawAT = Mathf.Clamp(
                rawAT,
                -Mathf.Max(0f, maximumPredictedTangentialAcceleration),
                Mathf.Max(0f, maximumPredictedTangentialAcceleration));

            float rawAN = Vector3.Dot(worldAcceleration, normal);
            rawAN = Mathf.Clamp(
                rawAN,
                -Mathf.Max(0f, maximumPredictedNormalAcceleration),
                Mathf.Max(0f, maximumPredictedNormalAcceleration));

            liveEqualizerTangentialAcceleration = Mathf.Lerp(
                liveEqualizerTangentialAcceleration,
                rawAT,
                Mathf.Clamp01(tangentialAccelerationFilter01));

            liveEqualizerNormalAcceleration = Mathf.Lerp(
                liveEqualizerNormalAcceleration,
                rawAN,
                Mathf.Clamp01(normalAccelerationFilter01));
        }
        else
        {
            liveEqualizerTangentialAcceleration = 0f;
            liveEqualizerNormalAcceleration = 0f;
            liveKinematicsValid = true;
        }

        float gravityAT = Vector3.Dot(Physics.gravity, tangent);
        float gravityAN = Vector3.Dot(Physics.gravity, normal);

        liveEqualizerResidualTangentialAcceleration =
            liveEqualizerTangentialAcceleration - gravityAT;
        liveEqualizerResidualNormalAcceleration =
            liveEqualizerNormalAcceleration - gravityAN;

        liveEqualizerTangentialSpeed = currentVT;
        liveEqualizerNormalSpeed = currentVN;
        liveEqualizerSideSpeed = currentVB;
        previousLiveVelocity = velocity;
        previousLiveKinematicsTime = now;
    }

    void UpdateMechanicalNormalStateValidity(
        NearestKnotDetector.GuideFrame guide)
    {
        if (!equalizerBody || !guide.valid)
        {
            InvalidateMechanicalNormalState(
                "MissingBodyOrGuide",
                clearPreviousPosition: false);
            return;
        }

        Vector3 position = equalizerBody.position;
        Vector3 velocity = equalizerBody.velocity;

        if (!IsFinite(position) || !IsFinite(velocity))
        {
            InvalidateMechanicalNormalState(
                "NonFiniteState",
                clearPreviousPosition: true);
            return;
        }

        mechanicalNormalLastStepMeters = 0f;
        if (mechanicalNormalPreviousPositionValid)
        {
            mechanicalNormalLastStepMeters =
                Vector3.Distance(
                    position,
                    previousMechanicalNormalPosition);

            if (mechanicalNormalLastStepMeters >
                Mathf.Max(0.25f, mechanicalNormalMaximumStepMeters))
            {
                previousMechanicalNormalPosition = position;
                InvalidateMechanicalNormalState(
                    "PositionJump",
                    clearPreviousPosition: false);
                return;
            }
        }

        previousMechanicalNormalPosition = position;
        mechanicalNormalPreviousPositionValid = true;

        if (equalizerSync &&
            equalizerSync.IsEmergencyVisualRecoveryActive)
        {
            InvalidateMechanicalNormalState(
                "EmergencyRecovery",
                clearPreviousPosition: false);
            return;
        }

        float absoluteSpeed = velocity.magnitude;
        if (absoluteSpeed >
            Mathf.Max(1f, mechanicalNormalMaximumAbsoluteSpeed))
        {
            InvalidateMechanicalNormalState(
                "AbsoluteSpeedTooHigh",
                clearPreviousPosition: false);
            return;
        }

        if (Mathf.Abs(liveEqualizerNormalSpeed) >
            Mathf.Max(1f, mechanicalNormalMaximumAbsoluteNormalSpeed))
        {
            InvalidateMechanicalNormalState(
                "NormalSpeedTooHigh",
                clearPreviousPosition: false);
            return;
        }

        if (!IsFiniteScalar(liveEqualizerNormalAcceleration) ||
            !IsFiniteScalar(liveEqualizerResidualNormalAcceleration))
        {
            InvalidateMechanicalNormalState(
                "InvalidNormalAcceleration",
                clearPreviousPosition: false);
            return;
        }

        mechanicalNormalStableFrames++;

        float now = Time.fixedTime;
        bool stableCountReached =
            mechanicalNormalStableFrames >=
            Mathf.Max(2, mechanicalNormalRequiredStableFrames);
        bool quietTimeReached =
            now - mechanicalNormalLastInvalidTime >=
            Mathf.Max(0f, mechanicalNormalRearmDelaySeconds);

        mechanicalNormalStateValid =
            stableCountReached &&
            quietTimeReached;

        if (mechanicalNormalStateValid)
            mechanicalNormalInvalidReason = string.Empty;
    }

    void InvalidateMechanicalNormalState(
        string reason,
        bool clearPreviousPosition)
    {
        mechanicalNormalStateValid = false;
        mechanicalNormalStableFrames = 0;
        mechanicalNormalLastInvalidTime = Time.fixedTime;
        mechanicalNormalInvalidReason =
            string.IsNullOrEmpty(reason)
                ? "Invalid"
                : reason;

        // Do not let a reset/emergency acceleration survive the validity gate
        // and later re-enter the predictor after the quiet-time re-arm.
        liveEqualizerNormalAcceleration = 0f;
        liveEqualizerResidualNormalAcceleration = 0f;

        if (clearPreviousPosition)
            mechanicalNormalPreviousPositionValid = false;
    }

    float PredictTravelDistanceMechanical(
        NearestKnotDetector.GuideFrame guide,
        float timeSeconds,
        float initialSpeed,
        float residualTangentialAcceleration)
    {
        float horizon = Mathf.Clamp(
            timeSeconds,
            0f,
            Mathf.Max(0.25f, maximumPredictionTimeSeconds));
        if (horizon <= Eps)
            return 0f;

        int steps = Mathf.Clamp(mechanicalIntegrationSubsteps, 4, 64);
        float dt = horizon / steps;
        float sMeters = 0f;
        float v = Mathf.Max(0f, initialSpeed);
        float speedLimit = Mathf.Max(
            v,
            Mathf.Max(0.01f, slopeCore.MaxGroundSpeedReadOnly));
        float residualDecayLength = Mathf.Max(
            0.10f,
            residualTangentialAccelerationDecayMeters);

        for (int i = 0; i < steps; i++)
        {
            if (!TryResolveFutureRouteTangent(
                    guide,
                    sMeters,
                    out Vector3 tangent))
            {
                tangent = NormalizeSafe(
                    correspondSubject.MapDirection(guide.tangent),
                    Vector3.forward);
            }

            float gravityAT = Vector3.Dot(Physics.gravity, tangent);
            float residualAT =
                residualTangentialAcceleration *
                Mathf.Exp(-sMeters / residualDecayLength);
            float a = Mathf.Clamp(
                gravityAT + residualAT,
                -Mathf.Max(0f, maximumPredictedTangentialAcceleration),
                Mathf.Max(0f, maximumPredictedTangentialAcceleration));

            // Midpoint-like velocity update keeps the short-horizon integral stable
            // while remaining transparent and mechanically interpretable.
            float vNext = Mathf.Clamp(v + a * dt, 0f, speedLimit);
            sMeters += 0.5f * (v + vNext) * dt;
            v = vNext;
        }

        return Mathf.Max(0f, sMeters);
    }

    bool TryResolveFutureRouteTangent(
        NearestKnotDetector.GuideFrame guide,
        float travelDistanceMeters,
        out Vector3 tangentVisual)
    {
        tangentVisual = Vector3.forward;
        if (!guide.valid)
            return false;

        float travel = Mathf.Max(0f, travelDistanceMeters);

        if (guide.isSlope)
        {
            float sectionLength = Mathf.Max(Eps, guide.sectionLength);
            float absoluteDistance =
                Mathf.Clamp01(guide.sectionProgress01) * sectionLength + travel;

            if (absoluteDistance <= sectionLength + Eps &&
                knotDetector.TryEvaluateSameSection(
                    guide,
                    Mathf.Clamp01(absoluteDistance / sectionLength),
                    out NearestKnotDetector.GuideSample sample))
            {
                tangentVisual = NormalizeSafe(
                    correspondSubject.MapDirection(sample.tangent),
                    Vector3.forward);
                return true;
            }

            if (knotDetector.TryEvaluateSameSection(
                    guide,
                    1f,
                    out NearestKnotDetector.GuideSample exitSample))
            {
                Vector3 exitTangent = NormalizeSafe(
                    correspondSubject.MapDirection(exitSample.tangent),
                    Vector3.forward);
                tangentVisual = NormalizeSafe(
                    Vector3.ProjectOnPlane(exitTangent, Vector3.up),
                    exitTangent);
                return true;
            }
            return false;
        }

        if (guide.nextIsSlope)
        {
            float distanceToEntry = Mathf.Max(0f, guide.distanceToNextSlope);
            if (travel <= distanceToEntry + Eps)
            {
                tangentVisual = NormalizeSafe(
                    correspondSubject.MapDirection(guide.tangent),
                    Vector3.forward);
                return true;
            }

            if (!knotDetector.TryEvaluateForwardSlopeSection(
                    guide,
                    0f,
                    out NearestKnotDetector.GuideSample entrySample))
            {
                return false;
            }

            float sectionLength = Mathf.Max(Eps, entrySample.sectionLength);
            float inside = travel - distanceToEntry;
            if (inside <= sectionLength + Eps &&
                knotDetector.TryEvaluateForwardSlopeSection(
                    guide,
                    Mathf.Clamp01(inside / sectionLength),
                    out NearestKnotDetector.GuideSample slopeSample))
            {
                tangentVisual = NormalizeSafe(
                    correspondSubject.MapDirection(slopeSample.tangent),
                    Vector3.forward);
                return true;
            }

            if (knotDetector.TryEvaluateForwardSlopeSection(
                    guide,
                    1f,
                    out NearestKnotDetector.GuideSample exitSample))
            {
                Vector3 exitTangent = NormalizeSafe(
                    correspondSubject.MapDirection(exitSample.tangent),
                    Vector3.forward);
                tangentVisual = NormalizeSafe(
                    Vector3.ProjectOnPlane(exitTangent, Vector3.up),
                    exitTangent);
                return true;
            }
        }

        tangentVisual = NormalizeSafe(
            correspondSubject.MapDirection(guide.tangent),
            Vector3.forward);
        return true;
    }

    float PredictTimeForDistanceMechanical(
        NearestKnotDetector.GuideFrame guide,
        float distanceMeters,
        float initialSpeed,
        float residualTangentialAcceleration)
    {
        float target = Mathf.Max(0f, distanceMeters);
        if (target <= Eps)
            return 0f;

        float hi = Mathf.Max(0.25f, maximumPredictionTimeSeconds);
        if (PredictTravelDistanceMechanical(
                guide,
                hi,
                initialSpeed,
                residualTangentialAcceleration) < target)
        {
            return hi;
        }

        float lo = 0f;
        for (int i = 0; i < 22; i++)
        {
            float mid = 0.5f * (lo + hi);
            if (PredictTravelDistanceMechanical(
                    guide,
                    mid,
                    initialSpeed,
                    residualTangentialAcceleration) < target)
            {
                lo = mid;
            }
            else
            {
                hi = mid;
            }
        }

        return 0.5f * (lo + hi);
    }



    void FinalizeMechanicalNormalPlan(
        ref CurvePlan plan,
        bool allowLiveResidualNormalAcceleration)
    {
        plan.mechanicalNormalValid = false;
        plan.mechanicalStartsOnActiveSlope =
            allowLiveResidualNormalAcceleration;
        plan.mechanicalInitialRelativeNormalSpeed = 0f;
        plan.mechanicalResidualNormalAcceleration0 = 0f;

        if (mechanicalNormalPredictionMode ==
            MechanicalNormalPredictionMode.ReferenceOnly)
        {
            lastMechanicalPlanValid = false;
            return;
        }

        if (!mechanicalNormalStateValid ||
            !IsFinite(plan.incidentVelocityPF) ||
            !IsFinite(plan.initialNormal) ||
            !IsFiniteScalar(plan.totalDuration) ||
            plan.totalDuration <= Eps)
        {
            lastMechanicalPlanValid = false;
            return;
        }

        float incidentVN =
            Vector3.Dot(
                plan.incidentVelocityPF,
                NormalizeSafe(plan.initialNormal, Vector3.up));

        if (!TryEstimateMechanicalCarrierNormalVelocityAtStart(
                plan,
                out float carrierVN))
        {
            lastMechanicalPlanValid = false;
            return;
        }

        float relativeVN = incidentVN - carrierVN;
        if (!IsFiniteScalar(relativeVN) ||
            Mathf.Abs(relativeVN) >
                Mathf.Max(
                    1f,
                    mechanicalNormalMaximumAbsoluteNormalSpeed))
        {
            lastMechanicalPlanValid = false;
            return;
        }

        float residualAN =
            allowLiveResidualNormalAcceleration
                ? liveEqualizerResidualNormalAcceleration
                : 0f;

        if (!IsFiniteScalar(residualAN))
        {
            lastMechanicalPlanValid = false;
            return;
        }

        residualAN = Mathf.Clamp(
            residualAN,
            -Mathf.Max(1f, maximumPredictedNormalAcceleration),
            Mathf.Max(1f, maximumPredictedNormalAcceleration));

        plan.mechanicalInitialRelativeNormalSpeed = relativeVN;
        plan.mechanicalResidualNormalAcceleration0 = residualAN;
        plan.mechanicalNormalValid = true;

        lastMechanicalPlanValid = true;
        lastMechanicalInitialRelativeNormalSpeed = relativeVN;
        lastMechanicalResidualNormalAcceleration = residualAN;
    }

    float ResolvePredictionElapsedTimeAtU(
        CurvePlan plan,
        float u)
    {
        u = Mathf.Clamp01(u);

        float total =
            Mathf.Max(0.001f, plan.totalDuration);
        float uT =
            Mathf.Clamp(plan.uT, 0.001f, 0.998f);
        float uM =
            Mathf.Clamp(plan.uM, uT + 0.001f, 0.999f);

        float tT =
            Mathf.Clamp(plan.timeToPT, 0f, total);
        float tM =
            Mathf.Clamp(plan.timeToPM, tT, total);

        // Degenerate intent timestamps are possible near coincident anchors.
        // Fall back to the spatial parameter fraction instead of creating a
        // discontinuous u<->time mapping.
        if (tT <= Eps)
            tT = total * uT;

        if (tM <= tT + Eps)
            tM = Mathf.Max(tT, total * uM);

        tT = Mathf.Clamp(tT, 0f, total);
        tM = Mathf.Clamp(tM, tT, total);

        if (u <= uT)
        {
            return Mathf.Lerp(
                0f,
                tT,
                Mathf.InverseLerp(0f, uT, u));
        }

        if (u <= uM)
        {
            return Mathf.Lerp(
                tT,
                tM,
                Mathf.InverseLerp(uT, uM, u));
        }

        return Mathf.Lerp(
            tM,
            total,
            Mathf.InverseLerp(uM, 1f, u));
    }

    float ResolvePredictionUAtElapsedTime(
        CurvePlan plan,
        float elapsedSeconds)
    {
        float total =
            Mathf.Max(0.001f, plan.totalDuration);
        float t =
            Mathf.Clamp(elapsedSeconds, 0f, total);

        float uT =
            Mathf.Clamp(plan.uT, 0.001f, 0.998f);
        float uM =
            Mathf.Clamp(plan.uM, uT + 0.001f, 0.999f);
        float tT =
            Mathf.Clamp(plan.timeToPT, 0f, total);
        float tM =
            Mathf.Clamp(plan.timeToPM, tT, total);

        if (tT <= Eps)
            tT = total * uT;

        if (tM <= tT + Eps)
            tM = Mathf.Max(tT, total * uM);

        tT = Mathf.Clamp(tT, 0f, total);
        tM = Mathf.Clamp(tM, tT, total);

        if (t <= Eps)
            return 0f;

        if (t <= tT && tT > Eps)
        {
            return Mathf.Lerp(
                0f,
                uT,
                Mathf.InverseLerp(0f, tT, t));
        }

        if (t <= tM && tM - tT > Eps)
        {
            return Mathf.Lerp(
                uT,
                uM,
                Mathf.InverseLerp(tT, tM, t));
        }

        if (total - tM <= Eps)
            return 1f;

        return Mathf.Lerp(
            uM,
            1f,
            Mathf.InverseLerp(tM, total, t));
    }

    bool TryEvaluateMechanicalCarrierCenter(
        CurvePlan plan,
        float u,
        out Vector3 center,
        out Vector3 tangent,
        out Vector3 normal)
    {
        center = Vector3.zero;
        tangent = Vector3.forward;
        normal = Vector3.up;

        EvaluateBaseline(
            plan,
            Mathf.Clamp01(u),
            out Vector3 baseline,
            out Vector3 derivative);

        if (!IsFinite(baseline) || !IsFinite(derivative))
            return false;

        tangent = NormalizeSafe(
            derivative,
            plan.tangentPF);

        normal = BuildInitialNormal(
            tangent,
            plan.initialNormal);

        Vector3 side = NormalizeSafe(
            Vector3.Cross(normal, tangent),
            Vector3.right);

        normal = NormalizeSafe(
            Vector3.Cross(tangent, side),
            normal);

        float bridgeN =
            baselineOnlyDiagnostic
                ? 0f
                : EvaluateBoundaryResidual(
                    plan,
                    u,
                    plan.residualNormal0,
                    plan.residualNormalSlopePerMeter0);

        float bridgeB =
            baselineOnlyDiagnostic
                ? 0f
                : EvaluateBoundaryResidual(
                    plan,
                    u,
                    plan.residualSide0,
                    plan.residualSideSlopePerMeter0);

        center =
            baseline +
            normal * bridgeN +
            side * bridgeB;

        return
            IsFinite(center) &&
            IsFinite(tangent) &&
            IsFinite(normal);
    }

    bool TryEstimateMechanicalCarrierNormalVelocityAtStart(
        CurvePlan plan,
        out float carrierNormalVelocity)
    {
        carrierNormalVelocity = 0f;

        float total =
            Mathf.Max(0.001f, plan.totalDuration);
        float dt =
            Mathf.Clamp(
                Mathf.Min(0.02f, total * 0.10f),
                0.0025f,
                total);

        if (dt <= Eps)
            return false;

        float u1 =
            ResolvePredictionUAtElapsedTime(
                plan,
                dt);

        if (!TryEvaluateMechanicalCarrierCenter(
                plan,
                0f,
                out Vector3 p0,
                out _,
                out Vector3 n0) ||
            !TryEvaluateMechanicalCarrierCenter(
                plan,
                u1,
                out Vector3 p1,
                out _,
                out _))
        {
            return false;
        }

        carrierNormalVelocity =
            Vector3.Dot(
                (p1 - p0) / dt,
                n0);

        return IsFiniteScalar(carrierNormalVelocity);
    }

    float EstimateMechanicalCarrierNormalAcceleration(
        CurvePlan plan,
        float elapsedSeconds)
    {
        float total =
            Mathf.Max(0.001f, plan.totalDuration);

        float sampleDt =
            Mathf.Clamp(
                Mathf.Min(0.02f, total * 0.05f),
                0.0025f,
                0.02f);

        float t =
            Mathf.Clamp(
                elapsedSeconds,
                0f,
                total);

        if (t < sampleDt ||
            t > total - sampleDt)
        {
            return 0f;
        }

        float u0 =
            ResolvePredictionUAtElapsedTime(
                plan,
                t - sampleDt);
        float u1 =
            ResolvePredictionUAtElapsedTime(
                plan,
                t);
        float u2 =
            ResolvePredictionUAtElapsedTime(
                plan,
                t + sampleDt);

        if (!TryEvaluateMechanicalCarrierCenter(
                plan,
                u0,
                out Vector3 p0,
                out _,
                out _) ||
            !TryEvaluateMechanicalCarrierCenter(
                plan,
                u1,
                out Vector3 p1,
                out _,
                out Vector3 normal) ||
            !TryEvaluateMechanicalCarrierCenter(
                plan,
                u2,
                out Vector3 p2,
                out _,
                out _))
        {
            return 0f;
        }

        Vector3 acceleration =
            (p2 - 2f * p1 + p0) /
            Mathf.Max(
                Eps,
                sampleDt * sampleDt);

        float aN =
            Vector3.Dot(
                acceleration,
                normal);

        return Mathf.Clamp(
            aN,
            -Mathf.Max(
                1f,
                mechanicalNormalMaximumCarrierAcceleration),
            Mathf.Max(
                1f,
                mechanicalNormalMaximumCarrierAcceleration));
    }

    float QuinticSmoothStep01(float x)
    {
        x = Mathf.Clamp01(x);
        return
            x * x * x *
            (10f +
             x * (-15f + 6f * x));
    }

    float ResolveMechanicalOscillatorControlAuthority01(
        CurvePlan plan,
        float elapsedSeconds)
    {
        if (plan.mechanicalStartsOnActiveSlope)
            return 1f;

        float delay =
            Mathf.Max(
                0f,
                mechanicalNormalFutureEntryDelaySeconds);

        if (elapsedSeconds <= delay)
            return 0f;

        float blend =
            Mathf.Max(
                0.01f,
                mechanicalNormalFutureEntryBlendSeconds);

        return
            QuinticSmoothStep01(
                (elapsedSeconds - delay) /
                blend);
    }

    bool TryEvaluateMechanicalReferenceKinematics(
        CurvePlan plan,
        float elapsedSeconds,
        out float targetNormalMeters,
        out float targetNormalSpeed,
        out float targetNormalAcceleration)
    {
        targetNormalMeters = 0f;
        targetNormalSpeed = 0f;
        targetNormalAcceleration = 0f;

        float total =
            Mathf.Max(
                0.001f,
                plan.totalDuration);

        float t =
            Mathf.Clamp(
                elapsedSeconds,
                0f,
                total);

        float sampleDt =
            Mathf.Clamp(
                Mathf.Min(
                    Mathf.Max(
                        0.005f,
                        Time.fixedDeltaTime),
                    total * 0.05f),
                0.005f,
                0.02f);

        float EvaluateAtTime(float sampleTime)
        {
            float sampleU =
                ResolvePredictionUAtElapsedTime(
                    plan,
                    Mathf.Clamp(
                        sampleTime,
                        0f,
                        total));

            return
                EvaluateReferenceNormalOffset(
                    plan,
                    sampleU);
        }

        targetNormalMeters =
            EvaluateAtTime(t);

        if (!IsFiniteScalar(targetNormalMeters))
            return false;

        // One-sided second-order stencil near each endpoint.
        if (t <= sampleDt &&
            total >= 2f * sampleDt)
        {
            float h0 =
                EvaluateAtTime(t);
            float h1 =
                EvaluateAtTime(t + sampleDt);
            float h2 =
                EvaluateAtTime(t + 2f * sampleDt);

            targetNormalSpeed =
                (-3f * h0 +
                 4f * h1 -
                 h2) /
                (2f * sampleDt);

            targetNormalAcceleration =
                (h2 -
                 2f * h1 +
                 h0) /
                (sampleDt * sampleDt);
        }
        else if (t >= total - sampleDt &&
                 total >= 2f * sampleDt)
        {
            float h0 =
                EvaluateAtTime(t);
            float h1 =
                EvaluateAtTime(t - sampleDt);
            float h2 =
                EvaluateAtTime(t - 2f * sampleDt);

            targetNormalSpeed =
                (3f * h0 -
                 4f * h1 +
                 h2) /
                (2f * sampleDt);

            targetNormalAcceleration =
                (h0 -
                 2f * h1 +
                 h2) /
                (sampleDt * sampleDt);
        }
        else
        {
            float t0 =
                Mathf.Max(
                    0f,
                    t - sampleDt);
            float t2 =
                Mathf.Min(
                    total,
                    t + sampleDt);

            float h0 =
                EvaluateAtTime(t0);
            float h1 =
                targetNormalMeters;
            float h2 =
                EvaluateAtTime(t2);

            float dt0 =
                Mathf.Max(
                    Eps,
                    t - t0);
            float dt1 =
                Mathf.Max(
                    Eps,
                    t2 - t);

            targetNormalSpeed =
                (h2 - h0) /
                Mathf.Max(
                    Eps,
                    t2 - t0);

            // Uneven-grid second derivative. This remains finite when the
            // prediction time mapping approaches either endpoint.
            targetNormalAcceleration =
                2f *
                (((h2 - h1) / dt1) -
                 ((h1 - h0) / dt0)) /
                Mathf.Max(
                    Eps,
                    dt0 + dt1);
        }

        float accelerationLimit =
            Mathf.Max(
                1f,
                maximumPredictedNormalAcceleration);

        targetNormalAcceleration =
            Mathf.Clamp(
                targetNormalAcceleration,
                -accelerationLimit,
                accelerationLimit);

        return
            IsFiniteScalar(targetNormalSpeed) &&
            IsFiniteScalar(targetNormalAcceleration);
    }

    bool TryEvaluateMechanicalNormalBallisticPrediction(
        CurvePlan plan,
        float u,
        out MechanicalNormalSolveResult result)
    {
        result =
            new MechanicalNormalSolveResult();

        if (!plan.mechanicalNormalValid)
            return false;

        float elapsed =
            ResolvePredictionElapsedTimeAtU(
                plan,
                u);

        if (elapsed <= Eps)
        {
            result.valid = true;
            result.normalOffsetMeters = 0f;
            result.relativeNormalSpeed =
                plan.mechanicalInitialRelativeNormalSpeed;
            result.firstUpperTurningTimeSeconds = -1f;
            result.firstUpperImpactTimeSeconds = -1f;
            return true;
        }

        int steps =
            Mathf.Clamp(
                Mathf.CeilToInt(
                    mechanicalIntegrationSubsteps *
                    elapsed /
                    Mathf.Max(
                        0.05f,
                        plan.totalDuration)),
                4,
                Mathf.Max(
                    4,
                    mechanicalIntegrationSubsteps));

        float dt =
            elapsed /
            Mathf.Max(
                1,
                steps);

        float h = 0f;
        float v =
            plan.mechanicalInitialRelativeNormalSpeed;

        float decaySeconds =
            Mathf.Max(
                0.01f,
                mechanicalNormalResidualAccelerationDecaySeconds);

        for (int i = 0; i < steps; i++)
        {
            float tMid =
                (i + 0.5f) * dt;

            float uMid =
                ResolvePredictionUAtElapsedTime(
                    plan,
                    tMid);

            if (!TryEvaluateMechanicalCarrierCenter(
                    plan,
                    uMid,
                    out _,
                    out _,
                    out Vector3 normal))
            {
                return false;
            }

            float gravityAN =
                Vector3.Dot(
                    Physics.gravity,
                    normal);

            float residualAN =
                plan.mechanicalResidualNormalAcceleration0 *
                Mathf.Exp(
                    -tMid / decaySeconds);

            float carrierAN =
                EstimateMechanicalCarrierNormalAcceleration(
                    plan,
                    tMid);

            float aRelative =
                gravityAN +
                residualAN -
                carrierAN;

            aRelative =
                Mathf.Clamp(
                    aRelative,
                    -Mathf.Max(
                        1f,
                        maximumPredictedNormalAcceleration),
                    Mathf.Max(
                        1f,
                        maximumPredictedNormalAcceleration));

            float vNext =
                v +
                aRelative * dt;

            h +=
                0.5f *
                (v + vNext) *
                dt;

            v =
                vNext;

            float radius =
                ResolveEqualizerWorldRadius();

            float maximumOffset =
                Mathf.Max(
                    1f,
                    mechanicalNormalMaximumOffsetR) *
                Mathf.Max(
                    0.0001f,
                    radius);

            if (!IsFiniteScalar(h) ||
                !IsFiniteScalar(v) ||
                Mathf.Abs(h) >
                    maximumOffset)
            {
                return false;
            }
        }

        result.valid = true;
        result.normalOffsetMeters = h;
        result.relativeNormalSpeed = v;
        result.firstUpperTurningTimeSeconds = -1f;
        result.firstUpperImpactTimeSeconds = -1f;
        return true;
    }

    bool TryEvaluateMechanicalNormalOscillatorPrediction(
        CurvePlan plan,
        float u,
        bool applyUpperReflection,
        out MechanicalNormalSolveResult result)
    {
        result =
            new MechanicalNormalSolveResult
            {
                firstUpperTurningTimeSeconds = -1f,
                firstUpperImpactTimeSeconds = -1f
            };

        if (!plan.mechanicalNormalValid)
            return false;

        float elapsed =
            ResolvePredictionElapsedTimeAtU(
                plan,
                u);

        if (elapsed <= Eps)
        {
            result.valid = true;
            result.normalOffsetMeters = 0f;
            result.relativeNormalSpeed =
                plan.mechanicalInitialRelativeNormalSpeed;
            return true;
        }

        int baseSteps =
            Mathf.Max(
                4,
                mechanicalIntegrationSubsteps);

        // The oscillator contains a 420 s^-2 spring by default, so it needs a
        // tighter step than the ballistic predictor. Keep it bounded and
        // deterministic rather than tying the result to frame rate.
        int oscillatorSteps =
            Mathf.CeilToInt(
                elapsed /
                Mathf.Max(
                    0.0025f,
                    Mathf.Min(
                        0.01f,
                        Time.fixedDeltaTime * 0.5f)));

        int steps =
            Mathf.Clamp(
                Mathf.Max(
                    baseSteps,
                    oscillatorSteps),
                4,
                128);

        float dt =
            elapsed /
            Mathf.Max(
                1,
                steps);

        float h = 0f;
        float v =
            plan.mechanicalInitialRelativeNormalSpeed;

        float controllerAcceleration = 0f;

        float decaySeconds =
            Mathf.Max(
                0.01f,
                mechanicalNormalResidualAccelerationDecaySeconds);

        float radius =
            Mathf.Max(
                0.0001f,
                ResolveEqualizerWorldRadius());

        float maximumOffset =
            Mathf.Max(
                1f,
                mechanicalNormalMaximumOffsetR) *
            radius;

        float accelerationLimit =
            Mathf.Max(
                1f,
                mechanicalNormalControllerAccelerationLimit);

        float jerkLimit =
            Mathf.Max(
                1f,
                mechanicalNormalControllerJerkLimit);

        float turnEpsilon =
            Mathf.Max(
                0f,
                mechanicalNormalTurningSpeedEpsilon);

        float restitution =
            Mathf.Clamp01(
                historicalUpperRestitution01 *
                Mathf.Max(
                    0f,
                    mechanicalNormalShadowUpperRestitutionScale));

        for (int i = 0; i < steps; i++)
        {
            float t0 =
                i * dt;
            float tMid =
                (i + 0.5f) * dt;
            float t1 =
                (i + 1f) * dt;

            float uMid =
                ResolvePredictionUAtElapsedTime(
                    plan,
                    tMid);

            if (!TryEvaluateMechanicalCarrierCenter(
                    plan,
                    uMid,
                    out _,
                    out _,
                    out Vector3 normal))
            {
                return false;
            }

            if (!TryEvaluateMechanicalReferenceKinematics(
                    plan,
                    tMid,
                    out float targetH,
                    out float targetV,
                    out float targetA))
            {
                return false;
            }

            float gravityAN =
                Vector3.Dot(
                    Physics.gravity,
                    normal);

            float gravityCompensationAN =
                -gravityAN *
                Mathf.Clamp(
                    mechanicalNormalGravityCompensation,
                    0f,
                    1.5f);

            float springAN =
                (targetH - h) *
                Mathf.Max(
                    0f,
                    mechanicalNormalSpringStrength);

            float damperAN =
                (targetV - v) *
                Mathf.Max(
                    0f,
                    mechanicalNormalDamper);

            float feedForwardAN =
                targetA *
                Mathf.Clamp(
                    mechanicalNormalReferenceAccelerationFeedForward,
                    0f,
                    1.5f);

            float desiredControllerAN =
                springAN +
                damperAN +
                gravityCompensationAN +
                feedForwardAN;

            desiredControllerAN =
                Mathf.Clamp(
                    desiredControllerAN,
                    -accelerationLimit,
                    accelerationLimit);

            float controlAuthority =
                ResolveMechanicalOscillatorControlAuthority01(
                    plan,
                    tMid);

            desiredControllerAN *=
                controlAuthority;

            controllerAcceleration =
                Mathf.MoveTowards(
                    controllerAcceleration,
                    desiredControllerAN,
                    jerkLimit * dt);

            float residualAN =
                plan.mechanicalResidualNormalAcceleration0 *
                Mathf.Exp(
                    -tMid / decaySeconds) *
                Mathf.Clamp01(
                    mechanicalNormalResidualAccelerationAuthority01);

            // BallVisualEqualizerSync adds support acceleration to the world
            // command before applying it. In carrier-relative coordinates that
            // term cancels the carrier acceleration, so do not subtract it a
            // second time here.
            float aRelative =
                gravityAN +
                controllerAcceleration +
                residualAN;

            aRelative =
                Mathf.Clamp(
                    aRelative,
                    -Mathf.Max(
                        accelerationLimit,
                        maximumPredictedNormalAcceleration),
                    Mathf.Max(
                        accelerationLimit,
                        maximumPredictedNormalAcceleration));

            float vBefore =
                v;

            float vNext =
                v +
                aRelative * dt;

            float hNext =
                h +
                0.5f *
                (v + vNext) *
                dt;

            if (applyUpperReflection)
            {
                float u0 =
                    ResolvePredictionUAtElapsedTime(
                        plan,
                        t0);

                float u1 =
                    ResolvePredictionUAtElapsedTime(
                        plan,
                        t1);

                ResolveHistoricalUpperSpanAtElapsedTime(
                    u0,
                    t0,
                    plan.totalDuration,
                    radius,
                    out _,
                    out _,
                    out _,
                    out float upper0);

                ResolveHistoricalUpperSpanAtElapsedTime(
                    u1,
                    t1,
                    plan.totalDuration,
                    radius,
                    out _,
                    out _,
                    out _,
                    out float upper1);

                float gap0 =
                    upper0 - h;

                float gap1 =
                    upper1 - hNext;

                if (gap0 > 0f &&
                    gap1 <= 0f &&
                    vNext > 0f)
                {
                    float denominator =
                        gap0 - gap1;

                    float crossing01 =
                        Mathf.Abs(denominator) > Eps
                            ? Mathf.Clamp01(
                                gap0 / denominator)
                            : 1f;

                    float impactTime =
                        Mathf.Lerp(
                            t0,
                            t1,
                            crossing01);

                    if (!result.predictedUpperImpact)
                    {
                        result.predictedUpperImpact = true;
                        result.firstUpperImpactTimeSeconds =
                            impactTime;
                    }

                    float overshoot =
                        Mathf.Max(
                            0f,
                            hNext - upper1);

                    hNext =
                        upper1 -
                        overshoot;

                    vNext =
                        -Mathf.Abs(vNext) *
                        restitution;
                }
            }

            if (vBefore > turnEpsilon &&
                vNext <= 0f)
            {
                float denominator =
                    vBefore - vNext;

                float crossing01 =
                    Mathf.Abs(denominator) > Eps
                        ? Mathf.Clamp01(
                            vBefore / denominator)
                        : 1f;

                float turningTime =
                    Mathf.Lerp(
                        t0,
                        t1,
                        crossing01);

                result.upperTurningCount++;

                if (!result.predictedUpperTurning)
                {
                    result.predictedUpperTurning = true;
                    result.firstUpperTurningTimeSeconds =
                        turningTime;
                }
            }

            h = hNext;
            v = vNext;

            if (!IsFiniteScalar(h) ||
                !IsFiniteScalar(v) ||
                !IsFiniteScalar(controllerAcceleration) ||
                Mathf.Abs(h) >
                    maximumOffset)
            {
                return false;
            }
        }

        result.valid = true;
        result.normalOffsetMeters = h;
        result.relativeNormalSpeed = v;
        return true;
    }

    bool TryEvaluateMechanicalNormalPredictionDetailed(
        CurvePlan plan,
        float u,
        out MechanicalNormalSolveResult result)
    {
        if (!enableMechanicalNormalOscillator)
        {
            return
                TryEvaluateMechanicalNormalBallisticPrediction(
                    plan,
                    u,
                    out result);
        }

        bool applyUpperReflection =
            mechanicalNormalPredictionMode ==
                MechanicalNormalPredictionMode.ShadowOnly &&
            mechanicalNormalEnableUpperReflectionInShadow;

        return
            TryEvaluateMechanicalNormalOscillatorPrediction(
                plan,
                u,
                applyUpperReflection,
                out result);
    }

    bool TryEvaluateMechanicalNormalPrediction(
        CurvePlan plan,
        float u,
        out float normalOffsetMeters,
        out float relativeNormalSpeed)
    {
        normalOffsetMeters = 0f;
        relativeNormalSpeed = 0f;

        if (!TryEvaluateMechanicalNormalPredictionDetailed(
                plan,
                u,
                out MechanicalNormalSolveResult result))
        {
            return false;
        }

        normalOffsetMeters =
            result.normalOffsetMeters;

        relativeNormalSpeed =
            result.relativeNormalSpeed;

        return
            result.valid;
    }

    float ResolveMechanicalNormalAuthority01(
        float elapsedSeconds)
    {
        float full =
            Mathf.Max(
                0f,
                mechanicalNormalFullAuthoritySeconds);
        float fadeEnd =
            Mathf.Max(
                full + 0.001f,
                mechanicalNormalFadeEndSeconds);

        if (elapsedSeconds <= full)
            return 1f;

        if (elapsedSeconds >= fadeEnd)
            return 0f;

        float x =
            Mathf.InverseLerp(
                full,
                fadeEnd,
                elapsedSeconds);

        float smooth =
            x * x * (3f - 2f * x);

        return 1f - smooth;
    }

    bool TryEvaluatePredictedNormalLayers(
        CurvePlan plan,
        float u,
        out float referenceNormal,
        out float mechanicalNormal,
        out float blendedNormal)
    {
        referenceNormal =
            EvaluateReferenceNormalOffset(
                plan,
                u);
        mechanicalNormal =
            referenceNormal;
        blendedNormal =
            referenceNormal;

        if (mechanicalNormalPredictionMode ==
            MechanicalNormalPredictionMode.ReferenceOnly)
        {
            return false;
        }

        if (!TryEvaluateMechanicalNormalPrediction(
                plan,
                u,
                out mechanicalNormal,
                out _))
        {
            mechanicalNormal =
                referenceNormal;
            blendedNormal =
                referenceNormal;
            return false;
        }

        float elapsed =
            ResolvePredictionElapsedTimeAtU(
                plan,
                u);

        float authority =
            ResolveMechanicalNormalAuthority01(
                elapsed);

        blendedNormal =
            Mathf.Lerp(
                referenceNormal,
                mechanicalNormal,
                authority);

        return true;
    }



    void SchedulePredictionProbes(CurvePlan plan)
    {
        if (!equalizerBody || predictionErrorHorizonsSeconds == null || predictionArcLengthTable.Count < 2)
            return;

        NearestKnotDetector.GuideFrame guide = knotDetector.CurrentGuide;
        if (!guide.valid)
            return;

        float now = Time.fixedTime;
        if (now - lastPredictionProbeIssueTime < Mathf.Max(0.01f, predictionProbeIssueIntervalSeconds))
            return;

        lastPredictionProbeIssueTime = now;
        EnsurePredictionErrorBuckets();

        Vector3 observationTangentAtIssue =
            NormalizeSafe(
                correspondSubject.MapDirection(guide.tangent),
                Vector3.forward);

        Vector3 observationNormalAtIssue =
            BuildInitialNormal(
                observationTangentAtIssue,
                correspondSubject.MapDirection(guide.normal));

        float observedRelativeNormalSpeedAtIssue =
            MeasureRelativeNormalSpeedOnFrozenNormal(
                observationNormalAtIssue);

        int realUpperReflectionCountAtIssue =
            equalizerSync
                ? equalizerSync.UpperReflectionCount
                : 0;

        for (int i = 0; i < predictionErrorHorizonsSeconds.Length; i++)
        {
            float horizon = predictionErrorHorizonsSeconds[i];
            if (!IsFiniteScalar(horizon) || horizon <= 0f)
                continue;

            float predictedTravel = PredictTravelDistanceMechanical(
                guide,
                horizon,
                Mathf.Max(minimumLiveTangentialSpeed, liveEqualizerTangentialSpeed),
                liveEqualizerResidualTangentialAcceleration);

            if (!TryMapPredictionTravelToU(predictedTravel, out float u) ||
                !TryEvaluateFinalAnalyticDiagnostic(
                    plan,
                    u,
                    out Vector3 predicted,
                    out Vector3 tangent,
                    out Vector3 normal,
                    out Vector3 side))
            {
                continue;
            }

            float referenceNormal =
                EvaluateReferenceNormalOffset(
                    plan,
                    u);

            float mechanicalNormal =
                referenceNormal;

            float blendedNormal =
                referenceNormal;

            MechanicalNormalSolveResult mechanicalSolve =
                new MechanicalNormalSolveResult
                {
                    firstUpperTurningTimeSeconds = -1f,
                    firstUpperImpactTimeSeconds = -1f
                };

            bool mechanicalNormalValid =
                mechanicalNormalPredictionMode !=
                    MechanicalNormalPredictionMode.ReferenceOnly &&
                TryEvaluateMechanicalNormalPredictionDetailed(
                    plan,
                    u,
                    out mechanicalSolve);

            if (mechanicalNormalValid)
            {
                mechanicalNormal =
                    mechanicalSolve.normalOffsetMeters;

                float mechanicalElapsed =
                    ResolvePredictionElapsedTimeAtU(
                        plan,
                        u);

                float mechanicalAuthority =
                    ResolveMechanicalNormalAuthority01(
                        mechanicalElapsed);

                blendedNormal =
                    Mathf.Lerp(
                        referenceNormal,
                        mechanicalNormal,
                        mechanicalAuthority);
            }

            float carrierBridgeNormal =
                baselineOnlyDiagnostic
                    ? 0f
                    : EvaluateBoundaryResidual(
                        plan,
                        u,
                        plan.residualNormal0,
                        plan.residualNormalSlopePerMeter0);

            pendingPredictionProbes.Add(new PredictionProbe
            {
                dueTime = now + horizon,
                horizonSeconds = horizon,
                predictedPosition = predicted,
                tangent = tangent,
                normal = normal,
                side = side,

                mechanicalNormalValid = mechanicalNormalValid,
                sourceSplineIndex = plan.sourceSplineIndex,
                sourceSectionIndex = plan.sourceSectionIndex,
                maxGroundSpeedCondition =
                    slopeCore
                        ? slopeCore.MaxGroundSpeedReadOnly
                        : 0f,
                realUpperReflectionCountAtIssue =
                    realUpperReflectionCountAtIssue,
                issueTime = now,
                observationNormalAtIssue =
                    observationNormalAtIssue,
                carrierBridgeNormalMeters = carrierBridgeNormal,
                referenceNormalMeters = referenceNormal,
                mechanicalNormalMeters = mechanicalNormal,
                blendedNormalMeters = blendedNormal,

                previousObservedRelativeNormalSpeed =
                    observedRelativeNormalSpeedAtIssue,
                observedUpperTurning = false,
                observedUpperTurningTimeSeconds = -1f,
                observedUpperImpact = false,
                observedUpperImpactTimeSeconds = -1f,
                observationInvalidated = false,
                observationInvalidReason = "None",
                predictedMechanicalNormalSpeed =
                    mechanicalNormalValid
                        ? mechanicalSolve.relativeNormalSpeed
                        : 0f,
                predictedUpperTurning =
                    mechanicalNormalValid &&
                    mechanicalSolve.predictedUpperTurning,
                predictedUpperTurningTimeSeconds =
                    mechanicalNormalValid
                        ? mechanicalSolve.firstUpperTurningTimeSeconds
                        : -1f,
                predictedUpperImpact =
                    mechanicalNormalValid &&
                    mechanicalSolve.predictedUpperImpact,
                predictedUpperImpactTimeSeconds =
                    mechanicalNormalValid
                        ? mechanicalSolve.firstUpperImpactTimeSeconds
                        : -1f
            });
        }

        if (pendingPredictionProbes.Count > 256)
            pendingPredictionProbes.RemoveRange(0, pendingPredictionProbes.Count - 256);
    }

    float MeasureRelativeNormalSpeedOnFrozenNormal(
        Vector3 frozenNormal)
    {
        if (!equalizerBody ||
            frozenNormal.sqrMagnitude <= Eps)
        {
            return 0f;
        }

        Vector3 equalizerVelocity =
            equalizerBody.velocity;

        Vector3 carrierVelocity =
            Vector3.zero;

        if (correspondSubject)
        {
            Rigidbody subjectBody =
                correspondSubject.SubjectBody;

            if (subjectBody &&
                IsFinite(subjectBody.velocity))
            {
                carrierVelocity =
                    subjectBody.velocity;
            }
        }

        if (!IsFinite(equalizerVelocity) ||
            !IsFinite(carrierVelocity))
        {
            return 0f;
        }

        return Vector3.Dot(
            equalizerVelocity - carrierVelocity,
            frozenNormal.normalized);
    }

    void InvalidatePredictionProbeObservation(
        ref PredictionProbe probe,
        string reason)
    {
        if (probe.observationInvalidated)
            return;

        probe.observationInvalidated = true;
        probe.observationInvalidReason =
            string.IsNullOrEmpty(reason)
                ? "Unknown"
                : reason;
    }

    void UpdatePredictionProbeObservations()
    {
        if (pendingPredictionProbes.Count == 0)
            return;

        float now = Time.fixedTime;

        for (int i = 0; i < pendingPredictionProbes.Count; i++)
        {
            PredictionProbe probe =
                pendingPredictionProbes[i];

            if (probe.observationInvalidated)
                continue;

            if (!probe.mechanicalNormalValid)
            {
                InvalidatePredictionProbeObservation(
                    ref probe,
                    "MechanicalSeedInvalid");
                pendingPredictionProbes[i] = probe;
                continue;
            }

            if (!mechanicalNormalStateValid)
            {
                InvalidatePredictionProbeObservation(
                    ref probe,
                    string.IsNullOrEmpty(mechanicalNormalInvalidReason)
                        ? "MechanicalStateInvalid"
                        : mechanicalNormalInvalidReason);
                pendingPredictionProbes[i] = probe;
                continue;
            }

            if (!equalizerSync)
            {
                InvalidatePredictionProbeObservation(
                    ref probe,
                    "MissingEqualizerSync");
                pendingPredictionProbes[i] = probe;
                continue;
            }

            if (equalizerSync.IsEmergencyVisualRecoveryActive)
            {
                InvalidatePredictionProbeObservation(
                    ref probe,
                    "EmergencyRecovery");
                pendingPredictionProbes[i] = probe;
                continue;
            }

            if (!equalizerBody ||
                !IsFinite(equalizerBody.position) ||
                !IsFinite(equalizerBody.velocity))
            {
                InvalidatePredictionProbeObservation(
                    ref probe,
                    "EqualizerStateInvalid");
                pendingPredictionProbes[i] = probe;
                continue;
            }

            float observedRelativeNormalSpeed =
                MeasureRelativeNormalSpeedOnFrozenNormal(
                    probe.observationNormalAtIssue);

            float turningEpsilon =
                Mathf.Max(
                    0f,
                    mechanicalNormalTurningSpeedEpsilon);

            if (!probe.observedUpperTurning &&
                probe.previousObservedRelativeNormalSpeed > turningEpsilon &&
                observedRelativeNormalSpeed <= 0f)
            {
                probe.observedUpperTurning = true;
                probe.observedUpperTurningTimeSeconds =
                    Mathf.Max(
                        0f,
                        now - probe.issueTime);
            }

            probe.previousObservedRelativeNormalSpeed =
                observedRelativeNormalSpeed;

            if (!probe.observedUpperImpact &&
                equalizerSync &&
                equalizerSync.UpperReflectionCount >
                    probe.realUpperReflectionCountAtIssue)
            {
                probe.observedUpperImpact = true;

                float observedEventTime =
                    equalizerSync.LastUpperReflectionFixedTime;

                probe.observedUpperImpactTimeSeconds =
                    observedEventTime >= probe.issueTime
                        ? observedEventTime - probe.issueTime
                        : Mathf.Max(0f, now - probe.issueTime);
            }

            pendingPredictionProbes[i] = probe;
        }
    }

    void BuildPredictionArcLengthTable(CurvePlan plan)
    {
        predictionArcLengthTable.Clear();
        int count = Mathf.Clamp(predictionArcLengthSamples, 24, 256);

        if (!TryEvaluateFinalAnalyticPoint(plan, 0f, out Vector3 previous))
            return;

        float distance = 0f;
        predictionArcLengthTable.Add(new PredictionArcLengthSample
        {
            u = 0f,
            distanceMeters = 0f
        });

        for (int i = 1; i <= count; i++)
        {
            float u = i / (float)count;
            if (!TryEvaluateFinalAnalyticPoint(plan, u, out Vector3 point))
            {
                predictionArcLengthTable.Clear();
                return;
            }

            distance += Vector3.Distance(previous, point);
            predictionArcLengthTable.Add(new PredictionArcLengthSample
            {
                u = u,
                distanceMeters = distance
            });
            previous = point;
        }

        lastPredictionArcLengthMeters = distance;
    }

    bool TryMapPredictionTravelToU(float travelMeters, out float u)
    {
        u = 0f;
        if (predictionArcLengthTable.Count < 2)
            return false;

        float target = Mathf.Max(0f, travelMeters);
        PredictionArcLengthSample last =
            predictionArcLengthTable[predictionArcLengthTable.Count - 1];
        if (target >= last.distanceMeters)
        {
            u = 1f;
            return true;
        }

        for (int i = 1; i < predictionArcLengthTable.Count; i++)
        {
            PredictionArcLengthSample b = predictionArcLengthTable[i];
            if (target > b.distanceMeters)
                continue;

            PredictionArcLengthSample a = predictionArcLengthTable[i - 1];
            float t = Mathf.InverseLerp(a.distanceMeters, b.distanceMeters, target);
            u = Mathf.Lerp(a.u, b.u, t);
            return true;
        }

        return false;
    }



    void EvaluateDuePredictionProbes()
    {
        if (!equalizerBody || pendingPredictionProbes.Count == 0)
            return;

        float now = Time.fixedTime;
        Vector3 actual = equalizerBody.position;
        if (!IsFinite(actual))
        {
            pendingPredictionProbes.Clear();
            return;
        }

        for (int i = pendingPredictionProbes.Count - 1; i >= 0; i--)
        {
            PredictionProbe probe = pendingPredictionProbes[i];
            if (probe.dueTime > now + 0.5f * Time.fixedDeltaTime)
                continue;

            if (probe.observationInvalidated)
            {
                if (enableMechanicalNormalShadowLogs &&
                    Mathf.Abs(
                        probe.horizonSeconds -
                        Mathf.Max(
                            0.01f,
                            mechanicalNormalShadowLogHorizonSeconds)) <= 0.005f)
                {
                    Debug.Log(
                        $"[NORMAL PREDICTION INVALIDATED] " +
                        $"mode={mechanicalNormalPredictionMode} " +
                        $"maxGround={probe.maxGroundSpeedCondition:F2}m/s " +
                        $"horizon={probe.horizonSeconds:F3}s " +
                        $"reason={probe.observationInvalidReason}",
                        this);
                }

                pendingPredictionProbes.RemoveAt(i);
                continue;
            }

            Vector3 error = actual - probe.predictedPosition;
            float eT = Vector3.Dot(error, probe.tangent);
            float eN = Vector3.Dot(error, probe.normal);
            float eB = Vector3.Dot(error, probe.side);
            float eP = error.magnitude;

            UpdatePredictionErrorBucket(
                probe.horizonSeconds,
                eP,
                eT,
                eN,
                eB);

            bool shadowWindowStillClean =
                probe.mechanicalNormalValid &&
                mechanicalNormalStateValid &&
                !(equalizerSync &&
                  equalizerSync.IsEmergencyVisualRecoveryActive);

            if (shadowWindowStillClean &&
                TryMeasureCurrentNeutralNormalOffset(
                    probe.sourceSplineIndex,
                    probe.sourceSectionIndex,
                    out float actualNeutralNormal))
            {
                float actualNormal =
                    actualNeutralNormal -
                    probe.carrierBridgeNormalMeters;

                float actualNormalSpeed =
                    MeasureRelativeNormalSpeedOnFrozenNormal(
                        probe.observationNormalAtIssue);

                bool observedUpperTurningByHorizon =
                    probe.observedUpperTurning;

                bool observedUpperImpactByHorizon =
                    probe.observedUpperImpact;

                UpdateMechanicalNormalPredictionErrorBucket(
                    probe.maxGroundSpeedCondition,
                    probe.horizonSeconds,
                    actualNormal,
                    probe.referenceNormalMeters,
                    probe.mechanicalNormalMeters,
                    probe.blendedNormalMeters,
                    actualNormalSpeed,
                    probe.predictedMechanicalNormalSpeed,
                    probe.predictedUpperTurning,
                    observedUpperTurningByHorizon,
                    probe.predictedUpperTurningTimeSeconds,
                    probe.observedUpperTurningTimeSeconds,
                    probe.predictedUpperImpact,
                    observedUpperImpactByHorizon,
                    probe.predictedUpperImpactTimeSeconds,
                    probe.observedUpperImpactTimeSeconds);

                bool shadowLogHorizon =
                    Mathf.Abs(
                        probe.horizonSeconds -
                        Mathf.Max(
                            0.01f,
                            mechanicalNormalShadowLogHorizonSeconds)) <=
                    0.005f;

                bool shadowLogInterval =
                    now - lastMechanicalNormalShadowLogTime >=
                    Mathf.Max(
                        0.10f,
                        mechanicalNormalShadowLogIntervalSeconds);

                if (enableMechanicalNormalShadowLogs &&
                    shadowLogHorizon &&
                    shadowLogInterval)
                {
                    lastMechanicalNormalShadowLogTime = now;

                    float turnTimeError =
                        probe.predictedUpperTurning &&
                        observedUpperTurningByHorizon &&
                        probe.predictedUpperTurningTimeSeconds >= 0f &&
                        probe.observedUpperTurningTimeSeconds >= 0f
                            ? Mathf.Abs(
                                probe.predictedUpperTurningTimeSeconds -
                                probe.observedUpperTurningTimeSeconds)
                            : -1f;

                    float upperTimeError =
                        probe.predictedUpperImpact &&
                        observedUpperImpactByHorizon &&
                        probe.predictedUpperImpactTimeSeconds >= 0f &&
                        probe.observedUpperImpactTimeSeconds >= 0f
                            ? Mathf.Abs(
                                probe.predictedUpperImpactTimeSeconds -
                                probe.observedUpperImpactTimeSeconds)
                            : -1f;

                    Debug.Log(
                        $"[NORMAL PREDICTION] " +
                        $"mode={mechanicalNormalPredictionMode} " +
                        $"maxGround={probe.maxGroundSpeedCondition:F2}m/s " +
                        $"horizon={probe.horizonSeconds:F3}s " +
                        $"actual={actualNormal:F4}m " +
                        $"ref={probe.referenceNormalMeters:F4}m " +
                        $"mech={probe.mechanicalNormalMeters:F4}m " +
                        $"blend={probe.blendedNormalMeters:F4}m " +
                        $"errRef={Mathf.Abs(actualNormal - probe.referenceNormalMeters):F4}m " +
                        $"errMech={Mathf.Abs(actualNormal - probe.mechanicalNormalMeters):F4}m " +
                        $"errBlend={Mathf.Abs(actualNormal - probe.blendedNormalMeters):F4}m " +
                        $"vObsFrozen={actualNormalSpeed:F4}m/s " +
                        $"vMechRel={probe.predictedMechanicalNormalSpeed:F4}m/s " +
                        $"turnPred={probe.predictedUpperTurning} " +
                        $"turnObs={observedUpperTurningByHorizon} " +
                        $"turnPredT={probe.predictedUpperTurningTimeSeconds:F4}s " +
                        $"turnObsT={probe.observedUpperTurningTimeSeconds:F4}s " +
                        $"turnErrT={turnTimeError:F4}s " +
                        $"upperPred={probe.predictedUpperImpact} " +
                        $"upperObs={observedUpperImpactByHorizon} " +
                        $"upperPredT={probe.predictedUpperImpactTimeSeconds:F4}s " +
                        $"upperObsT={probe.observedUpperImpactTimeSeconds:F4}s " +
                        $"upperErrT={upperTimeError:F4}s",
                        this);
                }
            }

            pendingPredictionProbes.RemoveAt(i);
        }
    }

    bool TryMeasureCurrentNeutralNormalOffset(
        int sourceSplineIndex,
        int sourceSectionIndex,
        out float normalOffsetMeters)
    {
        normalOffsetMeters = 0f;

        if (!equalizerBody ||
            !knotDetector ||
            !correspondSubject)
        {
            return false;
        }

        NearestKnotDetector.GuideFrame guide =
            knotDetector.CurrentGuide;

        if (!guide.valid ||
            !guide.isSlope ||
            guide.splineIndex != sourceSplineIndex ||
            guide.sectionIndex != sourceSectionIndex)
        {
            return false;
        }

        Vector3 tangent =
            NormalizeSafe(
                correspondSubject.MapDirection(
                    guide.tangent),
                Vector3.forward);

        Vector3 normal =
            BuildInitialNormal(
                tangent,
                correspondSubject.MapDirection(
                    guide.normal));

        float radius =
            ResolveEqualizerWorldRadius();

        float clearance =
            neutralCenterClearanceLatched &&
            IsFiniteScalar(
                latchedNeutralCenterClearanceMeters) &&
            latchedNeutralCenterClearanceMeters > 0f
                ? latchedNeutralCenterClearanceMeters
                : Mathf.Max(
                    0.0001f,
                    radius *
                    Mathf.Max(
                        0.25f,
                        baseCenterClearanceRadiusMultiplier));

        Vector3 neutralCenter =
            correspondSubject.MapPoint(
                guide.point) +
            normal * clearance;

        normalOffsetMeters =
            Vector3.Dot(
                equalizerBody.position -
                neutralCenter,
                normal);

        return IsFiniteScalar(normalOffsetMeters);
    }

    int FindOrCreateMechanicalNormalPredictionErrorBucket(
        float maxGroundSpeedCondition,
        float horizon)
    {
        for (int i = 0;
             i < mechanicalNormalPredictionErrors.Count;
             i++)
        {
            MechanicalNormalPredictionDiagnostic d =
                mechanicalNormalPredictionErrors[i];

            if (Mathf.Abs(
                    d.maxGroundSpeedCondition -
                    maxGroundSpeedCondition) <= 0.01f &&
                Mathf.Abs(
                    d.horizonSeconds -
                    horizon) <= 0.0001f)
            {
                return i;
            }
        }

        mechanicalNormalPredictionErrors.Add(
            new MechanicalNormalPredictionDiagnostic
            {
                maxGroundSpeedCondition =
                    maxGroundSpeedCondition,
                horizonSeconds =
                    horizon
            });

        return
            mechanicalNormalPredictionErrors.Count - 1;
    }

    bool SameSignedState(
        float a,
        float b,
        float deadZone)
    {
        deadZone =
            Mathf.Max(
                0f,
                deadZone);

        bool aNear =
            Mathf.Abs(a) <= deadZone;

        bool bNear =
            Mathf.Abs(b) <= deadZone;

        if (aNear || bNear)
            return aNear && bNear;

        return
            Mathf.Sign(a) ==
            Mathf.Sign(b);
    }

    void UpdateMechanicalNormalPredictionErrorBucket(
        float maxGroundSpeedCondition,
        float horizon,
        float actualNormal,
        float referenceNormal,
        float mechanicalNormal,
        float blendedNormal,
        float actualNormalSpeed,
        float predictedMechanicalNormalSpeed,
        bool predictedUpperTurning,
        bool observedUpperTurningByHorizon,
        float predictedTurningTimeSeconds,
        float observedTurningTimeSeconds,
        bool predictedUpperImpact,
        bool observedUpperImpactByHorizon,
        float predictedUpperImpactTimeSeconds,
        float observedUpperImpactTimeSeconds)
    {
        int index =
            FindOrCreateMechanicalNormalPredictionErrorBucket(
                maxGroundSpeedCondition,
                horizon);

        if (index < 0 ||
            index >= mechanicalNormalPredictionErrors.Count)
        {
            return;
        }

        MechanicalNormalPredictionDiagnostic d =
            mechanicalNormalPredictionErrors[index];

        int previousCount =
            d.samples;

        int nextCount =
            previousCount + 1;

        float inv =
            1f / nextCount;

        float refError =
            Mathf.Abs(
                actualNormal -
                referenceNormal);

        float mechError =
            Mathf.Abs(
                actualNormal -
                mechanicalNormal);

        float blendError =
            Mathf.Abs(
                actualNormal -
                blendedNormal);

        float speedError =
            Mathf.Abs(
                actualNormalSpeed -
                predictedMechanicalNormalSpeed);

        bool positionSignMatch =
            SameSignedState(
                actualNormal,
                mechanicalNormal,
                0.02f);

        bool velocitySignMatch =
            SameSignedState(
                actualNormalSpeed,
                predictedMechanicalNormalSpeed,
                Mathf.Max(
                    0.05f,
                    mechanicalNormalTurningSpeedEpsilon));

        bool turningAgreement =
            predictedUpperTurning ==
            observedUpperTurningByHorizon;

        bool upperImpactAgreement =
            predictedUpperImpact ==
            observedUpperImpactByHorizon;

        bool hasTurningTimePair =
            predictedUpperTurning &&
            observedUpperTurningByHorizon &&
            predictedTurningTimeSeconds >= 0f &&
            observedTurningTimeSeconds >= 0f;

        float turningTimeError =
            hasTurningTimePair
                ? Mathf.Abs(
                    predictedTurningTimeSeconds -
                    observedTurningTimeSeconds)
                : -1f;

        bool hasUpperImpactTimePair =
            predictedUpperImpact &&
            observedUpperImpactByHorizon &&
            predictedUpperImpactTimeSeconds >= 0f &&
            observedUpperImpactTimeSeconds >= 0f;

        float upperImpactTimeError =
            hasUpperImpactTimePair
                ? Mathf.Abs(
                    predictedUpperImpactTimeSeconds -
                    observedUpperImpactTimeSeconds)
                : -1f;

        float winsBefore =
            d.mechanicalWinRate01 *
            previousCount;

        float positionMatchesBefore =
            d.positionSignMatchRate01 *
            previousCount;

        float velocityMatchesBefore =
            d.velocitySignMatchRate01 *
            previousCount;

        float turningMatchesBefore =
            d.upperTurningAgreementRate01 *
            previousCount;

        float upperImpactMatchesBefore =
            d.upperImpactAgreementRate01 *
            previousCount;

        float win =
            mechError + 0.0001f <
            refError
                ? 1f
                : 0f;

        d.samples = nextCount;
        d.latestActualNormalMeters = actualNormal;
        d.latestReferenceNormalMeters = referenceNormal;
        d.latestMechanicalNormalMeters = mechanicalNormal;
        d.latestBlendedNormalMeters = blendedNormal;
        d.latestAbsReferenceErrorMeters = refError;
        d.latestAbsMechanicalErrorMeters = mechError;
        d.latestAbsBlendedErrorMeters = blendError;

        d.latestActualNormalSpeed =
            actualNormalSpeed;

        d.latestMechanicalNormalSpeed =
            predictedMechanicalNormalSpeed;

        d.latestAbsMechanicalSpeedError =
            speedError;

        d.latestPositionSignMatch =
            positionSignMatch;

        d.latestVelocitySignMatch =
            velocitySignMatch;

        d.latestPredictedUpperTurning =
            predictedUpperTurning;

        d.latestObservedUpperTurningByHorizon =
            observedUpperTurningByHorizon;

        d.latestPredictedTurningTimeSeconds =
            predictedTurningTimeSeconds;

        d.latestObservedTurningTimeSeconds =
            observedTurningTimeSeconds;

        d.latestTurningTimeErrorSeconds =
            turningTimeError;

        d.latestPredictedUpperImpact =
            predictedUpperImpact;

        d.latestObservedUpperImpactByHorizon =
            observedUpperImpactByHorizon;

        d.latestPredictedUpperImpactTimeSeconds =
            predictedUpperImpactTimeSeconds;

        d.latestObservedUpperImpactTimeSeconds =
            observedUpperImpactTimeSeconds;

        d.latestUpperImpactTimeErrorSeconds =
            upperImpactTimeError;

        if (hasTurningTimePair)
        {
            d.turningTimeSamples++;
            float turnInv =
                1f / Mathf.Max(1, d.turningTimeSamples);
            d.meanTurningTimeErrorSeconds +=
                (turningTimeError -
                 d.meanTurningTimeErrorSeconds) *
                turnInv;
        }

        if (hasUpperImpactTimePair)
        {
            d.upperImpactTimeSamples++;
            float upperInv =
                1f / Mathf.Max(1, d.upperImpactTimeSamples);
            d.meanUpperImpactTimeErrorSeconds +=
                (upperImpactTimeError -
                 d.meanUpperImpactTimeErrorSeconds) *
                upperInv;
        }

        d.meanAbsReferenceErrorMeters +=
            (refError -
             d.meanAbsReferenceErrorMeters) *
            inv;

        d.meanAbsMechanicalErrorMeters +=
            (mechError -
             d.meanAbsMechanicalErrorMeters) *
            inv;

        d.meanAbsBlendedErrorMeters +=
            (blendError -
             d.meanAbsBlendedErrorMeters) *
            inv;

        d.meanAbsMechanicalSpeedError +=
            (speedError -
             d.meanAbsMechanicalSpeedError) *
            inv;

        d.mechanicalWinRate01 =
            (winsBefore + win) /
            nextCount;

        d.positionSignMatchRate01 =
            (positionMatchesBefore +
             (positionSignMatch ? 1f : 0f)) /
            nextCount;

        d.velocitySignMatchRate01 =
            (velocityMatchesBefore +
             (velocitySignMatch ? 1f : 0f)) /
            nextCount;

        d.upperTurningAgreementRate01 =
            (turningMatchesBefore +
             (turningAgreement ? 1f : 0f)) /
            nextCount;

        d.upperImpactAgreementRate01 =
            (upperImpactMatchesBefore +
             (upperImpactAgreement ? 1f : 0f)) /
            nextCount;

        mechanicalNormalPredictionErrors[index] = d;
    }

    void EnsurePredictionErrorBuckets()
    {
        if (predictionErrorHorizonsSeconds == null)
            return;

        for (int i = 0; i < predictionErrorHorizonsSeconds.Length; i++)
        {
            float horizon = predictionErrorHorizonsSeconds[i];
            if (horizon <= 0f)
                continue;

            bool found = false;
            for (int j = 0; j < predictionErrors.Count; j++)
            {
                if (Mathf.Abs(predictionErrors[j].horizonSeconds - horizon) <= 0.0001f)
                {
                    found = true;
                    break;
                }
            }

            if (!found)
            {
                predictionErrors.Add(new PredictionErrorDiagnostic
                {
                    horizonSeconds = horizon
                });
            }
        }
    }

    void UpdatePredictionErrorBucket(
        float horizon,
        float positionError,
        float tangentError,
        float normalError,
        float sideError)
    {
        EnsurePredictionErrorBuckets();

        int index = -1;
        for (int i = 0; i < predictionErrors.Count; i++)
        {
            if (Mathf.Abs(predictionErrors[i].horizonSeconds - horizon) <= 0.0001f)
            {
                index = i;
                break;
            }
        }

        if (index < 0)
            return;

        PredictionErrorDiagnostic d = predictionErrors[index];
        int nextCount = d.samples + 1;
        float inv = 1f / nextCount;

        d.samples = nextCount;
        d.latestPositionErrorMeters = positionError;
        d.latestTangentErrorMeters = tangentError;
        d.latestNormalErrorMeters = normalError;
        d.latestSideErrorMeters = sideError;
        d.meanPositionErrorMeters += (positionError - d.meanPositionErrorMeters) * inv;
        d.meanAbsTangentErrorMeters += (Mathf.Abs(tangentError) - d.meanAbsTangentErrorMeters) * inv;
        d.meanAbsNormalErrorMeters += (Mathf.Abs(normalError) - d.meanAbsNormalErrorMeters) * inv;
        d.meanAbsSideErrorMeters += (Mathf.Abs(sideError) - d.meanAbsSideErrorMeters) * inv;

        predictionErrors[index] = d;
    }

    void FreezeEnteredStair(NearestKnotDetector.GuideFrame guide)
    {
        if (guide.sectionIndex < 0)
            return;

        long enteredKey = MakeKey(guide.splineIndex, guide.sectionIndex);

        if (generated.TryGetValue(enteredKey, out GeneratedStairSpline entered))
        {
            entered.frozen = true;
            LogSplineFrozen(entered, "ExactSectionMatch");

            if (autoArmVirtualUpperOnSlopeEntry)
                ArmVirtualUpper(enteredKey, "FreezeExactSection");
            return;
        }

        if (lastPredictedKey != long.MinValue &&
            generated.TryGetValue(lastPredictedKey, out GeneratedStairSpline last))
        {
            last.frozen = true;
            LogSplineFrozen(last, "LastPredictionFallback");

            if (autoArmVirtualUpperOnSlopeEntry)
                ArmVirtualUpper(lastPredictedKey, "FreezeFallback");
        }
    }

    void LogSplineCreatedOnce(GeneratedStairSpline target)
    {
        if (!enableMinimalDebugLog || target == null || target.creationLogged)
            return;

        target.creationLogged = true;

        bool orderValid =
            target.lastUT > 0f &&
            target.lastUM > target.lastUT &&
            target.lastUM < 1f;

        bool terminalValid =
            IsFinite(target.lastPL) &&
            target.lastTerminalErrorMeters <=
            Mathf.Max(0.0001f, terminalValidationToleranceMeters);

        bool knotValid = target.lastKnotCount >= 4;
        bool valid = orderValid && terminalValid && knotValid;

        string message =
            $"[FUTURE SPLINE CREATED] valid={valid} " +
            $"spline={target.sourceSplineIndex} section={target.sourceSectionIndex} " +
            $"knots={target.lastKnotCount} internalKnots={target.lastInternalKnotCount} " +
            $"renderEndU={target.lastRenderedEndU:F3} " +
            $"PFprogress={target.lastPfProgress01:P1} PTprogress={target.lastPtProgress01:P1} " +
            $"uT={target.lastUT:F3} uM={target.lastUM:F3} " +
            $"speed={target.lastPredictedSpeed:F3}m/s " +
            $"maxGround={(slopeCore ? slopeCore.MaxGroundSpeedReadOnly : 0f):F2}m/s " +
            $"TtoPT={target.lastTimeToPT:F3}s TtoPL={target.lastTimeToPL:F3}s " +
            $"waveA={target.lastWaveAmplitude:F3}m " +
            $"centerClearance={target.lastReferenceCenterClearance:F3}m " +
            $"incidentVN={target.lastIncidentNormalSpeed:F3}m/s " +
            $"maxBridgeN={target.lastMaxBridgeNormal:F3}m " +
            $"maxWave={target.lastMaxWave:F3}m " +
            $"maxNormalOffset={target.lastMaxTotalNormalOffset:F3}m " +
            $"terminalError={target.lastTerminalErrorMeters:F5}m\n" +
            $"UpperCandidate source={(string.IsNullOrEmpty(lastUpperContactCandidateSource) ? "None" : lastUpperContactCandidateSource)} " +
            $"u={lastUpperContactCandidateU:F3} gap={lastUpperContactCandidateGapMeters:F4}m " +
            $"closingVN={lastUpperContactCandidateClosingSpeed:F4}m/s " +
            $"Ein={lastUpperContactCandidateEnergyJ:F4}J " +
            $"searchSamples={lastUpperContactSearchSampleCount} " +
            $"protectedKnots={lastUpperContactProtectedKnotCount} " +
            $"reject={(string.IsNullOrEmpty(lastUpperContactCandidateRejectReason) ? "None" : lastUpperContactCandidateRejectReason)}\n" +
            $"MechanicalNormal mode={mechanicalNormalPredictionMode} " +
            $"oscillator={enableMechanicalNormalOscillator} " +
            $"stateValid={mechanicalNormalStateValid} " +
            $"planValid={lastMechanicalPlanValid} " +
            $"relVN0={lastMechanicalInitialRelativeNormalSpeed:F4}m/s " +
            $"residualAN0={lastMechanicalResidualNormalAcceleration:F4}m/s2 " +
            $"K={mechanicalNormalSpringStrength:F1} " +
            $"C={mechanicalNormalDamper:F1} " +
            $"aLimit={mechanicalNormalControllerAccelerationLimit:F1} " +
            $"jerk={mechanicalNormalControllerJerkLimit:F1} " +
            $"shadowUpper={mechanicalNormalEnableUpperReflectionInShadow} " +
            $"stateReason={(string.IsNullOrEmpty(mechanicalNormalInvalidReason) ? "None" : mechanicalNormalInvalidReason)}\n" +
            $"PF_GuideCenter={target.lastPFGuideCenter:F3}\n" +
            $"PF_Final={target.lastPF:F3}\n" +
            $"PT_Surface={target.lastPTSurface:F3}\n" +
            $"PT_CenterBase={target.lastPTCenterBase:F3}\n" +
            $"PT_Final={target.lastPTFinal:F3}\n" +
            $"PM={target.lastPM:F3}\n" +
            $"PL={target.lastPL:F3}";

        if (valid)
            Debug.Log(message, this);
        else
            Debug.LogWarning(message, this);
    }

    void LogSplineFrozen(GeneratedStairSpline target, string source)
    {
        if (!enableMinimalDebugLog || target == null)
            return;

        bool terminalValid =
            target.lastTerminalErrorMeters <=
            Mathf.Max(0.0001f, terminalValidationToleranceMeters);

        Debug.Log(
            $"[FUTURE SPLINE FROZEN] " +
            $"source={source} " +
            $"spline={target.sourceSplineIndex} section={target.sourceSectionIndex} " +
            $"knots={target.lastKnotCount} internalKnots={target.lastInternalKnotCount} " +
            $"renderEndU={target.lastRenderedEndU:F3} " +
            $"PFprogress={target.lastPfProgress01:P1} PTprogress={target.lastPtProgress01:P1} " +
            $"terminalOK={terminalValid} " +
            $"centerClearance={target.lastReferenceCenterClearance:F3}m " +
            $"incidentVN={target.lastIncidentNormalSpeed:F3}m/s " +
            $"incidentVB={target.lastIncidentSideSpeed:F3}m/s " +
            $"maxBridgeN={target.lastMaxBridgeNormal:F3}m " +
            $"maxWave={target.lastMaxWave:F3}m " +
            $"maxNormalOffset={target.lastMaxTotalNormalOffset:F3}m " +
            $"terminalError={target.lastTerminalErrorMeters:F5}m\n" +
            $"PF_GuideCenter={target.lastPFGuideCenter:F3}\n" +
            $"PF_Final={target.lastPF:F3}\n" +
            $"PT_Surface={target.lastPTSurface:F3}\n" +
            $"PT_CenterBase={target.lastPTCenterBase:F3}\n" +
            $"PT_Final={target.lastPTFinal:F3}\n" +
            $"PM={target.lastPM:F3}\n" +
            $"PL={target.lastPL:F3}",
            this);
    }

    void LogBuildFailureOnce(string code)
    {
        if (!logBuildFailureReason || string.IsNullOrEmpty(code))
            return;

        if (lastBuildFailureCode == code)
            return;

        lastBuildFailureCode = code;

        Debug.LogWarning(
            $"[FUTURE SPLINE BUILD SKIPPED] reason={code}",
            this);
    }

    /// <summary>
    /// Explicit turn hook for a turn controller with a cleaner boundary event than
    /// BallVisualSlopeDrive.IsTurnHandoffActive. A turn is accepted as a valid
    /// sample terminal only after Plane -> Stair sampling has begun.
    /// </summary>
    public void NotifyTurnBoundary()
    {
        manualTurnBoundaryPending = true;
    }

    /// <summary>
    /// Explicit invalidation hook for known abnormal physics states, such as the
    /// Equalizer becoming trapped by a real Collider or losing a trustworthy pose.
    /// </summary>
    public void InvalidateCurrentSample(string reason)
    {
        if (!IsCollectingFutureSample)
            return;

        RejectCurrentFutureSample(
            string.IsNullOrEmpty(reason) ? "ExternalInvalidation" : reason);
    }

    void UpdateIntegratedFutureSampleCollection(
        NearestKnotDetector.GuideFrame guide)
    {
        if (equalizerSync && equalizerSync.IsEmergencyVisualRecoveryActive)
        {
            if (IsCollectingFutureSample)
                RejectCurrentFutureSample("EmergencyVisualRecovery");

            // Do not restart a candidate in the middle of the same Stair after an
            // emergency. Wait for a fresh Plane -> Stair boundary.
            samplePlanePreRoll.Clear();
            samplePreviousGuideValid = guide.valid;
            samplePreviousWasSlope = guide.valid && guide.isSlope;
            samplePreviousTurnActive = ballVisualDrive && ballVisualDrive.IsTurnHandoffActive;
            manualTurnBoundaryPending = false;
            if (equalizerBody && IsFinite(equalizerBody.position))
            {
                previousSampleEqualizerPosition = equalizerBody.position;
                previousSampleEqualizerPositionValid = true;
            }
            return;
        }

        if (!guide.valid)
        {
            if (IsCollectingFutureSample)
                RejectCurrentFutureSample("GuideInvalid");

            samplePreviousGuideValid = false;
            return;
        }

        if (!TryBuildRawFutureSamplePoint(guide, out RawFutureSamplePoint raw))
        {
            if (IsCollectingFutureSample)
                RejectCurrentFutureSample("NonFiniteState");
            return;
        }

        if (!ValidateFutureSampleMotionContinuity(raw))
            return;

        bool turnActive = ballVisualDrive && ballVisualDrive.IsTurnHandoffActive;
        bool turnStarted =
            manualTurnBoundaryPending ||
            (turnActive && !samplePreviousTurnActive);

        manualTurnBoundaryPending = false;

        bool enteredSlope =
            samplePreviousGuideValid &&
            !samplePreviousWasSlope &&
            guide.isSlope;

        bool leftSlope =
            samplePreviousGuideValid &&
            samplePreviousWasSlope &&
            !guide.isSlope;

        if (futureSamplePhase == FutureSamplePhase.Idle)
        {
            if (!guide.isSlope)
                AppendFutureSamplePlanePreRoll(raw);

            if (enteredSlope)
                BeginFutureSampleCandidate(raw);
        }
        else
        {
            AppendCurrentFutureRawSample(raw);

            // User-defined alternative completion rule:
            // Plane -> Stair -> Turn is already a valid sample. The following
            // Plane and Stair are not required once the turn boundary is reached.
            if (turnStarted)
            {
                AcceptCurrentFutureSample(
                    FutureSampleCompletionKind.TurnBoundaryReached);
            }
            else if (
                futureSamplePhase == FutureSamplePhase.FirstStair &&
                leftSlope)
            {
                futureSamplePhase = FutureSamplePhase.NextPlane;
            }
            else if (
                futureSamplePhase == FutureSamplePhase.NextPlane &&
                enteredSlope)
            {
                AcceptCurrentFutureSample(
                    FutureSampleCompletionKind.NextStairReached);
            }
        }

        samplePreviousGuideValid = true;
        samplePreviousWasSlope = guide.isSlope;
        samplePreviousTurnActive = turnActive;
    }

    bool TryBuildRawFutureSamplePoint(
        NearestKnotDetector.GuideFrame guide,
        out RawFutureSamplePoint raw)
    {
        raw = default;

        if (!equalizerBody || !ballVisualBody)
            return false;

        Vector3 equalizerPosition = equalizerBody.position;
        Vector3 equalizerVelocity = equalizerBody.velocity;
        Vector3 ballPosition = ballVisualBody.position;
        Vector3 ballVelocity = ballVisualBody.velocity;

        if (!IsFinite(equalizerPosition) ||
            !IsFinite(equalizerVelocity) ||
            !IsFinite(ballPosition) ||
            !IsFinite(ballVelocity))
        {
            return false;
        }

        Vector3 tangent = NormalizeSafe(
            correspondSubject.MapDirection(guide.tangent),
            Vector3.forward);

        Vector3 normal = BuildInitialNormal(
            tangent,
            correspondSubject.MapDirection(guide.normal));

        Vector3 side = NormalizeSafe(
            Vector3.Cross(normal, tangent),
            Vector3.right);

        normal = NormalizeSafe(
            Vector3.Cross(tangent, side),
            normal);

        raw = new RawFutureSamplePoint
        {
            time = Time.fixedTime,
            equalizerPosition = equalizerPosition,
            equalizerVelocity = equalizerVelocity,
            ballPosition = ballPosition,
            ballVelocity = ballVelocity,
            guideCenter = correspondSubject.MapPoint(guide.point),
            tangent = tangent,
            normal = normal,
            side = side,
            slopeProgress01 = guide.isSlope
                ? Mathf.Clamp01(guide.sectionProgress01)
                : 0f,
            onSlope = guide.isSlope,
            splineIndex = guide.splineIndex,
            sectionIndex = guide.sectionIndex
        };

        return true;
    }

    bool ValidateFutureSampleMotionContinuity(RawFutureSamplePoint raw)
    {
        if (previousSampleEqualizerPositionValid)
        {
            float delta = Vector3.Distance(
                previousSampleEqualizerPosition,
                raw.equalizerPosition);

            if (delta > Mathf.Max(
                    0.01f,
                    maximumSampleSingleStepPositionJumpMeters))
            {
                if (IsCollectingFutureSample)
                {
                    RejectCurrentFutureSample(
                        $"PositionJump:{delta:F3}m");
                }

                previousSampleEqualizerPosition = raw.equalizerPosition;
                return false;
            }

            if (IsCollectingFutureSample &&
                delta <= Mathf.Max(
                    0f,
                    sampleStuckPositionDeltaThresholdMeters) &&
                raw.equalizerVelocity.magnitude <= Mathf.Max(
                    0f,
                    sampleStuckSpeedThresholdMetersPerSecond))
            {
                currentFutureSampleStuckSeconds += Time.fixedDeltaTime;

                if (currentFutureSampleStuckSeconds >= Mathf.Max(
                        0.05f,
                        sampleStuckDurationSeconds))
                {
                    RejectCurrentFutureSample(
                        "EqualizerStuckOrColliderTrapped");
                    previousSampleEqualizerPosition = raw.equalizerPosition;
                    return false;
                }
            }
            else
            {
                currentFutureSampleStuckSeconds = 0f;
            }
        }

        previousSampleEqualizerPosition = raw.equalizerPosition;
        previousSampleEqualizerPositionValid = true;
        return true;
    }

    void AppendFutureSamplePlanePreRoll(RawFutureSamplePoint raw)
    {
        AppendFutureSampleWithInterval(samplePlanePreRoll, raw);

        float oldestAllowed =
            raw.time - Mathf.Max(0f, samplePlanePreRollSeconds);

        int removeCount = 0;
        while (removeCount < samplePlanePreRoll.Count &&
               samplePlanePreRoll[removeCount].time < oldestAllowed)
        {
            removeCount++;
        }

        if (removeCount > 0)
            samplePlanePreRoll.RemoveRange(0, removeCount);
    }

    void BeginFutureSampleCandidate(RawFutureSamplePoint firstSlopePoint)
    {
        currentRawFutureSample.Clear();
        currentRawFutureSample.AddRange(samplePlanePreRoll);
        samplePlanePreRoll.Clear();

        futureSamplePhase = FutureSamplePhase.FirstStair;
        currentFutureSampleValid = true;
        lastFutureSampleInvalidReason = string.Empty;
        lastFutureSampleCompletionKind = FutureSampleCompletionKind.None;
        currentFutureSampleStuckSeconds = 0f;

        AppendCurrentFutureRawSample(firstSlopePoint);
    }

    void AppendCurrentFutureRawSample(RawFutureSamplePoint raw)
    {
        AppendFutureSampleWithInterval(currentRawFutureSample, raw);
        currentRawFutureSamplePointCount = currentRawFutureSample.Count;
    }

    void AppendFutureSampleWithInterval(
        List<RawFutureSamplePoint> destination,
        RawFutureSamplePoint raw)
    {
        float minInterval = Mathf.Max(0f, minimumSampleIntervalSeconds);

        if (destination.Count > 0 &&
            raw.time - destination[destination.Count - 1].time < minInterval)
        {
            return;
        }

        destination.Add(raw);
    }

    void AcceptCurrentFutureSample(
        FutureSampleCompletionKind completionKind)
    {
        if (!currentFutureSampleValid || currentRawFutureSample.Count < 2)
        {
            RejectCurrentFutureSample("InsufficientValidPoints");
            return;
        }

        FlattenCurrentFutureSample();
        AccumulateLatestValidSampleIntoSpatialTemplate();

        acceptedFutureSampleCount++;
        lastFutureSampleCompletionKind = completionKind;
        lastFutureSampleInvalidReason = string.Empty;

        ResetFutureSampleCandidateState(clearLatestValidSample: false);
    }

    void RejectCurrentFutureSample(string reason)
    {
        rejectedFutureSampleCount++;
        currentFutureSampleValid = false;
        lastFutureSampleInvalidReason = reason;
        lastFutureSampleCompletionKind = FutureSampleCompletionKind.None;
        latestFlattenedFutureSamplePointCount =
            latestValidFlattenedSample.Count;

        ResetFutureSampleCandidateState(clearLatestValidSample: false);
    }

    void ResetFutureSampleCandidateState(bool clearLatestValidSample)
    {
        currentRawFutureSample.Clear();
        currentRawFutureSamplePointCount = 0;
        futureSamplePhase = FutureSamplePhase.Idle;
        currentFutureSampleValid = false;
        currentFutureSampleStuckSeconds = 0f;
        samplePlanePreRoll.Clear();

        if (clearLatestValidSample)
        {
            latestValidFlattenedSample.Clear();
            latestFlattenedFutureSamplePointCount = 0;
        }
    }

    void ResetFutureSampleTrackingState()
    {
        samplePreviousGuideValid = false;
        samplePreviousWasSlope = false;
        samplePreviousTurnActive = false;
        manualTurnBoundaryPending = false;
        previousSampleEqualizerPositionValid = false;
        previousSampleEqualizerPosition = Vector3.zero;
    }

    void FlattenCurrentFutureSample()
    {
        latestValidFlattenedSample.Clear();
        if (currentRawFutureSample.Count < 2)
            return;

        float startTime = currentRawFutureSample[0].time;
        float endTime = currentRawFutureSample[currentRawFutureSample.Count - 1].time;
        float duration = Mathf.Max(Eps, endTime - startTime);

        int firstStairIndex = -1;
        float firstStairEntryN = 0f;
        int firstStairSpline = -1;
        int firstStairSection = -1;
        for (int i = 0; i < currentRawFutureSample.Count; i++)
        {
            RawFutureSamplePoint p = currentRawFutureSample[i];
            if (!p.onSlope)
                continue;

            firstStairIndex = i;
            firstStairSpline = p.splineIndex;
            firstStairSection = p.sectionIndex;
            firstStairEntryN = Vector3.Dot(
                p.equalizerPosition - p.guideCenter,
                p.normal);
            break;
        }

        float[] cumulativeS = new float[currentRawFutureSample.Count];
        float totalS = 0f;
        for (int i = 1; i < currentRawFutureSample.Count; i++)
        {
            RawFutureSamplePoint previous = currentRawFutureSample[i - 1];
            RawFutureSamplePoint current = currentRawFutureSample[i];
            Vector3 delta = current.equalizerPosition - previous.equalizerPosition;
            Vector3 tangent = NormalizeSafe(
                previous.tangent + current.tangent,
                current.tangent);
            totalS += Mathf.Max(0f, Vector3.Dot(delta, tangent));
            cumulativeS[i] = totalS;
        }

        float safeTotalS = Mathf.Max(Eps, totalS);

        for (int i = 0; i < currentRawFutureSample.Count; i++)
        {
            RawFutureSamplePoint raw = currentRawFutureSample[i];
            Vector3 equalizerDelta = raw.equalizerPosition - raw.guideCenter;
            Vector3 ballDelta = raw.ballPosition - raw.guideCenter;

            float equalizerN = Vector3.Dot(equalizerDelta, raw.normal);
            float equalizerVT = Vector3.Dot(raw.equalizerVelocity, raw.tangent);
            float equalizerVN = Vector3.Dot(raw.equalizerVelocity, raw.normal);
            bool isFirstStair =
                firstStairIndex >= 0 &&
                raw.onSlope &&
                raw.splineIndex == firstStairSpline &&
                raw.sectionIndex == firstStairSection;

            latestValidFlattenedSample.Add(new FlattenedSamplePoint
            {
                time01 = Mathf.Clamp01((raw.time - startTime) / duration),
                elapsedSeconds = raw.time - startTime,
                s01 = Mathf.Clamp01(cumulativeS[i] / safeTotalS),
                sMeters = cumulativeS[i],

                equalizerN = equalizerN,
                equalizerB = Vector3.Dot(equalizerDelta, raw.side),
                equalizerVT = equalizerVT,
                equalizerVN = equalizerVN,
                equalizerVB = Vector3.Dot(raw.equalizerVelocity, raw.side),
                equalizerDeltaNFromFirstStair = isFirstStair
                    ? equalizerN - firstStairEntryN
                    : 0f,
                equalizerDnDs = isFirstStair
                    ? Mathf.Clamp(
                        equalizerVN /
                        Mathf.Max(minimumLiveTangentialSpeed, Mathf.Abs(equalizerVT)),
                        -4f,
                        4f)
                    : 0f,

                ballN = Vector3.Dot(ballDelta, raw.normal),
                ballB = Vector3.Dot(ballDelta, raw.side),
                ballVT = Vector3.Dot(raw.ballVelocity, raw.tangent),
                ballVN = Vector3.Dot(raw.ballVelocity, raw.normal),
                ballVB = Vector3.Dot(raw.ballVelocity, raw.side),

                slopeProgress01 = raw.slopeProgress01,
                onSlope = raw.onSlope,
                splineIndex = raw.splineIndex,
                sectionIndex = raw.sectionIndex
            });
        }

        latestFlattenedFutureSamplePointCount = latestValidFlattenedSample.Count;
    }

    void AccumulateLatestValidSampleIntoSpatialTemplate()
    {
        EnsureSpatialTemplateStorage();
        lastSparseTemplateContributionCount = 0;
        if (latestValidFlattenedSample.Count < 2)
            return;

        int firstSlopeIndex = -1;
        int firstSlopeEndExclusive = -1;
        int firstSpline = -1;
        int firstSection = -1;

        for (int i = 0; i < latestValidFlattenedSample.Count; i++)
        {
            FlattenedSamplePoint p = latestValidFlattenedSample[i];

            if (firstSlopeIndex < 0)
            {
                if (!p.onSlope)
                    continue;

                firstSlopeIndex = i;
                firstSpline = p.splineIndex;
                firstSection = p.sectionIndex;
                continue;
            }

            // The template owns only the first Stair. Stop as soon as that Stair
            // ends; do not let the following Plane or next Stair contaminate it.
            if (!p.onSlope ||
                p.splineIndex != firstSpline ||
                p.sectionIndex != firstSection)
            {
                firstSlopeEndExclusive = i;
                break;
            }
        }

        if (firstSlopeIndex < 0)
            return;
        if (firstSlopeEndExclusive < 0)
            firstSlopeEndExclusive = latestValidFlattenedSample.Count;
        if (firstSlopeEndExclusive - firstSlopeIndex < 2)
            return;

        sparseTemplateSelectedIndices.Clear();
        sparseTemplateCandidateIndices.Clear();
        sparseTemplateCandidateScores.Clear();

        int baseCount = Mathf.Clamp(sparseTemplateBaseSamplesPerRun, 4, 16);
        for (int k = 0; k < baseCount; k++)
        {
            float targetProgress = k / (float)(baseCount - 1);
            int nearest = FindNearestFirstStairSampleIndex(
                firstSlopeIndex,
                firstSlopeEndExclusive,
                targetProgress);
            AddUniqueSparseSampleIndex(nearest);
        }

        // Extra samples are not uniformly dense. They are reserved for places where
        // dN/ds changes sharply, preserving repeatable Equalizer roughness without
        // importing every FixedUpdate sample.
        for (int i = firstSlopeIndex + 1; i < firstSlopeEndExclusive - 1; i++)
        {
            FlattenedSamplePoint previous = latestValidFlattenedSample[i - 1];
            FlattenedSamplePoint current = latestValidFlattenedSample[i];
            FlattenedSamplePoint next = latestValidFlattenedSample[i + 1];

            float slopeCurvature = Mathf.Abs(
                current.equalizerDnDs -
                0.5f * (previous.equalizerDnDs + next.equalizerDnDs));
            float slopeSwing = 0.5f * Mathf.Abs(
                next.equalizerDnDs - previous.equalizerDnDs);
            float roughness = slopeCurvature + slopeSwing;

            if (roughness < Mathf.Max(0f, roughnessEventThreshold))
                continue;

            int insert = sparseTemplateCandidateScores.Count;
            for (int j = 0; j < sparseTemplateCandidateScores.Count; j++)
            {
                if (roughness > sparseTemplateCandidateScores[j])
                {
                    insert = j;
                    break;
                }
            }
            sparseTemplateCandidateScores.Insert(insert, roughness);
            sparseTemplateCandidateIndices.Insert(insert, i);
        }

        int eventBudget = Mathf.Clamp(maximumRoughnessEventSamplesPerRun, 0, 12);
        int acceptedEvents = 0;
        for (int c = 0; c < sparseTemplateCandidateIndices.Count && acceptedEvents < eventBudget; c++)
        {
            int index = sparseTemplateCandidateIndices[c];
            float progress = latestValidFlattenedSample[index].slopeProgress01;
            bool tooClose = false;
            for (int j = 0; j < sparseTemplateSelectedIndices.Count; j++)
            {
                float other = latestValidFlattenedSample[sparseTemplateSelectedIndices[j]].slopeProgress01;
                if (Mathf.Abs(progress - other) <
                    Mathf.Max(0.005f, minimumRoughnessEventProgressSeparation01))
                {
                    tooClose = true;
                    break;
                }
            }

            if (tooClose)
                continue;

            AddUniqueSparseSampleIndex(index);
            acceptedEvents++;
        }

        int n = spatialTemplateBins.Length;
        float[] sumN = new float[n];
        float[] sumDnDs = new float[n];
        float[] sumRoughness = new float[n];
        int[] counts = new int[n];

        for (int j = 0; j < sparseTemplateSelectedIndices.Count; j++)
        {
            int index = sparseTemplateSelectedIndices[j];
            FlattenedSamplePoint p = latestValidFlattenedSample[index];
            int bin = Mathf.Clamp(
                Mathf.RoundToInt(Mathf.Clamp01(p.slopeProgress01) * (n - 1)),
                0,
                n - 1);

            float roughness = EstimateFlattenedRoughness(index, firstSlopeIndex, firstSlopeEndExclusive);
            sumN[bin] += p.equalizerDeltaNFromFirstStair;
            sumDnDs[bin] += p.equalizerDnDs;
            sumRoughness[bin] += roughness;
            counts[bin]++;
        }

        bool contributed = false;
        float residualLimit = Mathf.Max(0.02f, maximumTemplateNormalResidualPerRunMeters);
        float maximumAlpha = Mathf.Clamp(maximumTemplateLearningRate, 0.01f, 0.5f);

        for (int i = 0; i < n; i++)
        {
            if (counts[i] <= 0)
                continue;

            float runMeanN = sumN[i] / counts[i];
            float runMeanDnDs = sumDnDs[i] / counts[i];
            float runMeanRoughness = sumRoughness[i] / counts[i];
            SpatialTemplateBin b = spatialTemplateBins[i];

            if (b.runs <= 0)
            {
                b.meanDeltaN = runMeanN;
                b.meanDnDs = runMeanDnDs;
                b.meanRoughness = runMeanRoughness;
                b.m2DeltaN = 0f;
                b.runs = 1;
            }
            else
            {
                int next = b.runs + 1;
                float alpha = Mathf.Min(maximumAlpha, 1f / next);
                float rawResidual = runMeanN - b.meanDeltaN;
                float clippedResidual = Mathf.Clamp(rawResidual, -residualLimit, residualLimit);
                b.meanDeltaN += clippedResidual * alpha;
                b.meanDnDs += (runMeanDnDs - b.meanDnDs) * alpha;
                b.meanRoughness += (runMeanRoughness - b.meanRoughness) * alpha;
                b.m2DeltaN = Mathf.Lerp(
                    b.m2DeltaN,
                    clippedResidual * clippedResidual,
                    alpha);
                b.runs = next;
            }

            spatialTemplateBins[i] = b;
            contributed = true;
            lastSparseTemplateContributionCount++;
        }

        if (contributed)
            spatialTemplateAcceptedRuns++;

        UpdateSpatialTemplateDiagnostics();
    }

    int FindNearestFirstStairSampleIndex(
        int startIndex,
        int endExclusive,
        float targetProgress01)
    {
        int best = startIndex;
        float bestError = float.PositiveInfinity;
        for (int i = startIndex; i < endExclusive; i++)
        {
            float error = Mathf.Abs(
                latestValidFlattenedSample[i].slopeProgress01 -
                Mathf.Clamp01(targetProgress01));
            if (error < bestError)
            {
                bestError = error;
                best = i;
            }
        }
        return best;
    }

    void AddUniqueSparseSampleIndex(int index)
    {
        if (index < 0 || index >= latestValidFlattenedSample.Count)
            return;
        for (int i = 0; i < sparseTemplateSelectedIndices.Count; i++)
        {
            if (sparseTemplateSelectedIndices[i] == index)
                return;
        }
        sparseTemplateSelectedIndices.Add(index);
    }

    float EstimateFlattenedRoughness(
        int index,
        int firstSlopeIndex,
        int firstSlopeEndExclusive)
    {
        if (index <= firstSlopeIndex || index >= firstSlopeEndExclusive - 1)
            return 0f;

        FlattenedSamplePoint previous = latestValidFlattenedSample[index - 1];
        FlattenedSamplePoint current = latestValidFlattenedSample[index];
        FlattenedSamplePoint next = latestValidFlattenedSample[index + 1];

        float slopeCurvature = Mathf.Abs(
            current.equalizerDnDs -
            0.5f * (previous.equalizerDnDs + next.equalizerDnDs));
        float slopeSwing = 0.5f * Mathf.Abs(
            next.equalizerDnDs - previous.equalizerDnDs);
        return slopeCurvature + slopeSwing;
    }

    void RemoveInactiveSplines(NearestKnotDetector.GuideFrame guide)
    {
        long keepKey = lastPredictedKey;

        if (guide.isSlope && guide.sectionIndex >= 0)
            keepKey = MakeKey(guide.splineIndex, guide.sectionIndex);

        List<long> remove = null;

        foreach (KeyValuePair<long, GeneratedStairSpline> pair in generated)
        {
            if (pair.Key == keepKey)
                continue;

            if (remove == null)
                remove = new List<long>();

            remove.Add(pair.Key);
        }

        if (remove == null)
            return;

        for (int i = 0; i < remove.Count; i++)
        {
            long key = remove[i];
            if (generated.TryGetValue(key, out GeneratedStairSpline value) &&
                value != null &&
                value.gameObject)
            {
                Destroy(value.gameObject);
            }

            generated.Remove(key);
        }
    }

    // ---------------------------------------------------------------------
    // IDirectionFieldTeacher
    // ---------------------------------------------------------------------

    public bool TryEvaluateDirectionAtNormalizedU(
        int sourceSplineIndex,
        int sourceSectionIndex,
        float normalizedU,
        out Vector3 pointPhysics,
        out Vector3 tangentPhysics)
    {
        return TryEvaluateGeneratedDirectionPhysicsAtNormalizedU(
            sourceSplineIndex,
            sourceSectionIndex,
            normalizedU,
            out pointPhysics,
            out tangentPhysics);
    }

    public bool TryGetDirectionAtArcProgress(
        int sourceSplineIndex,
        int sourceSectionIndex,
        float normalizedArcProgress,
        int arcSamples,
        out float teacherU,
        out Vector3 pointPhysics,
        out Vector3 tangentPhysics)
    {
        return TryGetGeneratedDirectionPhysicsAtArcProgress(
            sourceSplineIndex,
            sourceSectionIndex,
            normalizedArcProgress,
            arcSamples,
            out teacherU,
            out pointPhysics,
            out tangentPhysics);
    }

    public bool TryProjectDirectionLocalNewton(
        int sourceSplineIndex,
        int sourceSectionIndex,
        Vector3 probePointPhysics,
        float seedU,
        float halfWindowU,
        int iterations,
        out float projectedU,
        out Vector3 pointPhysics,
        out Vector3 tangentPhysics,
        out float euclideanDistanceMeters)
    {
        return TryProjectGeneratedDirectionPhysicsLocalNewton(
            sourceSplineIndex,
            sourceSectionIndex,
            probePointPhysics,
            seedU,
            halfWindowU,
            iterations,
            out projectedU,
            out pointPhysics,
            out tangentPhysics,
            out euclideanDistanceMeters);
    }

    /// <summary>
    /// Direction-teacher seam for EqualizerFutureSpline.
    /// Evaluates only generated Spline geometry in canonical Physics space.
    /// No BallVisualEqualizer Rigidbody/phase/boundary state is exposed.
    /// </summary>
    public bool TryEvaluateGeneratedDirectionPhysicsAtNormalizedU(
        int sourceSplineIndex,
        int sourceSectionIndex,
        float normalizedU,
        out Vector3 referencePointPhysics,
        out Vector3 referenceTangentPhysics)
    {
        referencePointPhysics = Vector3.zero;
        referenceTangentPhysics = Vector3.forward;

        if (!correspondSubject)
            return false;

        long key =
            MakeKey(
                sourceSplineIndex,
                sourceSectionIndex);

        if (!generated.TryGetValue(
                key,
                out GeneratedStairSpline target) ||
            target == null ||
            !target.container ||
            target.container.Spline == null ||
            target.container.Spline.Count < 2)
        {
            return false;
        }

        if (!TryEvaluateGeneratedSplineWorldReadOnly(
                target.container,
                Mathf.Clamp01(normalizedU),
                out Vector3 pointVisual,
                out Vector3 tangentVisual))
        {
            return false;
        }

        referencePointPhysics =
            correspondSubject.InverseMapPoint(
                pointVisual);

        referenceTangentPhysics =
            NormalizeSafe(
                correspondSubject.InverseMapDirection(
                    tangentVisual),
                Vector3.forward);

        return
            IsFinite(referencePointPhysics) &&
            IsFinite(referenceTangentPhysics) &&
            referenceTangentPhysics.sqrMagnitude > Eps;
    }

    /// <summary>
    /// Maps normalized arc progress [0,1] to the generated teacher's raw Bezier U.
    /// This is used only as a topology-preserving seed for local projection.
    /// </summary>
    public bool TryGetGeneratedDirectionPhysicsAtArcProgress(
        int sourceSplineIndex,
        int sourceSectionIndex,
        float normalizedArcProgress,
        int arcSamples,
        out float referenceU,
        out Vector3 referencePointPhysics,
        out Vector3 referenceTangentPhysics)
    {
        referenceU = 0f;
        referencePointPhysics = Vector3.zero;
        referenceTangentPhysics = Vector3.forward;

        if (!correspondSubject)
            return false;

        long key =
            MakeKey(
                sourceSplineIndex,
                sourceSectionIndex);

        if (!generated.TryGetValue(
                key,
                out GeneratedStairSpline target) ||
            target == null ||
            !target.container ||
            target.container.Spline == null ||
            target.container.Spline.Count < 2)
        {
            return false;
        }

        int samples =
            Mathf.Clamp(
                arcSamples,
                16,
                256);

        if (!TryEvaluateGeneratedSplineWorldReadOnly(
                target.container,
                0f,
                out Vector3 previousVisual,
                out _))
        {
            return false;
        }

        Vector3 previousPhysics =
            correspondSubject.InverseMapPoint(
                previousVisual);

        float totalLength = 0f;

        for (int i = 1; i <= samples; i++)
        {
            float u =
                i / (float)samples;

            if (!TryEvaluateGeneratedSplineWorldReadOnly(
                    target.container,
                    u,
                    out Vector3 pointVisual,
                    out _))
            {
                return false;
            }

            Vector3 pointPhysics =
                correspondSubject.InverseMapPoint(
                    pointVisual);

            totalLength +=
                Vector3.Distance(
                    previousPhysics,
                    pointPhysics);

            previousPhysics =
                pointPhysics;
        }

        if (totalLength <= Eps ||
            !IsFiniteScalar(totalLength))
        {
            return false;
        }

        float targetLength =
            Mathf.Clamp01(
                normalizedArcProgress) *
            totalLength;

        if (!TryEvaluateGeneratedSplineWorldReadOnly(
                target.container,
                0f,
                out previousVisual,
                out _))
        {
            return false;
        }

        previousPhysics =
            correspondSubject.InverseMapPoint(
                previousVisual);

        float accumulated = 0f;
        float previousU = 0f;

        for (int i = 1; i <= samples; i++)
        {
            float currentU =
                i / (float)samples;

            if (!TryEvaluateGeneratedSplineWorldReadOnly(
                    target.container,
                    currentU,
                    out Vector3 currentVisual,
                    out _))
            {
                return false;
            }

            Vector3 currentPhysics =
                correspondSubject.InverseMapPoint(
                    currentVisual);

            float segmentLength =
                Vector3.Distance(
                    previousPhysics,
                    currentPhysics);

            if (accumulated + segmentLength >= targetLength ||
                i == samples)
            {
                float segmentT =
                    segmentLength > Eps
                        ? Mathf.Clamp01(
                            (targetLength - accumulated) /
                            segmentLength)
                        : 0f;

                referenceU =
                    Mathf.Lerp(
                        previousU,
                        currentU,
                        segmentT);

                return
                    TryEvaluateGeneratedDirectionPhysicsAtNormalizedU(
                        sourceSplineIndex,
                        sourceSectionIndex,
                        referenceU,
                        out referencePointPhysics,
                        out referenceTangentPhysics);
            }

            accumulated += segmentLength;
            previousPhysics = currentPhysics;
            previousU = currentU;
        }

        return false;
    }

    /// <summary>
    /// Refines a teacher correspondence around an arc-length seed by minimizing
    /// 1/2 |C(u)-P|^2 with a local Newton step. The search is clamped to a small
    /// U window so correspondence cannot jump to another distant wave.
    /// </summary>
    public bool TryProjectGeneratedDirectionPhysicsLocalNewton(
        int sourceSplineIndex,
        int sourceSectionIndex,
        Vector3 probePointPhysics,
        float seedU,
        float halfWindowU,
        int iterations,
        out float projectedU,
        out Vector3 referencePointPhysics,
        out Vector3 referenceTangentPhysics,
        out float referenceDistanceMeters)
    {
        projectedU = Mathf.Clamp01(seedU);
        referencePointPhysics = Vector3.zero;
        referenceTangentPhysics = Vector3.forward;
        referenceDistanceMeters = float.PositiveInfinity;

        if (!correspondSubject)
            return false;

        long key =
            MakeKey(
                sourceSplineIndex,
                sourceSectionIndex);

        if (!generated.TryGetValue(
                key,
                out GeneratedStairSpline target) ||
            target == null ||
            !target.container ||
            target.container.Spline == null ||
            target.container.Spline.Count < 2)
        {
            return false;
        }

        float window =
            Mathf.Clamp(
                halfWindowU,
                0.005f,
                0.50f);

        float lower =
            Mathf.Max(
                0f,
                projectedU - window);

        float upper =
            Mathf.Min(
                1f,
                projectedU + window);

        int count =
            Mathf.Clamp(
                iterations,
                1,
                12);

        for (int iteration = 0;
             iteration < count;
             iteration++)
        {
            if (!TryEvaluateGeneratedSplinePhysicsDifferentialReadOnly(
                    target.container,
                    projectedU,
                    out Vector3 pointPhysics,
                    out Vector3 firstDerivativePhysics,
                    out Vector3 secondDerivativePhysics))
            {
                return false;
            }

            Vector3 residual =
                pointPhysics -
                probePointPhysics;

            float numerator =
                Vector3.Dot(
                    residual,
                    firstDerivativePhysics);

            float denominator =
                firstDerivativePhysics.sqrMagnitude +
                Vector3.Dot(
                    residual,
                    secondDerivativePhysics);

            if (!IsFiniteScalar(denominator) ||
                Mathf.Abs(denominator) <= Eps)
            {
                break;
            }

            float step =
                numerator /
                denominator;

            step =
                Mathf.Clamp(
                    step,
                    -0.5f * window,
                    0.5f * window);

            float nextU =
                Mathf.Clamp(
                    projectedU - step,
                    lower,
                    upper);

            if (Mathf.Abs(nextU - projectedU) <= 0.00001f)
            {
                projectedU = nextU;
                break;
            }

            projectedU = nextU;
        }

        if (!TryEvaluateGeneratedDirectionPhysicsAtNormalizedU(
                sourceSplineIndex,
                sourceSectionIndex,
                projectedU,
                out referencePointPhysics,
                out referenceTangentPhysics))
        {
            return false;
        }

        referenceDistanceMeters =
            Vector3.Distance(
                probePointPhysics,
                referencePointPhysics);

        return
            IsFiniteScalar(projectedU) &&
            IsFiniteScalar(referenceDistanceMeters);
    }

    bool TryEvaluateGeneratedSplinePhysicsDifferentialReadOnly(
        SplineContainer container,
        float normalizedU,
        out Vector3 pointPhysics,
        out Vector3 firstDerivativePhysics,
        out Vector3 secondDerivativePhysics)
    {
        pointPhysics = Vector3.zero;
        firstDerivativePhysics = Vector3.zero;
        secondDerivativePhysics = Vector3.zero;

        if (!correspondSubject ||
            !TryEvaluateGeneratedSplineWorldDifferentialReadOnly(
                container,
                normalizedU,
                out Vector3 pointVisual,
                out Vector3 firstDerivativeVisual,
                out Vector3 secondDerivativeVisual))
        {
            return false;
        }

        pointPhysics =
            correspondSubject.InverseMapPoint(
                pointVisual);

        firstDerivativePhysics =
            correspondSubject.InverseMapDirection(
                firstDerivativeVisual);

        secondDerivativePhysics =
            correspondSubject.InverseMapDirection(
                secondDerivativeVisual);

        return
            IsFinite(pointPhysics) &&
            IsFinite(firstDerivativePhysics) &&
            IsFinite(secondDerivativePhysics) &&
            firstDerivativePhysics.sqrMagnitude > Eps;
    }

    static bool TryEvaluateGeneratedSplineWorldDifferentialReadOnly(
        SplineContainer container,
        float normalizedU,
        out Vector3 worldPoint,
        out Vector3 worldFirstDerivative,
        out Vector3 worldSecondDerivative)
    {
        worldPoint = Vector3.zero;
        worldFirstDerivative = Vector3.zero;
        worldSecondDerivative = Vector3.zero;

        if (!container ||
            container.Spline == null ||
            container.Spline.Count < 2)
        {
            return false;
        }

        Spline spline =
            container.Spline;

        int segmentCount =
            spline.Count - 1;

        float clampedU =
            Mathf.Clamp01(
                normalizedU);

        float scaled =
            clampedU *
            segmentCount;

        int segmentIndex =
            Mathf.Min(
                segmentCount - 1,
                Mathf.FloorToInt(
                    scaled));

        float t =
            clampedU >= 1f
                ? 1f
                : scaled - segmentIndex;

        BezierKnot a =
            spline[segmentIndex];

        BezierKnot b =
            spline[segmentIndex + 1];

        Vector3 p0 =
            ToVector3ReadOnly(
                a.Position);

        Vector3 p1 =
            p0 +
            ToVector3ReadOnly(
                a.TangentOut);

        Vector3 p3 =
            ToVector3ReadOnly(
                b.Position);

        Vector3 p2 =
            p3 +
            ToVector3ReadOnly(
                b.TangentIn);

        float omt =
            1f - t;

        float omt2 =
            omt * omt;

        float t2 =
            t * t;

        Vector3 localPoint =
            omt2 * omt * p0 +
            3f * omt2 * t * p1 +
            3f * omt * t2 * p2 +
            t2 * t * p3;

        Vector3 localFirstDerivativeT =
            3f * omt2 * (p1 - p0) +
            6f * omt * t * (p2 - p1) +
            3f * t2 * (p3 - p2);

        Vector3 localSecondDerivativeT =
            6f * omt * (p2 - 2f * p1 + p0) +
            6f * t * (p3 - 2f * p2 + p1);

        float duScale =
            segmentCount;

        Vector3 localFirstDerivativeU =
            localFirstDerivativeT *
            duScale;

        Vector3 localSecondDerivativeU =
            localSecondDerivativeT *
            duScale *
            duScale;

        worldPoint =
            container.transform.TransformPoint(
                localPoint);

        worldFirstDerivative =
            container.transform.TransformVector(
                localFirstDerivativeU);

        worldSecondDerivative =
            container.transform.TransformVector(
                localSecondDerivativeU);

        return
            IsFinite(worldPoint) &&
            IsFinite(worldFirstDerivative) &&
            IsFinite(worldSecondDerivative) &&
            worldFirstDerivative.sqrMagnitude > Eps;
    }

    /// <summary>
    /// EqualizerFutureSpline direction-only seam.
    /// Returns only generated Spline geometry in the canonical Physics frame.
    /// No BallVisualEqualizer Rigidbody/phase/boundary state is exposed.
    /// </summary>
    public bool TryGetGeneratedDirectionPhysicsForClosestPoint(
        int sourceSplineIndex,
        int sourceSectionIndex,
        Vector3 probePointPhysics,
        out Vector3 referencePointPhysics,
        out Vector3 referenceTangentPhysics,
        out float referenceDistanceMeters)
    {
        referencePointPhysics = Vector3.zero;
        referenceTangentPhysics = Vector3.forward;
        referenceDistanceMeters = float.PositiveInfinity;

        if (!correspondSubject)
            return false;

        long key = MakeKey(sourceSplineIndex, sourceSectionIndex);

        if (!generated.TryGetValue(key, out GeneratedStairSpline target) ||
            target == null ||
            !target.container ||
            target.container.Spline == null ||
            target.container.Spline.Count < 2)
        {
            return false;
        }

        const int coarseSamples = 64;
        float bestDistanceSqr = float.PositiveInfinity;
        float bestU = 0f;
        Vector3 bestPointPhysics = Vector3.zero;
        Vector3 bestTangentPhysics = Vector3.forward;

        for (int i = 0; i <= coarseSamples; i++)
        {
            float u = i / (float)coarseSamples;

            if (!TryEvaluateGeneratedSplineWorldReadOnly(
                    target.container,
                    u,
                    out Vector3 pointVisual,
                    out Vector3 tangentVisual))
            {
                continue;
            }

            Vector3 pointPhysics = correspondSubject.InverseMapPoint(pointVisual);
            Vector3 tangentPhysics = correspondSubject.InverseMapDirection(tangentVisual);
            float distanceSqr = (pointPhysics - probePointPhysics).sqrMagnitude;

            if (distanceSqr >= bestDistanceSqr)
                continue;

            bestDistanceSqr = distanceSqr;
            bestU = u;
            bestPointPhysics = pointPhysics;
            bestTangentPhysics = tangentPhysics;
        }

        if (float.IsInfinity(bestDistanceSqr))
            return false;

        float coarseStep = 1f / coarseSamples;
        float lower = Mathf.Max(0f, bestU - coarseStep);
        float upper = Mathf.Min(1f, bestU + coarseStep);

        // Small local refinement only. This API is called once per direction rebuild,
        // not as a per-frame motion controller.
        for (int iteration = 0; iteration < 5; iteration++)
        {
            float uA = Mathf.Lerp(lower, upper, 1f / 3f);
            float uB = Mathf.Lerp(lower, upper, 2f / 3f);

            float dA = EvaluateGeneratedDirectionDistanceSqrReadOnly(
                target.container,
                probePointPhysics,
                uA);

            float dB = EvaluateGeneratedDirectionDistanceSqrReadOnly(
                target.container,
                probePointPhysics,
                uB);

            if (dA <= dB)
                upper = uB;
            else
                lower = uA;
        }

        float refinedU = 0.5f * (lower + upper);

        if (TryEvaluateGeneratedSplineWorldReadOnly(
                target.container,
                refinedU,
                out Vector3 refinedPointVisual,
                out Vector3 refinedTangentVisual))
        {
            Vector3 refinedPointPhysics = correspondSubject.InverseMapPoint(refinedPointVisual);
            float refinedDistanceSqr = (refinedPointPhysics - probePointPhysics).sqrMagnitude;

            if (refinedDistanceSqr < bestDistanceSqr)
            {
                bestDistanceSqr = refinedDistanceSqr;
                bestPointPhysics = refinedPointPhysics;
                bestTangentPhysics = correspondSubject.InverseMapDirection(refinedTangentVisual);
            }
        }

        referencePointPhysics = bestPointPhysics;
        referenceTangentPhysics = NormalizeSafe(bestTangentPhysics, Vector3.forward);
        referenceDistanceMeters = Mathf.Sqrt(bestDistanceSqr);

        return
            IsFinite(referencePointPhysics) &&
            IsFinite(referenceTangentPhysics) &&
            referenceTangentPhysics.sqrMagnitude > Eps;
    }

    float EvaluateGeneratedDirectionDistanceSqrReadOnly(
        SplineContainer container,
        Vector3 probePointPhysics,
        float normalizedU)
    {
        if (!TryEvaluateGeneratedSplineWorldReadOnly(
                container,
                normalizedU,
                out Vector3 pointVisual,
                out _))
        {
            return float.PositiveInfinity;
        }

        Vector3 pointPhysics = correspondSubject.InverseMapPoint(pointVisual);
        return (pointPhysics - probePointPhysics).sqrMagnitude;
    }

    static bool TryEvaluateGeneratedSplineWorldReadOnly(
        SplineContainer container,
        float normalizedU,
        out Vector3 worldPoint,
        out Vector3 worldTangent)
    {
        worldPoint = Vector3.zero;
        worldTangent = Vector3.forward;

        if (!container ||
            container.Spline == null ||
            container.Spline.Count < 2)
        {
            return false;
        }

        Spline spline = container.Spline;
        int segmentCount = spline.Count - 1;

        float scaled = Mathf.Clamp01(normalizedU) * segmentCount;
        int segmentIndex = Mathf.Min(segmentCount - 1, Mathf.FloorToInt(scaled));
        float localT = normalizedU >= 1f ? 1f : scaled - segmentIndex;

        BezierKnot a = spline[segmentIndex];
        BezierKnot b = spline[segmentIndex + 1];

        Vector3 p0 = ToVector3ReadOnly(a.Position);
        Vector3 p1 = p0 + ToVector3ReadOnly(a.TangentOut);
        Vector3 p3 = ToVector3ReadOnly(b.Position);
        Vector3 p2 = p3 + ToVector3ReadOnly(b.TangentIn);

        float t = Mathf.Clamp01(localT);
        float omt = 1f - t;
        float omt2 = omt * omt;
        float t2 = t * t;

        Vector3 localPoint =
            omt2 * omt * p0 +
            3f * omt2 * t * p1 +
            3f * omt * t2 * p2 +
            t2 * t * p3;

        Vector3 localDerivative =
            3f * omt2 * (p1 - p0) +
            6f * omt * t * (p2 - p1) +
            3f * t2 * (p3 - p2);

        worldPoint = container.transform.TransformPoint(localPoint);
        worldTangent = container.transform.TransformVector(localDerivative);

        if (worldTangent.sqrMagnitude <= Eps)
            worldTangent = container.transform.TransformVector(p3 - p0);

        return
            IsFinite(worldPoint) &&
            IsFinite(worldTangent) &&
            worldTangent.sqrMagnitude > Eps;
    }

    static Vector3 ToVector3ReadOnly(float3 value)
    {
        return new Vector3(value.x, value.y, value.z);
    }

    static Vector3 BuildInitialNormal(
        Vector3 tangent,
        Vector3 preferredNormal)
    {
        tangent = NormalizeSafe(tangent, Vector3.forward);

        Vector3 normal = Vector3.ProjectOnPlane(
            preferredNormal,
            tangent);

        if (normal.sqrMagnitude <= Eps)
            normal = Vector3.ProjectOnPlane(Vector3.up, tangent);

        if (normal.sqrMagnitude <= Eps)
            normal = Vector3.ProjectOnPlane(Vector3.right, tangent);

        return NormalizeSafe(normal, Vector3.up);
    }

    static Vector3 NormalizeSafe(Vector3 value, Vector3 fallback)
    {
        if (value.sqrMagnitude <= Eps || !IsFinite(value))
            value = fallback;

        if (value.sqrMagnitude <= Eps)
            value = Vector3.forward;

        return value.normalized;
    }

    static bool IsFinite(Vector3 value)
    {
        return
            IsFiniteScalar(value.x) &&
            IsFiniteScalar(value.y) &&
            IsFiniteScalar(value.z);
    }

    static bool IsFiniteScalar(float value)
    {
        return !float.IsNaN(value) && !float.IsInfinity(value);
    }

    static bool Approximately(float a, float b)
    {
        return Mathf.Abs(a - b) <= 0.0005f;
    }

    static float3 ToFloat3(Vector3 value)
    {
        return new float3(value.x, value.y, value.z);
    }

    static long MakeKey(int splineIndex, int sectionIndex)
    {
        return ((long)splineIndex << 32) | (uint)sectionIndex;
    }
}
