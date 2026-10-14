using System;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Splines;

/// <summary>
/// Independent A+B (three orthogonal stage rails) / A+b (two wave splines).
/// V24: InSubject Knot observations + comoving terrain-frame acceleration wave drive.
/// One class only, with an automatically installed CONTACT_ONLY instance on InSubject.
/// Signed entry and exit corners are independently validated and
/// the published three rails are immutable until the next flat.
/// Reads visual StairWay*_Render / ArcSlab*_Render and an injected Rigidbody.
/// Independently drives a physics-free visual Transform. No physics driving.
/// Five owned SplineContainers remain isolated from existing wave systems.
/// Knot anchors are a bounded read-only sampling cache; zero knots are created for sonars.
/// The original ExampleBVE is never used as wave input; own output is renderer-only.
/// The original geometry and SplineContainer writers remain intact.
/// </summary>
[DefaultExecutionOrder(12440), DisallowMultipleComponent]
public sealed class IndependentFiveSplineWeaver : MonoBehaviour
{
    const float E = .00001f;
    const int Out = 0, Center = 1, Inside = 2, Observed = 3, Future = 4, Count = 5;
    static readonly string[] Names = { "OutCorner", "CenterCorn", "InCorner", "EqualizerObservedSpline", "EqualizerFutureBallSpline" };
    static readonly Color[] Colors = {
        new Color(1f,.55f,.12f), new Color(.1f,.9f,1f), new Color(.32f,1f,.28f),
        new Color(1f,.22f,.76f), new Color(.35f,.48f,1f)
    };

    [Header("Read-only display and renderer geometry")]
    [SerializeField] Transform displayedBall;         // Optional visual assigned in Inspector
    [SerializeField] Transform generatedVisualRoot;
    [Header("V21 fixed standalone authority (no ExampleBVE wave input)")]
    [SerializeField] string visualMode = "UNRESOLVED";
    [Header("Standalone visual output (only when Main BVE is unavailable)")]
    [SerializeField] Transform visualMotionSource;   // Optional: VisualPlayerRoot/subject, READ ONLY
    [SerializeField] bool createOwnVisualBall = true;
    [SerializeField] Material ownBallMaterial;
    [SerializeField] Mesh ownBallMesh;
    [SerializeField] bool debugVisualBinding;
    [SerializeField] string visualBindingStatus = "UNRESOLVED";
    [SerializeField, Range(.05f,1f)] float rendererScanSeconds = .25f;
    [SerializeField, Min(.5f)] float fallbackPieceLength = 9.9f;
    [SerializeField] float renderLocalHeight;


    [Header("Physics source (READ ONLY; SlopeStickCore keeps authority)")]
    [SerializeField] Rigidbody inSubject; // REQUIRED for non-inertial observation
    [SerializeField] Transform subjectFallback;
    [SerializeField] Transform physicsFrame, visualFrame;
    [SerializeField] LayerMask groundMask = ~0;

    [Header("V21: InSubject observation via transported Knot frame")]
    [SerializeField] bool driveVisualWithKnotSonar = true;
    [SerializeField] bool drawKnotSonarGizmos;
    [SerializeField, Range(.3f, 8f)] float knotSonarRadius = 2.75f;
    [SerializeField, Min(.05f)] float probeSpacing = .45f;
    [SerializeField, Min(.1f)] float probeHeight = 2f;
    [SerializeField, Min(.2f)] float probeDepth = 8f;
    [SerializeField] float bveInitialCY;
    [SerializeField, Min(.01f)] float entryBlendSeconds = .10f;
    [Header("Downhill entry and section diagnostics (not waveform control)")]
    // Separate from A+b Future spline settings: tuning a prediction cannot
    // silently change the actual rendered ball's physical wave program.
    [SerializeField, Min(.1f)] float bveSectionLength = 9.9f;
    [SerializeField, Range(.05f,.95f)] float bveFBoundary = 3f/7f;
    [SerializeField, Min(.1f)] float bveEntrySlopeDegrees = 12f;
    [SerializeField, Min(0f)] float bveMinimumEntryPlanarSpeed = 2f;
    [SerializeField, Min(.1f)] float bveTeleportThreshold = 2.5f;
    [SerializeField] bool bveAutoBeginOnDownhill = true;
    [SerializeField] bool bveAutoEndAtSection = true;
    [SerializeField] bool debugBveParityEvents = true;
    [SerializeField, Min(.01f)] float visualRadius = .5f;

    [Header("V21 non-inertial wave authority (STANDALONE only)")]
    [SerializeField, Range(.05f,1f)] float trackingAlpha = .65f;
    [SerializeField, Range(.001f,.7f)] float trackingBeta = .10f;
    [SerializeField, Min(.2f)] float nonInertialFrequencyHz = 1.4f;
    [SerializeField, Range(.2f,1.5f)] float nonInertialDampingRatio = .48f;
    [SerializeField, Range(0f,2f)] float nonInertialObservationGain = 1.0f;
    [SerializeField, Range(0f,1f)] float nonInertialImpactGain = .14f;
    [SerializeField, Range(0f,1f)] float nonInertialEntryGain = .40f;
    [SerializeField, Min(.1f)] float nonInertialMaxOffset = 3.5f;
    [SerializeField, Min(1f)] float nonInertialMaxSpeed = 20f;

    [Header("V22: real InSubject contact -> Knot-normal excitation")]
    [Tooltip("One-class collision listener is attached automatically to the InSubject GameObject. The Rigidbody is never written.")]
    [SerializeField] bool enablePhysicalContactExcitation = true;
    [Tooltip("Minimum contact relative-normal velocity (m/s) to reject steady support/friction impulses.")]
    [SerializeField, Min(0f)] float contactMinApproachSpeed = .12f;
    [Tooltip("Minimum collision impulse / Rigidbody mass (m/s) before it can excite the display.")]
    [SerializeField, Min(0f)] float contactMinImpulseSpeed = .08f;
    [Tooltip("Minimum alignment between actual contact normal and the Knot observer normal.")]
    [SerializeField, Range(0f,1f)] float contactMinNormalAlignment = .22f;
    [Tooltip("Fraction of measured collision delta-v transferred into visual-only oscillator velocity.")]
    [SerializeField, Range(0f,2f)] float contactWaveVelocityGain = .55f;
    [Tooltip("Limit the equivalent delta-v from collision impulses during one physics step.")]
    [SerializeField, Min(.1f)] float contactMaxVelocityKick = 3.5f;
    [Tooltip("OnCollisionStay is an impact only if the impulse rises above its recent steady-contact baseline by this ratio.")]
    [SerializeField, Range(1.05f,6f)] float contactImpulseRiseRatio = 1.8f;
    [Tooltip("Minimum absolute increase of impulse/mass (m/s) above steady-contact baseline.")]
    [SerializeField, Min(0f)] float contactImpulseRiseSpeed = .24f;
    [Tooltip("Minimum gap (s) between two impact excitations from the same collision partner.")]
    [SerializeField, Range(0f,.25f)] float contactImpactCooldown = .06f;
    [Tooltip("Soft saturation avoids flattening different strong impacts at one identical maximum kick.")]
    [SerializeField] bool contactSoftLimitKick = true;
    [SerializeField] bool debugContactExcitation = false;
    [Tooltip("Log one detailed before/after-solver trace for each qualified contact kick.")]
    [SerializeField] bool debugContactTransfer = false;
    [Tooltip("Per Wave 1/2/3 then 2/3: oscillator energy budget (diagnostic only; never pumps energy by wave index).")]
    [SerializeField] bool debugWaveEnergyBudget = true;
    [Tooltip("Check the tracking normal, wave solver normal and the current render-axis normal.")]
    [SerializeField] bool debugNormalAlignment = false;
    [SerializeField, Range(5f,90f)] float normalMismatchWarningDegrees = 25f;
    [SerializeField] bool debugKnotTracking = false;
    [SerializeField, Min(.1f)] float trackingLogSeconds = .25f;

    [Header("V26: energy-scaled comoving terrain observation (no fixed amplitude templates)")]
    [Tooltip("Observe stairs and physical velocity; drive a continuous normal acceleration even on ramp-like collision meshes.")]
    [SerializeField] bool enableTerrainExcitation = true;
    [Tooltip("Nominal spatial pitch (meters) for visually stair-stepped terrain when the PHYSICS mesh is smoothed. This is a display-only surrogate, not a measured collision.")]
    [SerializeField, Min(.5f)] float terrainSpatialPitch = 3.30f;
    [Tooltip("Display-only roughness height (meters), weighted by actual stair slope, speed, contact proximity and Knot normal.")]
    [SerializeField, Range(.02f,1.0f)] float terrainSpatialRoughnessHeight = .55f;
    [Tooltip("Maximum effective spatial forcing frequency. The actual phase still comes from traveled Knot station.")]
    [SerializeField, Range(2f,12f)] float terrainMaxFrameFrequencyHz = 9.0f;
    [Tooltip("Contribution of directly measured 3-point ground curvature to frame acceleration.")]
    [SerializeField, Range(0f,.35f)] float terrainMeasuredCurvatureGain = .06f;
    [SerializeField] bool debugTerrainExcitation = true;

    [Header("V26: non-inertial authority / spatial ground corrugation")]
    [Tooltip("Display-only scaling of the equivalent ground corrugation inferred from measured speed, grade and contact proximity; no wave-index amplitude template.")]
    [SerializeField, Range(.25f,3f)] float expressiveAmplitudeGain = 1.45f;
    [Tooltip("Where the FIRST ground corrugation reaches its high point. Changes the moving reference acceleration, NOT ball height.")]
    [SerializeField, Range(.50f,.95f)] float expressiveFirstApex = .82f;
    [Tooltip("Where subsequent spatial corrugations crest. NOT a prescribed ball apex.")]
    [SerializeField, Range(.30f,.75f)] float expressiveLaterApex = .50f;
    [Tooltip("Contribution of the directly observed moving Knot-frame acceleration; non-inertial physics source, no external BVE input.")]
    [SerializeField, Range(0f,2f)] float movingFrameTransportGain = .65f;
    [Tooltip("V26 oscillator speed limit, not a Rigidbody limit. Prevents the old 20m/s Inspector value from flattening 6-9Hz waves.")]
    [SerializeField, Range(20f,140f)] float niOscillatorMaxSpeed = 75f;
    [Tooltip("V26 moving-frame acceleration limit, independent of old V24 serialization.")]
    [SerializeField, Range(500f,20000f)] float niOscillatorAccelerationLimit = 9000f;
    [Tooltip("Coupling of the observed speed/slope-based support corrugation to the NONINERTIAL oscillator.")]
    [SerializeField, Range(.1f,1.5f)] float niCorrugationDriveGain = .52f;
    [Tooltip("Damping of the V26 high-frequency non-inertial oscillator, separate from old low-frequency V21 setting.")]
    [SerializeField, Range(.2f,1.5f)] float niHighFrequencyDamping = .75f;
    [Tooltip("Debug once per region: report observed physical state and resulting oscillator motion, never algebraic A1/A2/A3 commands.")]
    [SerializeField] bool debugExpressiveWave = true;

    [Header("Renderer-only presentation (non-inertial output)")]
    [SerializeField] bool softenVisualWave = true;
    [SerializeField] bool adaptiveVisualFilter = true;

    [SerializeField, Range(.005f,.10f)] float flightSmoothSeconds = .035f;

    [SerializeField, Min(0f)] float adaptiveSpeedGain = 1.4f;
    [SerializeField, Range(.5f,30f)] float adaptiveDerivativeHz = 8f;
    [SerializeField, Range(.005f,.04f)] float adaptiveMinimumSmoothSeconds = .008f;
    [SerializeField] bool resetFilterOnSharpNormalChange = true;
    [SerializeField, Range(30f,170f)] float filterNormalResetDegrees = 65f;
    [SerializeField] bool debugBveTransitions = false;
    [SerializeField] bool debugBveState = false; // one sample / .25s, OFF by default

    [Header("Ball trajectory: comparable MAIN / STANDALONE logs (diagnostic only)")]
    [SerializeField] bool debugBallTrajectory = true;
    [SerializeField, Range(.02f, .25f)] float ballTrajectoryLogInterval = .05f;

    [Header("One flat -> one downhill stair pair")]
    [SerializeField, Range(3f,25f)] float lookAheadDistance = 8f;
    [SerializeField, Range(2f,35f)] float minimumDownhillDegrees = 10f;
    [SerializeField, Range(1f,15f)] float maxEntryCrossTrack = 5f;
    [SerializeField, Range(0.1f,5f)] float exitConfirmDistance = .65f;
    [SerializeField, Range(2f,30f)] float teleportLogDistance = 16f;
    [SerializeField, Range(1f,20f)] float cornerToleranceDegrees = 8f;
    [Header("V8: A+B rail lifetime and corner geometry")]
    [SerializeField, Range(1f,30f)] float maximumCornerReach = 16f;
    [SerializeField, Range(.25f,5f)] float nextFlatArrivalTolerance = 1.6f;
    [Header("V13: entry-plane publish and exact Knot tubes")]
    [SerializeField, Range(.05f,.75f)] float cornerEntryLeadMeters = .35f;

    [Header("A+B: three independent straight/90-degree geometry rails")]
    [SerializeField, Range(.1f,8f)] float laneHalfWidth = 2.5f;
    [SerializeField] bool turnToPositiveZ = true;
    [SerializeField, Range(1f,8f)] float miterLimit = 4f;

    [Header("A+b: only on downhill StairWay renderer groups")]
    [SerializeField, Range(0,8)] int multiplierOverride;
    [SerializeField, Min(.5f)] float nominalStairLength = 9.9f;
    [SerializeField, Range(.05f,.95f)] float fBoundary = 3f/7f;
    [SerializeField, Range(33,201)] int futureMinimumKnots = 97;
    [SerializeField, Min(.01f)] float firstHeight = .45f;
    [SerializeField, Min(.01f)] float riseSpeed = 3.1367f;
    [SerializeField, Range(0f,1f)] float energyBlend = .35f;
    [SerializeField, Range(.05f,.99f)] float restitution = .86f;
    [SerializeField, Range(.05f,1f)] float lowerRatio = .75f;
    [SerializeField, Range(.15f,.95f)] float firstApex = .82f;
    [SerializeField, Range(.15f,.85f)] float laterApex = .50f;
    [SerializeField, Min(0f)] float dampingPerMeter = .02f;
    [SerializeField, Range(0f,.5f)] float firstRoughness = .08f;
    [SerializeField, Range(0f,.5f)] float secondRoughness = .22f;
    [SerializeField, Range(0f,.5f)] float thirdRoughness = .12f;
    [SerializeField, Range(1f,32f)] float fallbackSpeed = 18f;
    [SerializeField, Range(.005f,.2f)] float observedSpacing = .03f;
    [SerializeField, Range(64,4096)] int maxObservedKnots = 900;
    [SerializeField, Range(.6f,1f)] float postTurnHeadingDot = .94f;
    [SerializeField, Range(.1f,8f)] float maxLateralSync = 6f;
    [Header("V10: one-time lateral calibration, never continuous correction")]
    [SerializeField, Range(2,8)] int laneRequiredSamples = 3;
    [SerializeField, Range(.1f,1f)] float laneStableTolerance = .35f;

    [Header("Five NEW SplineContainer meshes (Game + Scene view)")]
    [SerializeField, Range(.008f,.15f)] float tubeRadius = .035f;
    [SerializeField, Range(3,10)] int tubeSides = 6;
    [SerializeField, Range(.02f,.5f)] float meshRefreshSeconds = .09f;
    [SerializeField] Material optionalTubeMaterial;
    [SerializeField] bool showFiveInGame = true;
    [Header("Spline lifecycle diagnostic (read-only during Play)")]
    [SerializeField] bool debugSplineLifecycle;


    [Header("Runtime (read-only)")]
    [SerializeField] int detectedPieces, detectedPairs, pairIndex = -1, stairGroup = -1;
    [SerializeField] string state = "IDLE", lastReason = "";
    [SerializeField] int multiplier, waveCount, routeRevisions, predictionRevisions;
    [SerializeField] int[] knotCounts = new int[Count];
    [SerializeField] bool isDownhillStair, hasFlatBefore, hasRightAngleTurn, isDetouring, isTurnBusy, laneSynced;
    [SerializeField] float entryDistance, stairStation, sectionLength, lateralOffset, futureLateralShift, supportHeight;
    [SerializeField] bool cornerBuilt, exitReached, entryLaneTrusted;
    [SerializeField] int railTurnCount;
    [SerializeField] Vector3 entryCornerPoint, exitCornerPoint;
    [SerializeField] bool hasEntryCornerPoint, hasExitCornerPoint;
    [SerializeField] float signedEntryYaw, signedExitYaw;
    [SerializeField] int beforePublishReplans;
    [Header("Knot / BVE diagnostic (read-only)")]
    [SerializeField] int sonarAnchorCount, sonarNearbyKnots, sonarGroundContacts;
    [SerializeField] float sonarWaveHeight, sonarStation, sonarDetectedGroundY;
    [SerializeField] Vector3 sonarWaveNormal = Vector3.up;
    [SerializeField] bool sonarGeometryDetected, bveActive, bveExiting, bveProbeValid;
    // WaveIndex / WaveType are progress-region labels for diagnostics ONLY.
    // They do not prescribe the standalone ball's wave shape.
    [SerializeField] int bveWaveIndex = -1, bveWaveType = -1, bveBeginCount;
    [SerializeField] bool bveFirstApexLogged;
    [SerializeField] float bveProgress, bveTraveled, bveCurrentAmplitude, bveBaseOffset;
    [SerializeField] float bveResidualOffset, bveTerminalOffset, bveRawOffset;
    [SerializeField] float bveSlopeDegrees;
    [SerializeField] float bvePresentationSeconds;
    const int SonarAnchorCapacity = 7;
    readonly Vector3[] sonarAnchors = new Vector3[SonarAnchorCapacity];
    readonly bool[] sonarAnchorIsObservation = new bool[SonarAnchorCapacity];
    readonly RaycastHit[] sonarHits = new RaycastHit[16];
    readonly Collider[] bveSampleColliders = new Collider[4];
    bool bveArmed, bveFrameReady;
    // BVE flat rearm / start time and the two-point entry gradient.
    float bveFlatElapsed, bveGradient, bveLastBegin = -999f;
    int bvePhysicsStairMultiplier = 1;
    float bveNextStateLog;
    float bveElapsed, bveExitElapsed, bveWaveLocal01 = 1f;
    // Snapshot at the wave's actual entry. PREPARED on the next stair must
    // never alter the outgoing stair's repeated 2/3 rhythm or section length.
    int bveCycleGroup = -1, bveCycleMultiplier;
    float bveCycleSectionLength;
    float bveLastOffset, bveLastOffsetSpeed;
    float bvePreviousFilterTarget, bveAdaptiveVisualSpeed, bvePresentedSpeed, bvePresentedOffset;
    bool bveFilterReady, bvePresentationReady;
    Vector3 bveT = Vector3.forward, bveN = Vector3.up;
    Vector3 bveHeading = Vector3.forward;
    Vector3 bvePreviousPhysicsPoint;
    Vector3 bvePreviousPresentationNormal = Vector3.up;
    bool sourcePositionValid;
    // Visual Carrier is display translation / spline lifecycle authority ONLY.
    // InSubject is the sole physical observation source for the wave.
    bool bveVisualPresentationHasSample;
    Transform bveLastVisualParent;
    Vector3 bveLastWrittenLocalPosition;
    Vector3 bveLastAppliedLocalOffset;
    Vector3 lastSourcePosition, fallbackVelocity;
    Transform ownedVisualBall;
    Material ownedVisualMaterial;
    MonoBehaviour activeMainBve;
    Transform inspectorVisualBall;
    bool mainBveMode;

    // Knot target tracking, inlined: one component, no external dependency.
    bool trackingValid;
    int trackingKnotIndex = -1;
    int trackingNearbyKnots;
    Vector3 trackingTangent = Vector3.forward;
    Vector3 trackingNormal = Vector3.up;
    Vector3 trackingSide = Vector3.right;
    Vector3 trackingReference;
    float trackingRelativeHeight;
    float trackingRelativeSpeed;
    float trackingWaveObservation;

    int trackingGroup = -1;
    float trackingBaseline, trackingTrend, trackingEstimate, trackingEstimateSpeed;
    float trackingComovingHeight, trackingPhysicalSpeed, trackingFrameSpeed;
    float trackingCoordinateSpeed, trackingReferenceSpeed;
    Vector3 trackingLastNormal, trackingLastRelative, trackingLastReference;
    bool trackingHasLastFrame;
    float nextKnotTrackingLog;
    // V21: anchored physical->visual map. Source physics POSITION is sampled each
    // FixedUpdate; the visual subject calibrates coordinate origin ONCE, never
    // serves as a substitute observation or an adaptive compensation signal.
    bool observedMapReady;
    Transform observedPhysicsParent, observedVisualParent, observedVisualAnchor;
    Vector3 observedPhysicsLocal0, observedVisualLocal0;
    string observedMapSource = "UNAVAILABLE";
    float nextMissingMapLog;
    [SerializeField] bool debugPhysicalObservation = false;
    [Header("V21.1: observation verification only (no waveform changes)")]
    [SerializeField, Range(0f,1f)] float observationContinuityWeight = .18f;
    [SerializeField, Min(.25f)] float observationMinimumStationTravel = 1.5f;
    [SerializeField, Min(1f)] float observationVelocityWindowFactor = 2.5f;
    [SerializeField, Min(0f)] float observationCrossTrackWarning = 5f;
    bool observationStationReady, observationSampleReady;
    int observationStationGroup = -1;
    int observationPairKnotCount;
    float observationLastStation, observationStationStep;
    Vector3 observationPreviousPhysicsPoint, observationPreviousMappedPoint;
    Vector3 observationMappedVelocity, observationMappedSourceVelocity;
    Vector3 observationMappedFrameVelocity, observationMappedRigidVelocity;
    float observationRigidbodyVelocityError, observationHeightRateError;
    float trackingRawHeight;
    // V21.2: failure reason and the last ACCEPTED station survive a transient miss.
    string observationRejectReason = "NONE";
    float observationLastStationTime, observationAllowedTravel, observationCandidateStation;
    int observationRejectedFrames, observationRebaseCount;
    float nextObservationRecoveryLog;

    float contactFraction;
    // V20 wave state: the measured comoving coordinate is authoritative.
    float nonInertialOffset, nonInertialVelocity, nonInertialTarget;
    float nonInertialLastInputVelocity, nonInertialInputJump;
    bool nonInertialHadInput;
    // Only ONE MonoBehaviour class is required: the manager on StairwaySimple
    // auto-installs an instance of THIS class as a collision-only listener on
    // InSubject. Unity dispatches collision callbacks to the Rigidbody host.
    // The attach marker must be set BEFORE AddComponent, because Awake runs
    // synchronously during AddComponent. Listener Awake exits before spline setup.
    static IndependentFiveSplineWeaver pendingContactOwner;
    static GameObject pendingContactObject;
    [SerializeField, HideInInspector] bool contactListenerOnly;
    [SerializeField, HideInInspector] IndependentFiveSplineWeaver contactListenerOwner;
    IndependentFiveSplineWeaver physicalContactProbe;
    bool ownsPhysicalContactProbe;
    // These listener fields are independent from the manager's Inspector knobs.
    Rigidbody contactTarget;
    LayerMask contactGroundMask;
    float contactListenerMinApproach;
    ContactSample contactPending;
    bool contactHasPending;
    int contactAllCallbacks;
    float nextContactStateLog;
    float lastContactImpulsePerMass, lastContactKick, lastContactNormalAlignment;
    float lastContactApproachSpeed;
    float lastContactVelocityBefore, lastContactVelocityAfter;
    float lastContactOffsetAtKick, lastContactTargetAtKick;
    int contactEventsSinceLog, contactQualifiedSinceLog, contactInjectedSinceLog;
    int contactSupportRejected, contactSpeedRejected, contactCooldownRejected;
    int contactImpulseRejected, contactOrientationRejected;
    // Energy is in display-oscillator units (mass=1), NOT real Rigidbody joules.
    bool waveEnergyActive;
    int waveEnergyGroup=-1, waveEnergyIndex=-1, waveEnergyType=-1;
    int waveEnergySteps, waveEnergyContactCount, waveEnergyTerrainCount;
    float waveEnergyStart, waveEnergyContactInput, waveEnergyTerrainInput, waveEnergyVelocityInput;
    float waveEnergyTargetWork, waveEnergyDampingLoss, waveEnergyMaxAbsOffset;
    bool contactTransferPending;
    // V25: spatial/temporal non-inertial reference acceleration. The frame
    // is constructed from the measured Knot station and real travel speed.
    Collider terrainBodyCollider;
    float terrainFrameAcceleration, terrainFrameOmega, terrainFrameSourceHz;
    float terrainFrameAmplitude, terrainMeasuredCurvature;
    float terrainLastTravelSpeed, terrainNextSummaryTime;
    int terrainFrameGroup = -1, terrainFrameDrivenSteps;
    bool terrainFrameUsingSurrogate;
    // V26 moving-frame transport theorem: the origin and rotating normal
    // contribute apparent acceleration, independently of true collision kicks.
    Vector3 niFramePreviousOrigin, niFramePreviousVelocity;
    Vector3 niFramePreviousNormal, niFramePreviousNormalRate;
    bool niFrameHistoryValid, niFrameHasVelocitySample;
    float niFrameTransportAcceleration, niFrameTransportRaw;
    int niRegionLogged = -1, niGroupLogged = -1;
    // Future predicts with the same local terrain/frame oscillator, but it
    // never receives live Observed spline peaks or an external BVE transform.
    float futureAmplitudeReference;
    bool futureEnergyFrozen;

    float nextNormalAlignmentLog;
    // Listener-owned memory: distinguish true impact onsets from continuous support.
    // Reuse the same dictionary on the auto-installed CONTACT_ONLY instance.
    readonly Dictionary<int, ContactHistory> contactHistory = new Dictionary<int, ContactHistory>(32);

    // Diagnostics only. These values never enter wave, physics, or spline calculations.
    bool ballTraceRunning, ballTraceLastMainMode, ballTraceWasExiting;
    int ballTraceGroup = -1, ballTraceWave = -1, ballTraceSamples;
    float ballTraceNextTime, ballTracePreviousTime, ballTracePreviousHeight;
    bool ballTracePreviousHeightValid;
    float ballTraceMinHeight, ballTraceMaxHeight, ballTraceMaxPhase;
    float ballTraceMinRaw, ballTraceMaxRaw, ballTraceEntryHeight;
    bool ballTraceEntryHeightValid;
    Vector3 ballTracePeakWorld;

    // Data transfer within the SAME class; no second MonoBehaviour script.
    struct ContactHistory {
        public float supportImpulsePerMass;
        public float lastSeenTime;
        public float lastInjectedTime;
        public bool initialized;
    }

    struct ContactSample {
        public float fixedTime;
        public int allEvents, qualifiedEvents;
        public int onsetEvents, spikeEvents;
        public int rejectedSupport, rejectedSpeed, rejectedCooldown, rejectedImpulse;
        public float qualifiedImpulsePerMass;
        public float peakApproachSpeed;
        public Vector3 qualifiedNormalWeighted;
    }

    sealed class Piece {
        public Transform transform;
        public int group, part;
        public bool stair;
        public Vector3 a,b,heading;
        public float slope;
    }
    sealed class Pair {
        public int group, key, first, last, flat, partCount;
        public bool downhill, turn90, outTurn90;
        public Vector3 entry, exit, heading, flatHeading, side;
        public Vector3 flatStart, flatEnd, corner;
        public Vector3 outgoingFlatStart, outgoingFlatEnd, outgoingHeading, outgoingCorner;
        public int outgoingFlat = -1;
        public bool hasCorner, hasOutgoingCorner;
        public float entryYaw, exitYaw;
        public float length;
        public Transform anchor;
        public readonly List<Vector3> axis = new List<Vector3>(32);
        public readonly List<float> stations = new List<float>(32);
    }
    readonly List<Piece> pieces = new List<Piece>(256);
    readonly List<Pair> pairs = new List<Pair>(64);
    readonly List<Vector3>[] paths = {
        new List<Vector3>(40),new List<Vector3>(40),new List<Vector3>(40),
        new List<Vector3>(900),new List<Vector3>(220)
    };
    readonly List<float> futureStations = new List<float>(220);
    readonly SplineContainer[] containers = new SplineContainer[Count];
    readonly Mesh[] meshes = new Mesh[Count];
    readonly MeshRenderer[] meshRenderers = new MeshRenderer[Count];
    readonly Material[] materials = new Material[Count];
    readonly bool[] meshDirty = new bool[Count];
    Transform owner;
    Matrix4x4 previousVisualRootMatrix;
    Transform followedVisualRoot;
    Pair active;
    float nextScan, nextMesh, previousSampleTime, measuredSpeed;
    float firstA, secondA, thirdA, lastObservedStation;
    // Future and Observed both live in VISUAL world space. Once at first
    // stair observation, measure the Future support in the STAIR-N frame,
    // never mistake world-Y height for a distance along an inclined normal.
    float futureModelSpeed, waveRiseSpeed, exitElapsed;
    bool futureNormalAnchored;
    readonly List<float> pendingLaneDeltas = new List<float>(8);
    float lastLaneSampleStation = -1f;
    Vector3 previousBall, previousObserved, moveStep;
    bool hasPreviousBall;
    int lastCompletedGroup = -1, stageZeroFirstKey, scanHash;
    int headingStableFrames;

    void LifecycleEvent(string tag) {
        if (!debugSplineLifecycle) return;
        Debug.Log($"[IFSW][LIFECYCLE] {tag} state={state} group={stairGroup} " +
            $"pieces={detectedPieces} pairs={detectedPairs} " +
            $"knots=[{knotCounts[0]},{knotCounts[1]},{knotCounts[2]},{knotCounts[3]},{knotCounts[4]}]", this);
    }



    void Awake() {
        // Unity calls Awake immediately while AddComponent is executing.
        // Identify the dynamically installed listener before any GameObjects,
        // spline containers, meshes or materials can be allocated.
        if (pendingContactOwner && pendingContactObject == gameObject) {
            contactListenerOnly = true;
            contactListenerOwner = pendingContactOwner;
            contactTarget = GetComponent<Rigidbody>();
            return;
        }
        if (contactListenerOnly) return; // domain-reload-disabled safety
        inspectorVisualBall = displayedBall; // Preserve the user's Inspector reference across mode changes.
        owner = new GameObject("__IFSW_DynamicsV15_OWNED").transform;
        for (int i = 0; i < Count; i++) {
            GameObject go = new GameObject("IFSW15_NEW_" + Names[i]);
            go.transform.SetParent(owner,false);
            containers[i] = go.AddComponent<SplineContainer>();
            containers[i].Spline = new Spline();
            meshes[i] = new Mesh { name="IFSW15_Tube_"+Names[i], indexFormat=IndexFormat.UInt32 };
            go.AddComponent<MeshFilter>().sharedMesh = meshes[i];
            meshRenderers[i] = go.AddComponent<MeshRenderer>();
            Shader shader = Shader.Find("Universal Render Pipeline/Unlit");
            if (!shader) shader = Shader.Find("Unlit/Color");
            if (!shader) shader = Shader.Find("Sprites/Default");
            if (optionalTubeMaterial || shader) {
                Material m = optionalTubeMaterial ? new Material(optionalTubeMaterial) : new Material(shader);
                m.name="IFSW15_Mat_"+Names[i];
                if (m.HasProperty("_BaseColor")) m.SetColor("_BaseColor",Colors[i]);
                if (m.HasProperty("_Color")) m.SetColor("_Color",Colors[i]);
                materials[i] = m;
                meshRenderers[i].sharedMaterial = m;
            }
            meshRenderers[i].receiveShadows = false;
            meshRenderers[i].shadowCastingMode = ShadowCastingMode.Off;
            meshDirty[i] = true;
        }
    }

    // One-time standalone ownership; the old BVE waveform cannot be selected.
    void Start() {
        if (contactListenerOnly) return;
        ResolveVisualMotionSource();
        EnsurePhysicalContactProbe();
        // V20 takes standalone ownership unconditionally. Even when an old
        // ExampleBVE is enabled, its deterministic wave cannot override this
        // non-inertial solver or feed back into the five-spline geometry.
        activeMainBve = null;
        mainBveMode = false;
        EnsureIndependentVisualBall();
        if (debugBveParityEvents)
            Debug.Log("[IFSW][NI_AUTHORITY] NONINERTIAL standalone owns the ball; " +
                "disable any old ExampleBallVisualEqualizer renderer to avoid duplicates", this);
        UpdateVisualStatus();
    }

    bool HasSource => inSubject || subjectFallback;
    Vector3 SourcePhysicsPoint => inSubject ? inSubject.position :
        subjectFallback ? subjectFallback.position : Vector3.zero;
    Vector3 SourcePhysicsVelocity => inSubject ? inSubject.velocity : fallbackVelocity;

    void OnEnable() {
        if (contactListenerOnly) return;
        if (owner) owner.gameObject.SetActive(true);
    }
    void OnDisable() {
        // The original sensor cleared queued samples when disabled.
        contactHasPending = false;
        contactPending = default(ContactSample);
        contactHistory.Clear();
        if (contactListenerOnly) return;
        FinishWaveEnergyBudget("DISABLED");
        if (owner) owner.gameObject.SetActive(false);
    }
    void OnDestroy() {
        if (contactListenerOnly) return;
        if (ownsPhysicalContactProbe && physicalContactProbe) {
            if (Application.isPlaying) Destroy(physicalContactProbe);
            else DestroyImmediate(physicalContactProbe);
        }
        if (owner) { if(Application.isPlaying) Destroy(owner.gameObject); else DestroyImmediate(owner.gameObject); }
        for(int i=0;i<Count;i++) {
            if(meshes[i]) { if(Application.isPlaying) Destroy(meshes[i]); else DestroyImmediate(meshes[i]); }
            if(materials[i]) { if(Application.isPlaying) Destroy(materials[i]); else DestroyImmediate(materials[i]); }
        }
        if (ownedVisualMaterial) {
            if (Application.isPlaying) Destroy(ownedVisualMaterial);
            else DestroyImmediate(ownedVisualMaterial);
        }
    }

    // The non-inertial observer owns its own entry / exit timing. A+B / A+b Spline
    // lifecycle can be PREPARED, ACTIVE or IDLE without prescribing wave shapes.
    // Rigidbody remains strictly READ ONLY.
    void FixedUpdate() {
        if (contactListenerOnly) return;
        if (mainBveMode) { UpdateReadOnlyMainSonar(); return; }
        if (!driveVisualWithKnotSonar) return;
        if (!inSubject) {
            // SubjectFallback is permitted by old five-spline lifecycle, but is
            // NOT an InSubject measurement and cannot drive the wave model.
            ResetKnotTracking();
            if (bveActive) ResetBveWave(true); // no lingering visual wave without physics
            if (debugPhysicalObservation && Time.fixedTime >= nextMissingMapLog) {
                nextMissingMapLog = Time.fixedTime + 2f;
                Debug.LogWarning("[IFSW][OBSERVATION_DISABLED] Assign the InSubject Rigidbody. " +
                    "No Visual Carrier substitution is permitted.", this);
            }
            return;
        }
        float dt = Mathf.Max(.001f, Time.fixedDeltaTime);
        // InSubject references can change during scene recreation. Rebind only
        // when necessary, not once every physics step on normal play.
        if (enablePhysicalContactExcitation &&
            (!physicalContactProbe || physicalContactProbe.gameObject != inSubject.gameObject))
            EnsurePhysicalContactProbe();
        Vector3 p = SourcePhysicsPoint;
        Vector3 v = SourcePhysicsVelocity;
        if (!bveFrameReady) InitializeStandaloneBveFrame(p, v);
        if (sourcePositionValid &&
            (p - lastSourcePosition).magnitude > bveTeleportThreshold) {
            ResetBveWave(true);
            ResetPhysicalObservationMap();
            InitializeStandaloneBveFrame(p, v);
            if (debugBveParityEvents)
                Debug.Log($"[IFSW][BVE_TELEPORT] t={Time.fixedTime:F3} " +
                    $"threshold={bveTeleportThreshold:F3}", this);
            sourcePositionValid = true;
            lastSourcePosition = p;
            return;
        }
        sourcePositionValid = true;
        lastSourcePosition = p;
        UpdateBveKnotSonar(p, v, dt);
    }

    // Unity invokes OnCollisionEnter/Stay on the InSubject Rigidbody host.
    // The manager remains on StairwaySimple; an instance of THIS VERY SAME
    // class is attached on InSubject in read-only listener mode.
    void EnsurePhysicalContactProbe() {
        if (!enablePhysicalContactExcitation || !inSubject) return;
        if (physicalContactProbe && physicalContactProbe.gameObject != inSubject.gameObject) {
            if (ownsPhysicalContactProbe) Destroy(physicalContactProbe);
            physicalContactProbe = null;
            ownsPhysicalContactProbe = false;
        }
        if (!physicalContactProbe) {
            IndependentFiveSplineWeaver existing =
                inSubject.GetComponent<IndependentFiveSplineWeaver>();
            if (existing) {
                // If the main Weaver is hosted on the Rigidbody itself, it
                // already gets collision callbacks; avoid another component.
                if (existing == this ||
                    (existing.contactListenerOnly && existing.contactListenerOwner == this)) {
                    physicalContactProbe = existing;
                } else {
                    Debug.LogWarning("[IFSW][CONTACT_BINDING] InSubject already has " +
                        "an unrelated IndependentFiveSplineWeaver. Contact binding " +
                        "is disabled to prevent hijacking its spline manager.", this);
                    return;
                }
            } else {
                // Single-threaded Unity main-thread AddComponent + Awake handoff.
                pendingContactOwner = this;
                pendingContactObject = inSubject.gameObject;
                try {
                    physicalContactProbe = inSubject.gameObject
                        .AddComponent<IndependentFiveSplineWeaver>();
                } finally {
                    pendingContactOwner = null;
                    pendingContactObject = null;
                }
                ownsPhysicalContactProbe = physicalContactProbe &&
                    physicalContactProbe.contactListenerOnly;
                if (ownsPhysicalContactProbe && debugContactExcitation)
                    Debug.Log("[IFSW][CONTACT_BINDING] class=IndependentFiveSplineWeaver " +
                        "role=CONTACT_ONLY host=" + inSubject.gameObject.name +
                        " splineContainers=0", this);
            }
        }
        if (physicalContactProbe)
            physicalContactProbe.ConfigureContactListener(
                inSubject, groundMask, contactMinApproachSpeed);
    }

    void ConfigureContactListener(Rigidbody body, LayerMask mask, float minApproach) {
        contactTarget = body;
        contactGroundMask = mask;
        contactListenerMinApproach = Mathf.Max(0f, minApproach);
        contactPending = default(ContactSample);
        contactHasPending = false;
        contactHistory.Clear();
    }

    void OnCollisionEnter(Collision collision) { RecordPhysicalContact(collision, true); }
    void OnCollisionStay(Collision collision) { RecordPhysicalContact(collision, false); }
    void OnCollisionExit(Collision collision) {
        if (contactTarget && collision.gameObject)
            contactHistory.Remove(collision.gameObject.GetInstanceID());
    }

    void RecordPhysicalContact(Collision collision, bool newContact) {
        if (!contactTarget || !collision.gameObject || collision.contactCount == 0)
            return;
        if (contactListenerOnly &&
            (!contactListenerOwner || !contactListenerOwner.isActiveAndEnabled))
            return;
        if ((contactGroundMask.value & (1 << collision.gameObject.layer)) == 0)
            return;

        contactAllCallbacks++;
        float time = Time.fixedTime;
        if (!contactHasPending || Mathf.Abs(time - contactPending.fixedTime) > .0001f) {
            contactPending = new ContactSample { fixedTime = time };
            contactHasPending = true;
        }
        contactPending.allEvents++;

        // The impulse itself also contains normal support from OnCollisionStay.
        // Require a measurable normal impact speed; then require either an
        // OnCollisionEnter or a sharp increase above this partner's support baseline.
        Vector3 normal = collision.GetContact(0).normal;
        if (normal.sqrMagnitude < 1e-8f) return;
        normal.Normalize();
        float normalSpeed = Mathf.Abs(Vector3.Dot(collision.relativeVelocity, normal));
        float deltaV = collision.impulse.magnitude / Mathf.Max(.001f, contactTarget.mass);
        if (float.IsNaN(deltaV) || float.IsInfinity(deltaV) || deltaV < 1e-5f) {
            contactPending.rejectedImpulse++;
            return;
        }
        int id = collision.gameObject.GetInstanceID();
        ContactHistory history;
        bool known = contactHistory.TryGetValue(id, out history) && history.initialized;
        float baseline = known ? history.supportImpulsePerMass : 0f;
        bool onset = newContact || !known || time - history.lastSeenTime > Time.fixedDeltaTime * 2.6f;
        IndependentFiveSplineWeaver settings = contactListenerOwner ? contactListenerOwner : this;
        bool spike = !onset &&
            deltaV - baseline >= settings.contactImpulseRiseSpeed &&
            deltaV >= Mathf.Max(settings.contactMinImpulseSpeed,
                baseline * settings.contactImpulseRiseRatio);
        bool cooldown = known && time - history.lastInjectedTime <
            settings.contactImpactCooldown;
        history.supportImpulsePerMass = known && !onset
            ? Mathf.Lerp(baseline, deltaV, .18f) : deltaV;
        history.lastSeenTime = time;
        history.initialized = true;
        bool qualified = normalSpeed >= contactListenerMinApproach &&
            (onset || spike) && !cooldown && deltaV >= 1e-5f;
        if (qualified) history.lastInjectedTime = time;
        contactHistory[id] = history;

        if (normalSpeed < contactListenerMinApproach) contactPending.rejectedSpeed++;
        else if (cooldown) contactPending.rejectedCooldown++;
        else if (!onset && !spike) contactPending.rejectedSupport++;
        else if (qualified) {
            contactPending.qualifiedEvents++;
            if (onset) contactPending.onsetEvents++;
            else contactPending.spikeEvents++;
            contactPending.qualifiedImpulsePerMass += deltaV;
            contactPending.peakApproachSpeed =
                Mathf.Max(contactPending.peakApproachSpeed, normalSpeed);
            contactPending.qualifiedNormalWeighted += normal * deltaV;
        }
        // The stage is generated dynamically. Bound listener memory to recent
        // contacts without scanning it on every physics callback.
        if (contactHistory.Count > 128) {
            List<int> stale = new List<int>();
            foreach (var pair in contactHistory)
                if (time - pair.Value.lastSeenTime > 2f) stale.Add(pair.Key);
            foreach (int key in stale) contactHistory.Remove(key);
        }
    }

    bool TryConsumeContact(out ContactSample result) {
        result = contactPending;
        if (!contactHasPending) return false;
        contactHasPending = false;
        contactPending = default(ContactSample);
        return true;
    }

    // Consume at most once per physics tick. The contact callback is usually
    // delivered after FixedUpdate, so a <=2.5-frame-old sample is valid.
    // A missing/invalid Knot frame must NEVER turn a contact into a visual kick.
    void ObservePhysicsContactExcitation(float dt) {
        lastContactImpulsePerMass = lastContactKick = 0f;
        lastContactNormalAlignment = lastContactApproachSpeed = 0f;
        contactTransferPending = false;
        if (!enablePhysicalContactExcitation || !physicalContactProbe || !inSubject) return;
        if (physicalContactProbe.TryConsumeContact(out ContactSample sample)) {
            if (Time.fixedTime - sample.fixedTime <= Mathf.Max(.06f, dt * 2.5f)) {
                contactEventsSinceLog += sample.allEvents;
                contactQualifiedSinceLog += sample.qualifiedEvents;
                contactSupportRejected += sample.rejectedSupport;
                contactSpeedRejected += sample.rejectedSpeed;
                contactCooldownRejected += sample.rejectedCooldown;
                contactImpulseRejected += sample.rejectedImpulse;
                lastContactImpulsePerMass = sample.qualifiedImpulsePerMass;
                lastContactApproachSpeed = sample.peakApproachSpeed;
                bool validKnot = trackingValid && active != null &&
                    active.group == bveCycleGroup && bveActive && !bveExiting;
                // Unlike the old high-pass q signal, a real collision can
                // excite the oscillator between *discrete* Knot positions.
                // Geometry (T,N,B) must still be valid; no guessed directions.
                if (validKnot && sample.qualifiedEvents > 0 &&
                    sample.qualifiedNormalWeighted.sqrMagnitude > E) {
                    Vector3 contactNormalInVisual = MapStandaloneDirection(
                        sample.qualifiedNormalWeighted.normalized);
                    // A normal opposite to the Knot outward direction is
                    // NOT an upward stair impact (e.g. an overhead obstacle).
                    lastContactNormalAlignment = Vector3.Dot(
                        contactNormalInVisual.normalized, trackingNormal);
                    float projectedImpulseSpeed = sample.qualifiedImpulsePerMass *
                        lastContactNormalAlignment;
                    if (lastContactNormalAlignment >= contactMinNormalAlignment &&
                        projectedImpulseSpeed >= contactMinImpulseSpeed) {
                        // Stair contacts push the visual ball away from the
                        // stair surface, along its outward Knot normal. Only
                        // physical contact magnitude is used; this does NOT
                        // create a periodic waveform independent of contacts.
                        float unclampedKick = Mathf.Max(0f,
                            projectedImpulseSpeed * contactWaveVelocityGain);
                        lastContactKick = contactSoftLimitKick
                            ? contactMaxVelocityKick * (1f - Mathf.Exp(
                                -unclampedKick / Mathf.Max(.1f, contactMaxVelocityKick)))
                            : Mathf.Min(unclampedKick, contactMaxVelocityKick);
                        float beforeContactEnergy = OscillatorDiagnosticEnergy();
                        lastContactVelocityBefore = nonInertialVelocity;
                        lastContactOffsetAtKick = nonInertialOffset;
                        lastContactTargetAtKick = nonInertialTarget;
                        nonInertialVelocity = Mathf.Clamp(
                            nonInertialVelocity + lastContactKick,
                            -niOscillatorMaxSpeed, niOscillatorMaxSpeed);
                        lastContactVelocityAfter = nonInertialVelocity;
                        // Record the *effective* kick after solver speed limits.
                        lastContactKick = Mathf.Max(0f,
                            lastContactVelocityAfter - lastContactVelocityBefore);
                        contactTransferPending = lastContactKick > 0f;
                        if (contactTransferPending) {
                            contactInjectedSinceLog++;
                            waveEnergyContactInput += OscillatorDiagnosticEnergy() - beforeContactEnergy;
                            waveEnergyContactCount++;
                        }
                        if (debugContactExcitation)
                            Debug.Log($"[IFSW][NI_CONTACT] t={Time.fixedTime:F3} " +
                                $"group={bveCycleGroup} knot={trackingKnotIndex} " +
                                $"impulsePerMass={sample.qualifiedImpulsePerMass:F4} " +
                                $"normalAlignment={lastContactNormalAlignment:F3} " +
                                $"approachSpeed={sample.peakApproachSpeed:F3} " +
                                $"onset={sample.onsetEvents} spike={sample.spikeEvents} " +
                                $"kick={lastContactKick:F4} " +
                                $"normal={trackingNormal:F4} " +
                                $"x={nonInertialOffset:F4} dx={nonInertialVelocity:F4}", this);
                    } else contactOrientationRejected++;
                }
            }
        }
        if (debugContactExcitation && Time.fixedTime >= nextContactStateLog) {
            nextContactStateLog = Time.fixedTime + .5f;
            Debug.Log($"[IFSW][CONTACT_SENSOR] t={Time.fixedTime:F3} " +
                $"bound={physicalContactProbe.gameObject.name} " +
                $"group={bveCycleGroup} validKnot={trackingValid} " +
                $"events={contactEventsSinceLog} qualified={contactQualifiedSinceLog} " +
                $"injected={contactInjectedSinceLog} " +
                $"rejectSupport={contactSupportRejected} " +
                $"rejectSpeed={contactSpeedRejected} " +
                $"rejectCooldown={contactCooldownRejected} " +
                $"rejectImpulse={contactImpulseRejected} " +
                $"rejectDirection={contactOrientationRejected} " +
                $"impulsePerMass={lastContactImpulsePerMass:F4} " +
                $"kick={lastContactKick:F4} " +
                $"sensorAllCallbacks={physicalContactProbe.contactAllCallbacks}", this);
            contactEventsSinceLog = contactQualifiedSinceLog = contactInjectedSinceLog = 0;
            contactSupportRejected = contactSpeedRejected = contactCooldownRejected = 0;
            contactImpulseRejected = contactOrientationRejected = 0;
        }
    }

    // Equivalent rise energy from the physical approach speed and local stair
    // pitch/height: qHeight ~ vNormalEquivalent^2 / (2*gNormal).
    // IMPORTANT: this does not claim an actual impact where no contact exists.
    // Unlike V25 it does NOT clamp speed to 19.8 m/s or grade to one plateau.
    float ComovingTerrainAmplitude(float speed, float slopeDegrees,
        float contactWeight, float normalAlignment) {
        float grade = Mathf.Sin(Mathf.Clamp(slopeDegrees,0f,89f)*Mathf.Deg2Rad);
        float riseOverRun = terrainSpatialRoughnessHeight /
            Mathf.Max(.5f,terrainSpatialPitch);
        float vEquivalent = Mathf.Max(0f,speed) * riseOverRun *
            (.85f + .5f*grade);
        float gNormal = Mathf.Max(3f,Physics.gravity.magnitude *
            Mathf.Max(.25f,normalAlignment));
        float eHeight = .5f*vEquivalent*vEquivalent/gNormal;
        return Mathf.Clamp(eHeight*Mathf.Clamp01(contactWeight)*
            Mathf.Clamp01(normalAlignment),0f,nonInertialMaxOffset*.95f);
    }

    // Shape the MOVING TERRAIN FRAME, never assign the shape to the ball.
    // h(s)=A*f(u), u=s/L. Its transport acceleration is
    // -[A*f''(u)*sDot^2/L^2 + A*f'(u)*sDDot/L].
    // Only the landscape/phase geometry is stylized. The solver's q and qDot
    // still come from integration and actual InSubject contact/observation.
    void SpatialFrameDerivatives(float u,float waveLength,int region,
        out float dhdu,out float d2hdu2) {
        float apex=region==0?expressiveFirstApex:expressiveLaterApex;
        float k,phase;
        u=Mathf.Clamp01(u);
        if(u<apex) {
            if(region==0) {
                k=Mathf.PI/Mathf.Max(.05f,2f*apex);
                phase=k*u;
                dhdu=k*Mathf.Cos(phase);
                d2hdu2=-k*k*Mathf.Sin(phase);
            } else {
                k=Mathf.PI/Mathf.Max(.05f,apex);
                phase=k*u;
                dhdu=k*Mathf.Sin(phase);
                d2hdu2=k*k*Mathf.Cos(phase);
            }
        } else {
            k=Mathf.PI/Mathf.Max(.05f,1f-apex);
            phase=k*(u-apex);
            dhdu=-k*Mathf.Sin(phase);
            d2hdu2=-k*k*Mathf.Cos(phase);
        }
    }

    // Normal-axis component of the full coordinate-transport identity:
    // q'' = N.(ax-aO) + 2*Ndot.(vx-vO) + Nddot.(x-O).
    // Physical ax is NOT replayed because contact impulses and the observed
    // displacement already carry it. Only moving-origin/rotating-axis terms
    // are injected here. Rebase skips prevent fictitious corner spikes.
    float ObserveMovingFrameTransport(Vector3 velocity,float dt,bool measured) {
        if(!measured || dt<=.00001f) {
            niFrameHistoryValid=niFrameHasVelocitySample=false;
            niFrameTransportRaw=niFrameTransportAcceleration=0f;
            return 0f;
        }
        Vector3 origin=trackingReference;
        Vector3 n=trackingNormal;
        if(!niFrameHistoryValid) {
            niFramePreviousOrigin=origin;
            niFramePreviousVelocity=Vector3.zero;
            niFramePreviousNormal=n;
            niFramePreviousNormalRate=Vector3.zero;
            niFrameHistoryValid=true;
            niFrameTransportRaw=niFrameTransportAcceleration=0f;
            return 0f;
        }
        Vector3 previousOrigin=niFramePreviousOrigin;
        Vector3 previousN=niFramePreviousNormal;
        Vector3 originVelocity=(origin-previousOrigin)/dt;
        Vector3 nRate=(n-previousN)/dt;
        float normalTurn=Vector3.Angle(n,previousN);
        float jump=(origin-previousOrigin).magnitude;
        if(normalTurn>24f || jump>Mathf.Max(2.5f,bveTeleportThreshold)) {
            niFramePreviousOrigin=origin;
            niFramePreviousVelocity=originVelocity;
            niFramePreviousNormal=n;
            niFramePreviousNormalRate=Vector3.zero;
            niFrameHasVelocitySample=false;
            niFrameTransportRaw=niFrameTransportAcceleration=0f;
            return 0f;
        }
        // Two measured origin velocities are required to differentiate origin
        // acceleration. Suppress the first-frame false aO spike.
        if(!niFrameHasVelocitySample) {
            niFrameHasVelocitySample=true;
            niFramePreviousOrigin=origin;
            niFramePreviousVelocity=originVelocity;
            niFramePreviousNormal=n;
            niFramePreviousNormalRate=nRate;
            return 0f;
        }
        Vector3 originAcceleration=(originVelocity-niFramePreviousVelocity)/dt;
        Vector3 nAcceleration=(nRate-niFramePreviousNormalRate)/dt;
        Vector3 vRel=MapStandaloneDirection(velocity)-originVelocity;
        // trackingLastRelative is the CURRENT relative point from the
        // immediately preceding ObserveKnotTarget call in this FixedUpdate.
        niFrameTransportRaw= -Vector3.Dot(n,originAcceleration)
            +2f*Vector3.Dot(nRate,vRel)
            +Vector3.Dot(nAcceleration,trackingLastRelative);
        niFrameTransportRaw=Mathf.Clamp(niFrameTransportRaw,-200f,200f);
        niFrameTransportAcceleration=Mathf.Lerp(niFrameTransportAcceleration,
            niFrameTransportRaw,.35f);
        niFramePreviousOrigin=origin;
        niFramePreviousVelocity=originVelocity;
        niFramePreviousNormal=n;
        niFramePreviousNormalRate=nRate;
        return niFrameTransportAcceleration*movingFrameTransportGain;
    }

    // V24 physical ground-height observer: raycasts read collision terrain only.
    // The spatial frame model below supplies separate DISPLAY-only forcing
    // when the physics collision geometry is smoothed into a slope.
    bool TryTerrainHeight(Vector3 p, out float y) {
        y = 0f;
        Vector3 origin = p + Vector3.up * Mathf.Max(.2f, probeHeight);
        int count = Physics.RaycastNonAlloc(origin, Vector3.down, sonarHits,
            Mathf.Max(.2f, probeHeight) + probeDepth, groundMask,
            QueryTriggerInteraction.Ignore);
        float closest = float.PositiveInfinity;
        for (int i=0; i<count; i++) {
            RaycastHit hit=sonarHits[i];
            Collider c=hit.collider;
            if (!c || (inSubject && c.attachedRigidbody == inSubject) ||
                hit.normal.y < .45f || hit.distance >= closest) continue;
            closest = hit.distance;
            y = hit.point.y;
        }
        return closest < float.PositiveInfinity;
    }

    // V26: measured ground curvature + transported Knot-frame inertia and
    // a separately disclosed DISPLAY-ONLY corrugation for smoothed colliders.
    // 1/2/3 is the SPATIAL support geometry, never an amplitude command.
    float ObserveTerrainFrameAcceleration(Vector3 point,Vector3 velocity,
        float dt,bool measured,float u,float waveLength,out float desiredOmega) {
        desiredOmega=2f*Mathf.PI*Mathf.Clamp(nonInertialFrequencyHz,.2f,5f);
        terrainFrameAcceleration=terrainFrameSourceHz=terrainFrameAmplitude=
            terrainMeasuredCurvature=0f;
        terrainFrameUsingSurrogate=false;
        float transport=ObserveMovingFrameTransport(velocity,dt,measured);
        if(!enableTerrainExcitation || !measured || !inSubject || active==null ||
           bveExiting || !bveProbeValid ||
           bveSlopeDegrees < bveEntrySlopeDegrees*.6f) {
            terrainLastTravelSpeed=0f;
            if(debugTerrainExcitation && bveActive && Time.fixedTime>=terrainNextSummaryTime) {
                terrainNextSummaryTime=Time.fixedTime+.5f;
                Debug.Log($"[IFSW][FRAME_WAVE_GATE] group={bveCycleGroup} " +
                    $"measured={measured} activePath={active!=null} " +
                    $"probe={bveProbeValid} slope={bveSlopeDegrees:F1} " +
                    $"exiting={bveExiting} enabled={enableTerrainExcitation}",this);
            }
            return transport;
        }
        if(terrainFrameGroup!=bveCycleGroup) {
            terrainFrameGroup=bveCycleGroup;
            terrainLastTravelSpeed=0f;
            terrainFrameDrivenSteps=0;
        }
        Vector3 vMapped=MapStandaloneDirection(velocity);
        float physicalSpeed=Mathf.Max(0f,Vector3.Dot(vMapped,trackingTangent));
        float stationSpeed=Mathf.Max(0f,observationStationStep/Mathf.Max(dt,.001f));
        float travelSpeed=Mathf.Clamp(stationSpeed>=.25f?stationSpeed:physicalSpeed,0f,55f);
        if(travelSpeed<bveMinimumEntryPlanarSpeed) {
            terrainLastTravelSpeed=0f;
            return transport;
        }
        float travelAcceleration=terrainLastTravelSpeed>.01f
            ? Mathf.Clamp((travelSpeed-terrainLastTravelSpeed)/Mathf.Max(dt,.001f),-60f,60f):0f;
        terrainLastTravelSpeed=travelSpeed;
        float upwardAlignment=Mathf.Max(0f,Vector3.Dot(trackingNormal,
            MapStandaloneDirection(Vector3.up).normalized));
        Vector3 forward=Vector3.ProjectOnPlane(velocity,Vector3.up);
        bool groundedSamples=false;
        float gap=0f;
        if(forward.sqrMagnitude>.01f) {
            forward.Normalize();
            float sample=Mathf.Clamp(probeSpacing,.35f,.9f);
            if(TryTerrainHeight(point-forward*sample,out float behindY) &&
               TryTerrainHeight(point,out float centerY) &&
               TryTerrainHeight(point+forward*sample,out float aheadY)) {
                groundedSamples=true;
                terrainMeasuredCurvature=(aheadY-2f*centerY+behindY)/(sample*sample);
                if(!terrainBodyCollider)
                    terrainBodyCollider=inSubject.GetComponentInChildren<Collider>();
                if(terrainBodyCollider && terrainBodyCollider.enabled)
                    gap=Mathf.Max(0f,terrainBodyCollider.bounds.min.y-centerY);
            }
        }
        float contactWeight=groundedSamples
            ? Mathf.Clamp01(1f-gap/Mathf.Max(1.2f,probeHeight)):.55f;
        // The envelope depends on actual speed^2, incline and proximity;
        // it does NOT become constant at speed=19.8 m/s like V25.1.
        float amplitude=ComovingTerrainAmplitude(travelSpeed,bveSlopeDegrees,
            contactWeight,upwardAlignment)*expressiveAmplitudeGain;
        terrainFrameAmplitude=amplitude;
        float L=Mathf.Max(.2f,waveLength);
        float frequency=Mathf.Min(travelSpeed/L,terrainMaxFrameFrequencyHz);
        desiredOmega=Mathf.Max(desiredOmega,2f*Mathf.PI*frequency);
        terrainFrameSourceHz=frequency;
        SpatialFrameDerivatives(u,L,bveWaveIndex,out float dhdu,out float d2hdu2);
        // Divide by actual Wave length, rather than a global 3.30m sine.
        // Longer sections contain more measured support pitches; scale only
        // this ground-frame input, never the ball's displacement directly.
        float supportCoherence=Mathf.Clamp(L/Mathf.Max(.5f,terrainSpatialPitch),.65f,2.5f);
        float surrogate=-amplitude*supportCoherence*niCorrugationDriveGain*
            (d2hdu2*travelSpeed*travelSpeed/(L*L)
            +dhdu*travelAcceleration/L);
        float measuredAcceleration=-upwardAlignment*terrainMeasuredCurvature*
            travelSpeed*travelSpeed*terrainMeasuredCurvatureGain;
        float measuredShare=groundedSamples
            ? Mathf.Clamp01(Mathf.Abs(terrainMeasuredCurvature)*.20f):0f;
        // Real collision impulses act on velocity, not again as terrain force.
        float terrainContribution=surrogate*(1f-measuredShare)+measuredAcceleration;
        if(Mathf.Abs(lastContactKick)>.001f) terrainContribution*=.35f;
        float drive=terrainContribution+transport;
        drive=Mathf.Clamp(drive,-niOscillatorAccelerationLimit,niOscillatorAccelerationLimit);
        terrainFrameUsingSurrogate=amplitude>.001f && (1f-measuredShare)>.05f;
        terrainFrameAcceleration=drive;
        if(Mathf.Abs(drive)>.01f) terrainFrameDrivenSteps++;
        if(debugTerrainExcitation && Time.fixedTime>=terrainNextSummaryTime) {
            terrainNextSummaryTime=Time.fixedTime+.4f;
            Debug.Log($"[IFSW][NI_FRAME_FORCE] group={bveCycleGroup} " +
                $"wave={bveWaveIndex} u={u:F3} station={observationLastStation:F2} " +
                $"speed={travelSpeed:F2} pitch={L:F2} hz={frequency:F2} " +
                $"envelope={amplitude:F3} slope={bveSlopeDegrees:F1} " +
                $"curvature={terrainMeasuredCurvature:F3} " +
                $"transport={transport:F2} terrain={terrainContribution:F2} " +
                $"drive={drive:F2} measured={groundedSamples} " +
                $"surrogate={terrainFrameUsingSurrogate} " +
                $"osc={nonInertialOffset:F3} vOsc={nonInertialVelocity:F3}",this);
        }
        return drive;
    }

    // Diagnostic-only energy balance: E = (v^2 + omega^2*x^2)/2, unit mass.
    // The moving observation target performs work, the damper dissipates energy.
    // Neither the Wave index nor this accounting changes the oscillator.
    float OscillatorDiagnosticEnergy() {
        float omega = Mathf.Max(2f * Mathf.PI * Mathf.Clamp(nonInertialFrequencyHz,.2f,5f),terrainFrameOmega);
        return .5f * (nonInertialVelocity * nonInertialVelocity +
            omega * omega * nonInertialOffset * nonInertialOffset);
    }

    void FinishWaveEnergyBudget(string reason) {
        if (!waveEnergyActive) return;
        if (debugWaveEnergyBudget) {
            float end = OscillatorDiagnosticEnergy();
            float residual = end - waveEnergyStart - waveEnergyContactInput -
                waveEnergyVelocityInput - waveEnergyTerrainInput - waveEnergyTargetWork + waveEnergyDampingLoss;
            Debug.Log($"[IFSW][WAVE_ENERGY] group={waveEnergyGroup} " +
                $"wave={waveEnergyIndex} type={waveEnergyType} reason={reason} " +
                $"samples={waveEnergySteps} energyStart={waveEnergyStart:F4} " +
                $"energyEnd={end:F4} contactDelta={waveEnergyContactInput:F4} " +
                $"velocityDelta={waveEnergyVelocityInput:F4} " +
                $"terrainWorkApprox={waveEnergyTerrainInput:F4} " +
                $"targetWorkApprox={waveEnergyTargetWork:F4} " +
                $"dampingLossApprox={waveEnergyDampingLoss:F4} " +
                $"numericResidual={residual:F4} " +
                $"contactKicks={waveEnergyContactCount} " +
                $"terrainDriveSteps={waveEnergyTerrainCount} " +
                $"peakAbsX={waveEnergyMaxAbsOffset:F4}", this);
        }
        waveEnergyActive = false;
    }

    void BeginWaveEnergyRegion() {
        if (waveEnergyActive && waveEnergyGroup == bveCycleGroup &&
            waveEnergyIndex == bveWaveIndex) return;
        FinishWaveEnergyBudget("REGION_CHANGE");
        waveEnergyActive = true;
        waveEnergyGroup = bveCycleGroup;
        waveEnergyIndex = bveWaveIndex;
        waveEnergyType = bveWaveType + 1;
        waveEnergySteps = waveEnergyContactCount = waveEnergyTerrainCount = 0;
        waveEnergyContactInput = waveEnergyVelocityInput = waveEnergyTerrainInput = 0f;
        waveEnergyTargetWork = waveEnergyDampingLoss = 0f;
        waveEnergyStart = OscillatorDiagnosticEnergy();
        waveEnergyMaxAbsOffset = Mathf.Abs(nonInertialOffset);
    }

    void InitializeStandaloneBveFrame(Vector3 p, Vector3 v) {
        bvePreviousPhysicsPoint = p;
        Vector3 planar = Vector3.ProjectOnPlane(v, Vector3.up);
        if (planar.sqrMagnitude > .01f) bveHeading = planar.normalized;
        bveT = bveHeading;
        bveN = Vector3.up;
        sonarWaveNormal = MapStandaloneDirection(bveN);
        bveArmed = false; // Original BVE must observe a real flat before arming.
        bveFlatElapsed = 0f;
        bveFrameReady = true;
        ResetPhysicalObservationMap();
        ResetKnotTracking();
        bveRawOffset = sonarWaveHeight = 0f;
        bveWaveLocal01 = 1f;
        ResetBvePresentation(0f);
    }

    void UpdateReadOnlyMainSonar() {
        if (!drawKnotSonarGizmos || !displayedBall || active == null ||
            state != "ACTIVE" || paths[Center].Count < 2) {
            sonarAnchorCount = sonarNearbyKnots = 0;
            return;
        }
        Vector3 point = displayedBall.position;
        ChooseKnotSonarAnchors(point);
        sonarNearbyKnots = 0;
        for (int i = 0; i < sonarAnchorCount; i++) {
            if (sonarAnchorIsObservation[i]) continue;
            Vector3 d = sonarAnchors[i] - point;
            if (d.x*d.x + d.z*d.z + .15f*d.y*d.y <= knotSonarRadius*knotSonarRadius)
                sonarNearbyKnots++;
        }
    }

    Vector3 ToVisualPoint(Vector3 point) => physicsFrame && visualFrame
        ? visualFrame.TransformPoint(physicsFrame.InverseTransformPoint(point)) : point;
    Vector3 ToVisualDirection(Vector3 v) => physicsFrame && visualFrame
        ? visualFrame.TransformDirection(physicsFrame.InverseTransformDirection(v)) : v;
    Vector3 ToPhysicsDirection(Vector3 v) => physicsFrame && visualFrame
        ? physicsFrame.TransformDirection(visualFrame.InverseTransformDirection(v)) : v;

    // ExampleBVE-style parent-frame direction fallback when no explicit root pair exists.
    Vector3 MapStandaloneDirection(Vector3 direction) {
        if (physicsFrame && visualFrame) return ToVisualDirection(direction);
        Transform src = inSubject ? inSubject.transform.parent :
            subjectFallback ? subjectFallback.parent : null;
        Transform dst = visualMotionSource ? visualMotionSource.parent : null;
        Quaternion rotation = (dst ? dst.rotation : Quaternion.identity) *
            Quaternion.Inverse(src ? src.rotation : Quaternion.identity);
        return rotation * direction;
    }

    // Inverse of the display-direction mapping. Never mix a VISUAL Knot N
    // with the PhysicsRoot normal when their parent transforms differ.
    Vector3 UnmapStandaloneDirection(Vector3 visualDirection) {
        if (physicsFrame && visualFrame) return ToPhysicsDirection(visualDirection);
        Transform src = inSubject ? inSubject.transform.parent :
            subjectFallback ? subjectFallback.parent : null;
        Transform dst = visualMotionSource ? visualMotionSource.parent : null;
        Quaternion reverse = (src ? src.rotation : Quaternion.identity) *
            Quaternion.Inverse(dst ? dst.rotation : Quaternion.identity);
        return reverse * visualDirection;
    }

    // A visual pose is NOT an observation. With explicit frame references we
    // convert the actual Rigidbody position; otherwise use a one-time affine
    // LOCAL-SPACE ALIGNMENT that stays fixed as both parents subsequently move.
    // Never subtract a moving Visual Carrier sample each physics step: doing
    // so cancels exactly the movement we need to observe.
    void ResetPhysicalObservationMap() {
        observedMapReady = false;
        observedPhysicsParent = observedVisualParent = observedVisualAnchor = null;
        observedMapSource = "UNAVAILABLE";
        ResetObservationVerification();
    }

    // Observation state is read-only telemetry. A path change cannot create
    // a synthetic velocity impulse in the damped wave model.
    void ResetObservationVerification() {
        observationStationReady = observationSampleReady = false;
        observationStationGroup = -1;
        observationPairKnotCount = 0;
        observationLastStation = observationStationStep = 0f;
        observationLastStationTime = observationAllowedTravel = observationCandidateStation = 0f;
        observationRejectReason = "NONE";
        observationRejectedFrames = 0;
        observationPreviousPhysicsPoint = observationPreviousMappedPoint = Vector3.zero;
        observationMappedVelocity = observationMappedSourceVelocity = Vector3.zero;
        observationMappedFrameVelocity = observationMappedRigidVelocity = Vector3.zero;
        observationRigidbodyVelocityError = observationHeightRateError = 0f;
    }

    static Vector3 InParentSpace(Transform parent, Vector3 world) =>
        parent ? parent.InverseTransformPoint(world) : world;

    static Vector3 FromParentSpace(Transform parent, Vector3 local) =>
        parent ? parent.TransformPoint(local) : local;

    bool TryMapObservedInSubject(out Vector3 visualPhysicsPoint) {
        visualPhysicsPoint = Vector3.zero;
        if (!inSubject) { observedMapSource = "MISSING_INSUBJECT"; return false; }
        // Explicit corresponding frame roots take precedence. NEVER assume
        // PhysicsRoot and VisualRoot world coordinates are interchangeable.
        if ((bool)physicsFrame != (bool)visualFrame) {
            observedMapSource = "INCOMPLETE_FRAME_PAIR";
            return false; // avoid silently mixing explicit and inferred frames
        }
        if (physicsFrame && visualFrame) {
            observedMapSource = "INSUBJECT_EXPLICIT_FRAMES";
            visualPhysicsPoint = ToVisualPoint(inSubject.position);
            return true;
        }
        // When Inspector frames are unset, anchor two PARENTS once. This can
        // map independent rigs without reading the visual carrier as a target.
        if (!visualMotionSource || visualMotionSource == ownedVisualBall ||
            visualMotionSource == inSubject.transform ||
            IsMainBveDisplay(visualMotionSource)) {
            observedMapSource = "NO_SAFE_FRAME_MAPPING";
            return false; // fail closed, never observe VISUAL_CARRIER
        }
        Transform physicalParent = inSubject.transform.parent;
        Transform visualParent = visualMotionSource.parent;
        if (!observedMapReady || observedPhysicsParent != physicalParent ||
            observedVisualParent != visualParent ||
            observedVisualAnchor != visualMotionSource) {
            observedPhysicsParent = physicalParent;
            observedVisualParent = visualParent;
            observedVisualAnchor = visualMotionSource;
            observedPhysicsLocal0 = InParentSpace(physicalParent, inSubject.position);
            observedVisualLocal0 = InParentSpace(visualParent, visualMotionSource.position);
            observedMapReady = true;
            // Calibration contributes an INITIAL offset, no ongoing movement.
            ResetKnotTracking();
            if (debugPhysicalObservation)
                Debug.Log("[IFSW][OBSERVATION_MAP] source=INSUBJECT_ANCHORED_PARENTS " +
                    $"physicsParent={(physicalParent ? physicalParent.name : "WORLD")} " +
                    $"visualParent={(visualParent ? visualParent.name : "WORLD")} " +
                    "(captured once; movement comes from InSubject)", this);
        }
        Vector3 physicsLocal = InParentSpace(physicalParent, inSubject.position);
        Vector3 translatedLocal = observedVisualLocal0 + (physicsLocal - observedPhysicsLocal0);
        visualPhysicsPoint = FromParentSpace(visualParent, translatedLocal);
        observedMapSource = "INSUBJECT_ANCHORED_PARENTS";
        return true;
    }

    // Re-evaluate the SAME map for an arbitrary physics point at CURRENT root
    // poses. Visual Carrier is never sampled here after one-time calibration.
    Vector3 MapObservedPhysicsPointAtCurrentFrames(Vector3 physicalPoint) {
        if (physicsFrame && visualFrame) return ToVisualPoint(physicalPoint);
        Vector3 local = InParentSpace(observedPhysicsParent, physicalPoint);
        return FromParentSpace(observedVisualParent,
            observedVisualLocal0 + (local - observedPhysicsLocal0));
    }

    // A pure vector transform: position-scale/rotation only; no root translation.
    Vector3 MapObservedPhysicsVelocity(Vector3 physicsWorldVelocity) {
        if (physicsFrame && visualFrame)
            return visualFrame.TransformVector(
                physicsFrame.InverseTransformVector(physicsWorldVelocity));
        Vector3 local = observedPhysicsParent
            ? observedPhysicsParent.InverseTransformVector(physicsWorldVelocity)
            : physicsWorldVelocity;
        return observedVisualParent
            ? observedVisualParent.TransformVector(local) : local;
    }

    // At the CURRENT frame pose, map both the previous and current InSubject
    // positions. This separates physical displacement from root-frame motion.
    // Diagnostic ONLY: no feedback to the oscillation or spline geometry.
    void AuditPhysicalObservation(Vector3 mapped, float dt) {
        if (!observationSampleReady || dt <= 0f) {
            observationMappedVelocity = observationMappedSourceVelocity = Vector3.zero;
            observationMappedFrameVelocity = observationMappedRigidVelocity = Vector3.zero;
            observationRigidbodyVelocityError = 0f;
            return;
        }
        Vector3 prevMappedAtCurrentFrames =
            MapObservedPhysicsPointAtCurrentFrames(observationPreviousPhysicsPoint);
        observationMappedSourceVelocity =
            (mapped - prevMappedAtCurrentFrames) / dt;
        observationMappedFrameVelocity =
            (prevMappedAtCurrentFrames - observationPreviousMappedPoint) / dt;
        observationMappedVelocity =
            (mapped - observationPreviousMappedPoint) / dt;
        observationMappedRigidVelocity = MapObservedPhysicsVelocity(inSubject.velocity);
        observationRigidbodyVelocityError =
            (observationMappedSourceVelocity - observationMappedRigidVelocity).magnitude;
    }

    // V21.2: follow the continuous PHYSICAL observation along an existing
    // Knot polyline. Allow a valid station to recover after temporarily
    // missing the path; never relabel a teleport as normal station travel.
    bool TryObservationStation(Pair pair, Vector3 mappedBall, float dt,
        out float station, out float crossTrack) {
        station = 0f;
        crossTrack = float.PositiveInfinity;
        observationRejectReason = "NONE";
        observationCandidateStation = 0f;
        observationAllowedTravel = 0f;
        if (pair == null || pair.axis.Count < 2 ||
            pair.axis.Count != pair.stations.Count) {
            observationRejectReason = "INVALID_PATH";
            return false;
        }
        // Pair objects may be rebuilt with the same group and topology. Do
        // not discard a valid observation simply because their identity changed.
        bool newTopology = observationStationGroup != pair.group ||
            observationPairKnotCount != pair.axis.Count;
        if (newTopology) {
            observationStationReady = false;
            observationSampleReady = false;
        }
        observationStationGroup = pair.group;
        observationPairKnotCount = pair.axis.Count;
        // Time since last ACCEPTED station, not just a single FixedUpdate.
        // Otherwise a one-frame miss can trap the tracker permanently.
        float elapsed = observationStationReady
            ? Mathf.Max(dt, Time.fixedTime - observationLastStationTime) : dt;
        float rbSpeed = inSubject ? inSubject.velocity.magnitude : 0f;
        observationAllowedTravel = Mathf.Max(observationMinimumStationTravel,
            rbSpeed * elapsed * observationVelocityWindowFactor + .25f);
        float bestScore = float.PositiveInfinity;
        bool found = false;
        for (int i = 0; i + 1 < pair.axis.Count; i++) {
            Vector3 a = pair.axis[i], d = pair.axis[i+1] - a;
            Vector3 flat = new Vector3(d.x, 0f, d.z);
            if (flat.sqrMagnitude < E ||
                pair.stations[i+1] <= pair.stations[i]) continue;
            Vector3 delta = new Vector3(mappedBall.x-a.x,0f,mappedBall.z-a.z);
            float u = Mathf.Clamp01(Vector3.Dot(delta, flat) / flat.sqrMagnitude);
            Vector3 foot = a + d * u;
            float dx = mappedBall.x-foot.x, dz = mappedBall.z-foot.z;
            float dy = mappedBall.y-foot.y;
            float candidate = Mathf.Lerp(pair.stations[i],pair.stations[i+1],u);
            float error = dx*dx+dz*dz+.12f*dy*dy;
            if (observationStationReady) {
                float excess = Mathf.Max(0f,
                    Mathf.Abs(candidate-observationLastStation)-observationAllowedTravel);
                error += observationContinuityWeight * excess * excess;
            }
            if (error >= bestScore) continue;
            bestScore = error;
            station = candidate;
            crossTrack = Mathf.Sqrt(dx*dx+dz*dz);
            found = true;
        }
        if (!found) {
            observationRejectReason = "NO_VALID_SEGMENT";
            return false;
        }
        observationCandidateStation = station;
        if (crossTrack > observationCrossTrackWarning) {
            observationRejectReason = state == "PREPARED"
                ? "WAITING_FOR_ENTRY" : "CROSS_TRACK";
            return false;
        }
        // A large jump is NOT silently accepted. This never moves the
        // physics body and never modifies the visual wave state.
        if (observationStationReady &&
            Mathf.Abs(station-observationLastStation) > observationAllowedTravel) {
            observationRejectReason = "STATION_JUMP";
            return false;
        }
        observationStationStep = observationStationReady
            ? station - observationLastStation : 0f;
        observationLastStation = station;
        observationLastStationTime = Time.fixedTime;
        observationStationReady = true;
        return true;
    }

    // Resolve an existing VISUAL-space motion authority without opening an
    // ExampleBallVisualEqualizer component. The generated display ball is output
    // only; it is NEVER a source for its own translation.
    // Resolve only when the reference is missing, not every quarter second.
    void ResolveVisualMotionSource() {
        if (visualMotionSource && visualMotionSource != ownedVisualBall) return;
        GameObject visual = GameObject.Find("VisualPlayerRoot");
        Transform root = visual ? visual.transform : generatedVisualRoot;
        if (!root) return;
        Transform t = root.Find("subject");
        if (!t) t = root.Find("Subject");
        if (!t) {
            Transform[] children = root.GetComponentsInChildren<Transform>(true);
            for (int i=0; i<children.Length; i++) {
                if (children[i].name.Equals("subject", StringComparison.OrdinalIgnoreCase)) {
                    t=children[i];
                    break;
                }
            }
        }
        if (t && t != ownedVisualBall && (!inSubject || t != inSubject.transform))
            visualMotionSource=t;
    }




    // Only a VISUAL-side reference or a coordinate-frame transform determines
    // the visual base. Never copy the raw PhysicsRoot world position to a visual.
    // The source of truth is VisualPlayerRoot/subject. A complete, EXPLICIT
    // physics/visual frame pair is the only permitted alternative.
    // Missing both: do not render at PhysicsRoot world coordinates.
    bool TryGetVisualMotionPoint(out Vector3 visualPoint) {
        if (visualMotionSource && visualMotionSource != ownedVisualBall) {
            visualPoint=visualMotionSource.position;
            return true;
        }
        if (HasSource && physicsFrame && visualFrame) {
            visualPoint=ToVisualPoint(SourcePhysicsPoint);
            return true;
        }
        visualPoint=Vector3.zero;
        return false;
    }


    static bool IsSafeVisualOutput(Transform t) {
        return t && !t.GetComponentInParent<Rigidbody>() &&
            !t.GetComponentInChildren<Rigidbody>(true) &&
            !t.GetComponentInChildren<Collider>(true);
    }

    void EnsureIndependentVisualBall() {
        if (mainBveMode) return;
        if (displayedBall && displayedBall != ownedVisualBall &&
            !IsMainBveDisplay(displayedBall)) return;
        // An explicitly assigned, independent nonphysical visual is allowed.
        // Never reuse the Main BVE generated output as standalone input/output.
        if (inspectorVisualBall && inspectorVisualBall != visualMotionSource &&
            !IsMainBveDisplay(inspectorVisualBall) &&
            IsSafeVisualOutput(inspectorVisualBall)) {
            displayedBall = inspectorVisualBall;
            if (ownedVisualBall) ownedVisualBall.gameObject.SetActive(false);
            return;
        }
        if (!createOwnVisualBall) { displayedBall = null; return; }
        if (!TryGetVisualMotionPoint(out Vector3 point)) { displayedBall = null; return; }
        if (!ownedVisualBall) {
            GameObject go = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            go.name = "IFSW_BallVisualEqualizer";
            // Primitive temporarily contains a collider. Disable immediately;
            // a visual-only ball must never affect SlopeStickCore physics.
            Collider collider = go.GetComponent<Collider>();
            if (collider) { collider.enabled = false; Destroy(collider); }
            go.transform.SetParent(owner, true);
            MeshFilter filter = go.GetComponent<MeshFilter>();
            if (ownBallMesh && filter) filter.sharedMesh = ownBallMesh;
            MeshRenderer renderer = go.GetComponent<MeshRenderer>();
            Shader shader = Shader.Find("Universal Render Pipeline/Lit");
            if (!shader) shader = Shader.Find("Standard");
            if (!shader) shader = Shader.Find("Unlit/Color");
            if (renderer && (ownBallMaterial || shader)) {
                ownedVisualMaterial = ownBallMaterial
                    ? new Material(ownBallMaterial) : new Material(shader);
                ownedVisualMaterial.name = "IFSW_StandaloneBVE_Material";
                if (!ownBallMaterial) {
                    Color color = new Color(.94f,.96f,1f,1f);
                    if (ownedVisualMaterial.HasProperty("_BaseColor"))
                        ownedVisualMaterial.SetColor("_BaseColor",color);
                    if (ownedVisualMaterial.HasProperty("_Color"))
                        ownedVisualMaterial.SetColor("_Color",color);
                }
                renderer.sharedMaterial = ownedVisualMaterial;
                renderer.shadowCastingMode = ShadowCastingMode.Off;
            }
            ownedVisualBall = go.transform;
            if (debugVisualBinding)
                Debug.Log("[IFSW][VISUAL_CREATED] IFSW_BallVisualEqualizer (no Rigidbody/Collider)",this);
        }
        displayedBall = ownedVisualBall;
        ownedVisualBall.gameObject.SetActive(true);
        displayedBall.localScale = Vector3.one * Mathf.Max(.02f,2f*visualRadius);
        displayedBall.position = point;
        bveVisualPresentationHasSample = false;
    }

    // Never import the ExampleBallVisualEqualizer type: the Weaver must still
    // compile and run after that script/component is physically removed.
    // Checking active MonoBehaviour type names does not read/change BVE state.


    static bool IsMainBveDisplay(Transform t) =>
        t && (t.name.StartsWith("ExampleBVE_Display_", StringComparison.Ordinal) ||
              t.name == "BallVisualEqualizer");



    Transform FindMainBveDisplay() {
        GameObject go = GameObject.Find("ExampleBVE_Display_BallVisualEqualizer");
        if (go && go.transform != ownedVisualBall) return go.transform;
        // A deliberately assigned visual can be used as a read-only Main BVE output.
        return inspectorVisualBall && inspectorVisualBall != ownedVisualBall
            ? inspectorVisualBall : null;
    }



    // Status is informational, never a gate for spline creation.
    void UpdateVisualStatus() {
        visualMode=mainBveMode ? "MAIN_BVE_READ_ONLY" : "NONINERTIAL_STANDALONE";
        visualBindingStatus=(displayedBall ? displayedBall.name : "WAITING_VISUAL")+
            " | "+(visualMotionSource ? "VISUAL_SUBJECT" :
                physicsFrame && visualFrame ? "EXPLICIT_FRAMES" : "NO_VISUAL_SOURCE");
        if (debugVisualBinding)
            Debug.Log("[IFSW][VISUAL] "+visualMode+" | "+visualBindingStatus,this);
    }

    // Non-inertial normal offset only in STANDALONE mode; never write Main BVE output.
    // Never write InSubject or a Rigidbody hierarchy.
    void UpdateDrivenVisual() {
        if (mainBveMode || !displayedBall || displayedBall == subjectFallback ||
            !IsSafeVisualOutput(displayedBall)) return;
        // Only the observed InSubject drives the non-inertial offset. The
        // independent visual carrier is a baseline, not a measured target.
        // Renderer-only filtering is independent.
        Vector3 normal = sonarWaveNormal.sqrMagnitude > E ? sonarWaveNormal.normalized : Vector3.up;
        float raw = driveVisualWithKnotSonar && HasSource ? sonarWaveHeight : 0f;
        if (resetFilterOnSharpNormalChange && bvePresentationReady &&
            Vector3.Angle(normal,bvePreviousPresentationNormal)>=filterNormalResetDegrees)
            ResetBvePresentation(raw);
        bvePreviousPresentationNormal=normal;
        // Presentation filtering only: the non-inertial ODE sets the wave.
        // Spatial phase affects inferred frame acceleration, not this filter.
        float baseSeconds = flightSmoothSeconds;
        if (softenVisualWave && driveVisualWithKnotSonar) {
            bvePresentationSeconds = adaptiveVisualFilter
                ? AdaptiveBveSeconds(raw,baseSeconds,Time.deltaTime) : baseSeconds;
            if (!adaptiveVisualFilter) bveFilterReady=false;
            // The old smoothing was tuned for ~1.4 Hz and could erase the
            // 6-9 Hz stair-frame waves. Bound smoothing by measured spatial
            // frequency, without affecting physics or any owned spline.
            if (terrainFrameAmplitude > .001f && terrainFrameSourceHz > 2f)
                bvePresentationSeconds = Mathf.Min(bvePresentationSeconds,
                    Mathf.Clamp(.065f/terrainFrameSourceHz,.004f,.012f));
            bvePresentedOffset = Mathf.SmoothDamp(bvePresentedOffset,raw,
                ref bvePresentedSpeed,bvePresentationSeconds,Mathf.Infinity,Time.deltaTime);
        } else {
            bvePresentationSeconds=0f;
            bvePresentedOffset=raw;
            bvePresentedSpeed=0f;
            bveFilterReady=false;
        }
        bvePresentationReady=true;

        // The self-owned output follows an independently read VISUAL trajectory.
        // A previous-frame wave offset never feeds the travel/lifecycle source.
        if (displayedBall == ownedVisualBall) {
            if (TryGetVisualMotionPoint(out Vector3 visualBase)) {
                // Independent Visual Carrier = output baseline ONLY.
                // InSubject = physical observation ONLY. Prevent double-bounce.
                displayedBall.position = visualBase +
                    MapStandaloneDirection(Vector3.up) * bveInitialCY +
                    normal * bvePresentedOffset;
            }
            return;
        }
        // Keep the existing component-free visual driver intact. Only an
        // incremental normal displacement is added to its externally set pose.
        Transform visualParent = displayedBall.parent;
        Vector3 visualBaseLocal = displayedBall.localPosition;
        if (bveVisualPresentationHasSample && visualParent == bveLastVisualParent &&
            (visualBaseLocal-bveLastWrittenLocalPosition).sqrMagnitude < .00000025f)
            visualBaseLocal -= bveLastAppliedLocalOffset;
        Vector3 waveDisplacement = normal * bvePresentedOffset +
            MapStandaloneDirection(Vector3.up) * bveInitialCY;
        Vector3 localOffset = visualParent
            ? visualParent.InverseTransformVector(waveDisplacement)
            : waveDisplacement;
        displayedBall.localPosition = visualBaseLocal + localOffset;
        bveLastVisualParent = visualParent;
        bveLastAppliedLocalOffset = localOffset;
        bveLastWrittenLocalPosition = displayedBall.localPosition;
        bveVisualPresentationHasSample = true;
    }

    void ResetBvePresentation(float offset) {
        bvePresentedOffset=offset;
        bvePresentedSpeed=0f;
        bvePreviousFilterTarget=offset;
        bveAdaptiveVisualSpeed=0f;
        bveFilterReady=false;
        bvePresentationReady=false;
    }

    float AdaptiveBveSeconds(float target,float baseSeconds,float dt) {
        float h=Mathf.Max(.0001f,dt);
        if (!bveFilterReady) { bvePreviousFilterTarget=target; bveAdaptiveVisualSpeed=0f; bveFilterReady=true; }
        float rawSpeed=(target-bvePreviousFilterTarget)/h;
        bvePreviousFilterTarget=target;
        float alpha=1f-Mathf.Exp(-2f*Mathf.PI*adaptiveDerivativeHz*h);
        bveAdaptiveVisualSpeed+=(rawSpeed-bveAdaptiveVisualSpeed)*alpha;
        float baseTime=Mathf.Max(.005f,baseSeconds);
        float cutoff=1f/(2f*Mathf.PI*baseTime)+adaptiveSpeedGain*Mathf.Abs(bveAdaptiveVisualSpeed);
        return Mathf.Clamp(1f/(2f*Mathf.PI*Mathf.Max(E,cutoff)),
            Mathf.Min(adaptiveMinimumSmoothSeconds,baseTime),baseTime);
    }

    // Transport-theorem tracking: only the OBSERVER is changed. A moving normal
    // must not turn a stationary ball into a false bounce. No Kalman/One-Euro
    // filter, additional ray probes, spline writes, or rigidbody writes.
    void ResetKnotTracking()
    {
        trackingValid = false;
        trackingKnotIndex = -1;
        trackingNearbyKnots = 0;
        trackingGroup = -1;
        trackingHasLastFrame = false;
        trackingBaseline = trackingTrend = trackingEstimate = trackingEstimateSpeed = 0f;
        trackingComovingHeight = trackingPhysicalSpeed = trackingFrameSpeed = 0f;
        trackingCoordinateSpeed = trackingReferenceSpeed = 0f;
        trackingLastNormal = Vector3.up;
        trackingLastRelative = trackingLastReference = Vector3.zero;
        trackingTangent = Vector3.forward;
        trackingNormal = Vector3.up;
        trackingSide = Vector3.right;
        trackingRelativeHeight = trackingRelativeSpeed = trackingWaveObservation = 0f;
        trackingRawHeight = observationHeightRateError = 0f;
    }

    // Knot positions/stations, carrier and tracked ball positions share VISUAL
    // world coordinates. The produced display-ball position is never an input.
    void ObserveKnotTracking(int currentGroup, IList<Vector3> knots, IList<float> stations,
        float station, Vector3 physicalBall, float dt,
        float radius, float alpha, float beta)
    {
        if (knots == null || stations == null || knots.Count < 2 ||
            knots.Count != stations.Count || dt <= 0f)
        {
            ResetKnotTracking();
            return;
        }
        if (trackingGroup != currentGroup)
        {
            ResetKnotTracking();
            trackingGroup = currentGroup;
        }

        int nearest = 0;
        float minDistance = float.PositiveInfinity;
        for (int i = 0; i < stations.Count; i++)
        {
            float d = Mathf.Abs(stations[i] - station);
            if (d < minDistance) { minDistance = d; nearest = i; }
        }
        int previousKnotIndex = trackingKnotIndex;
        trackingKnotIndex = nearest;
        trackingNearbyKnots = 0;
        float totalWeight = 0f;
        Vector3 direction = Vector3.zero;
        float spatialRadius = Mathf.Max(.1f, radius);
        for (int i = Mathf.Max(0, nearest - 2); i <= Mathf.Min(knots.Count - 1, nearest + 2); i++)
        {
            float distance = stations[i] - station;
            float weight = Mathf.Exp(-.5f * distance * distance / (spatialRadius * spatialRadius));
            if (Vector3.Distance(knots[i], physicalBall) <= spatialRadius * 2f)
                trackingNearbyKnots++;
            int a = Mathf.Max(0, i - 1);
            int b = Mathf.Min(knots.Count - 1, i + 1);
            Vector3 segment = knots[b] - knots[a];
            if (segment.sqrMagnitude > .00001f)
            {
                direction += segment.normalized * weight;
                totalWeight += weight;
            }
        }
        // Never assimilate a bad observation. During missing contact or Knot
        // references, the non-inertial oscillator decays without false input.
        // Nearby count describes sensor coverage, NOT geometric validity.
        // The continuous station and neighbouring Knot segments are enough to
        // define the observer even when the ball is between discrete knots.
        if (totalWeight < .00001f || direction.sqrMagnitude < .00001f)
        {
            ResetKnotTracking();
            observationRejectReason = "DEGENERATE_TANGENT";
            return;
        }

        Vector3 nextT = direction.normalized;
        Vector3 transported = trackingHasLastFrame
            ? Quaternion.FromToRotation(trackingTangent, nextT) * trackingNormal
            : Vector3.ProjectOnPlane(Vector3.up, nextT);
        Vector3 up = Vector3.ProjectOnPlane(Vector3.up, nextT).normalized;
        if (transported.sqrMagnitude < .00001f) transported = up;
        if (up.sqrMagnitude < .00001f) up = transported.normalized;
        if (Vector3.Dot(transported, up) < 0f) up = -up;
        trackingTangent = nextT;
        // No periodic rotation toward world-up: retain the transported
        // comoving normal. Up is used only for initial/degenerated frames.
        trackingNormal = Vector3.ProjectOnPlane(
            transported.normalized, trackingTangent).normalized;
        trackingSide = Vector3.Cross(trackingNormal, trackingTangent).normalized;

        int segmentIndex = Mathf.Clamp(nearest, 0, knots.Count - 2);
        if (stations[nearest] > station && segmentIndex > 0) segmentIndex--;
        float length = stations[segmentIndex + 1] - stations[segmentIndex];
        float u = length > .00001f ?
            Mathf.Clamp01((station - stations[segmentIndex]) / length) : 0f;
        trackingReference = Vector3.Lerp(knots[segmentIndex], knots[segmentIndex + 1], u);
        Vector3 relative = physicalBall - trackingReference;
        float rawHeight = Vector3.Dot(relative, trackingNormal);
        trackingRawHeight = rawHeight;
        observationHeightRateError = 0f;
        if (!trackingHasLastFrame)
        {
            trackingComovingHeight = rawHeight;
            trackingBaseline = trackingEstimate = rawHeight;
            trackingTrend = trackingEstimateSpeed = 0f;
            trackingLastRelative = relative;
            trackingLastReference = trackingReference;
            trackingLastNormal = trackingNormal;
            trackingHasLastFrame = true;
            trackingPhysicalSpeed = trackingFrameSpeed = trackingCoordinateSpeed = 0f;
            trackingReferenceSpeed = 0f;
        }
        else
        {
            // A change in track reference or coordinate frame is not a real
            // acceleration. Rebase, do not inject its jump into the wave.
            Vector3 relativeStep = relative - trackingLastRelative;
            float angularJump = Vector3.Angle(trackingLastNormal, trackingNormal);
            if (Mathf.Abs(nearest - previousKnotIndex) > 2 ||
                relativeStep.magnitude > Mathf.Max(1.5f, radius * 1.5f) ||
                angularJump > 60f)
            {
                // A corner or regenerated section may change its reference
                // abruptly. Rebase this observer WITHOUT manufacturing an
                // acceleration impulse and WITHOUT resetting the wave.
                observationRejectReason = angularJump > 60f ? "FRAME_REBASE" :
                    Mathf.Abs(nearest - previousKnotIndex) > 2
                        ? "KNOT_INDEX_REBASE" : "REFERENCE_REBASE";
                observationRebaseCount++;
                trackingComovingHeight = trackingBaseline = trackingEstimate = rawHeight;
                trackingTrend = trackingEstimateSpeed = 0f;
                trackingLastRelative = relative;
                trackingLastReference = trackingReference;
                trackingLastNormal = trackingNormal;
                trackingHasLastFrame = true;
                trackingRelativeHeight = trackingRelativeSpeed = 0f;
                trackingWaveObservation = 0f;
                trackingPhysicalSpeed = trackingFrameSpeed = trackingCoordinateSpeed = 0f;
                trackingReferenceSpeed = 0f;
                trackingValid = false; // skip impulse for this rebase frame
                return;
            }

            // Exact finite-difference transport-identity for q = r dot N:
            // (q[k]-q[k-1])/dt = (dr/dt) dot N[k]
            //                   + r[k-1] dot (dN/dt).
            // First term: relative motion of ball vs path-reference point.
            // Second term: apparent height change due ONLY to frame rotation.
            // Both contain no inferred acceleration and no accumulated bias.
            trackingPhysicalSpeed = Vector3.Dot(relativeStep, trackingNormal) / dt;
            trackingFrameSpeed = Vector3.Dot(
                trackingLastRelative, trackingNormal - trackingLastNormal) / dt;
            trackingCoordinateSpeed = trackingPhysicalSpeed + trackingFrameSpeed;
            trackingReferenceSpeed = (trackingReference - trackingLastReference).magnitude / dt;
            float previousRawHeight = Vector3.Dot(trackingLastRelative, trackingLastNormal);
            // Algebraic consistency check for transport theorem (NOT proof of
            // a correct physics/visual transform). Compare to finite-diff q.
            observationHeightRateError =
                (rawHeight - previousRawHeight) / dt - trackingCoordinateSpeed;

            // Track frame-corrected displacement of the physical ball, not raw q. This prevents
            // frame rotation alone from producing a fake rising/falling wave.
            trackingComovingHeight += trackingPhysicalSpeed * dt;
            float prediction = trackingEstimate + trackingEstimateSpeed * dt;
            float innovation = Mathf.Clamp(trackingComovingHeight - prediction, -1.5f, 1.5f);
            trackingEstimate = prediction + Mathf.Clamp01(alpha) * innovation;
            // Measured TRANSLATIONAL speed, no /dt innovation amplification.
            trackingEstimateSpeed = Mathf.Lerp(trackingEstimateSpeed,
                trackingPhysicalSpeed, Mathf.Clamp01(beta));
            trackingLastRelative = relative;
            trackingLastReference = trackingReference;
            trackingLastNormal = trackingNormal;
        }
        trackingRelativeHeight = trackingEstimate - trackingBaseline;
        trackingRelativeSpeed = trackingEstimateSpeed;
        trackingTrend = Mathf.Lerp(trackingTrend, trackingRelativeHeight,
            1f - Mathf.Exp(-2f * Mathf.PI * 1.3f * dt));
        trackingWaveObservation = Mathf.Clamp(trackingRelativeHeight - trackingTrend,
            -nonInertialMaxOffset, nonInertialMaxOffset);
        trackingValid = true;
    }

    // At most five read-only Knot markers for editor gizmos; the tracking observer uses
    // the ACTUAL n-th knot and its +/-2 neighbours, without making GameObjects.
    void ChooseKnotSonarAnchors(Vector3 visual, float observedStation = float.NaN)
    {
        sonarAnchorCount = sonarNearbyKnots = 0;
        if (active == null || active.axis.Count < 2) return;
        float station = float.IsNaN(observedStation)
            ? StationOn(active, visual, out _, out _) : observedStation;
        int n = 0;
        float best = float.PositiveInfinity;
        for (int i = 0; i < active.stations.Count; ++i)
        {
            float d = Mathf.Abs(active.stations[i] - station);
            if (d < best) { best = d; n = i; }
        }
        for (int i = Mathf.Max(0,n-2); i <= Mathf.Min(active.axis.Count-1,n+2); ++i)
        {
            Vector3 knot = active.axis[i];
            sonarAnchors[sonarAnchorCount] = knot;
            sonarAnchorIsObservation[sonarAnchorCount] = false;
            sonarAnchorCount++;
            if ((knot-visual).sqrMagnitude <= knotSonarRadius*knotSonarRadius)
                sonarNearbyKnots++;
        }
        sonarStation = station;
    }

    void ObserveKnotTarget(float dt)
    {
        // Display motion is optional telemetry; NEVER required to observe
        // InSubject when explicit frames are provided.
        bool hasCarrier = TryGetVisualMotionPoint(out Vector3 carrier);
        if (!TryMapObservedInSubject(out Vector3 mappedPhysical)) {
            ResetKnotTracking();
            ResetObservationVerification();
            sonarAnchorCount = sonarNearbyKnots = 0;
            if (debugPhysicalObservation && Time.fixedTime >= nextMissingMapLog) {
                nextMissingMapLog = Time.fixedTime + 2f;
                Debug.LogWarning("[IFSW][OBSERVATION_DISABLED] " + observedMapSource +
                    ". Configure InSubject and a valid physics/visual mapping.", this);
            }
            return;
        }
        if (active == null || active.axis.Count < 2 ||
            (state != "PREPARED" && state != "ACTIVE") ||
            (bveActive && !bveExiting && active.group != bveCycleGroup)) {
            ResetKnotTracking();
            ResetObservationVerification();
            sonarAnchorCount = sonarNearbyKnots = 0;
            return;
        }
        if (!TryObservationStation(active, mappedPhysical, dt,
            out float station, out float crossTrack)) {
            // Preserve last accepted station/time to allow local reacquisition.
            // Do not derive a velocity across a missing observation; the
            // independent oscillator continues to decay on its own.
            trackingValid = false;
            trackingNearbyKnots = 0;
            trackingHasLastFrame = false;
            observationSampleReady = false;
            observationRejectedFrames++;
            sonarAnchorCount = sonarNearbyKnots = 0;
            if (debugPhysicalObservation && Time.fixedTime >= nextMissingMapLog) {
                nextMissingMapLog = Time.fixedTime+2f;
                Debug.LogWarning($"[IFSW][OBSERVATION_REJECT] " +
                    $"reason={observationRejectReason} group={active.group} " +
                    $"candidateStation={observationCandidateStation:F3} " +
                    $"lastStation={observationLastStation:F3} " +
                    $"allowedTravel={observationAllowedTravel:F3} " +
                    $"crossTrack={crossTrack:F3} " +
                    $"misses={observationRejectedFrames}", this);
            }
            return;
        }
        if (observationSampleReady)
            AuditPhysicalObservation(mappedPhysical, dt);
        else {
            observationMappedVelocity = observationMappedSourceVelocity = Vector3.zero;
            observationMappedFrameVelocity = observationMappedRigidVelocity = Vector3.zero;
            observationRigidbodyVelocityError = 0f;
        }
        // Choose AND measure the same InSubject-derived station. Other
        // consumers of ChooseKnotSonarAnchors retain original behaviour.
        ChooseKnotSonarAnchors(mappedPhysical, station);
        ObserveKnotTracking(active.group,active.axis,active.stations,station,
            mappedPhysical,dt,knotSonarRadius,trackingAlpha,trackingBeta);
        if (!trackingValid) {
            // An accepted station can still require an observer-frame rebase.
            // Never integrate a discontinuous q/dq into the wave.
            observationSampleReady = false;
            observationRejectedFrames++;
            if (debugPhysicalObservation && Time.fixedTime >= nextMissingMapLog) {
                nextMissingMapLog = Time.fixedTime+2f;
                string eventName = observationRejectReason.EndsWith("REBASE")
                    ? "OBSERVATION_REBASE" : "OBSERVATION_REJECT";
                Debug.LogWarning($"[IFSW][{eventName}] " +
                    $"reason={observationRejectReason} source={observedMapSource} " +
                    $"group={active.group} station={station:F3} " +
                    $"mapped={mappedPhysical:F3} crossTrack={crossTrack:F3} " +
                    $"near={trackingNearbyKnots} misses={observationRejectedFrames} " +
                    $"rebases={observationRebaseCount}", this);
            }
            return;
        }
        if (observationRejectedFrames > 0 && debugPhysicalObservation &&
            Time.fixedTime >= nextObservationRecoveryLog) {
            nextObservationRecoveryLog = Time.fixedTime + 1f;
            Debug.Log($"[IFSW][OBSERVATION_RECOVERED] source={observedMapSource} " +
                $"group={active.group} station={station:F3} " +
                $"missedFrames={observationRejectedFrames}", this);
        }
        observationRejectedFrames = 0;
        observationPreviousPhysicsPoint = inSubject.position;
        observationPreviousMappedPoint = mappedPhysical;
        observationSampleReady = true;
        if (debugPhysicalObservation && Time.fixedTime >= nextKnotTrackingLog) {
            nextKnotTrackingLog = Time.fixedTime + Mathf.Max(.1f, trackingLogSeconds);
            Debug.Log($"[IFSW][OBSERVATION] t={Time.fixedTime:F3} " +
                $"source={observedMapSource} group={active.group} " +
                $"physicalPosition={inSubject.position:F4} " +
                $"mappedPosition={mappedPhysical:F4} " +
                $"carrier={(hasCarrier ? carrier.ToString("F4") : "NA")} " +
                $"knotIndex={trackingKnotIndex} near={trackingNearbyKnots} " +
                $"station={station:F4} ds={observationStationStep:F4} " +
                $"crossTrack={crossTrack:F4} allowedTravel={observationAllowedTravel:F4} " +
                $"status=TRACKING near0Allowed=True " +
                $"T={trackingTangent:F4} N={trackingNormal:F4} B={trackingSide:F4} " +
                $"relativeHeight={trackingRelativeHeight:F4} rawHeight={trackingRawHeight:F4} " +
                $"measuredNormalSpeed={trackingPhysicalSpeed:F4} " +
                $"frameRotationSpeed={trackingFrameSpeed:F4} " +
                $"coordinateSpeed={trackingCoordinateSpeed:F4} " +
                $"velocityConsistencyError={observationHeightRateError:F6} " +
                $"mappedVelocity={observationMappedVelocity:F4} " +
                $"physicalMotion={observationMappedSourceVelocity:F4} " +
                $"mappingFrameMotion={observationMappedFrameVelocity:F4} " +
                $"rbVelocity={observationMappedRigidVelocity:F4} " +
                $"rbVelocityError={observationRigidbodyVelocityError:F4} " +
                $"osc={trackingWaveObservation:F4}", this);
            if (debugKnotTracking)
                Debug.Log($"[IFSW][KNOT_TRACK] t={Time.fixedTime:F3} " +
                    $"source={observedMapSource} group={active.group} n={trackingKnotIndex} " +
                    $"near={trackingNearbyKnots} q={trackingRelativeHeight:F4} " +
                    $"dq={trackingRelativeSpeed:F4} vRel={trackingPhysicalSpeed:F4} " +
                    $"vFrame={trackingFrameSpeed:F4} vCoord={trackingCoordinateSpeed:F4} " +
                    $"vRef={trackingReferenceSpeed:F4} osc={trackingWaveObservation:F4} " +
                    $"T={trackingTangent:F4} N={trackingNormal:F4}", this);
        }
        // Existing KNOT_TRACK makes V21.1 logs comparable with V21 runs.
        if (trackingValid && debugKnotTracking && !debugPhysicalObservation &&
            Time.fixedTime >= nextKnotTrackingLog) {
            nextKnotTrackingLog = Time.fixedTime + Mathf.Max(.1f,trackingLogSeconds);
            Debug.Log($"[IFSW][KNOT_TRACK] t={Time.fixedTime:F3} " +
                $"source={observedMapSource} group={active.group} n={trackingKnotIndex} " +
                $"near={trackingNearbyKnots} q={trackingRelativeHeight:F4} " +
                $"dq={trackingRelativeSpeed:F4} vRel={trackingPhysicalSpeed:F4} " +
                $"vFrame={trackingFrameSpeed:F4} vCoord={trackingCoordinateSpeed:F4} " +
                $"vRef={trackingReferenceSpeed:F4} osc={trackingWaveObservation:F4} " +
                $"T={trackingTangent:F4} N={trackingNormal:F4}", this);
        }
    }

    // Original two-ray entry sensor is retained for stable stage detection;
    // the independent terrain-frame observer can add smooth spatial forcing.
    // This is NOT an OverlapSphere search: the observed physics ball is known.
    void ObserveMinimalContact(Vector3 position, Vector3 velocity)
    {
        Vector3 planar = Vector3.ProjectOnPlane(velocity, Vector3.up);
        if (planar.sqrMagnitude > .01f) bveHeading = planar.normalized;
        float spacing = Mathf.Max(.1f, probeSpacing * 2f);
        int hits = 0;
        float behind = 0f, ahead = 0f;
        for (int i = 0; i < 2; i++)
        {
            Vector3 origin = position + bveHeading * ((i-.5f)*spacing) + Vector3.up*probeHeight;
            int n = Physics.RaycastNonAlloc(origin, Vector3.down, sonarHits,
                probeHeight+probeDepth, groundMask, QueryTriggerInteraction.Ignore);
            float closest = float.PositiveInfinity;
            bveSampleColliders[i] = null;
            float height = 0f;
            for (int j=0;j<n;j++)
            {
                RaycastHit hit = sonarHits[j];
                Collider c=hit.collider;
                if (!c || (inSubject && c.attachedRigidbody==inSubject) ||
                    hit.distance >= closest) continue;
                closest = hit.distance;
                bveSampleColliders[i] = c;
                height = hit.point.y;
            }
            if (closest < float.PositiveInfinity) { hits++; if (i==0) behind=height; else ahead=height; }
        }
        bveProbeValid = hits==2;
        sonarGroundContacts = hits;
        contactFraction = hits*.5f;
        sonarGeometryDetected = bveProbeValid;
        bveGradient = bveProbeValid ? (ahead-behind)/spacing : 0f;
        bveSlopeDegrees = -Mathf.Atan(bveGradient)*Mathf.Rad2Deg;
        sonarDetectedGroundY = bveProbeValid ? (behind+ahead)*.5f : 0f;
    }

    // The moving Knot frame drives the wave, backed by observed REAL steps.
    // No fixed A1/A2/A3 heights, no 123/23 wave forcing, no FutureSpline feedback. ResolveBveCycle
    // only labels where we are along the stair for old-compatible logs.
    void UpdateBveKnotSonar(Vector3 point, Vector3 velocity, float dt)
    {
        ObserveKnotTarget(dt);
        ObserveMinimalContact(point, velocity); // only entry / section identification
        Vector3 planar = Vector3.ProjectOnPlane(velocity, Vector3.up);
        if (!bveActive || bveExiting)
        {
            bveFlatElapsed = bveProbeValid && bveSlopeDegrees < bveEntrySlopeDegrees*.5f
                ? bveFlatElapsed + dt : 0f;
            if (bveFlatElapsed > .12f && Time.fixedTime - bveLastBegin > .4f)
                bveArmed = true;
        }
        bool justBegan = false;
        if ((!bveActive || bveExiting) && bveAutoBeginOnDownhill && bveArmed &&
            bveProbeValid && bveSlopeDegrees >= bveEntrySlopeDegrees &&
            planar.magnitude >= bveMinimumEntryPlanarSpeed)
        {
            BeginBveWave();
            justBegan = bveActive && !bveExiting;
        }
        if (!bveActive)
        {
            // Consume and discard off-stair contacts, preventing stale impulses
            // from exciting the next region after a fresh entry.
            ObservePhysicsContactExcitation(dt);
            terrainFrameAcceleration = terrainFrameAmplitude = 0f;
            // Free solver is never fed by a rendered output. Renderer filtering
            // may smoothly return the visual sphere to the independent carrier.
            nonInertialOffset = nonInertialVelocity = nonInertialTarget = 0f;
            nonInertialHadInput = false;
            niFrameHistoryValid=niFrameHasVelocitySample=false;
            bveBaseOffset = bveResidualOffset = bveTerminalOffset = 0f;
            bveRawOffset = sonarWaveHeight = 0f;
            bveWaveLocal01 = 1f;
            bvePreviousPhysicsPoint = point;
            return;
        }
        Vector3 delta = justBegan ? Vector3.zero : point - bvePreviousPhysicsPoint;
        bveElapsed += dt;
        if (bveExiting) bveExitElapsed += dt;
        bveTraveled += Mathf.Max(0f, Vector3.Dot(delta, bveT));
        bveProgress = Mathf.Clamp01(bveTraveled / Mathf.Max(.1f, bveCycleSectionLength));
        int before = bveWaveIndex;
        ResolveBveCycle(bveProgress, out float u, out float waveLength, out int priorType);
        bveWaveLocal01 = u;
        BeginWaveEnergyRegion();
        if (debugBveTransitions && before != bveWaveIndex)
            Debug.Log($"[IFSW][NI_REGION] group={bveCycleGroup} region={bveWaveIndex} " +
                $"progress={bveProgress:F4} (diagnostic, NOT a prescribed bounce)", this);
        if (bveAutoEndAtSection && bveProgress >= 1f) EndBveWave();
        // Read contact AFTER Wave resolution so its energy is charged to the
        // segment in which the physical collision was observed.
        ObservePhysicsContactExcitation(dt);

        // A stale group or a temporarily invisible knot must never inject
        // energy into the ball. The oscillator simply decays to its baseline.
        bool measured = trackingValid && active != null &&
            active.group == bveCycleGroup && observationStationReady && !bveExiting;
        nonInertialTarget = measured
            ? Mathf.Clamp(trackingWaveObservation * nonInertialObservationGain,
                -nonInertialMaxOffset, nonInertialMaxOffset) : 0f;
        float spatialOmega;
        float terrainDrive = ObserveTerrainFrameAcceleration(point,velocity,dt,measured,u,waveLength,out spatialOmega);
        terrainFrameOmega = spatialOmega;
        nonInertialInputJump = 0f;
        if (measured)
        {
            // The impulse is a bounded change in measured relative normal
            // velocity, not an acceleration inferred across reference resets.
            float velocityInFrame = Mathf.Clamp(trackingPhysicalSpeed,
                -nonInertialMaxSpeed, nonInertialMaxSpeed);
            if (nonInertialHadInput)
            {
                // Do not add the same collision twice through velocity-difference and Collision.impulse.
                if (lastContactKick <= 0f) {
                    nonInertialInputJump = Mathf.Clamp(
                        velocityInFrame - nonInertialLastInputVelocity,
                        -nonInertialMaxSpeed * .25f, nonInertialMaxSpeed * .25f);
                    float beforeVelocityEnergy = OscillatorDiagnosticEnergy();
                    nonInertialVelocity += nonInertialInputJump * nonInertialImpactGain;
                    waveEnergyVelocityInput += OscillatorDiagnosticEnergy() - beforeVelocityEnergy;
                }
            }
            nonInertialLastInputVelocity = velocityInFrame;
            nonInertialHadInput = true;
        }
        else nonInertialHadInput = false;

        // Stable semi-implicit damped second-order system:
        // x''+2*zeta*omega*x'+omega^2*(x-gain*q_highpass)=0.
        // Angular speed, damping and drive depend on COMOVING measurements,
        // never on a fixed staircase wave number, amplitude or FutureSpline.
        float omega = Mathf.Max(2f * Mathf.PI * Mathf.Clamp(nonInertialFrequencyHz,.2f,5f),terrainFrameOmega);
        float damp = niHighFrequencyDamping;
        if (bveExiting) damp = Mathf.Max(1f, damp);
        // Integrate with short steps to avoid numerical instability even if
        // users increase fixedDeltaTime or frequency in the Inspector.
        int steps = Mathf.Clamp(Mathf.CeilToInt(dt * omega / .22f), 1, 12);
        float h = dt / steps;
        for (int i = 0; i < steps; i++)
        {
            // Work/loss estimates are diagnostics; do not feed them back.
            float diagnosticV = nonInertialVelocity;
            waveEnergyTargetWork += omega * omega * nonInertialTarget * diagnosticV * h;
            waveEnergyDampingLoss += 2f * damp * omega * diagnosticV * diagnosticV * h;
            float accel = omega * omega * (nonInertialTarget - nonInertialOffset)
                          - 2f * damp * omega * nonInertialVelocity
                          + terrainDrive; // -moving terrain-frame acceleration
            waveEnergyTerrainInput += terrainDrive * diagnosticV * h;
            if (i==0 && Mathf.Abs(terrainDrive) > .01f) waveEnergyTerrainCount++;
            nonInertialVelocity = Mathf.Clamp(nonInertialVelocity + accel * h,
                -niOscillatorMaxSpeed, niOscillatorMaxSpeed);
            nonInertialOffset = Mathf.Clamp(nonInertialOffset + nonInertialVelocity * h,
                -nonInertialMaxOffset, nonInertialMaxOffset);
            if (Mathf.Abs(nonInertialOffset) >= nonInertialMaxOffset &&
                Mathf.Sign(nonInertialOffset) == Mathf.Sign(nonInertialVelocity))
                nonInertialVelocity = 0f;
        }

        waveEnergySteps++;
        waveEnergyMaxAbsOffset = Mathf.Max(waveEnergyMaxAbsOffset, Mathf.Abs(nonInertialOffset));

        // Diagnostic only: check whether the physical impulse really reached
        // the oscillator and how much the spring/damper changed it this frame.
        if (contactTransferPending && debugContactTransfer) {
            float angle = trackingValid ?
                Vector3.Angle(trackingNormal, MapStandaloneDirection(bveN)) : -1f;
            float omegaForEnergy = omega;
            float energyBefore = .5f * (
                lastContactVelocityBefore * lastContactVelocityBefore +
                omegaForEnergy * omegaForEnergy *
                Mathf.Pow(lastContactOffsetAtKick - lastContactTargetAtKick, 2f));
            float energyAfter = .5f * (
                lastContactVelocityAfter * lastContactVelocityAfter +
                omegaForEnergy * omegaForEnergy *
                Mathf.Pow(lastContactOffsetAtKick - lastContactTargetAtKick, 2f));
            Debug.Log($"[IFSW][CONTACT_TRANSFER] t={Time.fixedTime:F3} " +
                $"group={bveCycleGroup} station={observationLastStation:F3} " +
                $"xAtKick={lastContactOffsetAtKick:F4} " +
                $"xAfterIntegration={nonInertialOffset:F4} " +
                $"velocityBeforeKick={lastContactVelocityBefore:F4} " +
                $"velocityAfterKick={lastContactVelocityAfter:F4} " +
                $"velocityAfterIntegration={nonInertialVelocity:F4} " +
                $"effectiveKick={lastContactKick:F4} " +
                $"target={nonInertialTarget:F4} " +
                $"energyAtKickBefore={energyBefore:F4} energyAtKickAfter={energyAfter:F4} " +
                $"observerToSolverNormalAngle={angle:F2}", this);
        }

        // V26: the integrated NONINERTIAL state owns 100% of the wave.
        // No Hermite height, no external BVE pose and no 20% residual overlay.
        bveBaseOffset=nonInertialOffset;
        bveResidualOffset=bveTerminalOffset=0f;
        if(debugExpressiveWave && measured &&
           (niGroupLogged!=bveCycleGroup || niRegionLogged!=bveWaveIndex)) {
            niGroupLogged=bveCycleGroup;
            niRegionLogged=bveWaveIndex;
            Debug.Log($"[IFSW][NI_REGION] group={bveCycleGroup} " +
                $"wave={bveWaveIndex} type={bveWaveType+1} u={u:F3} " +
                $"terrainEnvelope={terrainFrameAmplitude:F3} " +
                $"q={nonInertialOffset:F3} v={nonInertialVelocity:F3} " +
                $"target={nonInertialTarget:F3} aFrame={niFrameTransportAcceleration:F2} " +
                $"aDrive={terrainDrive:F2} apexHint=" +
                $"{(bveWaveIndex==0?expressiveFirstApex:expressiveLaterApex):F2} " +
                $"source=NONINERTIAL",this);
        }
        bveCurrentAmplitude = Mathf.Abs(bveBaseOffset);
        float entryGain = Mathf.SmoothStep(0f, 1f,
            bveElapsed / Mathf.Max(.01f, entryBlendSeconds));
        bveRawOffset = Mathf.Clamp((bveBaseOffset+bveResidualOffset)*entryGain,
            -nonInertialMaxOffset,nonInertialMaxOffset);
        sonarWaveHeight = bveRawOffset;
        if (measured)
        {
            Vector3 normal = UnmapStandaloneDirection(trackingNormal);
            if (normal.sqrMagnitude > E)
                bveN = Vector3.RotateTowards(bveN, normal.normalized,
                    Mathf.Deg2Rad * 180f * dt, 0f).normalized;
        }
        sonarWaveNormal = MapStandaloneDirection(bveN).normalized;
        // All three vectors are in VISUAL world space. The diagnostic doesn't
        // change the wave normal or any of the five splines.
        if (debugNormalAlignment && trackingValid && active != null &&
            active.group == bveCycleGroup &&
            Time.fixedTime >= nextNormalAlignmentLog) {
            Vector3 renderNormal = Vector3.up;
            if (TryGetVisualMotionPoint(out Vector3 carrierPoint) && active.axis.Count >= 2) {
                float renderStation = StationOn(active,carrierPoint,out _,out _);
                PointOn(active,renderStation,out Vector3 renderTangent);
                renderNormal = Vector3.ProjectOnPlane(Vector3.up,renderTangent).normalized;
            }
            float observerSolverAngle = Vector3.Angle(trackingNormal,sonarWaveNormal);
            float observerRenderAngle = Vector3.Angle(trackingNormal,renderNormal);
            float solverRenderAngle = Vector3.Angle(sonarWaveNormal,renderNormal);
            if (observerSolverAngle >= normalMismatchWarningDegrees ||
                solverRenderAngle >= normalMismatchWarningDegrees ||
                observerRenderAngle >= normalMismatchWarningDegrees) {
                Debug.Log($"[IFSW][NORMAL_ALIGNMENT] group={bveCycleGroup} " +
                    $"station={observationLastStation:F3} " +
                    $"observerSolverDeg={observerSolverAngle:F2} " +
                    $"observerRenderDeg={observerRenderAngle:F2} " +
                    $"solverRenderDeg={solverRenderAngle:F2} " +
                    $"observerN={trackingNormal:F4} " +
                    $"solverN={sonarWaveNormal:F4} " +
                    $"renderN={renderNormal:F4}", this);
            }
            nextNormalAlignmentLog = Time.fixedTime + .5f;
        }
        float speed = (bveRawOffset - bveLastOffset) / dt;
        if (!bveFirstApexLogged && bveLastOffsetSpeed > .02f && speed <= 0f &&
            bveLastOffset > .05f)
        {
            bveFirstApexLogged = true;
            if (debugBveTransitions)
                Debug.Log($"[IFSW][NI_APEX] group={bveCycleGroup} " +
                    $"progress={bveProgress:F4} offset={bveLastOffset:F4}", this);
        }
        bveLastOffsetSpeed = speed;
        bveLastOffset = bveRawOffset;
        if (debugBveState && Time.fixedTime >= bveNextStateLog)
        {
            bveNextStateLog = Time.fixedTime + .25f;
            Debug.Log($"[IFSW][NI_STATE] group={bveCycleGroup} " +
                $"region={bveWaveIndex} q={trackingRelativeHeight:F4} " +
                $"vRel={trackingPhysicalSpeed:F4} vFrame={trackingFrameSpeed:F4} " +
                $"drive={nonInertialTarget:F4} impulse={nonInertialInputJump:F4} " +
                $"contactImpulse={lastContactImpulsePerMass:F4} " +
                $"contactKick={lastContactKick:F4} " +
                $"x={nonInertialOffset:F4} dx={nonInertialVelocity:F4} " +
                $"valid={measured} raw={bveRawOffset:F4}", this);
        }
        bvePreviousPhysicsPoint = point;
        if (bveExiting && bveExitElapsed > .35f &&
            (Mathf.Abs(nonInertialOffset) < .01f &&
             Mathf.Abs(nonInertialVelocity) < .15f || bveExitElapsed > 2f))
            ResetBveWave(false);
    }

    // Siblings of a minimally sampled StairWay*_Physics determine the multiplier,
    // as in ExampleBVE. Spline/render partCount is ONLY a fallback if there is
    // no physics sibling match; the oscillation does not require the splines.
    int ResolveBvePhysicsMultiplier() {
        for(int k=0;k<4;k++) {
            int index=k==0?0:k==1?1:k==2?2:3;
            Collider collider=bveSampleColliders[index];
            if (!collider) continue;
            Transform t=collider.transform;
            string group;
            while(t && !TryParsePhysicsStairGroup(t.name,out group)) t=t.parent;
            if (!t || !t.parent || !TryParsePhysicsStairGroup(t.name,out group)) continue;
            int count=0;
            Transform root=t.parent;
            for(int i=0;i<root.childCount;i++) {
                if (TryParsePhysicsStairGroup(root.GetChild(i).name,out string sibling) &&
                    sibling==group) count++;
            }
            if (count>0) return Mathf.Clamp(count,1,8);
        }
        if (active != null && active.partCount>0) return Mathf.Clamp(active.partCount,1,8);
        return 1;
    }

    static bool TryParsePhysicsStairGroup(string name,out string group) {
        group=null;
        if (string.IsNullOrEmpty(name) ||
            !name.EndsWith("_Physics",StringComparison.Ordinal)) return false;
        string core=name.Substring(0,name.Length-"_Physics".Length);
        int i=core.LastIndexOf('_');
        if (i<=0 || !int.TryParse(core.Substring(i+1),out int childIndex)) return false;
        group=core.Substring(0,i);
        return group.StartsWith("StairWay",StringComparison.Ordinal);
    }

    void BeginBveWave() {
        if (mainBveMode || !driveVisualWithKnotSonar || !HasSource ||
            !bveProbeValid || (bveActive && !bveExiting) ||
            !trackingValid || active == null || trackingGroup != active.group ||
            !observationStationReady) return;
        FinishWaveEnergyBudget("NEW_GROUP");
        Vector3 p = SourcePhysicsPoint, v = SourcePhysicsVelocity;
        Vector3 planar = Vector3.ProjectOnPlane(v, Vector3.up);
        if (planar.sqrMagnitude > .01f) bveHeading = planar.normalized;
        Vector3 entryT = bveHeading + Vector3.up * bveGradient;
        if (entryT.sqrMagnitude < E) { bveArmed = false; return; }
        bveT = entryT.normalized;
        Vector3 n = trackingValid && active != null
            ? UnmapStandaloneDirection(trackingNormal)
            : Vector3.ProjectOnPlane(Vector3.up, bveT);
        if (n.sqrMagnitude < E) { bveArmed = false; return; }
        bveN = n.normalized;
        sonarWaveNormal = MapStandaloneDirection(bveN).normalized;
        bvePhysicsStairMultiplier = ResolveBvePhysicsMultiplier();
        bveCycleMultiplier = bvePhysicsStairMultiplier;
        bveCycleGroup = active != null ? active.group : bveBeginCount;
        bveCycleSectionLength = bveSectionLength * bveCycleMultiplier;
        bveElapsed = bveExitElapsed = bveTraveled = bveProgress = bveFlatElapsed = 0f;
        bveResidualOffset = bveTerminalOffset = bveRawOffset = sonarWaveHeight = 0f;
        bveBaseOffset = bveCurrentAmplitude = 0f;
        bvePreviousPhysicsPoint = p;
        bveWaveIndex = bveWaveType = -1;
        bveFirstApexLogged = false;
        bveNextStateLog = Time.fixedTime;
        bveLastOffset = bveLastOffsetSpeed = 0f;
        bveWaveLocal01 = 0f;
        nonInertialOffset = nonInertialTarget = 0f;
        nonInertialVelocity = 0f;
        nonInertialHadInput = false;
        terrainFrameGroup=-1;
        terrainFrameAcceleration=terrainFrameOmega=terrainLastTravelSpeed=0f;
        niFrameHistoryValid=niFrameHasVelocitySample=false;
        niFrameTransportAcceleration=niFrameTransportRaw=0f;
        niRegionLogged=niGroupLogged=-1;
        // Only a REAL measured comoving velocity can excite the initial wave.
        if (trackingValid && active != null && active.group == bveCycleGroup) {
            nonInertialVelocity = Mathf.Clamp(trackingPhysicalSpeed * nonInertialEntryGain,
                -nonInertialMaxSpeed, nonInertialMaxSpeed);
            nonInertialLastInputVelocity = trackingPhysicalSpeed;
            nonInertialHadInput = true;
        }
        bveActive = true;
        bveExiting = bveArmed = false;
        bveLastBegin = Time.fixedTime;
        bveBeginCount++;
        ResetBvePresentation(0f);
        if (debugBveTransitions || debugBveParityEvents)
            Debug.Log($"[IFSW][NI_BEGIN] t={Time.fixedTime:F3} group={bveCycleGroup} " +
                $"length={bveCycleSectionLength:F3} multiplier={bveCycleMultiplier} " +
                $"entryRelSpeed={nonInertialVelocity:F4} tracking={trackingValid} " +
                $"N={bveN:F4} probe={bveProbeValid}", this);
    }

    void EndBveWave() {
        if (!bveActive || bveExiting) return;
        bveExiting = true;
        bveExitElapsed = 0f;
        if (debugBveTransitions)
            Debug.Log($"[IFSW][NI_END] group={bveCycleGroup} progress={bveProgress:F3}", this);
    }

    void ResetBveWave(bool resetPresentation) {
        FinishWaveEnergyBudget("RESET");
        terrainFrameGroup = -1;
        terrainFrameAcceleration = terrainFrameOmega = terrainLastTravelSpeed = 0f;
        bveActive=bveExiting=false;
        niFrameHistoryValid=niFrameHasVelocitySample=false;
        niFrameTransportAcceleration=niFrameTransportRaw=0f;
        niRegionLogged=niGroupLogged=-1;
        bveBaseOffset=bveResidualOffset=bveTerminalOffset=bveRawOffset=sonarWaveHeight=0f;
        nonInertialOffset = nonInertialVelocity = nonInertialTarget = 0f;
        nonInertialHadInput = false;
        bveWaveLocal01=1f;
        bveWaveIndex=bveWaveType=-1;
        bveFirstApexLogged=false;
        bveCycleGroup=-1;
        bveCycleMultiplier=0;
        bveCycleSectionLength=0f;
        if (resetPresentation) ResetBvePresentation(0f);
    }

    // Exactly BVE topology: 123 before F; then repeated 23 chunks.
    void ResolveBveCycle(float progress,out float u,out float waveLength,out int priorType) {
        int repeat=Mathf.Max(0,bveCycleMultiplier-1);
        if (repeat==0 || progress<bveFBoundary) {
            float zone=repeat==0?1f:bveFBoundary;
            float x=Mathf.Clamp01(progress/zone)*3f;
            int k=Mathf.Clamp(Mathf.FloorToInt(x),0,2);
            bveWaveIndex=k;bveWaveType=k;
            priorType=Mathf.Max(0,k-1);
            u=k==2 && x>=3f?1f:Mathf.Clamp01(x-k);
            waveLength=bveCycleSectionLength*zone/3f;
        } else {
            float x=Mathf.Clamp01((progress-bveFBoundary)/(1f-bveFBoundary))*repeat;
            // Exact chunk boundaries belong to their LEFT chunk (BVE convention).
            int chunk=Mathf.Clamp(Mathf.CeilToInt(x)-1,0,repeat-1);
            float local=Mathf.Clamp((x-chunk)*2f,0f,2f);
            int k=local>=2f?1:Mathf.Clamp(Mathf.FloorToInt(local),0,1);
            bveWaveIndex=3+2*chunk+k;
            bveWaveType=k+1;
            priorType=k==0?2:1;
            u=k==1 && local>=2f?1f:Mathf.Clamp01(local-k);
            waveLength=bveCycleSectionLength*(1f-bveFBoundary)/(repeat*2f);
        }
    }

    void OnDrawGizmosSelected() {
        if (contactListenerOnly || !drawKnotSonarGizmos) return;
        for (int i = 0; i < sonarAnchorCount; ++i) {
            Gizmos.color = sonarAnchorIsObservation[i] ? new Color(.95f,.25f,.8f,.35f) :
                           new Color(.2f,.9f,1f,.55f);
            Gizmos.DrawWireSphere(sonarAnchors[i], knotSonarRadius);
        }
    }

    void LateUpdate() {
        if (contactListenerOnly) return;
        // V26 is self-contained: external BVE is never an input.
        // The five-spline lifecycle continues independently.
        UpdateVisualRootPose();
        if(Time.time>=nextScan) {
            nextScan=Time.time+rendererScanSeconds;
            ScanStage();
        }
        // Kept as a read-only compatibility branch for old scenes, but Start()
        // never acquires old BVE ownership in V20.
        // Independent-only: never read an external BVE component at runtime.
        bool mainAvailable=false;
        if (mainAvailable != mainBveMode) {
            mainBveMode=mainAvailable;
            displayedBall=null;
            hasPreviousBall=false; // ownership change is not a teleport
            ResetBveWave(true);
            sourcePositionValid=false;
            bveFrameReady=false;
            if (ownedVisualBall) ownedVisualBall.gameObject.SetActive(!mainBveMode);
            UpdateVisualStatus();
        }
        if (mainBveMode) {
            if (!displayedBall || displayedBall == ownedVisualBall) {
                displayedBall=FindMainBveDisplay();
                if (displayedBall) UpdateVisualStatus();
            }
        } else {
            if (!visualMotionSource) ResolveVisualMotionSource();
            if (!displayedBall || !displayedBall.gameObject.activeInHierarchy) {
                EnsureIndependentVisualBall();
                if (displayedBall) UpdateVisualStatus();
            }
        }
        // Run the shorter version's EXACT pair lifecycle in VISUAL world space.
        // Main BVE output is an observation, NEVER a transform we write.
        // When its output is not yet created, the independent visual-motion
        // reference still permits five-spline creation and state transitions.
        bool hasLifecyclePoint = false;
        Vector3 ball = Vector3.zero;
        if (mainBveMode && displayedBall) {
            ball = displayedBall.position;
            hasLifecyclePoint = true;
        } else if (TryGetVisualMotionPoint(out Vector3 visualPoint)) {
            ball = visualPoint;
            hasLifecyclePoint = true;
        } else if (displayedBall && displayedBall != ownedVisualBall) {
            ball = displayedBall.position;
            hasLifecyclePoint = true;
        }
        if (hasLifecyclePoint && pairs.Count>0) {
            Vector3 observed = displayedBall ? displayedBall.position : ball;
            moveStep=hasPreviousBall ? ball-previousBall : Vector3.zero;
            if(hasPreviousBall) {
                float dt=Mathf.Max(.001f,Time.time-previousSampleTime);
                float v=Vector3.ProjectOnPlane(moveStep,Vector3.up).magnitude/dt;
                if(v<80f) measuredSpeed=Mathf.Lerp(measuredSpeed,v,1f-Mathf.Exp(-6f*dt));
            }
            previousBall=ball;
            previousSampleTime=Time.time;
            hasPreviousBall=true;
            ProcessPair(ball,observed);
            exitElapsed = state=="CORNER_HOLD" ? exitElapsed+Time.deltaTime : 0f;
        }
        // BVE renders afterwards, so its own output cannot change the
        // SAME-FRAME stair entry/exit decision from the short lifecycle.
        if (!mainBveMode && displayedBall)
            UpdateDrivenVisual();
        // Observe the FINAL rendered Transform, after the standalone visual filter.
        // This is also the same reference measurement when Main BVE is enabled.
        LogBallTrajectory();
        // Display-only safety: clearing knots must also remove the tube mesh.
        // This does NOT change the shorter code's creation/deletion conditions.
        FlushOwnedSplineMeshes();
    }

    // Read-only trajectory recorder. Logs actual display output, not just oscillator Y.
    // Works with Main BVE enabled OR the standalone IFSW wave implementation.
    // Both modes share the same [IFSW][BALL_TRACE] column names for direct comparison.
    void LogBallTrajectory() {
        if (!debugBallTrajectory) return;
        bool stageActive = active != null && (state == "ACTIVE" || state == "CORNER_HOLD");
        bool running = mainBveMode ? stageActive : bveActive;
        if (!running || !displayedBall || !displayedBall.gameObject.activeInHierarchy) {
            if (ballTraceRunning) {
                FinishBallTraceWave("STOP");
                Debug.Log($"[IFSW][BALL_END] t={Time.time:F3} fixed={Time.fixedTime:F3} " +
                    $"mode={(ballTraceLastMainMode ? "MAIN_BVE" : "STANDALONE")} group={ballTraceGroup}", this);
            }
            ballTraceRunning = false;
            ballTraceWave = -1;
            ballTracePreviousHeightValid = false;
            ballTraceEntryHeightValid = false;
            return;
        }

        int group = mainBveMode ? (active != null ? active.group : -1) : bveCycleGroup;
        bool newCycle = !ballTraceRunning || mainBveMode != ballTraceLastMainMode ||
            group != ballTraceGroup;
        if (newCycle) {
            if (ballTraceRunning) FinishBallTraceWave("GROUP_CHANGE");
            ballTraceGroup = group;
            ballTraceLastMainMode = mainBveMode;
            ballTraceRunning = true;
            ballTraceWasExiting = false;
            ballTraceWave = -1;
            ballTraceSamples = 0;
            ballTracePreviousHeightValid = false;
            ballTraceEntryHeightValid = false;
            ballTraceNextTime = 0f;
            Debug.Log($"[IFSW][BALL_BEGIN] t={Time.time:F3} fixed={Time.fixedTime:F3} " +
                $"mode={(mainBveMode ? "MAIN_BVE" : "STANDALONE")} group={group} " +
                $"length={(mainBveMode ? (active != null ? active.length : 0f) : bveCycleSectionLength):F3} " +
                $"regions={(mainBveMode ? waveCount : 3 + 2 * Mathf.Max(0, bveCycleMultiplier - 1))} " +
                $"engine={(mainBveMode ? "EXAMPLE_BVE" : "NONINERTIAL")}", this);
        }

        // Coordinate-system invariant: compare the displayed ball against the
        // same VISUAL-side carrier, in the currently observed staircase frame.
        bool hasBase = TryGetVisualMotionPoint(out Vector3 carrier);
        Vector3 visualPosition = displayedBall.position;
        Vector3 tangent = Vector3.forward;
        Vector3 normal = mainBveMode ? Vector3.up : sonarWaveNormal;
        float station = -1f, sectionProgress = -1f;
        if (active != null && active.length > E && active.axis.Count >= 2) {
            station = StationOn(active, hasBase ? carrier : visualPosition, out _, out _);
            sectionProgress = Mathf.Clamp01(station / active.length);
            PointOn(active, station, out tangent);
            normal = Vector3.ProjectOnPlane(Vector3.up, tangent).normalized;
        }
        if (tangent.sqrMagnitude <= E) tangent = Vector3.forward;
        if (normal.sqrMagnitude <= E) normal = Vector3.up;
        Vector3 side = Vector3.Cross(normal, tangent).normalized;
        if (side.sqrMagnitude <= E) side = Vector3.right;
        Vector3 relative = visualPosition - carrier;
        float heightN = hasBase ? Vector3.Dot(relative, normal) : float.NaN;
        float alongT = hasBase ? Vector3.Dot(relative, tangent) : float.NaN;
        float crossB = hasBase ? Vector3.Dot(relative, side) : float.NaN;
        if (hasBase && !ballTraceEntryHeightValid) {
            ballTraceEntryHeight = heightN;
            ballTraceEntryHeightValid = true;
        }
        float entryRelativeN = hasBase && ballTraceEntryHeightValid
            ? heightN - ballTraceEntryHeight : float.NaN;

        // These legacy 'WAVE' logs now measure path REGIONS; the physical
        // oscillator has no prescribed per-region bounce or amplitude.
        if (!mainBveMode && bveWaveIndex >= 0) {
            if (ballTraceWave != bveWaveIndex) {
                FinishBallTraceWave("WAVE_CHANGE");
                ballTraceWave = bveWaveIndex;
                ballTraceSamples = 0;
                ballTraceMinHeight = float.PositiveInfinity;
                ballTraceMaxHeight = float.NegativeInfinity;
                ballTraceMinRaw = float.PositiveInfinity;
                ballTraceMaxRaw = float.NegativeInfinity;
                Debug.Log($"[IFSW][BALL_REGION] t={Time.time:F3} fixed={Time.fixedTime:F3} " +
                    $"group={group} wave={ballTraceWave} type={bveWaveType + 1} " +
                    $"progress={bveProgress:F5} u={bveWaveLocal01:F5} " +
                    $"amplitude={bveCurrentAmplitude:F5}", this);
            }
            if (hasBase) {
                ballTraceSamples++;
                if (heightN > ballTraceMaxHeight) {
                    ballTraceMaxHeight = heightN;
                    ballTraceMaxPhase = bveProgress;
                    ballTracePeakWorld = visualPosition;
                }
                ballTraceMinHeight = Mathf.Min(ballTraceMinHeight, heightN);
            }
            ballTraceMinRaw = Mathf.Min(ballTraceMinRaw, bveRawOffset);
            ballTraceMaxRaw = Mathf.Max(ballTraceMaxRaw, bveRawOffset);
        }
        if (!mainBveMode && bveExiting && !ballTraceWasExiting)
            Debug.Log($"[IFSW][BALL_EXIT] t={Time.time:F3} group={group} " +
                $"wave={bveWaveIndex} raw={bveRawOffset:F5} presented={bvePresentedOffset:F5}", this);
        ballTraceWasExiting = !mainBveMode && bveExiting;

        if (Time.time < ballTraceNextTime) return;
        ballTraceNextTime = Time.time + Mathf.Max(.02f, ballTrajectoryLogInterval);
        float visualSpeedN = hasBase && ballTracePreviousHeightValid
            ? (heightN - ballTracePreviousHeight) / Mathf.Max(.0001f, Time.time - ballTracePreviousTime)
            : float.NaN;
        ballTracePreviousHeight = heightN;
        ballTracePreviousHeightValid = hasBase;
        ballTracePreviousTime = Time.time;
        string raw = mainBveMode ? "NA" : bveRawOffset.ToString("F5");
        string presented = mainBveMode ? "NA" : bvePresentedOffset.ToString("F5");
        string baseWave = mainBveMode ? "NA" : bveBaseOffset.ToString("F5");
        string residual = mainBveMode ? "NA" : bveResidualOffset.ToString("F5");
        string terminal = mainBveMode ? "NA" : bveTerminalOffset.ToString("F5");
        string amplitude = mainBveMode ? "NA" : bveCurrentAmplitude.ToString("F5");
        Debug.Log($"[IFSW][BALL_TRACE] t={Time.time:F3} fixed={Time.fixedTime:F3} " +
            $"mode={(mainBveMode ? "MAIN_BVE" : "STANDALONE")} group={group} stage={state} " +
            $"station={station:F4} section01={sectionProgress:F5} " +
            $"wave={(mainBveMode ? -1 : bveWaveIndex)} type={(mainBveMode ? -1 : bveWaveType + 1)} " +
            $"wave01={(mainBveMode ? -1f : bveWaveLocal01):F5} " +
            $"engine={(mainBveMode ? "MAIN_BVE" : "NONINERTIAL")} " +
            $"modelProgress={(mainBveMode ? -1f : bveProgress):F5} " +
            $"physicalTravel={(mainBveMode ? -1f : bveTraveled):F4} amplitude={amplitude} " +
            $"baseWave={baseWave} residual={residual} terminal={terminal} " +
            $"raw={raw} presented={presented} " +
            $"baseValid={hasBase} displayN={heightN:F5} " +
            $"entryRelativeN={entryRelativeN:F5} displaySpeedN={visualSpeedN:F5} " +
            $"displayT={alongT:F5} displayB={crossB:F5} " +
            $"displayPos={visualPosition:F4} carrierPos={(hasBase ? carrier.ToString("F4") : "NA")} " +
            $"frameN={normal:F4} " +
            $"solverN={(mainBveMode ? "NA" : bveN.ToString("F4"))} " +
            $"probe={(mainBveMode ? "NA" : bveProbeValid.ToString())} " +
            $"filter={(mainBveMode ? "MAIN_BVE" : !softenVisualWave ? "OFF" : adaptiveVisualFilter ? "ADAPTIVE" : "SMOOTH")}", this);
    }

    void FinishBallTraceWave(string reason) {
        if (ballTraceWave < 0) return;
        if (ballTraceSamples > 0) {
            Debug.Log($"[IFSW][BALL_REGION_RESULT] t={Time.time:F3} group={ballTraceGroup} " +
                $"wave={ballTraceWave} reason={reason} samples={ballTraceSamples} " +
                $"peakDisplayN={ballTraceMaxHeight:F5} troughDisplayN={ballTraceMinHeight:F5} " +
                $"peakToTrough={ballTraceMaxHeight - ballTraceMinHeight:F5} " +
                $"peakRelativeN={(ballTraceEntryHeightValid ? ballTraceMaxHeight - ballTraceEntryHeight : float.NaN):F5} " +
                $"troughRelativeN={(ballTraceEntryHeightValid ? ballTraceMinHeight - ballTraceEntryHeight : float.NaN):F5} " +
                $"peakModelProgress={ballTraceMaxPhase:F5} peakWorld={ballTracePeakWorld:F4} " +
                $"maxRaw={ballTraceMaxRaw:F5} minRaw={ballTraceMinRaw:F5}", this);
        }
        ballTraceWave = -1;
        ballTraceSamples = 0;
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
                            BuildRails(active); // paths[0..2] only; no SplineContainer writes
                            beforePublishReplans++;
                        }
                        if(state!="CORNER_HOLD") BuildFuture(active);
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
        pair.downhill=maxSlope>=minimumDownhillDegrees && pair.axis.Count>=2;
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

    // The next pair is selected by STAGE order first; proximity is only a gate.
    // No global nearest-chain switch, no +12 part index limit, no 5m join split.
    int FindCandidate(Vector3 ball) {
        for(int i=0;i<pairs.Count;i++) {
            Pair pair=pairs[i];
            if(!pair.downhill || pair.group<=lastCompletedGroup ||
               (active!=null && i<pairIndex)) continue;
            float station=StationOn(pair,ball,out Vector3 foot,out float across);
            Vector3 toEntry=pair.entry-ball;
            float gap=Vector3.ProjectOnPlane(toEntry,Vector3.up).magnitude;
            float projected=Vector3.Dot(toEntry,pair.flatHeading);
            // A clamped closest point at the far EXIT of a stale stair is not
            // evidence that we have entered that stair. This caused the t=0
            // StairWay1 false positive in the V7 trace.
            bool entered=station>.12f && station<pair.length-.15f &&
                across<maxEntryCrossTrack && Mathf.Abs(ball.y-foot.y)<3.5f;
            bool approaching=gap<=lookAheadDistance && projected>=-1f &&
                Mathf.Abs(ball.y-pair.entry.y)<4f &&
                Mathf.Abs(Vector3.Dot(toEntry,Side(pair.flatHeading)))<maxEntryCrossTrack+laneHalfWidth;
            if(entered || approaching) return i;
        }
        return -1;
    }

    // Four moments: (1) flat A+B precompute, (2) stair entry publish,
    // (3) ACTIVE without rail rebuild, (4) hold rails until NEXT FLAT.
    void ProcessPair(Vector3 ball, Vector3 observed) {
        int candidate=FindCandidate(ball);
        if(active==null) {
            if(candidate>=0) Prepare(candidate,ball);
            else return;
        }
        if(active==null) return;
        if(state=="PREPARED" && !cornerBuilt && candidate>=0 && candidate>pairIndex) {
            // A nearer later pair won the chronological/geometry gate while
            // the prior candidate never reached its entry boundary.
            Finish("PRE_ENTRY_PAIR_CHANGED");
            Prepare(candidate,ball);
        }
        if(active==null) return;
        Pair p=active;
        float geometricStation=StationOn(p,ball,out Vector3 point,out float crossTrack);
        stairStation=geometricStation/Mathf.Max(.01f,p.length)*sectionLength;
        entryDistance=Vector3.ProjectOnPlane(p.entry-ball,Vector3.up).magnitude;
        // Publish against the STAIR ENTRY PLANE in X/Z, not StationOn's
        // vertical-weighted nearest point. During an A+B right-angle turn,
        // the ball may be high/low or laterally offset at that plane; V12's
        // height gate delayed publication by as much as several meters.
        // A small lead allows one discrete frame of travel at game speed.
        Vector3 entryHeading=Flat(p.axis[1]-p.axis[0]);
        Vector3 entryDelta=Vector3.ProjectOnPlane(ball-p.entry,Vector3.up);
        float entrySignedXZ=Vector3.Dot(entryDelta,entryHeading);
        float entryRadialXZ=entryDelta.magnitude;
        float entryRadius=Mathf.Max(5f,maxEntryCrossTrack+laneHalfWidth);
        bool crossedEntry=entrySignedXZ>=-cornerEntryLeadMeters &&
            entrySignedXZ<=Mathf.Min(8f,p.length*.65f) &&
            entryRadialXZ<=entryRadius;
        if(state=="PREPARED" && !cornerBuilt && crossedEntry) {
            if(paths[Center].Count>=2) {
                for(int rail=Out;rail<=Inside;rail++) WriteSpline(rail);
                cornerBuilt=true;
                LifecycleEvent("CORNER_PUBLISHED");
            }
        }
        // Preserve V10 A+b entrance timing and wave observations unchanged.
        bool geometricallyInside=geometricStation>.12f &&
            geometricStation<p.length-.10f && crossTrack<maxEntryCrossTrack &&
            Mathf.Abs(ball.y-point.y)<3.5f;

        if(state=="PREPARED" && geometricallyInside) {
            state="ACTIVE";
            LifecycleEvent("STAIR_ACTIVE");
            // Wave entry is handled by FixedUpdate's physical slope probe,
            // not by the visual Spline entry timing.
            // Fix the lane based on the actual STAIR frame on arrival. Previous
            // pre-entry measurements at a right-angle turn are only provisional.
            float stairLane=Vector3.Dot(ball-point,p.side);
            if(Finite(stairLane) && Mathf.Abs(stairLane)<=laneHalfWidth+1.5f) {
                lateralOffset=stairLane;
                entryLaneTrusted=true;
            }
            // Freeze the entry model before any calibration so later lane syncs
            // do not accidentally change the vertical Hermite phase.
            futureModelSpeed=Mathf.Max(1f,measuredSpeed>1f?measuredSpeed:fallbackSpeed);
            // The first actual observation defines support along the stair's
            // local N. A world-Y flat-board gap is NOT a stair-normal offset.
            PointOn(p,geometricStation,out Vector3 entryTangent);
            Vector3 entryNormal=Vector3.ProjectOnPlane(Vector3.up,entryTangent).normalized;
            if(entryNormal.sqrMagnitude<E) entryNormal=Vector3.up;
            float entryU=Mathf.Clamp01(geometricStation/Mathf.Max(.01f,p.length));
            float g = inSubject && displayedBall
                ? Mathf.Max(.1f, -Vector3.Dot(Physics.gravity, entryNormal))
                : Mathf.Max(.1f, Physics.gravity.magnitude);
            float expectedWave=WaveAt(entryU,futureModelSpeed,g);
            float normalSupport=Vector3.Dot(observed-point,entryNormal)-expectedWave;
            if(Finite(normalSupport) && Mathf.Abs(normalSupport)<=4f) {
                supportHeight=normalSupport;
                futureNormalAnchored=true;
            }
            pendingLaneDeltas.Clear();
            lastLaneSampleStation = -1f;
            // The first measured entry gets ONE independent forecast energy
            // snapshot; no Later Wave Observed amplitude is ever copied.
            futureEnergyFrozen=false;
            BuildFuture(p);
        }
        if(state=="PREPARED") return;

        Vector3 tangent;
        PointOn(p,geometricStation,out tangent);
        Vector3 observedHeading=Flat(moveStep);
        Vector3 expected=Flat(tangent);
        float headingDot=observedHeading.sqrMagnitude>E?Vector3.Dot(observedHeading,expected):-1f;
        if(headingDot>=postTurnHeadingDot) headingStableFrames++;
        else headingStableFrames=0;
        isTurnBusy=hasRightAngleTurn && headingStableFrames<3;
        float sidewaysDistance=Mathf.Abs(Vector3.Dot(ball-point,Side(expected)));
        float sidewaysStep=Mathf.Abs(Vector3.Dot(moveStep,Side(expected)));
        // A constant lateral lane (even beyond +/-laneHalfWidth) is NOT by
        // itself a detour. V9 wrongly blocked lane sync on StairWay9/10 here.
        isDetouring=(sidewaysStep>.08f && headingDot<.82f) ||
            (sidewaysDistance>laneHalfWidth+1.5f && headingDot<.92f);

        if(state=="ACTIVE" && geometricallyInside) {
            // Future retains its INDEPENDENT entry-state prediction.
            // Never calibrate it from this Observed display or from BVE.
            if(paths[Observed].Count==0 || Vector3.Distance(previousObserved,observed)>=observedSpacing) {
                paths[Observed].Add(observed);
                previousObserved=observed;
                if(paths[Observed].Count>maxObservedKnots) paths[Observed].RemoveAt(0);
                WriteSpline(Observed);
            }
            if(!laneSynced)
                GatherLaneCalibration(observed, geometricStation, expected, headingDot);
            lastObservedStation=Mathf.Max(lastObservedStation,geometricStation);
        }

        if(state=="ACTIVE" && lastObservedStation>=p.length*.70f) {
            Vector3 exitDirection=Flat(p.exit-p.axis[p.axis.Count-2]);
            bool pastExit=Vector3.Dot(ball-p.exit,exitDirection)>=exitConfirmDistance;
            // If the stage turns immediately onto its next flat, the player
            // may already be on that flat before crossing the old exit axis.
            if(pastExit || IsOnFollowingFlat(p,ball)) {
                exitReached=true;
                state="CORNER_HOLD";
                LifecycleEvent("WAVES_CLEAR_AT_EXIT");
                // Visual Spline exit does not end the independent physical wave.
                sonarAnchorCount=sonarNearbyKnots=0;
                // A+b exists only on the actual downhill stair. Keep A+B on
                // the next flat, but NEVER display a stale Future for minutes.
                paths[Observed].Clear(); WriteSpline(Observed);
                paths[Future].Clear(); futureStations.Clear(); WriteSpline(Future);
            }
        }
        // The flat, not a distance threshold beyond the stair, owns the lifetime.
        if(state=="CORNER_HOLD" && IsOnFollowingFlat(p,ball)) {
            Finish("NEXT_FLAT_ARRIVAL");
            if(candidate>=0 && pairs[candidate].group>p.group) Prepare(candidate,ball);
            return;
        }
        // If the player is already on a later StairWay, the intervening flat
        // has necessarily been passed. This also handles a narrow/rotated flat
        // which never satisfied the old 1.6m flat proximity gate.
        if(state=="CORNER_HOLD") {
            int reached=FindReachedSuccessor(ball);
            if(reached>=0) {
                Finish("NEXT_STAIR_REACHED");
                Prepare(reached,ball);
                return;
            }
        }
    }

    // A successor counts as reached only when the display point is INSIDE
    // its own visible, downhill stair geometry (not merely near its entrance).
    // This never reads an external component/SplineContainer.
    int FindReachedSuccessor(Vector3 ball) {
        if(active==null) return -1;
        for(int i=pairIndex+1;i<pairs.Count;i++) {
            Pair next=pairs[i];
            if(!next.downhill || next.axis.Count<2) continue;
            float s=StationOn(next,ball,out Vector3 foot,out float cross);
            if(s>.15f && s<next.length-.15f &&
               cross<maxEntryCrossTrack && Mathf.Abs(ball.y-foot.y)<3.5f)
                return i;
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

    void Prepare(int index,Vector3 ball) {
        Pair pair=pairs[index];
        if(!pair.downhill || pair.axis.Count<2) return;
        active=pair;
        pairIndex=index;
        stairGroup=pair.group;
        state="PREPARED";
        isDownhillStair=true;
        hasFlatBefore=pair.flat>=0;
        hasRightAngleTurn=pair.turn90;
        isDetouring=isTurnBusy=laneSynced=false;
        cornerBuilt=exitReached=entryLaneTrusted=hasEntryCornerPoint=false;
        hasExitCornerPoint=false;
        signedEntryYaw=pair.entryYaw;
        signedExitYaw=pair.exitYaw;
        beforePublishReplans=0;
        railTurnCount=0;
        futureLateralShift=0f;
        lastObservedStation=0f;
        headingStableFrames=0;
        futureNormalAnchored=false;
        futureAmplitudeReference=0f;
        futureEnergyFrozen=false;
        pendingLaneDeltas.Clear();
        lastLaneSampleStation=-1f;
        futureModelSpeed=Mathf.Max(1f,measuredSpeed>1f?measuredSpeed:fallbackSpeed);
        lastReason="";
        waveRiseSpeed = riseSpeed;
        if (inSubject && physicsFrame && visualFrame) {
            PointOn(pair, .01f, out Vector3 entryTangent);
            Vector3 entryNormal = Vector3.ProjectOnPlane(Vector3.up, entryTangent).normalized;
            float vN = Vector3.Dot(ToVisualDirection(inSubject.velocity), entryNormal);
            if (vN > .1f) waveRiseSpeed = vN;
        }
        multiplier=multiplierOverride>0?multiplierOverride:Mathf.Clamp(pair.partCount,1,8);
        waveCount=3+2*(multiplier-1);
        sectionLength=nominalStairLength*multiplier;
        paths[Observed].Clear();
        WriteSpline(Observed);
        Vector3 foot;
        float lateral;
        if(pair.flat>=0) {
            Piece flat=pieces[pair.flat];
            Vector3 segment=flat.b-flat.a;
            Vector3 projected=Flat(segment);
            float t=projected.sqrMagnitude>E
                ?Mathf.Clamp01(Vector3.Dot(Vector3.ProjectOnPlane(ball-flat.a,Vector3.up),projected)/
                    Mathf.Max(.01f,Vector3.ProjectOnPlane(segment,Vector3.up).magnitude)) : 0f;
            foot=Vector3.Lerp(flat.a,flat.b,t);
            Vector3 flatSide=Side(pair.flatHeading);
            float yaw=Vector3.SignedAngle(pair.flatHeading,pair.heading,Vector3.up);
            Vector3 rotatedSide=Quaternion.AngleAxis(yaw,Vector3.up)*flatSide;
            float sign=Vector3.Dot(rotatedSide,pair.side)>=0f?1f:-1f;
            float measuredLane=Vector3.Dot(ball-foot,flatSide);
            // At a 90-degree turn the entry can be close by in 3D while the
            // display still belongs to a previous arc. Never clamp an invalid
            // lateral estimate to +/-2.5m and call it a measurement.
            float horizontalError=Vector3.ProjectOnPlane(ball-foot,Vector3.up).magnitude;
            bool trusted=horizontalError<=laneHalfWidth+maxEntryCrossTrack &&
                Mathf.Abs(ball.y-foot.y)<2.5f && Mathf.Abs(measuredLane)<=laneHalfWidth+1.5f;
            lateral=trusted?measuredLane*sign:0f;
            entryLaneTrusted=trusted;
            } else {
            StationOn(pair,ball,out foot,out _);
            lateral=Vector3.Dot(ball-foot,pair.side);
            entryLaneTrusted=false;
        }
        lateralOffset=Mathf.Clamp(lateral,-laneHalfWidth,laneHalfWidth);
        float support=ball.y-foot.y;
        // Support is estimated only on the preceding FLAT. No platform ->
        // conservative neutral height, never a feedback loop into the ball.
        supportHeight=(pair.flat>=0 && entryLaneTrusted && support>-.5f && support<3f) ? support : .75f;
        // A+B: precompute all 3 independent world-space rails while on the
        // flat. Do not publish to their SplineContainers until entry crossing.
        for(int i=Out;i<=Inside;i++) { paths[i].Clear(); WriteSpline(i); }
        BuildRails(pair);
        BuildFuture(pair);
        predictionRevisions++;
        sonarAnchorCount=sonarNearbyKnots=sonarGroundContacts=0;
        LifecycleEvent("PREPARED");
    }

    void Finish(string reason) {
        if(active!=null) {
            lastCompletedGroup=Mathf.Max(lastCompletedGroup,active.group);
        }
        active=null; pairIndex=-1; stairGroup=-1;
        state="IDLE";
        lastReason=reason;
        isDownhillStair=hasFlatBefore=hasRightAngleTurn=isDetouring=isTurnBusy=laneSynced=false;
        cornerBuilt=exitReached=entryLaneTrusted=hasEntryCornerPoint=false;
        hasExitCornerPoint=false;
        railTurnCount=0;
        lastObservedStation=0f;
        headingStableFrames=0;
        futureLateralShift=0f;
        futureModelSpeed=0f;
        futureNormalAnchored=false;
        futureAmplitudeReference=0f;
        futureEnergyFrozen=false;
        pendingLaneDeltas.Clear();
        lastLaneSampleStation=-1f;
        for(int i=0;i<Count;i++) { paths[i].Clear(); WriteSpline(i); }
        sonarAnchorCount=sonarNearbyKnots=sonarGroundContacts=0;
        sonarGeometryDetected=false;
        // ExampleBVE wave state outlives the A+b visual splines and ends at
        // its OWN traveled-distance boundary. Never reset here.
        LifecycleEvent("FINISH_" + reason);
    }

    // A+B: precompute three SHARP world-space rails; this method must never
    // publish a SplineContainer or re-run once Corner has been latched.
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

    // No per-type amplitude table: this is an estimated energy envelope,
    // used as a forcing magnitude in a future non-inertial ODE only.
    void ApplySharedFutureAmplitudes() {
        firstA=secondA=thirdA=Mathf.Max(.001f,futureAmplitudeReference);
    }

    // Estimate the SAME terrain-intensity model while still on the approach.
    // The first physically observed intensity supersedes this provisional
    // estimate ONCE after ACTIVE, never continuously or on every Wave.
    float PredictEntryTerrainAmplitude(Pair pair, Vector3 normal,Vector3 tangent) {
        Vector3 visualVelocity=ToVisualDirection(SourcePhysicsVelocity);
        float speed=Mathf.Max(0f,Vector3.Dot(visualVelocity,tangent));
        if(speed<.25f) speed=Mathf.Max(0f,futureModelSpeed);
        float slope=Mathf.Asin(Mathf.Clamp(-tangent.y,0f,1f))*Mathf.Rad2Deg;
        float frameAlignment=Mathf.Max(0f,Vector3.Dot(normal,Vector3.up));
        float contactWeight=.55f; // same missing-samples case as real terrain observer
        if (inSubject && TryTerrainHeight(SourcePhysicsPoint,out float groundY)) {
            Collider body=terrainBodyCollider;
            if(!body) body=inSubject.GetComponentInChildren<Collider>();
            if(body && body.enabled) {
                float gap=Mathf.Max(0f,body.bounds.min.y-groundY);
                contactWeight=Mathf.Clamp01(1f-gap/Mathf.Max(1.2f,probeHeight));
            }
        }
        return ComovingTerrainAmplitude(speed,slope,contactWeight,frameAlignment)*
            expressiveAmplitudeGain;
    }

    void BuildFuture(Pair pair) {
        paths[Future].Clear(); futureStations.Clear();
        if(!pair.downhill || pair.length<E) { WriteSpline(Future); return; }
        PointOn(pair, .01f, out Vector3 entryTangent);
        Vector3 entryNormal=Vector3.ProjectOnPlane(Vector3.up,entryTangent).normalized;
        if(entryNormal.sqrMagnitude<E) entryNormal=Vector3.up;
        // Spline geometry and wave normals are visual-world vectors.
        float g=Mathf.Max(.1f,-Vector3.Dot(Physics.gravity, entryNormal));
        if(!futureEnergyFrozen) {
            futureAmplitudeReference=PredictEntryTerrainAmplitude(pair,entryNormal,entryTangent);
            futureEnergyFrozen=true;
        }
        ApplySharedFutureAmplitudes(); // diagnostics, not wave-height commands
        float speed=Mathf.Max(1f,futureModelSpeed);
        int samples=Mathf.Max(futureMinimumKnots,waveCount*12+1);
        float predictedMinimum=float.PositiveInfinity, predictedMaximum=float.NegativeInfinity;
        for(int i=0;i<samples;i++) {
            float u=i/(float)(samples-1);
            Vector3 point=PointOn(pair,pair.length*u,out Vector3 tangent);
            Vector3 normal=Vector3.ProjectOnPlane(Vector3.up,tangent).normalized;
            if(normal.sqrMagnitude<E) normal=Vector3.up;
            Vector3 lateralSide=Side(tangent);
            if(lateralSide.sqrMagnitude<E) lateralSide=pair.side;
            float predictedWave=WaveAt(u,speed,g);
            predictedMinimum=Mathf.Min(predictedMinimum,predictedWave);
            predictedMaximum=Mathf.Max(predictedMaximum,predictedWave);
            Vector3 position=point+lateralSide*(lateralOffset+futureLateralShift) +
                normal*(supportHeight+predictedWave);
            paths[Future].Add(position);
            futureStations.Add(sectionLength*u);
        }
        WriteSpline(Future);
        if(debugExpressiveWave)
            Debug.Log($"[IFSW][NI_FUTURE_PREDICT] group={pair.group} " +
                $"independent=True speed={speed:F2} envelope={futureAmplitudeReference:F3} " +
                $"waveMin={predictedMinimum:F3} waveMax={predictedMaximum:F3} " +
                $"peakToTrough={predictedMaximum-predictedMinimum:F3} " +
                $"samples={samples} noExternalBVE=True",this);
    }

    // Calibrate Future B ONCE, using a short stable observation window.
    // A single frame near a 90-degree turn is not a trustworthy lane reading.
    void GatherLaneCalibration(Vector3 observed,float station,Vector3 heading,float headingDot) {
        if(active==null || laneSynced || futureStations.Count<2 || state!="ACTIVE") return;
        float sidewaysStep=Mathf.Abs(Vector3.Dot(moveStep,Side(heading)));
        if(headingDot < Mathf.Min(.84f,postTurnHeadingDot) || sidewaysStep>.20f ||
           station<.35f || station>Mathf.Min(active.length*.65f,12f) ||
           moveStep.magnitude>teleportLogDistance) return;
        float waveStation=station/Mathf.Max(.01f,active.length)*sectionLength;
        int k=1;
        while(k<futureStations.Count-1 && futureStations[k]<waveStation) k++;
        float t=Mathf.InverseLerp(futureStations[k-1],futureStations[k],waveStation);
        Vector3 futurePoint=Vector3.Lerp(paths[Future][k-1],paths[Future][k],t);
        float delta=Vector3.Dot(observed-futurePoint,Side(heading));
        if(!Finite(delta) || Mathf.Abs(delta)>maxLateralSync) return;
        if(station-lastLaneSampleStation<.12f) return;
        lastLaneSampleStation=station;
        if(pendingLaneDeltas.Count>0 &&
           Mathf.Abs(delta-pendingLaneDeltas[pendingLaneDeltas.Count-1])>laneStableTolerance) {
            pendingLaneDeltas.Clear(); // discard a turn/detour transition, never accumulate bad samples
        }
        pendingLaneDeltas.Add(delta);
        if(pendingLaneDeltas.Count>8) pendingLaneDeltas.RemoveAt(0);
        if(pendingLaneDeltas.Count<laneRequiredSamples) return;
        List<float> sorted=new List<float>(pendingLaneDeltas);
        sorted.Sort();
        float median=sorted[sorted.Count/2];
        SyncFutureLaterally(median);
    }

    void SyncFutureLaterally(float correction) {
        if(active==null || laneSynced) return;
        futureLateralShift+=correction;
        laneSynced=true;
        BuildFuture(active);
        pendingLaneDeltas.Clear();
    }


    static bool Finite(float x) => !float.IsNaN(x) && !float.IsInfinity(x);

    // Predict with the same SECOND-ORDER physical display oscillator, from
    // the independently measured entry speed and stage geometry only.
    // No live Observed samples, displayed-ball poses, BVE, or future knowledge
    // of actual collision impulses may change this forecast.
    void ResolveFuturePhase(float progress,out int region,out float u,out float L) {
        int repeat=Mathf.Max(0,multiplier-1);
        if(repeat==0 || progress<fBoundary) {
            float zone=repeat==0?1f:fBoundary;
            float x=Mathf.Clamp01(progress/zone)*3f;
            region=Mathf.Clamp(Mathf.FloorToInt(x),0,2);
            u=region==2 && x>=3f?1f:Mathf.Clamp01(x-region);
            L=Mathf.Max(.2f,sectionLength*zone/3f);
        } else {
            float x=Mathf.Clamp01((progress-fBoundary)/(1f-fBoundary))*repeat;
            int chunk=Mathf.Clamp(Mathf.CeilToInt(x)-1,0,repeat-1);
            float local=Mathf.Clamp((x-chunk)*2f,0f,2f);
            int index=local>=2f?1:Mathf.Clamp(Mathf.FloorToInt(local),0,1);
            region=3+2*chunk+index;
            u=index==1 && local>=2f?1f:Mathf.Clamp01(local-index);
            L=Mathf.Max(.2f,sectionLength*(1f-fBoundary)/(repeat*2f));
        }
    }

    float PredictNonInertialOffset(float progress,float speed,float gravity) {
        float targetDistance=Mathf.Clamp01(progress)*Mathf.Max(.2f,sectionLength);
        if(targetDistance<=.00001f) return 0f;
        float vTravel=Mathf.Max(1f,speed);
        float x=0f;
        // Entrance vN is a READ-ONLY physical snapshot, never an ExampleBVE kick.
        float v=Mathf.Clamp(waveRiseSpeed*nonInertialEntryGain,
            -nonInertialMaxSpeed,nonInertialMaxSpeed);
        int count=Mathf.Clamp(Mathf.CeilToInt(targetDistance/.15f),1,1500);
        float ds=targetDistance/count;
        float dt=ds/vTravel;
        float amp=futureAmplitudeReference;
        for(int i=0;i<count;i++) {
            float fraction=((i+.5f)*ds)/Mathf.Max(.2f,sectionLength);
            ResolveFuturePhase(fraction,out int region,out float localU,out float L);
            SpatialFrameDerivatives(localU,L,region,out float d1,out float d2);
            float coherence=Mathf.Clamp(L/Mathf.Max(.5f,terrainSpatialPitch),.65f,2.5f);
            float drive=-amp*d2*vTravel*vTravel/(L*L)*coherence*niCorrugationDriveGain;
            drive=Mathf.Clamp(drive,-niOscillatorAccelerationLimit,niOscillatorAccelerationLimit);
            float omega=Mathf.Max(2f*Mathf.PI*nonInertialFrequencyHz,
                2f*Mathf.PI*Mathf.Min(vTravel/L,terrainMaxFrameFrequencyHz));
            float a= -omega*omega*x-2f*niHighFrequencyDamping*omega*v+drive;
            v=Mathf.Clamp(v+a*dt,-niOscillatorMaxSpeed,niOscillatorMaxSpeed);
            x=Mathf.Clamp(x+v*dt,-nonInertialMaxOffset,nonInertialMaxOffset);
        }
        return x;
    }

    // In V26 Future is an independent integration of the SAME non-inertial
    // oscillator family.  No precomputed Hermite A1/A2/A3 waveform remains.
    float WaveAt(float progress,float speed,float gravity) {
        return PredictNonInertialOffset(progress,speed,gravity);
    }

    // World-space paths are the only source; each independently owned container
    // holds a local-space Spline. Linear tangents guarantee sharp 90-degree rails.
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
